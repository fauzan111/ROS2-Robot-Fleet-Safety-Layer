"""Render the demo video from the bag recorded by `make demo`.

    uv run python scripts/render_demo.py results/demo_bag --out docs/media

Left: the overhead Gazebo camera with each robot's protective zones drawn from its ground-truth
pose, coloured by its live safety state. Right: what the detector sees (robot1, or robot3 while
its camera is fogged) and one status row per robot. Captions come from the safety states and
the demo_director events in the same bag, so nothing in the video is staged after the fact.
Writes demo.mp4 (H.264) and a short demo.gif of the first person stop for the README.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import imageio_ffmpeg
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/fleet_bringup"))
from fleet_bringup.sim_assets import OVERHEAD, person_position, project_overhead  # noqa: E402

W, H, FPS = 1280, 720, 15
ROBOTS = ("robot1", "robot2", "robot3")
STOP_ZONE, WARN_ZONE = (1.0, 0.4), (2.0, 0.6)   # length ahead, half width (safety_monitor)
HEARTBEAT_TIMEOUT_S = 0.5
OOD_THRESHOLD = json.loads((ROOT / "src/fleet_ml/models/ood_stats.json").read_text())["threshold"]
# RGB colours (PIL) - converted for OpenCV where needed
BG, PANEL, TEXT, MUTED = (16, 18, 22), (28, 31, 37), (235, 237, 240), (150, 156, 166)
STATE_RGB = {"normal": (46, 204, 113), "slow": (243, 156, 18), "stop": (231, 76, 60),
             "no heartbeat": (190, 80, 230), "camera_timeout": (190, 80, 230),
             "lidar_timeout": (190, 80, 230), "starting": (120, 126, 136)}
LABEL = {"normal": "NORMAL", "slow": "SLOW 30%", "stop": "STOP", "no heartbeat": "NO HEARTBEAT",
         "camera_timeout": "CAMERA TIMEOUT", "lidar_timeout": "LIDAR TIMEOUT",
         "starting": "STARTING"}

EVENT_CAPTIONS = {
    "start": ("Three robots, three independent safety chains",
              "Two people cross robot1's corridor. Each robot only obeys its own speed gate."),
    "monitor_killed": ("robot2: safety monitor killed (SIGKILL)",
                       "No heartbeat for 0.5 s, so robot2's gate stops it. "
                       "robot1 and robot3 keep working."),
    "monitor_restarted": ("robot2: monitor restarted",
                          "Heartbeat is back, so robot2 drives on."),
    "fog_on": ("robot3: camera fogged",
               "The frame is out-of-distribution, so the detector can't be trusted: SLOW."),
    "fog_off": ("robot3: fog cleared",
                "OOD score back under the threshold: NORMAL."),
}
STATE_CAPTIONS = {
    "slow": ("robot1: person within 2 m ahead", "Warning zone: slow to 30%."),
    "stop": ("robot1: person in the stop zone", "Camera + lidar: STOP within ~0.5 s."),
}


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    for path, index in (("/System/Library/Fonts/HelveticaNeue.ttc", 1 if bold else 0),
                        ("/System/Library/Fonts/Helvetica.ttc", 1 if bold else 0),
                        ("/usr/share/fonts/truetype/dejavu/DejaVuSans"
                         + ("-Bold" if bold else "") + ".ttf", 0)):
        try:
            return ImageFont.truetype(path, size, index=index)
        except OSError:
            continue
    return ImageFont.load_default(size)


F = {"title": font(44, True), "h1": font(26, True), "h2": font(19, True), "body": font(17),
     "small": font(14), "chip": font(14, True), "big": font(30, True)}


def zone_polygon(x: float, y: float, yaw: float, length: float, half_w: float):
    """Rectangle ahead of the robot (base_link x forward) as world-frame corners."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py)
            for px, py in ((0, -half_w), (length, -half_w), (length, half_w), (0, half_w))]


def yaw_from_quaternion(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


@dataclass
class Live:
    """Latest value of every stream at the current video time."""
    overhead: np.ndarray | None = None
    cams: dict = field(default_factory=dict)
    pose: dict = field(default_factory=dict)            # robot -> (x, y, yaw)
    state: dict = field(default_factory=dict)           # robot -> (state, t)
    speed: dict = field(default_factory=dict)           # robot -> commanded m/s after the gate
    scale: dict = field(default_factory=dict)
    ood: dict = field(default_factory=dict)
    fogged: bool = False

    def robot_state(self, robot: str, t: float) -> str:
        st = self.state.get(robot)
        if st is None:
            return "starting"
        return "no heartbeat" if t - st[1] > HEARTBEAT_TIMEOUT_S else st[0]


def to_bgr(msg) -> np.ndarray:
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img.copy()


class Captions:
    """Director events win: a fault or fog caption stays up until the event ends; robot1's
    slow/stop transitions are narrated only in between (first few only)."""

    HOLD_UNTIL_NEXT = {"monitor_killed", "fog_on"}

    def __init__(self) -> None:
        self.current: tuple[str, str] | None = None
        self.until = -1.0
        self.event_active = False
        self.stops_shown = 0

    def event(self, t: float, kind: str) -> None:
        if kind in EVENT_CAPTIONS:
            self.current = EVENT_CAPTIONS[kind]
            self.event_active = kind in self.HOLD_UNTIL_NEXT
            self.until = math.inf if self.event_active else t + 6.0

    def robot1(self, t: float, state: str) -> None:
        if state in STATE_CAPTIONS and not self.event_active and t > self.until - 2.0 \
                and self.stops_shown < 4:
            self.current, self.until = STATE_CAPTIONS[state], t + 4.0
            self.stops_shown += 1

    def get(self, t: float):
        if t > self.until:
            self.current = None
        return self.current


def draw_overhead(live: Live, t: float, people: dict) -> np.ndarray:
    img = live.overhead.copy() if live.overhead is not None else np.zeros(
        (OVERHEAD["size_px"], OVERHEAD["size_px"], 3), np.uint8)
    layer = img.copy()
    for robot in ROBOTS:
        if robot not in live.pose:
            continue
        x, y, yaw = live.pose[robot]
        rgb = STATE_RGB[live.robot_state(robot, t)]
        bgr = rgb[::-1]
        for (length, hw), alpha_fill in ((WARN_ZONE, False), (STOP_ZONE, True)):
            pts = np.array([project_overhead(px, py) for px, py in
                            zone_polygon(x, y, yaw, length, hw)], np.int32)
            if alpha_fill:
                cv2.fillPoly(layer, [pts], bgr)
            cv2.polylines(img, [pts], True, bgr, 1, cv2.LINE_AA)
        u, v = project_overhead(x, y, 0.1)
        cv2.circle(img, (int(u), int(v)), 18, bgr, 2, cv2.LINE_AA)
    img = cv2.addWeighted(layer, 0.35, img, 0.65, 0)
    for p in people.values():
        px, py = person_position(p["waypoints"], t)
        u, v = project_overhead(px, py, 1.0)
        cv2.circle(img, (int(u), int(v)), 5, (255, 255, 255), -1, cv2.LINE_AA)
    return img


def compose(live: Live, t: float, t_start: float, speedup: float, caption, people) -> np.ndarray:
    frame = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(frame)
    # overhead view
    ov = cv2.resize(draw_overhead(live, t, people), (600, 600), interpolation=cv2.INTER_AREA)
    frame.paste(Image.fromarray(ov[:, :, ::-1]), (20, 100))
    for robot in ROBOTS:
        if robot in live.pose:
            x, y, _ = live.pose[robot]
            u, v = project_overhead(x, y, 0.1)
            d.text((20 + u + 22, 100 + v - 26), robot, font=F["chip"], fill=TEXT,
                   stroke_width=3, stroke_fill=BG)
    # caption bar
    if caption:
        d.rounded_rectangle((20, 14, W - 20, 86), 10, fill=PANEL)
        d.text((36, 22), caption[0], font=F["h1"], fill=TEXT)
        d.text((36, 56), caption[1], font=F["body"], fill=MUTED)
    else:
        d.text((36, 22), "ROS 2 fleet safety layer", font=F["h1"], fill=TEXT)
        d.text((36, 56), "Gazebo simulation, recorded live, no edits to the data",
               font=F["body"], fill=MUTED)
    # camera panel
    cam_robot = "robot3" if live.fogged and "robot3" in live.cams else "robot1"
    x0, y0 = 650, 100
    d.rounded_rectangle((x0, y0, W - 20, y0 + 390), 10, fill=PANEL)
    d.text((x0 + 14, y0 + 10), f"{cam_robot} camera: YOLOv4-tiny + depth", font=F["h2"],
           fill=TEXT)
    cam = live.cams.get(cam_robot)
    if cam is not None:
        cam = cv2.resize(cam, (448, 336), interpolation=cv2.INTER_LINEAR)
        frame.paste(Image.fromarray(cam[:, :, ::-1]), (x0 + 82, y0 + 42))
    # status rows
    y = y0 + 404
    for robot in ROBOTS:
        st = live.robot_state(robot, t)
        rgb = STATE_RGB[st]
        d.rounded_rectangle((x0, y, W - 20, y + 58), 10, fill=PANEL)
        d.text((x0 + 14, y + 8), robot, font=F["h2"], fill=TEXT)
        chip = LABEL[st]
        cw = d.textlength(chip, font=F["chip"]) + 20
        d.rounded_rectangle((x0 + 100, y + 8, x0 + 100 + cw, y + 32), 12, fill=rgb)
        d.text((x0 + 110, y + 12), chip, font=F["chip"], fill=(255, 255, 255))
        v = live.speed.get(robot, 0.0)
        ood = (f"   OOD {live.ood[robot]:.1f} (limit {OOD_THRESHOLD:.1f})"
               if robot in live.ood else "")
        d.text((x0 + 14, y + 34), f"{v:.2f} m/s{ood}", font=F["small"], fill=MUTED)
        # allowed speed bar (the gate's scale)
        bx, bw = x0 + 300, 230
        sc = 0.0 if st == "no heartbeat" else live.scale.get(robot, 0.0)
        d.text((bx, y + 8), "speed allowed", font=F["small"], fill=MUTED)
        d.rounded_rectangle((bx, y + 30, bx + bw, y + 42), 6, fill=(50, 54, 62))
        if sc > 0:
            d.rounded_rectangle((bx, y + 30, bx + max(12, int(bw * sc)), y + 42), 6, fill=rgb)
        y += 66
    d.text((20, H - 22), f"sim time {t - t_start:5.1f} s   playback x{speedup:.1f}",
           font=F["small"], fill=MUTED)
    return np.asarray(frame)[:, :, ::-1]


def card(lines: list[tuple[str, str, tuple]]) -> np.ndarray:
    frame = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(frame)
    y = 120
    for text, style, colour in lines:
        if not text:
            y += 24
            continue
        d.text((90, y), text, font=F[style], fill=colour)
        y += {"title": 64, "h1": 44, "big": 48, "body": 30, "small": 24}.get(style, 30)
    return np.asarray(frame)[:, :, ::-1]


TITLE = [("Fleet safety layer for mobile robots", "title", TEXT),
         ("ROS 2 Jazzy  ·  Nav2  ·  Gazebo Harmonic  ·  YOLOv4-tiny  ·  C++ / Python", "body",
          MUTED), ("", "", TEXT),
         ("Three robots, each with its own safety chain:", "h1", TEXT),
         ("camera + lidar protective zones: slow or stop", "body", TEXT),
         ("fail-safe heartbeat: if the safety monitor dies, the robot stops", "body", TEXT),
         ("degraded-camera detection: if the image can't be trusted, slow down", "body", TEXT),
         ("", "", TEXT), ("A learning demonstrator, not a certified safety function.", "small",
                          MUTED)]

RESULTS = [("Measured against simulator ground truth", "h1", TEXT), ("", "", TEXT),
           ("Missed stop-zone entries:  camera only 4 of 13, camera + lidar 0 of 16", "body",
            TEXT),
           ("Stop latency (median, n=3):  command 0.13 s, robot halted 0.46 s", "body", TEXT),
           ("False stops:  0 in 24.5 robot-minutes without people", "body", TEXT),
           ("Killed safety monitor:  robot halted in 0.80 s, the other two kept driving", "body",
            TEXT),
           ("Degraded-camera detector:  AUROC 0.93 (ImageNet energy score: 0.45)", "body", TEXT),
           ("", "", TEXT), ("Small samples: engineering numbers, not safety claims.", "small",
                            MUTED),
           ("github.com/fauzan111/ROS2-Robot-Fleet-Safety-Layer", "h1", (120, 180, 255))]


def ffmpeg_writer(path: Path) -> subprocess.Popen:
    return subprocess.Popen(
        [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "rawvideo",
         "-pix_fmt", "bgr24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
         "-pix_fmt", "yuv420p", "-crf", "24", "-preset", "slow", "-movflags", "+faststart",
         str(path)], stdin=subprocess.PIPE)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag")
    ap.add_argument("--out", default="docs/media")
    ap.add_argument("--people", default=str(ROOT / "ros2_ws/src/fleet_bringup/config/"
                                                   "people_crossing.yaml"))
    ap.add_argument("--tail-s", type=float, default=45.0,
                    help="keep this much sim time after the last director event (0 = all)")
    ap.add_argument("--target-s", type=float, default=100.0,
                    help="length of the main section; playback is sped up to fit")
    args = ap.parse_args()
    people = yaml.safe_load(Path(args.people).read_text())["people"]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    typestore = get_typestore(Stores.ROS2_JAZZY)
    with AnyReader([Path(args.bag)], default_typestore=typestore) as reader:
        conns = reader.connections
        # pass 1: events and time span (small topics only)
        events, t_end = [], reader.end_time / 1e9
        light = [c for c in conns if c.topic == "/demo/event"]
        for c, _ts, raw in reader.messages(connections=light) if light else ():
            events.append(json.loads(reader.deserialize(raw, c.msgtype).data))
        start = next((e["t"] for e in events if e["kind"] == "start"), reader.start_time / 1e9)
        t_start = start - 3.0
        if events and args.tail_s > 0:  # after the last scripted event robots just finish up
            t_end = min(t_end, max(e["t"] for e in events) + args.tail_s)
        speedup = max(1.0, (t_end - t_start) / args.target_s)
        print(f"events: {[(e['kind'], e['t']) for e in events]}; main section "
              f"{t_end - t_start:.0f} s sim at x{speedup:.2f}")

        writer = ffmpeg_writer(out / "demo.mp4")
        title = card(TITLE)
        for _ in range(int(5 * FPS)):
            writer.stdin.write(title.tobytes())

        live, caps = Live(), Captions()
        pending = sorted(events, key=lambda e: e["t"])
        dt = speedup / FPS
        t_next = t_start
        gif_marks = []
        prev_r1 = None
        for c, ts, raw in reader.messages(connections=conns):
            t = ts / 1e9
            while t >= t_next and t_next <= t_end:
                while pending and pending[0]["t"] <= t_next:
                    e = pending.pop(0)
                    caps.event(t_next, e["kind"])
                    live.fogged = {"fog_on": True, "fog_off": False}.get(e["kind"], live.fogged)
                if t_next >= t_start:
                    r1 = live.robot_state("robot1", t_next)
                    if r1 != prev_r1:
                        caps.robot1(t_next, r1)
                        if r1 == "stop":
                            gif_marks.append(t_next)
                        prev_r1 = r1
                    writer.stdin.write(compose(live, t_next, t_start, speedup,
                                               caps.get(t_next), people).tobytes())
                t_next += dt
            if t < t_start - 2 and (c.topic == "/demo/overhead"
                                    or c.topic.endswith("/annotated")):
                continue  # images before the video starts: skip decoding
            msg = reader.deserialize(raw, c.msgtype)
            robot = c.topic.split("/")[1]
            if c.topic == "/demo/overhead":
                live.overhead = to_bgr(msg)
            elif c.topic.endswith("/safety/annotated"):
                live.cams[robot] = to_bgr(msg)
            elif c.topic.endswith("/ground_truth/pose"):
                p = msg.pose
                live.pose[robot] = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
            elif c.topic.endswith("/safety/state"):
                live.state[robot] = (msg.data, t)
            elif c.topic.endswith("/safety/speed_scale"):
                live.scale[robot] = msg.data
            elif c.topic.endswith("/cmd_vel_safe"):
                live.speed[robot] = abs(msg.linear.x)
            elif c.topic.endswith("/safety/ood_score") and robot == "robot3":
                live.ood[robot] = msg.data

        results = card(RESULTS)
        for _ in range(int(9 * FPS)):
            writer.stdin.write(results.tobytes())
        writer.stdin.close()
        writer.wait()

    total = 5 + (t_end - t_start) / speedup + 9
    print(f"wrote {out / 'demo.mp4'} ({total:.0f} s)")
    if gif_marks:  # README teaser: the first stop for a person, in real time
        gif_at = 5 + (gif_marks[0] - t_start) / speedup - 5.0
        subprocess.run(
            [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-ss",
             f"{max(gif_at, 5):.2f}", "-t", "10", "-i", str(out / "demo.mp4"), "-vf",
             "fps=8,scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];"
             "[b][p]paletteuse=dither=bayer", str(out / "demo.gif")], check=True)
        print(f"wrote {out / 'demo.gif'} (from {gif_at:.1f} s)")


if __name__ == "__main__":
    main()
