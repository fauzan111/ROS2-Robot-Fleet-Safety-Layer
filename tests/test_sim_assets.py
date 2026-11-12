"""Simulation asset patching (fleet_bringup.sim_assets)."""

import xml.dom.minidom

import pytest
from fleet_bringup.sim_assets import (
    actor_sdf,
    person_position,
    with_actors,
    with_rgbd_camera,
)

UPSTREAM = """<sdf><model name="tb3"><link name="camera_link">
<sensor name="intel_realsense_r200_depth" type="depth"><update_rate>5</update_rate>
<camera><image><width>320</width></image></camera></sensor></link></model></sdf>"""


def test_rgbd_camera_replaces_depth_sensor_with_namespaced_topic():
    out = with_rgbd_camera(UPSTREAM, "robot2")
    assert "intel_realsense_r200_depth" not in out
    assert "<topic>robot2/camera</topic>" in out
    assert 'type="rgbd_camera"' in out
    assert "<gz_frame_id>camera_rgb_optical_frame</gz_frame_id>" in out
    xml.dom.minidom.parseString(out)  # still well-formed


def test_rgbd_camera_single_robot_and_missing_sensor():
    assert "<topic>camera</topic>" in with_rgbd_camera(UPSTREAM, "")
    with pytest.raises(ValueError, match="found 0"):
        with_rgbd_camera("<sdf/>", "robot1")


WALK = [{"t": 0, "x": 0.0, "y": 0.0}, {"t": 4, "x": 2.0, "y": 0.0},
        {"t": 6, "x": 2.0, "y": 0.0}, {"t": 10, "x": 0.0, "y": 0.0}]


def test_actor_sdf_is_wellformed_and_faces_direction_of_travel():
    out = actor_sdf("person1", WALK, "/opt/fleet_models/actor/walk.dae")
    doc = xml.dom.minidom.parseString(out)
    poses = [p.firstChild.data.split() for p in doc.getElementsByTagName("pose")]
    assert [float(p[5]) for p in poses] == pytest.approx([0.0, 0.0, 3.1416, 3.1416], abs=1e-3)
    assert doc.getElementsByTagName("loop")[0].firstChild.data == "true"


def test_actor_sdf_rejects_bad_waypoints():
    with pytest.raises(ValueError, match="at least 2"):
        actor_sdf("p", WALK[:1], "m.dae")
    with pytest.raises(ValueError, match="increase"):
        actor_sdf("p", [WALK[1], WALK[0]], "m.dae")


def test_with_actors_inserts_before_closing_world_tag():
    world = "<sdf><world name='w'><model name='a'/></world></sdf>"
    out = with_actors(world, ["<actor name='p1'/>", "<actor name='p2'/>"])
    assert out.index("p2") < out.index("</world>")
    xml.dom.minidom.parseString(out)


def test_person_position_interpolates_and_loops():
    assert person_position(WALK, 2.0) == pytest.approx((1.0, 0.0))
    assert person_position(WALK, 5.0) == pytest.approx((2.0, 0.0))   # standing still
    assert person_position(WALK, 12.0) == pytest.approx((1.0, 0.0))  # 12 s = 2 s into loop 2


def test_pose_publisher_added_inside_model():
    from fleet_bringup.sim_assets import with_pose_publisher

    out = with_pose_publisher(UPSTREAM)
    assert out.index("PosePublisher") < out.index("</model>")
    assert "<publish_model_pose>true</publish_model_pose>" in out
    xml.dom.minidom.parseString(out)
    with pytest.raises(ValueError, match="model"):
        with_pose_publisher("<sdf/>")
