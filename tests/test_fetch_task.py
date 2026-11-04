"""Fast deterministic checks for the PyBullet fetch task internals."""
import math
import io
import json

import pybullet as p
from PIL import Image

from engine.fetch_environment import FetchEnvironment
from engine.fetch_state import FetchPhase, FetchState
from engine.pickup_manager import PickupManager
from engine import fetch_server
from agent.claude_agent import _groq_messages, _groq_payload, _groq_retry_delay, _keep_latest_image
from agent.fetch_tools import FETCH_TOOLS
from agent.prompts import FETCH_SYSTEM_PROMPT
from evaluation.fetch_logger import FetchLogger
from engine import config
from scripts.diff_drive_robot import DiffDriveRobot


def test_seeded_room_camera_and_fixed_constraint_delivery():
    client = p.connect(p.DIRECT)
    try:
        env = FetchEnvironment(19).build()
        assert len(env.wall_ids) == 4
        assert len(env.object_ids) == 6
        x, y, _ = env.robot_start
        robot = DiffDriveRobot("urdf/diff_drive_robot.urdf", [x, y, 0.1], 0)
        assert (robot.left_wheel_joint, robot.right_wheel_joint) == (0, 1)
        assert (robot.camera_width, robot.camera_height) == (640, 480)
        rgb, depth = robot.get_camera_image()
        assert rgb.shape == (480, 640, 3)
        assert depth.shape == (480, 640)
        p.resetBasePositionAndOrientation(robot.robot_id, [x, y, 0.1],
                                          p.getQuaternionFromEuler([0, 0, math.pi / 2]))
        rotated_rgb, _ = robot.get_camera_image()
        assert not (rgb == rotated_rgb).all()

        body = env.object_ids[0]
        p.resetBasePositionAndOrientation(body, [0.4, 0.0, 0.15], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(robot.robot_id, [0, 0, 0.1], [0, 0, 0, 1])
        manager = PickupManager(env, robot)
        task = FetchState()
        task.start()
        task.action()
        assert task.phase == FetchPhase.APPROACH
        assert manager.pickup() is True
        task.picked_up(True, True)
        assert task.phase == FetchPhase.CARRY
        assert manager.constraint_id is not None
        p.stepSimulation()
        env.drop_center = (1.3, 1.0)
        p.resetBasePositionAndOrientation(env.drop_id, [1.3, 1.0, 0.002], [0, 0, 0, 1])
        # The test moves the held body as the physics action layer would.
        p.resetBasePositionAndOrientation(body, [1.3, 1.0, 0.3], [0, 0, 0, 1])
        released, in_zone = manager.drop()
        assert released and in_zone
        task.dropped(released, in_zone, False)
        assert task.phase == FetchPhase.DONE
        assert task.outcome == "pass"
    finally:
        p.disconnect(client)


def test_pickup_angle_and_range_limits():
    client = p.connect(p.DIRECT)
    try:
        env = FetchEnvironment(5).build()
        robot = DiffDriveRobot("urdf/diff_drive_robot.urdf", [0, 0, 0.1], 0)
        body = env.object_ids[0]
        p.resetBasePositionAndOrientation(body, [0.8, 0.8, 0.15], [0, 0, 0, 1])
        manager = PickupManager(env, robot)
        assert manager.pickup() is False
    finally:
        p.disconnect(client)


def test_drop_outside_zone_fails_validation_and_keeps_carry_state():
    client = p.connect(p.DIRECT)
    try:
        env = FetchEnvironment(6).build()
        robot = DiffDriveRobot("urdf/diff_drive_robot.urdf", [0, 0, 0.1], 0)
        body = env.object_ids[0]
        p.resetBasePositionAndOrientation(body, [0.4, 0, 0.15], [0, 0, 0, 1])
        manager = PickupManager(env, robot)
        assert manager.pickup()
        p.resetBasePositionAndOrientation(body, [2.5, 2.5, 0.3], [0, 0, 0, 1])
        released, in_zone = manager.drop()
        state = FetchState()
        state.start()
        state.picked_up(True, False)
        state.dropped(released, in_zone, False)
        assert released and not in_zone
        assert state.phase == FetchPhase.CARRY
        assert state.retry_count == 1
        assert state.outcome is None
    finally:
        p.disconnect(client)


def test_all_six_objects_pickup_and_reject_bad_range_or_heading():
    client = p.connect(p.DIRECT)
    try:
        env = FetchEnvironment(42).build()
        robot = DiffDriveRobot("urdf/diff_drive_robot.urdf", [0, 0, 0.1], 0)
        manager = PickupManager(env, robot)
        assert len(env.object_ids) == 6
        for body in env.object_ids:
            position, _ = p.getBasePositionAndOrientation(body)
            p.resetBasePositionAndOrientation(body, [4.0, 4.0, position[2]], [0, 0, 0, 1])
        robot = DiffDriveRobot("urdf/diff_drive_robot.urdf", [1, 0, 0.1], 0)
        manager = PickupManager(env, robot)
        for index, body in enumerate(env.object_ids):
            for other in env.object_ids:
                if other != body:
                    other_pos, _ = p.getBasePositionAndOrientation(other)
                    p.resetBasePositionAndOrientation(other, [4.0, 4.0, other_pos[2]], [0, 0, 0, 1])
            angle = index * 0.04
            p.resetBasePositionAndOrientation(body, [1 + 0.48 * math.cos(angle), 0.48 * math.sin(angle), 0.18], [0, 0, 0, 1])
            assert manager.pickup()
            assert manager.held_body == body
            assert manager.constraint_id is not None
            assert p.getConstraintInfo(manager.constraint_id)[4] == p.JOINT_FIXED
            assert not manager.pickup()
            assert p.getBasePositionAndOrientation(body)[0][0] < 1.6
            p.removeConstraint(manager.constraint_id)
            manager.constraint_id = None
            manager.held_body = None
        # Beyond range.
        p.resetBasePositionAndOrientation(robot.robot_id, [1, 0, 0.1], [0, 0, 0, 1])
        for other in env.object_ids[1:]:
            other_pos, _ = p.getBasePositionAndOrientation(other)
            p.resetBasePositionAndOrientation(other, [4, 4, other_pos[2]], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(env.object_ids[0], [2.0, 0, 0.18], [0, 0, 0, 1])
        assert not manager.pickup()
        # Within reach but outside the +/- 30 degree forward cone.
        p.resetBasePositionAndOrientation(env.object_ids[0], [1.35, 0.35, 0.18], [0, 0, 0, 1])
        assert not manager.pickup()
    finally:
        p.disconnect(client)


def test_constraint_carries_with_robot_and_drop_validation_settles():
    client = p.connect(p.DIRECT)
    try:
        env = FetchEnvironment(19).build()
        body = env.object_ids[0]
        p.resetBasePositionAndOrientation(body, [0.4, 0, 0.15], [0, 0, 0, 1])
        robot = DiffDriveRobot("urdf/diff_drive_robot.urdf", [0, 0, 0.1], 0)
        manager = PickupManager(env, robot)
        assert manager.pickup()
        constraint = manager.constraint_id
        before_robot = robot.get_pose().copy()
        for _ in range(60):
            robot.set_velocity(0.25, 0.0)
            p.stepSimulation()
        robot.stop()
        after_robot = robot.get_pose().copy()
        body_pos, _ = p.getBasePositionAndOrientation(body)
        assert constraint is not None and abs(after_robot[0] - before_robot[0]) > 0.005
        assert math.dist(body_pos[:2], after_robot[:2]) < 0.6
        # Explicit turn while the fixed constraint remains attached.
        robot.turn_angle(30, speed_deg_s=60, collision_check=False)
        assert manager.constraint_id == constraint

        env.drop_center = (1.0, 1.0)
        p.resetBasePositionAndOrientation(env.drop_id, [1, 1, 0.002], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(body, [1, 1, 0.4], [0, 0, 0, 1])
        released, success = manager.drop()
        assert released and success
        assert manager.held_body is None and manager.constraint_id is None
        assert p.getBasePositionAndOrientation(body)[0][2] < 0.4

        p.resetBasePositionAndOrientation(robot.robot_id, [2.0, 2.5, 0.1], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(body, [2.35, 2.5, 0.4], [0, 0, 0, 1])
        assert manager.pickup()
        p.resetBasePositionAndOrientation(body, [2.5, 2.5, 0.4], [0, 0, 0, 1])
        released, success = manager.drop()
        assert released and not success
    finally:
        p.disconnect(client)


def test_three_collision_retries_terminate():
    state = FetchState()
    state.start()
    for _ in range(3):
        state.collision()
        if state.phase == FetchPhase.BLOCKED:
            state.recover()
    assert state.phase == FetchPhase.DONE
    assert state.outcome == "fail_collision"
    assert state.retry_count == 3


def test_wall_collision_and_backoff_recovery():
    fetch_server._initialize(31)
    try:
        p.resetBasePositionAndOrientation(fetch_server._robot.robot_id,
                                          [2.65, 0.0, 0.1], [0, 0, 0, 1])
        p.resetBaseVelocity(fetch_server._robot.robot_id, [0, 0, 0], [0, 0, 0])
        result = fetch_server._move(0.5)
        assert result["collided"] is True
        assert result["state"] == "BLOCKED"
        assert fetch_server._state.retry_count == 1
        recovery = fetch_server._move(-0.12)
        assert recovery["collided"] is False
        assert recovery["state"] == "LOCATE"
        assert fetch_server._state.retry_count == 1
    finally:
        p.disconnect()
        fetch_server._started = False


def test_distance_turn_cardinal_directions_and_think_freeze():
    fetch_server._initialize(71)
    try:
        robot = fetch_server._robot
        start = robot.get_pose().copy()
        forward = robot.drive_distance(1.0, speed=0.5)
        after_forward = robot.get_pose().copy()
        assert not forward["collided"]
        assert abs(forward["moved_m"] - 1.0) < 0.01
        assert abs(math.hypot(after_forward[0] - start[0], after_forward[1] - start[1]) - 1.0) < 0.01

        turned = robot.turn_angle(90.0, speed_deg_s=90.0)
        assert not turned["collided"]
        assert abs(turned["turned_deg"] - 90.0) < 1.0
        rgb, _ = robot.get_camera_image()
        assert rgb.shape == (480, 640, 3)

        cardinals = []
        for distance, yaw, pos in ((0.12, 0.0, [1, 0, 0.1]), (-0.12, 0.0, [1, 1, 0.1]),
                                   (0.12, math.pi / 2, [0, -1, 0.1]),
                                   (-0.12, math.pi / 2, [1, -1, 0.1])):
            p.resetBasePositionAndOrientation(robot.robot_id, pos, p.getQuaternionFromEuler([0, 0, yaw]))
            p.resetBaseVelocity(robot.robot_id, [0, 0, 0], [0, 0, 0])
            cardinals.append(robot.drive_distance(distance, speed=0.25))
        assert all(not item["collided"] and abs(abs(item["moved_m"]) - 0.12) < 0.01 for item in cardinals)

        p.resetBasePositionAndOrientation(robot.robot_id, [2.65, 0.0, 0.1], [0, 0, 0, 1])
        p.resetBaseVelocity(robot.robot_id, [0, 0, 0], [0, 0, 0])
        collision = robot.drive_distance(0.5, speed=0.5)
        assert collision["collided"]
        assert collision["moved_m"] < 0.2

        frozen_pose = robot.get_pose().copy()
        for _ in range(3):
            fetch_server._status()
            fetch_server._look()
        assert (robot.get_pose() == frozen_pose).all()
    finally:
        p.disconnect()
        fetch_server._started = False


def test_agent_history_keeps_only_current_image_and_logs_png(tmp_path):
    messages = [
        {"role": "user", "content": [{"type": "text", "text": "first"},
                                        {"type": "image", "source": {"data": "old"}}]},
        {"role": "assistant", "content": [{"type": "tool_use", "id": "1"}]},
        {"role": "user", "content": [{"type": "tool_result", "tool_use_id": "1"},
                                        {"type": "image", "source": {"data": "new"}}]},
    ]
    _keep_latest_image(messages)
    images = [block for message in messages for block in message["content"]
              if isinstance(block, dict) and block.get("type") == "image"]
    assert len(images) == 1
    assert images[0]["source"]["data"] == "new"
    prompt_and_tools = FETCH_SYSTEM_PROMPT.lower() + " " + " ".join(
        tool["description"].lower() for tool in FETCH_TOOLS)
    assert all(name.lower() not in prompt_and_tools for _, name in config.PALETTE)
    assert all(shape not in prompt_and_tools for shape in ("box", "sphere", "cylinder", "cone"))

    buffer = io.BytesIO()
    Image.new("RGB", (640, 480), "white").save(buffer, format="JPEG")
    logger = FetchLogger("test_run", "test command", root=str(tmp_path))
    logger.record("turn", {"degrees": 10, "what_i_see": "view", "why": "scan"},
                  {"state": "LOCATE"}, buffer.getvalue(), {"state": "LOCATE"})
    row = json.loads(logger.path.read_text(encoding="utf-8"))
    assert row["args"]["why"] == "scan"
    assert (logger.frames / "step_000.png").exists()


def test_groq_payload_keeps_camera_as_image_content():
    conversation = [
        {"role": "user", "content": [
            {"type": "text", "text": "Inspect the view."},
            {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": "pixels"}},
        ]},
    ]
    payload = _groq_messages(conversation)
    assert payload[0]["role"] == "system"
    assert payload[1]["content"][1]["type"] == "image_url"
    assert payload[1]["content"][1]["image_url"]["url"] == "data:image/jpeg;base64,pixels"
    request = _groq_payload(conversation, "qwen/qwen3.8-27b")
    assert request["response_format"]["json_schema"]["strict"] is True
    assert "tools" not in request
    assert request["response_format"]["json_schema"]["schema"]["required"] == [
        "action", "distance_m", "degrees", "what_i_see", "why"]


def test_groq_rate_limit_retry_uses_server_wait_hint():
    class RateLimitResponse:
        headers = {}

        @staticmethod
        def json():
            return {"error": {"message": "Please try again in 4.037142857s."}}

    assert _groq_retry_delay(RateLimitResponse(), 0) == 4.237142857

