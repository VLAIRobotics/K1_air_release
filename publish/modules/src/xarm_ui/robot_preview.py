"""Read-only product model assets and received ROS feedback for the browser viewer.

No publishers, control commands, subprocesses, CAN access or ROS shutdown live here.
The camera runtime owns its node/executor; this module only adds subscriptions.
"""
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from flask import Response, abort, jsonify, send_file


# Teleop keeps the shared xarm topic contract, including on L1. Only received
# joint names are mapped to the selected product's model prefix.
PRODUCT_MODELS = {
    "X1": {"label": "X1", "package": "xarm_description",
           "file": "v10_bimanual_generated.urdf", "joint_prefix": "xarm_"},
    "K1": {"label": "K1 双臂", "package": "xarm_description",
           "file": "v10_bimanual_generated.urdf", "joint_prefix": "xarm_"},
    "L1": {"label": "L1", "package": "l1_description",
           "file": "l1.urdf", "joint_prefix": "l1_"},
}


def _product_profile(product=None):
    if product is None:
        identity = Path(__file__).resolve().parent / "product.json"
        policy = Path(__file__).resolve().parents[4] / "product-policy.json"
        source = identity if identity.is_file() else policy
        if not source.is_file():
            raise ValueError("Product identity is missing")
        product = json.loads(source.read_text())["product"]
    if product not in PRODUCT_MODELS:
        raise ValueError("Unsupported product model")
    return product, PRODUCT_MODELS[product]


ARM_TOPICS = {
    side: f"/xarm_{side}_follower/arm/position" for side in ("left", "right")
}
HAND_TOPICS = {
    side: f"/xarm_{side}_follower/hand/position" for side in ("left", "right")
}
STALE_SECONDS = 1.0


class RobotPreviewManager:
    def __init__(self, ros_manager=None, description_dir=None, clock=time.monotonic, product=None):
        self.product, profile = _product_profile(product)
        self.model_label = profile["label"]
        self.description_package = profile["package"]
        self.joint_prefix = profile["joint_prefix"]
        self.description_dir = Path(description_dir or
            Path(__file__).resolve().parent.parent / self.description_package).resolve()
        self.mesh_dir = (self.description_dir / "meshes").resolve()
        self.model_file = self.description_dir / profile["file"]
        self._runtime = ros_manager
        self._clock = clock
        self._lock = threading.RLock()
        self._subscribe_lock = threading.Lock()
        self._samples = {"joint_states": {}, "follower": {}}
        self._subscriptions = []
        self._error = None

    def joint_names(self, side):
        return [f"{self.joint_prefix}{side}_joint{i}" for i in range(1, 8)]

    def _store(self, source, values, topic):
        now = self._clock()
        with self._lock:
            for name, position in values.items():
                self._samples[source][name] = (position, now, topic)

    def receive_arm(self, side, msg):
        """Seven measured motor positions, rad, from the follower state callback."""
        if side not in ARM_TOPICS:
            return False
        try:
            data = [float(value) for value in msg.data]
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False
        if len(data) != 7 or not all(math.isfinite(value) for value in data):
            return False
        self._store("follower", dict(zip(self.joint_names(side), data)), ARM_TOPICS[side])
        return True

    def receive_hand(self, side, msg):
        """Teleop publishes the motor angle, not finger travel in metres."""
        if side not in HAND_TOPICS:
            return False
        try:
            data = [float(value) for value in msg.data]
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False
        if len(data) != 1 or not math.isfinite(data[0]):
            return False
        # Same conversion as XArm_v10HW::motor_radians_to_joint.
        self._store("follower", {f"{self.joint_prefix}{side}_finger_joint1":
            0.044 * data[0] / -1.0472}, HAND_TOPICS[side])
        return True

    def receive_joint_states(self, msg):
        expected = {name for side in ARM_TOPICS for name in self.joint_names(side)}
        expected.update(f"{self.joint_prefix}{side}_finger_joint1" for side in ARM_TOPICS)
        try:
            names, positions = list(msg.name), list(msg.position)
            if len(names) != len(positions) or len(set(names)) != len(names):
                return False
            values = {name: float(value) for name, value in zip(names, positions)
                      if name in expected}
            if not all(math.isfinite(value) for value in values.values()):
                return False
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False
        if not values:
            return False
        self._store("joint_states", values, "/joint_states")
        return True

    def ensure_subscriptions(self):
        with self._subscribe_lock:
            if self._subscriptions:
                return True
            if self._runtime is None:
                self._error = "ROS反馈未连接"
                return False
            created = []
            try:
                result = self._runtime._ensure_runtime()
                if not result.get("success"):
                    self._error = "ROS反馈运行环境不可用"
                    return False
                from sensor_msgs.msg import JointState
                from std_msgs.msg import Float64MultiArray
                from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
                node = self._runtime._node
                created.append(node.create_subscription(
                    JointState, "/joint_states", self.receive_joint_states, qos))
                for side in ARM_TOPICS:
                    created.append(node.create_subscription(Float64MultiArray, ARM_TOPICS[side],
                        lambda msg, side=side: self.receive_arm(side, msg), qos))
                    created.append(node.create_subscription(Float64MultiArray, HAND_TOPICS[side],
                        lambda msg, side=side: self.receive_hand(side, msg), qos))
                self._subscriptions = created
                self._error = None
                return True
            except Exception:
                # Do not return host paths or exception internals to the browser.
                self._error = "ROS反馈订阅不可用"
                for subscription in created:
                    try:
                        self._runtime._node.destroy_subscription(subscription)
                    except Exception:
                        pass
                return False

    def snapshot(self):
        now = self._clock()
        output = {"success": True, "model": self.model_label, "product": self.product,
                  "joint_prefix": self.joint_prefix, "stale_after_ms": 1000,
                  "feedback_only": True, "runtime_error": self._error,
                  "positions": {}, "joints": {}, "arms": {}}
        with self._lock:
            for side in ARM_TOPICS:
                names = self.joint_names(side)
                fresh, seen, sources, ages = 0, 0, set(), []
                for name in names + [f"{self.joint_prefix}{side}_finger_joint1"]:
                    follower = self._samples["follower"].get(name)
                    standard = self._samples["joint_states"].get(name)
                    candidates = [sample for sample in (follower, standard) if sample is not None]
                    if not candidates:
                        continue
                    # Prefer the complete teleop feedback if fresh, otherwise the
                    # standard JointState source. Never combine targets/leader data.
                    if follower and now - follower[1] <= STALE_SECONDS:
                        sample = follower
                    elif standard and now - standard[1] <= STALE_SECONDS:
                        sample = standard
                    else:
                        sample = max(candidates, key=lambda value: value[1])
                    position, received, topic = sample
                    age = max(0.0, now - received)
                    output["positions"][name] = position
                    output["joints"][name] = {"age_ms": round(age * 1000),
                                               "fresh": age <= STALE_SECONDS, "source": topic}
                    if name in names:
                        seen += 1
                        fresh += age <= STALE_SECONDS
                        sources.add(topic)
                        ages.append(age)
                state = "live" if fresh == 7 else "partial" if fresh else "stale" if seen else "unknown"
                hand = output["joints"].get(f"{self.joint_prefix}{side}_finger_joint1")
                output["arms"][side] = {"state": state, "fresh_joints": fresh,
                    "received_joints": seen, "expected_joints": 7, "sources": sorted(sources),
                    "age_ms": round(max(ages) * 1000) if ages else None,
                    "gripper_state": "live" if hand and hand["fresh"] else "stale" if hand else "unknown"}
        return output

    def model_xml(self):
        tree = ET.parse(self.model_file)
        root = tree.getroot()
        if root.tag != "robot":
            raise ValueError("Invalid model")
        for mesh in root.iter("mesh"):
            filename = mesh.get("filename", "")
            prefix = f"package://{self.description_package}/meshes/"
            if not filename.startswith(prefix):
                raise ValueError("Model references an unsupported asset")
            relative = filename[len(prefix):]
            self.mesh_path(relative)
            revision = hashlib.sha256(self.mesh_path(relative).read_bytes()).hexdigest()[:16]
            mesh.set("filename", "/api/robot-preview/meshes/_v/" + revision + "/" + relative)
        return ET.tostring(root, encoding="utf-8", xml_declaration=True)

    def mesh_path(self, relative):
        if not relative or "\\" in relative:
            raise ValueError("Invalid asset path")
        path = (self.mesh_dir / relative).resolve()
        try:
            path.relative_to(self.mesh_dir)
        except ValueError:
            raise ValueError("Invalid asset path") from None
        if path.suffix.lower() not in {".stl", ".dae", ".png", ".jpg", ".jpeg", ".tga"} or not path.is_file():
            raise ValueError("Unknown model asset")
        return path


def register_robot_preview(app, ros_manager=None, description_dir=None, product=None):
    manager = RobotPreviewManager(ros_manager=ros_manager, description_dir=description_dir, product=product)
    app.extensions["robot_preview"] = manager

    @app.get("/api/robot-preview/model")
    def robot_preview_model():
        try:
            response = Response(manager.model_xml(), mimetype="application/xml")
            response.headers["Cache-Control"] = "no-store"
            return response
        except (OSError, ValueError, ET.ParseError):
            return jsonify(success=False, message="模型资源不可用"), 503

    @app.get("/api/robot-preview/meshes/<path:relative>")
    @app.get("/api/robot-preview/meshes/_v/<revision>/<path:relative>")
    def robot_preview_mesh(relative, revision=None):
        try:
            path = manager.mesh_path(relative)
            if revision is not None and revision != hashlib.sha256(path.read_bytes()).hexdigest()[:16]:
                abort(404)
            response = send_file(path, conditional=True, max_age=0)
            response.headers["Cache-Control"] = "no-cache, must-revalidate"
            return response
        except (OSError, ValueError):
            abort(404)

    @app.get("/api/robot-preview/state")
    def robot_preview_state():
        manager.ensure_subscriptions()
        return jsonify(manager.snapshot())

    return manager
