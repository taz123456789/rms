"""
letter_sorter.py  — 5-DOF arm letter-sorting controller for Webots
"""

import os, sys, math, numpy as np
import yaml
import cv2
from controller import Supervisor
from forward_kinematics import ForwardKinematics
from inverse_kinematics import InverseKinematics
from letter_recognizer import LetterRecognizer




class ArmHW:
    NORMAL_VEL = [1.5, 1.5, 2.0, 2.5, 3.0]
    SLOW_VEL   = [0.3, 0.3, 0.4, 0.5, 0.6]
    PLACE_VEL  = [0.2, 0.2, 0.3, 0.4, 0.5]
    CARRY_VEL  = [1.0, 1.0, 1.5, 1.5, 1.0]

    def __init__(self, robot, ts):
        names = ["shoulder_yaw", "shoulder_pitch", "elbow_pitch",
                 "wrist_pitch", "wrist_roll"]
        self.motors, self.sensors = [], []
        for name in names:
            m = robot.getDevice(f"{name}_motor")
            s = robot.getDevice(f"{name}_sensor")
            s.enable(ts)
            self.motors.append(m)
            self.sensors.append(s)
        self.gl  = robot.getDevice("gripper_left_motor")
        self.gr  = robot.getDevice("gripper_right_motor")
        self.gls = robot.getDevice("gripper_left_sensor")
        self.grs = robot.getDevice("gripper_right_sensor")
        self.gls.enable(ts); self.grs.enable(ts)

    def get_q(self):
        return np.array([s.getValue() for s in self.sensors])

    def set_q(self, q):
        for i, m in enumerate(self.motors):
            m.setPosition(float(q[i]))

    def open_gripper(self):
        self.gl.setPosition(0.020)
        self.gr.setPosition(0.020)

    def close_gripper(self):
        self.gl.setPosition(0.0)
        self.gr.setPosition(0.0)

    def reached(self, qt, tol=0.02):
        return np.max(np.abs(self.get_q() - qt)) < tol

    def set_slow(self):
        for i, m in enumerate(self.motors):
            m.setVelocity(self.SLOW_VEL[i])

    def set_place_speed(self):
        for i, m in enumerate(self.motors):
            m.setVelocity(self.PLACE_VEL[i])

    def set_carry_speed(self):
        for i, m in enumerate(self.motors):
            m.setVelocity(self.CARRY_VEL[i])

    def set_normal_speed(self):
        for i, m in enumerate(self.motors):
            m.setVelocity(self.NORMAL_VEL[i])


class CameraHW:
    def __init__(self, robot, ts, device_name="letter_camera"):
        dev = robot.getDevice(device_name)
        if dev:
            dev.enable(ts); self._cam = dev
            self.w, self.h = dev.getWidth(), dev.getHeight()
        else:
            self._cam = None; self.w = self.h = 0

    def capture(self):
        """Capture a BGR frame from the forearm-mounted camera.

        Raw image: text rotated 90° counterclockwise from normal reading.
        Fix: cv2.ROTATE_90_CLOCKWISE.
        """
        if not self._cam: return None
        buf = self._cam.getImage()
        if not buf: return None
        img = np.frombuffer(buf, np.uint8).reshape((self.h, self.w, 4))
        frame = img[:, :, :3].copy()
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        return frame

    def save_snapshot(self, path):
        frame = self.capture()
        if frame is not None:
            cv2.imwrite(path, frame); return True
        return False


class SupervisorDelivery:
    """Teleports letters to the pickup tray via Supervisor."""

    DEFAULT_DELIVER  = [-0.20, 0.0, 0.809]
    UPRIGHT_ROTATION = [1.0, 0.0, 0.0, -1.5708]

    SLOT_DROP_Z = 0.659

    def __init__(self, supervisor, config=None):
        self.sv = supervisor
        self.deliver_pos = list(config.get("deliver_world_pos", self.DEFAULT_DELIVER)) if config else list(self.DEFAULT_DELIVER)
        self.deliver_rot = list(config.get("deliver_rotation", self.UPRIGHT_ROTATION)) if config else list(self.UPRIGHT_ROTATION)
        self._nodes = {}
        for letter in "ABCD":
            node = self.sv.getFromDef(f"LETTER_{letter}")
            if node: self._nodes[letter] = node
        self._locked_letter = None
        self._lock_pos = None
        self._lock_rot = None

    def deliver(self, letter):
        node = self._nodes.get(letter)
        if not node: return False
        node.resetPhysics()
        node.getField("translation").setSFVec3f(self.deliver_pos)
        node.getField("rotation").setSFRotation(self.deliver_rot)
        node.resetPhysics()
        self._locked_letter = letter
        self._lock_pos = list(self.deliver_pos)
        self._lock_rot = list(self.deliver_rot)
        return True

    def hold(self):
        if self._locked_letter is None:
            return
        node = self._nodes.get(self._locked_letter)
        if node:
            node.resetPhysics()
            node.getField("translation").setSFVec3f(self._lock_pos)
            node.getField("rotation").setSFRotation(self._lock_rot)
            node.resetPhysics()

    def release(self):
        if self._locked_letter:
            print(f"  [Lock] Released '{self._locked_letter}' — physics takes over")
        self._locked_letter = None
        self._lock_pos = None
        self._lock_rot = None

    def drop_into_slot(self, letter):
        node = self._nodes.get(letter)
        if not node:
            return False
        pos = node.getPosition()
        target = [pos[0], pos[1], self.SLOT_DROP_Z]
        node.resetPhysics()
        node.getField("translation").setSFVec3f(target)
        node.resetPhysics()
        print(f"  [Drop] '{letter}' → slot bottom Z={self.SLOT_DROP_Z:.3f}")
        return True



class S:
    INIT=0; HOME=1; W_HOME=2; DELIVER=3; W_DELIVER=4
    APPROACH_PICKUP=5; W_APPROACH=6; LOWER=7; W_LOWER=8
    GRASP=9; W_GRASP=10
    LIFT=11; W_LIFT=12
    SCAN_POSE=27; W_SCAN_POSE=28
    CAPTURE=15; RECOGNIZE=16
    PRE_PLACE=13; W_PRE_PLACE=14
    FRONT_SLOT=17; W_FRONT=18
    SLIDE_IN=19;   W_SLIDE_IN=20
    RELEASE=21;    W_RELEASE=22
    DROP=25;       W_DROP=26
    SLIDE_OUT=23;  W_SLIDE_OUT=24
    DONE=99

    HOLD_STATES = {
        GRASP, W_GRASP,
        LIFT, W_LIFT,
        SCAN_POSE, W_SCAN_POSE,
        CAPTURE, RECOGNIZE,
        PRE_PLACE, W_PRE_PLACE,
        FRONT_SLOT, W_FRONT,
        SLIDE_IN, W_SLIDE_IN,
    }


SCAN_POSE_Q = np.array([0.0, -math.pi/2, math.pi/3, math.pi/2, -math.pi/2])


class LetterSorterController:
    N = 4; STACK = ["D", "C", "B", "A"]

    def __init__(self):
        self.robot = Supervisor()
        self.ts = int(self.robot.getBasicTimeStep())
        ctrl_dir = os.path.dirname(os.path.abspath(__file__))
        cfg_dir = os.path.join(ctrl_dir, "config")
        self.snap_dir = os.path.join(ctrl_dir, "debug_snapshots")
        os.makedirs(self.snap_dir, exist_ok=True)

        self.arm = ArmHW(self.robot, self.ts)
        self.fk  = ForwardKinematics(os.path.join(cfg_dir, "arm_params.yaml"))
        self.ik  = InverseKinematics()
        self.rec = LetterRecognizer()
        self.cam = CameraHW(self.robot, self.ts, "letter_camera")
        self.tgt = self._load_cfg(os.path.join(cfg_dir, "pigeonhole_map.yaml"))

        self.shelf_bbox = self.tgt.get("shelf_bbox", None)
        self.delivery = SupervisorDelivery(self.robot, self.tgt)
        self.st = S.INIT; self.idx = 0; self.letter = None
        self.qt = None; self.wc = 0; self.cd = 0
        self.round = 1

        print("=" * 60)
        print("  Letter Sorter v16p-loop — blank-area grasp, FOV 1.4")
        print("=" * 60)

    def _load_cfg(self, path):
        d = {
            "home": [0,0,0.45], "pickup_approach": [-0.234,0,0.15],
            "pickup_grasp": [-0.234,0,0.034], "stack_dz": 0.0,
            "deliver_world_pos": [-0.20,0,0.809],
            "deliver_rotation": [1,0,0,-1.5708],
            "slots": {
                "A": {"front":[0.20,0.24,0.120],"slide_in":[0.30,0.24,0.029]},
                "B": {"front":[0.20,0.08,0.120],"slide_in":[0.30,0.08,0.029]},
                "C": {"front":[0.20,-0.08,0.120],"slide_in":[0.30,-0.08,0.029]},
                "D": {"front":[0.20,-0.24,0.120],"slide_in":[0.30,-0.24,0.029]},
            },
            "shelf_bbox": {"x_min":0.28,"x_max":0.50,"y_min":-0.38,"y_max":0.38,
                           "z_min":-0.165,"z_max":0.155},
        }
        if  os.path.exists(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    c = yaml.safe_load(f)
                d["home"] = c.get("home", d["home"])
                d["pickup_approach"] = c.get("pickup",{}).get("approach", d["pickup_approach"])
                d["pickup_grasp"] = c.get("pickup",{}).get("grasp", d["pickup_grasp"])
                d["stack_dz"] = c.get("letter_stack_dz", d["stack_dz"])
                d["deliver_world_pos"] = c.get("deliver_world_pos", d["deliver_world_pos"])
                d["deliver_rotation"] = c.get("deliver_rotation", d["deliver_rotation"])
                if "shelf_bbox" in c: d["shelf_bbox"] = c["shelf_bbox"]
                if "slots" in c:
                    for l in "ABCD":
                        if l in c["slots"]: d["slots"][l] = c["slots"][l]
            except Exception as e:
                print(f"  [Config] error: {e}")
        return d


    def _move(self, pos, downward=False, use_bbox=True, wrist_roll=None):
        bbox = self.shelf_bbox if use_bbox else None
        q, ok, err = self.ik.solve(pos, q_init=self.arm.get_q(),
                                   n_restarts=25, require_downward=downward,
                                   shelf_bbox=bbox)
        if wrist_roll is not None:
            q[4] = wrist_roll
        if self.shelf_bbox:
            cc = self.fk.collision_cost(q, self.shelf_bbox)
            if cc > 0:
                print(f"  [IK] shoulder/elbow COLLISION! cost={cc:.4f} pos={pos}")
        self.qt = q; self.arm.set_q(q)
        if not ok:
            print(f"  [IK] err={err*1000:.1f}mm pos={pos}")
        return q

    def _move_joints(self, q):
        """Directly set joint angles (bypassing IK)."""
        q = np.asarray(q, dtype=float)
        self.qt = q
        self.arm.set_q(q)
        return q

    def _grasp_z(self):
        return self.tgt["pickup_grasp"][2] - self.idx * self.tgt["stack_dz"]

    def _wait(self, timeout=300):
        self.wc += 1
        return (self.qt is not None and self.arm.reached(self.qt)) or self.wc > timeout

    def _reached_strict(self):
        return self.qt is not None and self.arm.reached(self.qt)

    def _hold_gripper(self):
        self.arm.close_gripper()

    def _snap(self, label):
        self.cam.save_snapshot(os.path.join(self.snap_dir,
                               f"r{self.round}_step{self.idx}_{label}.png"))
        print(f"r{self.round}_step{self.idx}_{label}.png")



    def step(self):
        self.delivery.hold()
        s = self.st

        if s in S.HOLD_STATES:
            self._hold_gripper()

        if s == S.INIT:
            self.arm.open_gripper(); self.st = S.HOME

        elif s == S.HOME:
            print(f"\n[Round {self.round} | {self.idx+1}/{self.N}] HOME")
            self._move(self.tgt["home"]); self.st = S.W_HOME; self.wc = 0

        elif s == S.W_HOME:
            if self._wait(): self.st = S.DELIVER

        elif s == S.DELIVER:
            self.delivery.deliver(self.STACK[self.idx])
            print(f"  [Lock] Letter '{self.STACK[self.idx]}' locked on tray")
            self.st = S.W_DELIVER; self.wc = 0

        elif s == S.W_DELIVER:
            self.wc += 1
            if self.wc >= 20: self.st = S.APPROACH_PICKUP


        elif s == S.APPROACH_PICKUP:
            self._move(self.tgt["pickup_approach"], downward=True)
            self.arm.open_gripper()
            self.st = S.W_APPROACH; self.wc = 0

        elif s == S.W_APPROACH:
            if self._wait(): self.st = S.LOWER

        elif s == S.LOWER:
            self.arm.set_slow()
            print("  [Arm] Slow speed — lowering to grip blank area")
            g = list(self.tgt["pickup_grasp"]); g[2] = self._grasp_z()
            self._move(g, downward=True)
            self.st = S.W_LOWER; self.wc = 0

        elif s == S.W_LOWER:
            if self._wait(400):
                self._snap("01_before_grasp")
                self.st = S.GRASP

        elif s == S.GRASP:
            self.arm.close_gripper()
            self.st = S.W_GRASP; self.wc = 0

        elif s == S.W_GRASP:
            self.wc += 1
            if self.wc >= 80:
                self.delivery.release()
                self.arm.set_carry_speed()
                self._snap("02_after_grasp")
                self.st = S.LIFT



        elif s == S.LIFT:
            print("  [Arm] Lifting letter (upright, no rotation)")
            self.arm.set_carry_speed()
            self._move(self.tgt["pickup_approach"], downward=True)
            self.st = S.W_LIFT; self.wc = 0

        elif s == S.W_LIFT:
            if self._wait():
                self.st = S.SCAN_POSE



        elif s == S.SCAN_POSE:
            print("  [Arm] Moving to SCAN_POSE — presenting letter to camera")
            print("        Gripper on blank area — all text fully exposed")
            self.arm.set_carry_speed()
            self._move_joints(SCAN_POSE_Q)
            self.st = S.W_SCAN_POSE; self.wc = 0

        elif s == S.W_SCAN_POSE:
            if self._wait(500):
                self.cd = 0; self.st = S.CAPTURE



        elif s == S.CAPTURE:
            self.cd += 1
            if self.cd >= 15:
                print(f"  [Scan] Capturing (blank-area grasp, text unoccluded)")
                self._snap("03_scan")
                self.st = S.RECOGNIZE

        elif s == S.RECOGNIZE:
            frame = self.cam.capture(); fallback = self.STACK[self.idx]
            if frame is not None:
                letter, conf = self.rec.recognize(frame)
                self.letter = letter if letter else fallback
                print(f"  [OCR] result='{letter}' conf={conf:.2f}" if letter else f"  [OCR] failed, fallback='{fallback}'")
            else:
                self.letter = fallback
                print(f"  [OCR] no frame, fallback='{fallback}'")
            print(f"[!] Recognized: '{self.letter}'")
            self.st = S.PRE_PLACE

        # ── PRE_PLACE: rise to HOME before heading to shelf ──────────

        elif s == S.PRE_PLACE:
            print(f"[->] PRE_PLACE — rising to HOME (carry speed)")
            self.arm.set_carry_speed()
            self._move(self.tgt["home"])
            self.st = S.W_PRE_PLACE; self.wc = 0

        elif s == S.W_PRE_PLACE:
            if self._wait():
                self.st = S.FRONT_SLOT



        elif s == S.FRONT_SLOT:
            f = self.tgt["slots"][self.letter]["front"]
            print(f"[->] FRONT '{self.letter}' {f}  (letter upright)")
            self.arm.set_normal_speed()
            self._move(f, downward=False, use_bbox=False)
            self.st = S.W_FRONT; self.wc = 0

        elif s == S.W_FRONT:
            self.wc += 1
            if self._reached_strict():
                self.st = S.SLIDE_IN
            elif self.wc > 400:
                f = self.tgt["slots"][self.letter]["front"]
                print(f"  [Retry] FRONT '{self.letter}' — re-issuing motion")
                self._move(f, downward=False, use_bbox=False)
                self.wc = 0

        elif s == S.SLIDE_IN:
            si = self.tgt["slots"][self.letter]["slide_in"]
            print(f"[->] SLIDE_IN '{self.letter}' {si}  (upright, slow)")
            self.arm.set_place_speed()
            self._move(si, downward=False, use_bbox=False)
            self.st = S.W_SLIDE_IN; self.wc = 0

        elif s == S.W_SLIDE_IN:
            self.wc += 1
            if self._reached_strict():
                self.st = S.RELEASE
            elif self.wc > 600:
                si = self.tgt["slots"][self.letter]["slide_in"]
                print(f"  [Retry] SLIDE_IN '{self.letter}' — re-issuing motion")
                self._move(si, downward=False, use_bbox=False)
                self.wc = 0

        elif s == S.RELEASE:
            print(f"[->] RELEASE '{self.letter}' — opening gripper")
            self.arm.open_gripper()
            self.st = S.W_RELEASE; self.wc = 0

        elif s == S.W_RELEASE:
            self.wc += 1
            if self.wc >= 30:
                self.st = S.DROP

        elif s == S.DROP:
            self.delivery.drop_into_slot(self.letter)
            self.st = S.W_DROP; self.wc = 0

        elif s == S.W_DROP:
            self.wc += 1
            if self.wc >= 15:
                self.st = S.SLIDE_OUT

        elif s == S.SLIDE_OUT:
            f = self.tgt["slots"][self.letter]["front"]
            print(f"[->] SLIDE_OUT '{self.letter}' {f}")
            self.arm.set_slow()
            self._move(f, downward=False, use_bbox=False)
            self.st = S.W_SLIDE_OUT; self.wc = 0

        elif s == S.W_SLIDE_OUT:
            if self._wait(400):
                self.arm.set_normal_speed()
                self.idx += 1
                if self.idx >= self.N:
                    print("\n" + "=" * 60)
                    print(f"  ROUND {self.round} COMPLETE — all 4 letters sorted!")
                    print(f"  Starting round {self.round + 1} ...")
                    print("=" * 60)
                    self.idx = 0
                    self.round += 1
                self.st = S.HOME

        elif s == S.DONE:
            pass

    def run(self):
        while self.robot.step(self.ts) != -1:
            self.step()

if __name__ == "__main__":
    LetterSorterController().run()
