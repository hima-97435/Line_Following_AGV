"""PyBullet world: textured tape floor + differential-drive AGV.

The robot is generated as a URDF from ``RobotParams`` so that the kinematic
constants used by the controller (wheel radius, track width) are exactly the
ones simulated by the physics engine.
"""
import hashlib
import math
import os
import tempfile

import numpy as np
import pybullet as pb
from pybullet_utils import bullet_client

from .config import SimConfig
from .track import Track

CACHE_DIR = os.path.join(tempfile.gettempdir(), "agv_sim_cache")
TEXTURE_VERSION = "v2"   # bump when the texture rasterisation changes

URDF_TEMPLATE = """<?xml version="1.0"?>
<robot name="agv">
  <material name="body"><color rgba="0.95 0.55 0.10 1"/></material>
  <material name="dark"><color rgba="0.15 0.15 0.15 1"/></material>
  <material name="green"><color rgba="0.10 0.60 0.25 1"/></material>
  <link name="base_link">
    <inertial>
      <origin xyz="0.02 0 0.03"/>
      <mass value="{mass}"/>
      <inertia ixx="{ixx}" iyy="{iyy}" izz="{izz}" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <origin xyz="0.02 0 0.03"/>
      <geometry><box size="0.30 0.20 0.05"/></geometry>
      <material name="body"/>
    </visual>
    <collision>
      <origin xyz="0.02 0 0.03"/>
      <geometry><box size="0.30 0.20 0.05"/></geometry>
    </collision>
    <!-- IR sensor bar (visual only) -->
    <visual>
      <origin xyz="{look} 0 {bar_z}"/>
      <geometry><box size="0.015 {bar_w} 0.008"/></geometry>
      <material name="green"/>
    </visual>
    <!-- camera mast and head (visual only) -->
    <visual>
      <origin xyz="{cam_x} 0 {mast_z}"/>
      <geometry><box size="0.015 0.015 {mast_h}"/></geometry>
      <material name="dark"/>
    </visual>
    <visual>
      <origin xyz="{cam_x} 0 {cam_z}"/>
      <geometry><box size="0.03 0.04 0.025"/></geometry>
      <material name="dark"/>
    </visual>
  </link>
  {wheels}
  {casters}
</robot>
"""

WHEEL = """
  <link name="{name}">
    <inertial><mass value="0.25"/>
      <inertia ixx="1.2e-4" iyy="2.0e-4" izz="1.2e-4" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{r}" length="0.025"/></geometry><material name="dark"/></visual>
    <collision><origin rpy="1.5708 0 0"/>
      <geometry><cylinder radius="{r}" length="0.025"/></geometry></collision>
  </link>
  <joint name="{name}_joint" type="continuous">
    <parent link="base_link"/><child link="{name}"/>
    <origin xyz="0 {y} 0"/><axis xyz="0 1 0"/>
  </joint>"""

CASTER = """
  <link name="{name}">
    <inertial><mass value="0.05"/>
      <inertia ixx="5e-6" iyy="5e-6" izz="5e-6" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><sphere radius="{cr}"/></geometry><material name="dark"/></visual>
    <collision><geometry><sphere radius="{cr}"/></geometry></collision>
  </link>
  <joint name="{name}_joint" type="fixed">
    <parent link="base_link"/><child link="{name}"/>
    <origin xyz="{x} 0 {z}"/>
  </joint>"""


def build_urdf(cfg: SimConfig) -> str:
    rp, sp = cfg.robot, cfg.sensor
    r = rp.wheel_radius
    m = rp.chassis_mass
    a, b, c = 0.30, 0.20, 0.05
    caster_r = 0.015
    cam_h_local = sp.cam_height_m - r            # base frame sits on the axle
    wheels = WHEEL.format(name="left_wheel", r=r, y=rp.track_width / 2) + \
        WHEEL.format(name="right_wheel", r=r, y=-rp.track_width / 2)
    casters = CASTER.format(name="front_caster", cr=caster_r, x=0.13, z=-(r - caster_r)) + \
        CASTER.format(name="rear_caster", cr=caster_r, x=-0.10, z=-(r - caster_r))
    return URDF_TEMPLATE.format(
        mass=m, ixx=m * (b * b + c * c) / 12, iyy=m * (a * a + c * c) / 12, izz=m * (a * a + b * b) / 12,
        look=sp.lookahead, bar_z=-(r - 0.012), bar_w=sp.n_ir * sp.ir_pitch + 0.01,
        cam_x=sp.cam_x, mast_z=0.055 + (cam_h_local - 0.055) / 2, mast_h=max(cam_h_local - 0.055, 0.01),
        cam_z=cam_h_local, wheels=wheels, casters=casters)


def _floor_assets(track: Track):
    """Write (once) the floor texture PNG and a UV-mapped quad OBJ for a track."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    key = hashlib.md5((repr(track.segments) + TEXTURE_VERSION).encode()).hexdigest()[:8]
    png = os.path.join(CACHE_DIR, f"floor_{track.name}_{key}.png")
    obj = os.path.join(CACHE_DIR, f"floor_{track.name}_{key}.obj")
    lo, hi = track.bounds()
    if not (os.path.exists(png) and os.path.exists(obj)):
        # write to per-process temp names, then rename atomically (parallel workers)
        tmp = f".{os.getpid()}"
        track.render_texture(path=png + tmp + ".png")
        with open(obj + tmp, "w") as f:
            f.write(f"v {lo[0]} {lo[1]} 0\nv {hi[0]} {lo[1]} 0\nv {hi[0]} {hi[1]} 0\nv {lo[0]} {hi[1]} 0\n")
            f.write("vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n")
            f.write("f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")
        os.replace(png + tmp + ".png", png)
        os.replace(obj + tmp, obj)
    return png, obj


class World:
    def __init__(self, cfg: SimConfig, track: Track):
        self.cfg = cfg
        self.track = track
        self.p = bullet_client.BulletClient(connection_mode=pb.GUI if cfg.gui else pb.DIRECT)
        p = self.p
        if cfg.gui:
            p.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(cfg.physics_dt)
        p.setPhysicsEngineParameter(numSolverIterations=50)

        # --- floor: infinite collision plane + textured visual quad
        png, obj = _floor_assets(track)
        col = p.createCollisionShape(pb.GEOM_PLANE)
        vis = p.createVisualShape(pb.GEOM_MESH, fileName=obj, meshScale=[1, 1, 1])
        self.floor = p.createMultiBody(0, col, vis, basePosition=[0, 0, 0])
        tex = p.loadTexture(png)
        p.changeVisualShape(self.floor, -1, textureUniqueId=tex, rgbaColor=[1, 1, 1, 1])
        p.changeDynamics(self.floor, -1, lateralFriction=1.0)

        # --- robot
        urdf_path = os.path.join(CACHE_DIR, f"agv_{os.getpid()}.urdf")
        os.makedirs(CACHE_DIR, exist_ok=True)
        with open(urdf_path, "w") as f:
            f.write(build_urdf(cfg))
        x0, y0, yaw0 = self.start_pose()
        self.robot = p.loadURDF(urdf_path, [x0, y0, cfg.robot.wheel_radius + 0.002],
                                p.getQuaternionFromEuler([0, 0, yaw0]))
        self.inertial_pos = np.asarray(p.getDynamicsInfo(self.robot, -1)[3])
        self.reset_pose(x0, y0, yaw0)
        names = {p.getJointInfo(self.robot, j)[1].decode(): j for j in range(p.getNumJoints(self.robot))}
        self.jl, self.jr = names["left_wheel_joint"], names["right_wheel_joint"]
        for j in (names["front_caster_joint"], names["rear_caster_joint"]):
            p.changeDynamics(self.robot, j, lateralFriction=0.0, rollingFriction=0.0, spinningFriction=0.0)
        for j in (self.jl, self.jr):
            p.changeDynamics(self.robot, j, lateralFriction=cfg.robot.floor_friction)
        if cfg.robot.payload_mass > 0:
            p.changeDynamics(self.robot, -1, mass=cfg.robot.chassis_mass + cfg.robot.payload_mass)
        # actuator state (first-order motor lag)
        self.w_act = np.zeros(2)
        self.w_cmd = np.zeros(2)
        for _ in range(60):          # let the robot settle on its wheels
            self._apply_wheels()
            p.stepSimulation()

    def start_pose(self):
        t = self.track
        th = t.heading[0]
        n = t.normal[0]
        x, y = t.pts[0] + n * self.cfg.initial_offset
        return float(x), float(y), float(th + self.cfg.initial_heading)

    # --------------------------------------------------------------- actuation
    def set_wheel_targets(self, w_left, w_right):
        lim = self.cfg.robot.wheel_max_speed
        self.w_cmd[:] = np.clip([w_left, w_right], -lim, lim)

    def _apply_wheels(self):
        p = self.p
        tau = max(self.cfg.robot.motor_tau, 1e-6)
        self.w_act += (self.cfg.physics_dt / tau) * (self.w_cmd - self.w_act)
        f = self.cfg.robot.wheel_max_torque
        p.setJointMotorControl2(self.robot, self.jl, pb.VELOCITY_CONTROL, targetVelocity=self.w_act[0], force=f)
        p.setJointMotorControl2(self.robot, self.jr, pb.VELOCITY_CONTROL, targetVelocity=self.w_act[1], force=f)

    def step(self, n=1):
        for _ in range(n):
            self._apply_wheels()
            self.p.stepSimulation()

    # ------------------------------------------------------------------ state
    def pose(self):
        """Pose of the URDF base link (axle centre).

        PyBullet reports the base *centre-of-mass* frame, which the URDF places
        2 cm ahead of and 3 cm above the axle, so the inertial offset is removed.
        """
        com, orn = self.p.getBasePositionAndOrientation(self.robot)
        R = np.asarray(self.p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        pos = np.asarray(com) - R @ self.inertial_pos
        yaw = self.p.getEulerFromQuaternion(orn)[2]
        return np.array([pos[0], pos[1], yaw]), tuple(pos), orn

    def reset_pose(self, x, y, yaw):
        """Place the base link (axle centre) at (x, y, yaw) on the floor."""
        orn = self.p.getQuaternionFromEuler([0, 0, yaw])
        R = np.asarray(self.p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        com = np.array([x, y, self.cfg.robot.wheel_radius + 0.001]) + R @ self.inertial_pos
        self.p.resetBasePositionAndOrientation(self.robot, com.tolist(), orn)

    def velocity(self):
        lin, ang = self.p.getBaseVelocity(self.robot)
        _, _, orn = self.pose()
        yaw = self.p.getEulerFromQuaternion(orn)[2]
        v = lin[0] * math.cos(yaw) + lin[1] * math.sin(yaw)
        return v, ang[2]

    def wheel_speeds(self):
        return np.array([self.p.getJointState(self.robot, self.jl)[1],
                         self.p.getJointState(self.robot, self.jr)[1]])

    def close(self):
        try:
            self.p.disconnect()
        except Exception:
            pass
