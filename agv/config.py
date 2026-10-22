"""Central parameter definitions for the line-following AGV simulation.

Every experiment builds a ``SimConfig`` (optionally overriding fields) so that
all numbers that appear in the report can be traced back to this file.
"""
from dataclasses import dataclass, field, replace
from typing import Optional


@dataclass
class RobotParams:
    wheel_radius: float = 0.040        # m
    track_width: float = 0.240         # m, distance between wheel contact points
    chassis_mass: float = 4.0          # kg (empty AGV)
    payload_mass: float = 0.0          # kg, extra load carried on the deck
    wheel_max_speed: float = 25.0      # rad/s  -> 1.0 m/s top linear speed
    wheel_max_torque: float = 2.5      # N m per wheel motor
    motor_tau: float = 0.060           # s, first-order lag of the motor speed loop
    floor_friction: float = 0.9        # wheel/floor Coulomb friction coefficient


@dataclass
class SensorParams:
    kind: str = "ir"                   # "ir" (ray-cast reflectance array) or "camera"
    n_ir: int = 8                      # number of IR reflectance sensors
    ir_pitch: float = 0.014            # m, spacing between neighbouring sensors
    ir_spot_radius: float = 0.006      # m, radius of the IR footprint on the floor
    lookahead: float = 0.150           # m, sensor bar distance ahead of the wheel axle
    noise_std: float = 0.02            # reflectance noise (fraction of full scale)
    adc_bits: int = 10                 # ADC resolution
    lost_threshold: float = 0.35       # darkness below this = line lost
    ir_method: str = "window"          # "window": centroid of peak sensor +-1, "centroid": all sensors
    # camera
    cam_width: int = 80
    cam_height: int = 60
    cam_fov_deg: float = 70.0
    cam_height_m: float = 0.18         # camera height above floor
    cam_pitch_deg: float = 55.0        # downward pitch from horizontal
    cam_x: float = 0.19                # camera position ahead of the axle (front bumper)
    cam_lookahead: float = 0.28        # m, where the camera evaluates the lateral error

    @property
    def effective_lookahead(self) -> float:
        return self.cam_lookahead if self.kind == "camera" else self.lookahead

    @property
    def half_span(self) -> float:
        """Largest lateral line offset the sensor can measure at its look-ahead."""
        return 0.12 if self.kind == "camera" else (self.n_ir - 1) / 2 * self.ir_pitch


@dataclass
class ControlParams:
    controller: str = "pid"            # pid | bangbang | fuzzy | relay
    kp: float = 6.0                    # rad/s per metre of lateral error
    ki: float = 0.0
    kd: float = 0.0
    d_filter_n: float = 15.0           # derivative filter coefficient N (rad/s)
    omega_max: float = 5.0             # rad/s, commanded yaw-rate saturation
    base_speed: float = 0.40           # m/s
    adaptive_speed: bool = False
    v_min: float = 0.15                # m/s, floor for adaptive speed
    speed_gain_e: float = 6.0          # slow-down per metre of |error|
    speed_gain_w: float = 0.18         # slow-down per rad/s of |omega|
    gain_schedule: bool = False        # look-ahead gain scheduling: kp = kp_factor * 2 v / L^2
    kp_factor: float = 1.0
    edge_guard: bool = False           # extra steering when the line nears the sensor edge
    edge_frac: float = 0.65            # guard starts at this fraction of the sensor half-span
    edge_gain: float = 150.0           # rad/s per metre beyond the guard threshold
    use_kalman: bool = False
    kf_q: float = 4.0                  # process noise spectral density (m/s^2)^2
    kf_r: float = 4.0e-5               # measurement variance (m^2), ~6 mm std
    bangbang_deadband: float = 0.004   # m
    fuzzy_e_scale: float = 0.05        # m, error mapped to +-1
    fuzzy_de_scale: float = 0.30       # m/s, error rate mapped to +-1
    relay_amp: float = 1.5             # rad/s, relay output for auto-tuning
    relay_hyst: float = 0.001          # m, relay hysteresis


@dataclass
class SimConfig:
    track: str = "oval"
    robot: RobotParams = field(default_factory=RobotParams)
    sensor: SensorParams = field(default_factory=SensorParams)
    control: ControlParams = field(default_factory=ControlParams)
    physics_dt: float = 1.0 / 240.0
    control_every: int = 4             # control at 60 Hz
    max_time: float = 120.0            # s
    initial_offset: float = 0.0        # m, lateral start offset (step-response tests)
    initial_heading: float = 0.0       # rad
    fail_distance: float = 0.20        # m, cross-track error that counts as derailment
    lost_timeout: float = 2.0          # s with no line before the run is aborted
    seed: int = 0
    gui: bool = False
    record_every: int = 1              # log every n-th control step
    laps: int = 1                      # for closed tracks
    stop_at_distance: Optional[float] = None  # m along track (overrides lap end)

    @property
    def control_dt(self) -> float:
        return self.physics_dt * self.control_every


def with_overrides(cfg: SimConfig, **kw) -> SimConfig:
    """Return a copy of ``cfg`` with dotted overrides, e.g. control__kp=3."""
    robot, sensor, control = cfg.robot, cfg.sensor, cfg.control
    top = {}
    for k, v in kw.items():
        if k.startswith("robot__"):
            robot = replace(robot, **{k[7:]: v})
        elif k.startswith("sensor__"):
            sensor = replace(sensor, **{k[8:]: v})
        elif k.startswith("control__"):
            control = replace(control, **{k[9:]: v})
        else:
            top[k] = v
    return replace(cfg, robot=robot, sensor=sensor, control=control, **top)
