"""
===============================================================================
COMPLETE FLYING OBJECT BLUEPRINT SYSTEM
Includes: base blueprint system + all extensions (material library, performance
estimator, optimizer, version manager, simulation integration, exporters, CLI)
===============================================================================
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Union, Callable
from abc import ABC, abstractmethod
import json
from enum import Enum
import hashlib
from datetime import datetime
import copy
import os
import glob
import tempfile
import csv

# ============================================================================
#  PART 1: FLYING OBJECT SIMULATION FRAMEWORK (from original)
# ============================================================================

def euler_to_rotation(phi: float, theta: float, psi: float) -> np.ndarray:
    """Return rotation matrix from body to inertial frame (ZYX convention)."""
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    R = np.array([
        [cth*cpsi, sphi*sth*cpsi - cphi*spsi, cphi*sth*cpsi + sphi*spsi],
        [cth*spsi, sphi*sth*spsi + cphi*cpsi, cphi*sth*spsi - sphi*cpsi],
        [-sth,     sphi*cth,               cphi*cth]
    ])
    return R

@dataclass
class Environment:
    """Atmospheric and gravity model."""
    rho0: float = 1.225        # sea level air density [kg/m^3]
    g: np.ndarray = field(default_factory=lambda: np.array([0, 0, -9.81]))
    wind: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def air_density(self, altitude: float) -> float:
        return self.rho0 * np.exp(-altitude / 8500.0)

class FlyingObject(ABC):
    """Abstract base class for any flying vehicle."""
    def __init__(self,
                 mass: float,
                 inertia: np.ndarray,
                 env: Environment,
                 initial_state: Optional[np.ndarray] = None):
        self.mass = mass
        self.inertia = inertia
        self.inv_inertia = np.linalg.inv(inertia)
        self.env = env
        if initial_state is None:
            self.state = np.zeros(12)
            self.state[2] = -100.0
        else:
            self.state = initial_state.copy()
        self.controls = np.zeros(self.control_dim())

    @property
    def position(self) -> np.ndarray:
        return self.state[0:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.state[3:6]

    @property
    def euler(self) -> np.ndarray:
        return self.state[6:9]

    @property
    def angular_rates(self) -> np.ndarray:
        return self.state[9:12]

    def rotation_matrix(self) -> np.ndarray:
        return euler_to_rotation(*self.euler)

    def air_velocity(self) -> np.ndarray:
        return self.velocity - self.env.wind

    @abstractmethod
    def control_dim(self) -> int:
        pass

    @abstractmethod
    def compute_forces_and_moments(self, state: np.ndarray, controls: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pass

    def derivatives(self, t: float, state: np.ndarray) -> np.ndarray:
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
        R = euler_to_rotation(phi, theta, psi)
        F_body, M_body = self.compute_forces_and_moments(state, self.controls)
        gravity_body = R.T @ self.env.g
        F_body += self.mass * gravity_body
        accel_inertial = R @ (F_body / self.mass)
        omega = np.array([p, q, r])
        omega_dot = self.inv_inertia @ (M_body - np.cross(omega, self.inertia @ omega))
        phi_dot = p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
        theta_dot = q * np.cos(phi) - r * np.sin(phi)
        psi_dot = (q * np.sin(phi) + r * np.cos(phi)) / np.cos(theta)
        deriv = np.zeros_like(state)
        deriv[0:3] = state[3:6]
        deriv[3:6] = accel_inertial
        deriv[6:9] = [phi_dot, theta_dot, psi_dot]
        deriv[9:12] = omega_dot
        return deriv

    def step(self, dt: float, method: str = 'rk4') -> None:
        if method == 'euler':
            self.state += dt * self.derivatives(0, self.state)
        elif method == 'rk4':
            k1 = self.derivatives(0, self.state)
            k2 = self.derivatives(dt/2, self.state + 0.5*dt*k1)
            k3 = self.derivatives(dt/2, self.state + 0.5*dt*k2)
            k4 = self.derivatives(dt, self.state + dt*k3)
            self.state += (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        else:
            raise ValueError(f"Unknown method: {method}")

@dataclass
class AirplaneParams:
    mass: float = 10.0
    wing_area: float = 2.0
    span: float = 4.0
    chord: float = 0.5
    CL0: float = 0.2
    CL_alpha: float = 5.0
    CD0: float = 0.03
    CD_induced: float = 0.05
    CM0: float = 0.0
    CM_alpha: float = -0.5
    CM_q: float = -5.0
    thrust_max: float = 100.0
    inertia: np.ndarray = field(default_factory=lambda: np.diag([1.0, 5.0, 5.0]))

class Airplane(FlyingObject):
    def __init__(self, params: AirplaneParams, env: Environment, initial_state=None):
        self.params = params
        super().__init__(params.mass, params.inertia, env, initial_state)

    def control_dim(self) -> int:
        return 3

    def compute_forces_and_moments(self, state, controls):
        phi, theta, psi = state[6:9]
        p, q, r = state[9:12]
        v_air = self.air_velocity()
        V = np.linalg.norm(v_air)
        if V < 1e-6:
            return np.zeros(3), np.zeros(3)
        R = euler_to_rotation(phi, theta, psi)
        v_body = R.T @ v_air
        alpha = np.arctan2(v_body[2], v_body[0])
        beta = np.arcsin(v_body[1] / V)
        rho = self.env.air_density(-state[2])
        qbar = 0.5 * rho * V**2
        CL = self.params.CL0 + self.params.CL_alpha * alpha
        CD = self.params.CD0 + self.params.CD_induced * CL**2
        CM = self.params.CM0 + self.params.CM_alpha * alpha + self.params.CM_q * (q * self.params.chord / (2*V))
        S = self.params.wing_area
        F_wind = np.array([-CD * qbar * S, 0, -CL * qbar * S])
        c, s = np.cos(alpha), np.sin(alpha)
        R_wind_to_body = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
        F_aero_body = R_wind_to_body @ F_wind
        throttle = controls[0]
        thrust = throttle * self.params.thrust_max
        F_thrust = np.array([thrust, 0, 0])
        elevator = controls[1]
        aileron = controls[2]
        Cl_da = 0.1
        Cm_de = -0.5
        Mx = qbar * S * self.params.span * (Cl_da * aileron)
        My = qbar * S * self.params.chord * (CM + Cm_de * elevator)
        Mz = 0.0
        return F_aero_body + F_thrust, np.array([Mx, My, Mz])

class Simulator:
    def __init__(self, vehicle: FlyingObject, dt: float = 0.01):
        self.vehicle = vehicle
        self.dt = dt
        self.history = None
        self.t = 0.0

    def run(self, duration: float, controller: Optional[Callable] = None) -> np.ndarray:
        steps = int(duration / self.dt)
        self.history = np.zeros((steps+1, 13))
        self.history[0, 0] = self.t
        self.history[0, 1:] = self.vehicle.state
        for i in range(steps):
            if controller is not None:
                self.vehicle.controls = controller(self.vehicle, self.t)
            self.vehicle.step(self.dt)
            self.t += self.dt
            self.history[i+1, 0] = self.t
            self.history[i+1, 1:] = self.vehicle.state
        return self.history

# ============================================================================
#  PART 2: BLUEPRINT SYSTEM (original)
# ============================================================================

class ComponentType(Enum):
    WING = "wing"
    FUSELAGE = "fuselage"
    MOTOR = "motor"
    CONTROL_SURFACE = "control_surface"
    BATTERY = "battery"
    LANDING_GEAR = "landing_gear"
    AVIONICS = "avionics"
    PAYLOAD = "payload"
    PROPELLER = "propeller"
    FUEL_TANK = "fuel_tank"

class WingShape(Enum):
    RECTANGULAR = "rectangular"
    TAPERED = "tapered"
    ELLIPTICAL = "elliptical"
    SWEPT = "swept"
    DELTA = "delta"

class AirfoilType(Enum):
    SYMMETRIC = "symmetric"
    CAMBERED = "cambered"
    HIGH_LIFT = "high_lift"
    SUPERCRITICAL = "supercritical"
    LAMINAR = "laminar"

@dataclass
class ComponentBlueprint:
    id: str
    name: str
    component_type: ComponentType
    mass: float
    material: str = "aluminum"
    cost: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def get_id(self) -> str:
        return self.id

@dataclass
class WingBlueprint(ComponentBlueprint):
    span: float
    root_chord: float
    tip_chord: float
    sweep_angle: float = 0.0
    dihedral: float = 0.0
    airfoil: AirfoilType = AirfoilType.SYMMETRIC
    aspect_ratio: float = 0.0
    taper_ratio: float = 1.0

    def __post_init__(self):
        self.component_type = ComponentType.WING
        if self.aspect_ratio == 0 and self.span > 0:
            avg_chord = (self.root_chord + self.tip_chord) / 2
            self.aspect_ratio = self.span / avg_chord
        if self.taper_ratio == 1.0 and self.tip_chord != self.root_chord:
            self.taper_ratio = self.tip_chord / self.root_chord

    @property
    def wing_area(self) -> float:
        return (self.root_chord + self.tip_chord) / 2 * self.span

    @property
    def mean_aerodynamic_chord(self) -> float:
        return (2/3) * self.root_chord * (1 + self.taper_ratio + self.taper_ratio**2) / (1 + self.taper_ratio)

@dataclass
class MotorBlueprint(ComponentBlueprint):
    thrust_max: float
    power_max: float
    efficiency: float = 0.85
    motor_type: str = "electric"
    kv_rating: float = 0.0
    rpm_max: float = 0.0

    def __post_init__(self):
        self.component_type = ComponentType.MOTOR

@dataclass
class ControlSurfaceBlueprint(ComponentBlueprint):
    surface_type: str
    area: float
    chord: float
    deflection_range: Tuple[float, float] = (-30.0, 30.0)
    effectiveness: float = 1.0
    hinge_moment: float = 0.0

    def __post_init__(self):
        self.component_type = ComponentType.CONTROL_SURFACE

@dataclass
class BatteryBlueprint(ComponentBlueprint):
    capacity: float
    voltage: float
    discharge_rate: float = 1.0
    chemistry: str = "LiPo"
    cells: int = 0

    def __post_init__(self):
        self.component_type = ComponentType.BATTERY
        self.cells = int(self.voltage / 3.7) if self.cells == 0 else self.cells

    @property
    def energy(self) -> float:
        return self.capacity * self.voltage

# --- extended components ---
@dataclass
class PropellerBlueprint(ComponentBlueprint):
    diameter: float
    pitch: float
    blade_count: int = 2
    efficiency: float = 0.8

    def __post_init__(self):
        self.component_type = ComponentType.PROPELLER

@dataclass
class LandingGearBlueprint(ComponentBlueprint):
    type: str = "tricycle"
    weight_on_nose: float = 0.1
    tire_diameter: float = 0.3
    shock_stroke: float = 0.1

    def __post_init__(self):
        self.component_type = ComponentType.LANDING_GEAR

@dataclass
class AvionicsBlueprint(ComponentBlueprint):
    gps: bool = True
    imu: bool = True
    magnetometer: bool = True
    airspeed_sensor: bool = True
    autopilot: bool = False
    communication_range: float = 10.0

    def __post_init__(self):
        self.component_type = ComponentType.AVIONICS

@dataclass
class FuelTankBlueprint(ComponentBlueprint):
    capacity: float
    fuel_type: str = "avgas"

    def __post_init__(self):
        self.component_type = ComponentType.FUEL_TANK

# ----------------------------------------------------------------------------
# VehicleBlueprint
# ----------------------------------------------------------------------------
@dataclass
class VehicleBlueprint:
    id: str
    name: str
    version: str = "1.0"
    description: str = ""
    author: str = ""
    created: str = field(default_factory=lambda: datetime.now().isoformat())
    modified: str = field(default_factory=lambda: datetime.now().isoformat())
    components: Dict[str, ComponentBlueprint] = field(default_factory=dict)
    mass_empty: float = 0.0
    max_payload_mass: float = 0.0
    fuel_capacity: float = 0.0
    wing_config: Optional[Dict] = None
    propulsion_config: Optional[Dict] = None
    control_config: Optional[Dict] = None
    cd0: float = 0.03
    cl_max: float = 1.5
    cl_alpha: float = 5.0
    cm_alpha: float = -0.5
    cm_q: float = -5.0
    inertia: Optional[List[List[float]]] = None
    max_speed: float = 100.0
    max_altitude: float = 5000.0
    max_g_load: float = 4.0
    tags: List[str] = field(default_factory=list)
    notes: str = ""
    signature: str = ""

    def __post_init__(self):
        if self.inertia is None:
            self.inertia = [[10.0, 0, 0], [0, 50.0, 0], [0, 0, 50.0]]
        self.update_mass()
        self.generate_signature()

    def update_mass(self):
        total_mass = 0.0
        for comp in self.components.values():
            total_mass += comp.mass
        self.mass_empty = total_mass

    def add_component(self, component: ComponentBlueprint):
        self.components[component.id] = component
        self.update_mass()
        self.modified = datetime.now().isoformat()

    def remove_component(self, component_id: str):
        if component_id in self.components:
            del self.components[component_id]
            self.update_mass()
            self.modified = datetime.now().isoformat()

    def get_component_by_type(self, comp_type: ComponentType) -> List[ComponentBlueprint]:
        return [c for c in self.components.values() if c.component_type == comp_type]

    def get_wing(self) -> Optional[WingBlueprint]:
        wings = self.get_component_by_type(ComponentType.WING)
        return wings[0] if wings else None

    def get_motor(self) -> Optional[MotorBlueprint]:
        motors = self.get_component_by_type(ComponentType.MOTOR)
        return motors[0] if motors else None

    def generate_signature(self):
        data = f"{self.id}:{self.name}:{self.version}:{self.mass_empty}"
        for comp_id, comp in sorted(self.components.items()):
            data += f":{comp_id}:{comp.mass}"
        self.signature = hashlib.md5(data.encode()).hexdigest()[:8]

    def validate(self) -> Tuple[bool, List[str]]:
        errors = []
        wings = self.get_component_by_type(ComponentType.WING)
        if len(wings) == 0:
            errors.append("No wing component found")
        motors = self.get_component_by_type(ComponentType.MOTOR)
        if len(motors) == 0:
            errors.append("No motor component found")
        computed_mass = sum(c.mass for c in self.components.values())
        if abs(computed_mass - self.mass_empty) > 1e-6:
            errors.append(f"Mass mismatch: components sum = {computed_mass}, empty mass = {self.mass_empty}")
        wing = self.get_wing()
        if wing:
            wing_loading = self.mass_empty / wing.wing_area * 9.81
            if wing_loading > 1000:
                errors.append(f"High wing loading: {wing_loading:.1f} N/m²")
        motor = self.get_motor()
        if motor:
            tw_ratio = motor.thrust_max / (self.mass_empty * 9.81)
            if tw_ratio < 0.5:
                errors.append(f"Low thrust-to-weight ratio: {tw_ratio:.2f}")
            if tw_ratio > 3.0:
                errors.append(f"Very high thrust-to-weight ratio: {tw_ratio:.2f}")
        return len(errors) == 0, errors

    def to_dict(self) -> Dict[str, Any]:
        data = {
            "id": self.id,
            "name": self.name,
            "version": self.version,
            "description": self.description,
            "author": self.author,
            "created": self.created,
            "modified": self.modified,
            "mass_empty": self.mass_empty,
            "max_payload_mass": self.max_payload_mass,
            "fuel_capacity": self.fuel_capacity,
            "cd0": self.cd0,
            "cl_max": self.cl_max,
            "cl_alpha": self.cl_alpha,
            "cm_alpha": self.cm_alpha,
            "cm_q": self.cm_q,
            "inertia": self.inertia,
            "max_speed": self.max_speed,
            "max_altitude": self.max_altitude,
            "max_g_load": self.max_g_load,
            "tags": self.tags,
            "notes": self.notes,
            "signature": self.signature,
            "components": {}
        }
        for comp_id, comp in self.components.items():
            comp_data = {
                "type": comp.component_type.value,
                "name": comp.name,
                "mass": comp.mass,
                "material": comp.material,
                "cost": comp.cost,
                "metadata": comp.metadata
            }
            if isinstance(comp, WingBlueprint):
                comp_data.update({
                    "span": comp.span,
                    "root_chord": comp.root_chord,
                    "tip_chord": comp.tip_chord,
                    "sweep_angle": comp.sweep_angle,
                    "dihedral": comp.dihedral,
                    "airfoil": comp.airfoil.value,
                    "aspect_ratio": comp.aspect_ratio,
                    "taper_ratio": comp.taper_ratio
                })
            elif isinstance(comp, MotorBlueprint):
                comp_data.update({
                    "thrust_max": comp.thrust_max,
                    "power_max": comp.power_max,
                    "efficiency": comp.efficiency,
                    "motor_type": comp.motor_type,
                    "kv_rating": comp.kv_rating,
                    "rpm_max": comp.rpm_max
                })
            elif isinstance(comp, ControlSurfaceBlueprint):
                comp_data.update({
                    "surface_type": comp.surface_type,
                    "area": comp.area,
                    "chord": comp.chord,
                    "deflection_range": comp.deflection_range,
                    "effectiveness": comp.effectiveness,
                    "hinge_moment": comp.hinge_moment
                })
            elif isinstance(comp, BatteryBlueprint):
                comp_data.update({
                    "capacity": comp.capacity,
                    "voltage": comp.voltage,
                    "discharge_rate": comp.discharge_rate,
                    "chemistry": comp.chemistry,
                    "cells": comp.cells
                })
            elif isinstance(comp, PropellerBlueprint):
                comp_data.update({
                    "diameter": comp.diameter,
                    "pitch": comp.pitch,
                    "blade_count": comp.blade_count,
                    "efficiency": comp.efficiency
                })
            elif isinstance(comp, LandingGearBlueprint):
                comp_data.update({
                    "type": comp.type,
                    "weight_on_nose": comp.weight_on_nose,
                    "tire_diameter": comp.tire_diameter,
                    "shock_stroke": comp.shock_stroke
                })
            elif isinstance(comp, AvionicsBlueprint):
                comp_data.update({
                    "gps": comp.gps,
                    "imu": comp.imu,
                    "magnetometer": comp.magnetometer,
                    "airspeed_sensor": comp.airspeed_sensor,
                    "autopilot": comp.autopilot,
                    "communication_range": comp.communication_range
                })
            elif isinstance(comp, FuelTankBlueprint):
                comp_data.update({
                    "capacity": comp.capacity,
                    "fuel_type": comp.fuel_type
                })
            data["components"][comp_id] = comp_data
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'VehicleBlueprint':
        blueprint = cls(
            id=data["id"],
            name=data["name"],
            version=data.get("version", "1.0"),
            description=data.get("description", ""),
            author=data.get("author", ""),
            mass_empty=data.get("mass_empty", 0.0),
            max_payload_mass=data.get("max_payload_mass", 0.0),
            fuel_capacity=data.get("fuel_capacity", 0.0),
            cd0=data.get("cd0", 0.03),
            cl_max=data.get("cl_max", 1.5),
            cl_alpha=data.get("cl_alpha", 5.0),
            cm_alpha=data.get("cm_alpha", -0.5),
            cm_q=data.get("cm_q", -5.0),
            inertia=data.get("inertia", [[10,0,0],[0,50,0],[0,0,50]]),
            max_speed=data.get("max_speed", 100.0),
            max_altitude=data.get("max_altitude", 5000.0),
            max_g_load=data.get("max_g_load", 4.0),
            tags=data.get("tags", []),
            notes=data.get("notes", ""),
            signature=data.get("signature", "")
        )
        for comp_id, comp_data in data.get("components", {}).items():
            comp_type = ComponentType(comp_data["type"])
            if comp_type == ComponentType.WING:
                comp = WingBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {}),
                    span=comp_data["span"],
                    root_chord=comp_data["root_chord"],
                    tip_chord=comp_data["tip_chord"],
                    sweep_angle=comp_data.get("sweep_angle", 0.0),
                    dihedral=comp_data.get("dihedral", 0.0),
                    airfoil=AirfoilType(comp_data.get("airfoil", "symmetric")),
                    aspect_ratio=comp_data.get("aspect_ratio", 0.0),
                    taper_ratio=comp_data.get("taper_ratio", 1.0)
                )
            elif comp_type == ComponentType.MOTOR:
                comp = MotorBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {}),
                    thrust_max=comp_data["thrust_max"],
                    power_max=comp_data["power_max"],
                    efficiency=comp_data.get("efficiency", 0.85),
                    motor_type=comp_data.get("motor_type", "electric"),
                    kv_rating=comp_data.get("kv_rating", 0.0),
                    rpm_max=comp_data.get("rpm_max", 0.0)
                )
            elif comp_type == ComponentType.CONTROL_SURFACE:
                comp = ControlSurfaceBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {}),
                    surface_type=comp_data["surface_type"],
                    area=comp_data["area"],
                    chord=comp_data["chord"],
                    deflection_range=tuple(comp_data.get("deflection_range", (-30, 30))),
                    effectiveness=comp_data.get("effectiveness", 1.0),
                    hinge_moment=comp_data.get("hinge_moment", 0.0)
                )
            elif comp_type == ComponentType.BATTERY:
                comp = BatteryBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {}),
                    capacity=comp_data["capacity"],
                    voltage=comp_data["voltage"],
                    discharge_rate=comp_data.get("discharge_rate", 1.0),
                    chemistry=comp_data.get("chemistry", "LiPo"),
                    cells=comp_data.get("cells", 0)
                )
            elif comp_type == ComponentType.PROPELLER:
                comp = PropellerBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {}),
                    diameter=comp_data["diameter"],
                    pitch=comp_data["pitch"],
                    blade_count=comp_data.get("blade_count", 2),
                    efficiency=comp_data.get("efficiency", 0.8)
                )
            elif comp_type == ComponentType.LANDING_GEAR:
                comp = LandingGearBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {}),
                    type=comp_data.get("type", "tricycle"),
                    weight_on_nose=comp_data.get("weight_on_nose", 0.1),
                    tire_diameter=comp_data.get("tire_diameter", 0.3),
                    shock_stroke=comp_data.get("shock_stroke", 0.1)
                )
            elif comp_type == ComponentType.AVIONICS:
                comp = AvionicsBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {}),
                    gps=comp_data.get("gps", True),
                    imu=comp_data.get("imu", True),
                    magnetometer=comp_data.get("magnetometer", True),
                    airspeed_sensor=comp_data.get("airspeed_sensor", True),
                    autopilot=comp_data.get("autopilot", False),
                    communication_range=comp_data.get("communication_range", 10.0)
                )
            elif comp_type == ComponentType.FUEL_TANK:
                comp = FuelTankBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {}),
                    capacity=comp_data["capacity"],
                    fuel_type=comp_data.get("fuel_type", "avgas")
                )
            else:
                comp = ComponentBlueprint(
                    id=comp_id,
                    name=comp_data["name"],
                    component_type=comp_type,
                    mass=comp_data["mass"],
                    material=comp_data.get("material", "aluminum"),
                    cost=comp_data.get("cost", 0.0),
                    metadata=comp_data.get("metadata", {})
                )
            blueprint.components[comp_id] = comp
        return blueprint

    def save_json(self, filepath: str):
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load_json(cls, filepath: str) -> 'VehicleBlueprint':
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_dict(data)

# ============================================================================
#  PART 3: BLUEPRINT REGISTRY
# ============================================================================
class BlueprintRegistry:
    def __init__(self):
        self._blueprints: Dict[str, VehicleBlueprint] = {}
        self._categories: Dict[str, List[str]] = {}

    def register(self, blueprint: VehicleBlueprint, category: str = "general"):
        self._blueprints[blueprint.id] = blueprint
        if category not in self._categories:
            self._categories[category] = []
        if blueprint.id not in self._categories[category]:
            self._categories[category].append(blueprint.id)

    def unregister(self, blueprint_id: str):
        if blueprint_id in self._blueprints:
            del self._blueprints[blueprint_id]
            for category, ids in self._categories.items():
                if blueprint_id in ids:
                    ids.remove(blueprint_id)

    def get(self, blueprint_id: str) -> Optional[VehicleBlueprint]:
        return self._blueprints.get(blueprint_id)

    def list_all(self) -> List[VehicleBlueprint]:
        return list(self._blueprints.values())

    def list_by_category(self, category: str) -> List[VehicleBlueprint]:
        if category not in self._categories:
            return []
        return [self._blueprints[id] for id in self._categories[category] if id in self._blueprints]

    def search(self, query: str) -> List[VehicleBlueprint]:
        results = []
        query_lower = query.lower()
        for bp in self._blueprints.values():
            if query_lower in bp.name.lower() or query_lower in bp.description.lower():
                results.append(bp)
            elif any(query_lower in tag.lower() for tag in bp.tags):
                results.append(bp)
        return results

    def import_from_directory(self, directory: str, pattern: str = "*.json"):
        for filepath in glob.glob(os.path.join(directory, pattern)):
            try:
                bp = VehicleBlueprint.load_json(filepath)
                self.register(bp, "imported")
            except Exception as e:
                print(f"Failed to load {filepath}: {e}")

# ============================================================================
#  PART 4: BLUEPRINT FACTORY
# ============================================================================
class BlueprintFactory:
    @staticmethod
    def create_from_blueprint(blueprint: VehicleBlueprint, env: Optional[Environment] = None) -> FlyingObject:
        if env is None:
            env = Environment()
        wing = blueprint.get_wing()
        motor = blueprint.get_motor()
        if wing is None:
            raise ValueError("Blueprint must contain a wing component")
        params = AirplaneParams(
            mass=blueprint.mass_empty,
            wing_area=wing.wing_area,
            span=wing.span,
            chord=wing.mean_aerodynamic_chord,
            CL0=0.2,
            CL_alpha=blueprint.cl_alpha,
            CD0=blueprint.cd0,
            CD_induced=0.05,
            CM0=0.0,
            CM_alpha=blueprint.cm_alpha,
            CM_q=blueprint.cm_q,
            thrust_max=motor.thrust_max if motor else 100.0,
            inertia=np.array(blueprint.inertia) if blueprint.inertia else np.diag([1,5,5])
        )
        aircraft = Airplane(params, env)
        aircraft.blueprint_id = blueprint.id
        aircraft.blueprint = blueprint
        return aircraft

    @staticmethod
    def create_initial_state(blueprint: VehicleBlueprint, altitude: float = 100.0,
                             speed: float = 30.0, heading: float = 0.0) -> np.ndarray:
        state = np.zeros(12)
        state[2] = -altitude
        state[3] = speed * np.cos(heading)
        state[4] = speed * np.sin(heading)
        state[5] = 0.0
        state[6] = 0.0
        state[7] = 0.0
        state[8] = heading
        return state

# ============================================================================
#  PART 5: BLUEPRINT LIBRARY (built-in designs)
# ============================================================================
class BlueprintLibrary:
    @staticmethod
    def create_trainer_airplane() -> VehicleBlueprint:
        bp = VehicleBlueprint(
            id="trainer_v1",
            name="Basic Trainer",
            version="1.0",
            description="A simple high-wing trainer aircraft",
            author="Design Library",
            max_payload_mass=5.0,
            fuel_capacity=10.0,
            cd0=0.035,
            cl_max=1.6,
            cl_alpha=5.5,
            tags=["trainer", "fixed-wing", "beginner"]
        )
        wing = WingBlueprint(
            id="wing_main",
            name="Main Wing",
            mass=3.0,
            span=4.0,
            root_chord=0.6,
            tip_chord=0.4,
            sweep_angle=2.0,
            dihedral=3.0,
            airfoil=AirfoilType.CAMBERED,
            material="aluminum"
        )
        bp.add_component(wing)
        motor = MotorBlueprint(
            id="motor_main",
            name="Main Motor",
            mass=1.5,
            thrust_max=50.0,
            power_max=1000.0,
            efficiency=0.85,
            motor_type="electric"
        )
        bp.add_component(motor)
        battery = BatteryBlueprint(
            id="battery_main",
            name="Main Battery",
            mass=1.2,
            capacity=5.0,
            voltage=22.2,
            discharge_rate=25.0,
            chemistry="LiPo"
        )
        bp.add_component(battery)
        elevator = ControlSurfaceBlueprint(
            id="elevator_main",
            name="Elevator",
            mass=0.3,
            surface_type="elevator",
            area=0.12,
            chord=0.15,
            deflection_range=(-25, 25)
        )
        bp.add_component(elevator)
        aileron = ControlSurfaceBlueprint(
            id="aileron_main",
            name="Ailerons",
            mass=0.25,
            surface_type="aileron",
            area=0.08,
            chord=0.12,
            deflection_range=(-20, 20)
        )
        bp.add_component(aileron)
        bp.validate()
        return bp

    @staticmethod
    def create_racing_drone() -> VehicleBlueprint:
        bp = VehicleBlueprint(
            id="drone_racer_v1",
            name="Racing Drone",
            version="1.0",
            description="High-performance FPV racing drone",
            author="Design Library",
            max_payload_mass=0.5,
            cd0=0.05,
            cl_max=2.0,
            max_speed=40.0,
            tags=["drone", "racing", "multirotor"]
        )
        for i in range(4):
            motor = MotorBlueprint(
                id=f"motor_{i+1}",
                name=f"Motor {i+1}",
                mass=0.035,
                thrust_max=8.0,
                power_max=150.0,
                efficiency=0.9,
                motor_type="brushless",
                kv_rating=2300.0,
                rpm_max=50000.0
            )
            bp.add_component(motor)
        battery = BatteryBlueprint(
            id="battery_racer",
            name="Racing Battery",
            mass=0.1,
            capacity=1.3,
            voltage=14.8,
            discharge_rate=100.0,
            chemistry="LiPo"
        )
        bp.add_component(battery)
        bp.inertia = [[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.02]]
        bp.mass_empty = 0.35
        bp.validate()
        return bp

    @staticmethod
    def create_jumbo_jet() -> VehicleBlueprint:
        bp = VehicleBlueprint(
            id="jumbo_jet_v1",
            name="Jumbo Jet",
            version="1.0",
            description="Large commercial passenger aircraft",
            author="Design Library",
            max_payload_mass=40000.0,
            fuel_capacity=150000.0,
            cd0=0.02,
            cl_max=1.8,
            cl_alpha=4.8,
            max_speed=250.0,
            max_altitude=13000.0,
            max_g_load=2.5,
            tags=["commercial", "passenger", "large"]
        )
        wing = WingBlueprint(
            id="wing_main",
            name="Main Wing",
            mass=15000.0,
            span=60.0,
            root_chord=8.0,
            tip_chord=2.5,
            sweep_angle=30.0,
            dihedral=5.0,
            airfoil=AirfoilType.SUPERCRITICAL,
            material="aluminum"
        )
        bp.add_component(wing)
        for i in range(4):
            motor = MotorBlueprint(
                id=f"engine_{i+1}",
                name=f"Turbofan Engine {i+1}",
                mass=5000.0,
                thrust_max=300000.0,
                power_max=10000000.0,
                efficiency=0.7,
                motor_type="turbofan",
                rpm_max=8000.0
            )
            bp.add_component(motor)
        bp.inertia = [[1000000, 0, 0], [0, 10000000, 0], [0, 0, 10000000]]
        bp.mass_empty = 150000.0
        bp.validate()
        return bp

# ============================================================================
#  PART 6: ORIGINAL BLUEPRINT MANAGER
# ============================================================================
class BlueprintManager:
    def __init__(self):
        self.registry = BlueprintRegistry()
        self.factory = BlueprintFactory()
        self._load_builtins()

    def _load_builtins(self):
        library = BlueprintLibrary()
        self.registry.register(library.create_trainer_airplane(), "trainer")
        self.registry.register(library.create_racing_drone(), "drone")
        self.registry.register(library.create_jumbo_jet(), "commercial")

    def create_blueprint(self, name: str, **kwargs) -> VehicleBlueprint:
        bp = VehicleBlueprint(
            id=f"{name.lower().replace(' ', '_')}_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            name=name,
            **kwargs
        )
        return bp

    def save_blueprint(self, blueprint: VehicleBlueprint, filepath: str):
        blueprint.save_json(filepath)

    def load_blueprint(self, filepath: str, register: bool = True) -> VehicleBlueprint:
        bp = VehicleBlueprint.load_json(filepath)
        if register:
            self.registry.register(bp, "loaded")
        return bp

    def create_vehicle(self, blueprint_id: str, env=None) -> FlyingObject:
        blueprint = self.registry.get(blueprint_id)
        if blueprint is None:
            raise ValueError(f"Blueprint '{blueprint_id}' not found")
        return self.factory.create_from_blueprint(blueprint, env)

    def list_blueprints(self) -> Dict[str, List[VehicleBlueprint]]:
        result = {}
        for category in self.registry._categories:
            result[category] = self.registry.list_by_category(category)
        return result

    def export_catalog(self, filepath: str):
        catalog = {
            "version": "1.0",
            "generated": datetime.now().isoformat(),
            "blueprints": []
        }
        for bp in self.registry.list_all():
            catalog["blueprints"].append({
                "id": bp.id,
                "name": bp.name,
                "version": bp.version,
                "description": bp.description,
                "mass": bp.mass_empty,
                "tags": bp.tags,
                "signature": bp.signature
            })
        with open(filepath, 'w') as f:
            json.dump(catalog, f, indent=2)

# ============================================================================
#  PART 7: EXTENSIONS
# ============================================================================

# ---- 7.1 Material Library ----
@dataclass
class Material:
    name: str
    density: float
    young_modulus: float
    yield_strength: float
    ultimate_strength: float
    cost_per_kg: float
    thermal_expansion: float = 0.0

class MaterialLibrary:
    MATERIALS = {
        "aluminum_2024": Material("Aluminum 2024-T3", 2780, 73, 324, 469, 2.5),
        "aluminum_7075": Material("Aluminum 7075-T6", 2810, 71, 503, 572, 3.2),
        "steel_4130": Material("Steel 4130", 7850, 205, 435, 670, 1.8),
        "titanium_6al4v": Material("Titanium 6Al-4V", 4430, 114, 880, 950, 30.0),
        "carbon_fiber": Material("Carbon Fiber Composite", 1600, 70, 600, 700, 50.0),
        "glass_fiber": Material("Glass Fiber Composite", 1900, 30, 200, 300, 15.0),
        "wood_spruce": Material("Spruce Wood", 500, 10, 40, 50, 5.0),
        "foam_pvc": Material("PVC Foam", 80, 0.5, 2, 3, 8.0)
    }

    @classmethod
    def get(cls, name: str) -> Optional[Material]:
        return cls.MATERIALS.get(name)

    @classmethod
    def list_materials(cls) -> List[str]:
        return list(cls.MATERIALS.keys())

# ---- 7.2 Performance Estimator ----
class PerformanceEstimator:
    @staticmethod
    def stall_speed(blueprint: VehicleBlueprint) -> float:
        wing = blueprint.get_wing()
        if wing is None:
            return float('nan')
        rho = 1.225
        mass = blueprint.mass_empty + blueprint.max_payload_mass
        cl_max = blueprint.cl_max
        wing_area = wing.wing_area
        return np.sqrt(2 * mass * 9.81 / (rho * cl_max * wing_area))

    @staticmethod
    def cruise_speed(blueprint: VehicleBlueprint) -> float:
        motor = blueprint.get_motor()
        wing = blueprint.get_wing()
        if motor is None or wing is None:
            return float('nan')
        mass = blueprint.mass_empty + blueprint.max_payload_mass
        rho = 1.225
        wing_area = wing.wing_area
        cd0 = blueprint.cd0
        thrust = motor.thrust_max * 0.7
        return np.sqrt(2 * thrust / (rho * wing_area * cd0))

    @staticmethod
    def range_breguet(blueprint: VehicleBlueprint, fuel_mass: float = None) -> float:
        motor = blueprint.get_motor()
        wing = blueprint.get_wing()
        if motor is None or wing is None:
            return float('nan')
        mass_empty = blueprint.mass_empty
        mass_fuel = fuel_mass or blueprint.fuel_capacity * 0.72
        if mass_fuel <= 0:
            return 0.0
        L_over_D = 10.0
        c_sfc = 0.5  # kg/N/hour
        g = 9.81
        V = PerformanceEstimator.cruise_speed(blueprint)
        if np.isnan(V):
            V = 30.0
        c_sfc_s = c_sfc / 3600.0
        Wi = mass_empty + mass_fuel
        Wf = mass_empty
        return (V / (c_sfc_s * g)) * L_over_D * np.log(Wi / Wf)

    @staticmethod
    def climb_rate(blueprint: VehicleBlueprint) -> float:
        motor = blueprint.get_motor()
        wing = blueprint.get_wing()
        if motor is None or wing is None:
            return float('nan')
        mass = blueprint.mass_empty + blueprint.max_payload_mass
        thrust = motor.thrust_max
        V = PerformanceEstimator.cruise_speed(blueprint)
        if np.isnan(V):
            V = 30.0
        D = 0.5 * 1.225 * V**2 * wing.wing_area * blueprint.cd0
        excess_thrust = thrust - D
        if excess_thrust < 0:
            return 0.0
        return (excess_thrust * V) / (mass * 9.81)

    @staticmethod
    def estimate_all(blueprint: VehicleBlueprint) -> Dict[str, float]:
        return {
            "stall_speed_mps": PerformanceEstimator.stall_speed(blueprint),
            "cruise_speed_mps": PerformanceEstimator.cruise_speed(blueprint),
            "range_m": PerformanceEstimator.range_breguet(blueprint),
            "climb_rate_mps": PerformanceEstimator.climb_rate(blueprint),
            "wing_loading_Npm2": blueprint.mass_empty * 9.81 / blueprint.get_wing().wing_area if blueprint.get_wing() else float('nan'),
            "thrust_to_weight": blueprint.get_motor().thrust_max / (blueprint.mass_empty * 9.81) if blueprint.get_motor() else float('nan')
        }

# ---- 7.3 Design Optimizer ----
class DesignOptimizer:
    @staticmethod
    def optimize_wing_for_stall_speed(blueprint: VehicleBlueprint, target_stall_speed: float,
                                      bounds: Tuple[float, float] = (1.0, 20.0)) -> VehicleBlueprint:
        bp = copy.deepcopy(blueprint)
        wing = bp.get_wing()
        if wing is None:
            raise ValueError("Blueprint has no wing")
        mass = bp.mass_empty + bp.max_payload_mass
        rho = 1.225
        cl_max = bp.cl_max
        required_area = 2 * mass * 9.81 / (rho * cl_max * target_stall_speed**2)
        required_area = np.clip(required_area, bounds[0], bounds[1])
        AR = wing.aspect_ratio
        new_span = np.sqrt(required_area * AR)
        new_chord = required_area / new_span
        wing.span = new_span
        wing.root_chord = new_chord
        wing.tip_chord = new_chord
        wing.aspect_ratio = AR
        bp.update_mass()
        bp.generate_signature()
        return bp

    @staticmethod
    def optimize_aspect_ratio(blueprint: VehicleBlueprint, target_range: float,
                              max_span: float = 20.0) -> VehicleBlueprint:
        bp = copy.deepcopy(blueprint)
        wing = bp.get_wing()
        if wing is None:
            raise ValueError("Blueprint has no wing")
        current_range = PerformanceEstimator.range_breguet(bp)
        if current_range <= 0:
            current_range = 1.0
        target_AR = wing.aspect_ratio * (target_range / current_range)**2
        target_AR = np.clip(target_AR, 3.0, 30.0)
        new_span = np.sqrt(target_AR * wing.wing_area)
        if new_span > max_span:
            new_span = max_span
        wing.span = new_span
        wing.aspect_ratio = target_AR
        wing.root_chord = wing.wing_area / new_span
        wing.tip_chord = wing.root_chord
        bp.update_mass()
        bp.generate_signature()
        return bp

# ---- 7.4 Version Manager ----
class BlueprintVersionManager:
    def __init__(self, blueprint: VehicleBlueprint):
        self.blueprint = blueprint
        self.history: List[Tuple[str, VehicleBlueprint]] = []
        self._save_version()

    def _save_version(self):
        timestamp = datetime.now().isoformat()
        self.history.append((timestamp, copy.deepcopy(self.blueprint)))

    def update(self, new_blueprint: VehicleBlueprint):
        self.blueprint = new_blueprint
        self._save_version()

    def rollback(self, index: int = -1) -> VehicleBlueprint:
        if abs(index) > len(self.history):
            raise ValueError("Index out of range")
        self.blueprint = self.history[index][1]
        return self.blueprint

    def diff(self, version1: int, version2: int) -> Dict[str, Any]:
        if version1 >= len(self.history) or version2 >= len(self.history):
            raise ValueError("Version index out of range")
        bp1 = self.history[version1][1]
        bp2 = self.history[version2][1]
        diff = {}
        comps1 = {c.id: c.mass for c in bp1.components.values()}
        comps2 = {c.id: c.mass for c in bp2.components.values()}
        all_ids = set(comps1.keys()) | set(comps2.keys())
        for cid in all_ids:
            m1 = comps1.get(cid, 0)
            m2 = comps2.get(cid, 0)
            if m1 != m2:
                diff[f"component_{cid}_mass"] = (m1, m2)
        if bp1.mass_empty != bp2.mass_empty:
            diff["mass_empty"] = (bp1.mass_empty, bp2.mass_empty)
        return diff

    def get_history_summary(self) -> List[Dict[str, str]]:
        return [{"version": i, "timestamp": t} for i, (t, _) in enumerate(self.history)]

# ---- 7.5 Constraint Checker ----
class ConstraintChecker:
    @staticmethod
    def check_all(blueprint: VehicleBlueprint, constraints: Dict[str, float]) -> Dict[str, Tuple[bool, float, float]]:
        results = {}
        perf = PerformanceEstimator.estimate_all(blueprint)
        for key, required in constraints.items():
            if key in perf:
                actual = perf[key]
                if 'max' in key:
                    passed = actual <= required
                elif 'min' in key:
                    passed = actual >= required
                else:
                    passed = True
                results[key] = (passed, actual, required)
            else:
                results[key] = (False, None, required)
        return results

# ---- 7.6 Simulation Adapter ----
class SimulationAdapter:
    @staticmethod
    def create_airplane_params(blueprint: VehicleBlueprint) -> AirplaneParams:
        wing = blueprint.get_wing()
        motor = blueprint.get_motor()
        if wing is None:
            raise ValueError("No wing component")
        return AirplaneParams(
            mass=blueprint.mass_empty,
            wing_area=wing.wing_area,
            span=wing.span,
            chord=wing.mean_aerodynamic_chord,
            CL0=0.2,
            CL_alpha=blueprint.cl_alpha,
            CD0=blueprint.cd0,
            CD_induced=0.05,
            CM0=0.0,
            CM_alpha=blueprint.cm_alpha,
            CM_q=blueprint.cm_q,
            thrust_max=motor.thrust_max if motor else 100.0,
            inertia=np.array(blueprint.inertia) if blueprint.inertia else np.diag([1,5,5])
        )

    @staticmethod
    def create_environment(blueprint: VehicleBlueprint) -> Environment:
        return Environment(g=np.array([0, 0, -9.81]))

    @staticmethod
    def create_initial_state(blueprint: VehicleBlueprint, altitude: float = 100.0,
                             speed: Optional[float] = None) -> np.ndarray:
        if speed is None:
            speed = PerformanceEstimator.cruise_speed(blueprint)
            if np.isnan(speed):
                speed = 30.0
        state = np.zeros(12)
        state[2] = -altitude
        state[3] = speed
        state[8] = 0.0
        return state

# ---- 7.7 Exporters ----
class BlueprintExporter:
    @staticmethod
    def to_csv(blueprint: VehicleBlueprint, filepath: str):
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Component ID', 'Name', 'Type', 'Mass (kg)', 'Material', 'Cost (USD)'])
            for comp in blueprint.components.values():
                writer.writerow([comp.id, comp.name, comp.component_type.value, comp.mass, comp.material, comp.cost])

    @staticmethod
    def performance_to_csv(blueprint: VehicleBlueprint, filepath: str):
        perf = PerformanceEstimator.estimate_all(blueprint)
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Metric', 'Value'])
            for key, val in perf.items():
                writer.writerow([key, val])

    @staticmethod
    def to_xml(blueprint: VehicleBlueprint, filepath: str):
        import xml.etree.ElementTree as ET
        root = ET.Element("VehicleBlueprint", id=blueprint.id, version=blueprint.version)
        ET.SubElement(root, "name").text = blueprint.name
        ET.SubElement(root, "description").text = blueprint.description
        ET.SubElement(root, "mass_empty").text = str(blueprint.mass_empty)
        comps_elem = ET.SubElement(root, "components")
        for comp in blueprint.components.values():
            comp_elem = ET.SubElement(comps_elem, "component", id=comp.id, type=comp.component_type.value)
            ET.SubElement(comp_elem, "name").text = comp.name
            ET.SubElement(comp_elem, "mass").text = str(comp.mass)
            ET.SubElement(comp_elem, "material").text = comp.material
            ET.SubElement(comp_elem, "cost").text = str(comp.cost)
        tree = ET.ElementTree(root)
        tree.write(filepath, encoding='utf-8', xml_declaration=True)

# ---- 7.8 Extended Blueprint Manager ----
class BlueprintManagerExtended(BlueprintManager):
    def __init__(self):
        super().__init__()
        self.version_managers: Dict[str, BlueprintVersionManager] = {}

    def register(self, blueprint: VehicleBlueprint, category: str = "general"):
        super().register(blueprint, category)
        self.version_managers[blueprint.id] = BlueprintVersionManager(blueprint)

    def update_blueprint(self, blueprint_id: str, new_blueprint: VehicleBlueprint):
        if blueprint_id not in self.version_managers:
            raise ValueError(f"Blueprint {blueprint_id} not found")
        self.registry._blueprints[blueprint_id] = new_blueprint
        self.version_managers[blueprint_id].update(new_blueprint)

    def rollback_blueprint(self, blueprint_id: str, version_index: int = -1) -> VehicleBlueprint:
        if blueprint_id not in self.version_managers:
            raise ValueError(f"Blueprint {blueprint_id} not found")
        bp = self.version_managers[blueprint_id].rollback(version_index)
        self.registry._blueprints[blueprint_id] = bp
        return bp

    def analyze_performance(self, blueprint_id: str) -> Dict[str, float]:
        bp = self.registry.get(blueprint_id)
        if bp is None:
            raise ValueError(f"Blueprint {blueprint_id} not found")
        return PerformanceEstimator.estimate_all(bp)

    def optimize_stall_speed(self, blueprint_id: str, target_stall_speed: float) -> VehicleBlueprint:
        bp = self.registry.get(blueprint_id)
        if bp is None:
            raise ValueError(f"Blueprint {blueprint_id} not found")
        optimized = DesignOptimizer.optimize_wing_for_stall_speed(bp, target_stall_speed)
        self.update_blueprint(blueprint_id, optimized)
        return optimized

    def simulate_blueprint(self, blueprint_id: str, duration: float = 30.0,
                           controller: Optional[Callable] = None) -> np.ndarray:
        blueprint = self.registry.get(blueprint_id)
        if blueprint is None:
            raise ValueError(f"Blueprint {blueprint_id} not found")
        env = SimulationAdapter.create_environment(blueprint)
        vehicle = self.create_vehicle(blueprint_id, env)
        initial_state = SimulationAdapter.create_initial_state(blueprint)
        vehicle.state = initial_state
        sim = Simulator(vehicle, dt=0.02)
        history = sim.run(duration, controller)
        return history

# ---- 7.9 CLI Interface ----
def blueprint_cli():
    import argparse
    parser = argparse.ArgumentParser(description="Flying Object Blueprint System")
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="List all blueprints")
    create_parser = subparsers.add_parser("create", help="Create new blueprint")
    create_parser.add_argument("name", help="Blueprint name")
    save_parser = subparsers.add_parser("save", help="Save blueprint to file")
    save_parser.add_argument("id", help="Blueprint ID")
    save_parser.add_argument("file", help="Output JSON file")
    load_parser = subparsers.add_parser("load", help="Load blueprint from file")
    load_parser.add_argument("file", help="JSON file")
    analyze_parser = subparsers.add_parser("analyze", help="Analyze performance")
    analyze_parser.add_argument("id", help="Blueprint ID")
    opt_parser = subparsers.add_parser("optimize", help="Optimize wing for stall speed")
    opt_parser.add_argument("id", help="Blueprint ID")
    opt_parser.add_argument("target_stall", type=float, help="Target stall speed (m/s)")

    args = parser.parse_args()
    manager = BlueprintManagerExtended()

    if args.command == "list":
        for category, bps in manager.list_blueprints().items():
            print(f"\nCategory: {category}")
            for bp in bps:
                print(f"  {bp.id}: {bp.name} (v{bp.version}) - {bp.mass_empty:.1f} kg")
    elif args.command == "create":
        bp = manager.create_blueprint(args.name)
        manager.register(bp, "custom")
        print(f"Created blueprint: {bp.id}")
    elif args.command == "save":
        bp = manager.registry.get(args.id)
        if bp:
            bp.save_json(args.file)
            print(f"Saved to {args.file}")
        else:
            print(f"Blueprint {args.id} not found")
    elif args.command == "load":
        bp = manager.load_blueprint(args.file)
        print(f"Loaded blueprint: {bp.id}")
    elif args.command == "analyze":
        perf = manager.analyze_performance(args.id)
        print(f"Performance for {args.id}:")
        for key, val in perf.items():
            print(f"  {key}: {val:.2f}")
    elif args.command == "optimize":
        bp = manager.optimize_stall_speed(args.id, args.target_stall)
        print(f"Optimized blueprint: {bp.id}, new stall speed ~ {PerformanceEstimator.stall_speed(bp):.2f} m/s")

# ============================================================================
#  PART 8: EXAMPLE USAGE
# ============================================================================
def example_usage():
    print("=" * 60)
    print("FLYING OBJECT BLUEPRINT SYSTEM (with extensions)")
    print("=" * 60)

    manager = BlueprintManagerExtended()

    # List blueprints
    print("\n📋 Available Blueprints:")
    for category, bps in manager.list_blueprints().items():
        print(f"\n  Category: {category.upper()}")
        for bp in bps:
            print(f"    - {bp.name} (v{bp.version}) [{bp.id}]")
            print(f"      Mass: {bp.mass_empty:.1f} kg, Components: {len(bp.components)}")

    # Analyze trainer
    trainer = manager.registry.get("trainer_v1")
    if trainer:
        print("\n🔍 Trainer Performance:")
        perf = PerformanceEstimator.estimate_all(trainer)
        for k, v in perf.items():
            print(f"  {k}: {v:.2f}")

        # Optimize
        print("\n⚙️ Optimizing for stall speed 15 m/s...")
        opt_trainer = DesignOptimizer.optimize_wing_for_stall_speed(trainer, 15.0)
        manager.update_blueprint("trainer_v1", opt_trainer)
        perf2 = PerformanceEstimator.estimate_all(opt_trainer)
        print("New performance:")
        for k, v in perf2.items():
            print(f"  {k}: {v:.2f}")

        # Export
        BlueprintExporter.to_csv(trainer, "trainer_components.csv")
        BlueprintExporter.performance_to_csv(trainer, "trainer_performance.csv")
        print("\n📁 Exported to CSV files.")

    # Version history
    vm = manager.version_managers.get("trainer_v1")
    if vm:
        print("\n📜 Version history:")
        for entry in vm.get_history_summary():
            print(f"  Version {entry['version']}: {entry['timestamp']}")

    # Simulate (if controller is provided, here we simulate with simple level flight)
    print("\n✈️ Simulating trainer for 5 seconds...")
    def simple_controller(vehicle, t):
        # Keep altitude and speed
        alt_error = 100.0 - (-vehicle.state[2])
        pitch_cmd = np.clip(0.01 * alt_error, -0.2, 0.2)
        elevator = np.clip(0.5 * (pitch_cmd - vehicle.state[7]), -1, 1)
        V = np.linalg.norm(vehicle.state[3:6])
        throttle = np.clip(0.5 + 0.01 * (30 - V), 0, 1)
        return np.array([throttle, elevator, 0.0])

    try:
        history = manager.simulate_blueprint("trainer_v1", duration=5.0, controller=simple_controller)
        print(f"  Final position: {history[-1, 1:4]}")
    except Exception as e:
        print(f"  Simulation error: {e}")

    print("\n" + "=" * 60)
    print("✅ Blueprint system demonstration complete!")

if __name__ == "__main__":
    # Uncomment to run the CLI:
    # blueprint_cli()
    example_usage()