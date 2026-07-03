import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Union
from abc import ABC, abstractmethod
import json
from enum import Enum
import hashlib
from datetime import datetime
import copy

# ============================================================================
#  Core Component Types
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

# ============================================================================
#  Component Blueprints
# ============================================================================

@dataclass
class ComponentBlueprint:
    """Base class for all component blueprints"""
    id: str
    name: str
    component_type: ComponentType
    mass: float  # kg
    material: str = "aluminum"
    cost: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def get_id(self) -> str:
        return self.id

@dataclass
class WingBlueprint(ComponentBlueprint):
    """Wing design specifications"""
    span: float  # meters
    root_chord: float  # meters
    tip_chord: float  # meters
    sweep_angle: float = 0.0  # degrees
    dihedral: float = 0.0  # degrees
    airfoil: AirfoilType = AirfoilType.SYMMETRIC
    aspect_ratio: float = 0.0  # computed if zero
    taper_ratio: float = 1.0  # tip/root chord ratio
    
    def __post_init__(self):
        self.component_type = ComponentType.WING
        if self.aspect_ratio == 0 and self.span > 0:
            avg_chord = (self.root_chord + self.tip_chord) / 2
            self.aspect_ratio = self.span / avg_chord
        if self.taper_ratio == 1.0 and self.tip_chord != self.root_chord:
            self.taper_ratio = self.tip_chord / self.root_chord
    
    @property
    def wing_area(self) -> float:
        """Planform area in m^2"""
        return (self.root_chord + self.tip_chord) / 2 * self.span
    
    @property
    def mean_aerodynamic_chord(self) -> float:
        """Mean aerodynamic chord in meters"""
        return (2/3) * self.root_chord * (1 + self.taper_ratio + self.taper_ratio**2) / (1 + self.taper_ratio)

@dataclass
class MotorBlueprint(ComponentBlueprint):
    """Motor/engine specifications"""
    thrust_max: float  # Newtons
    power_max: float  # Watts
    efficiency: float = 0.85  # 0-1
    motor_type: str = "electric"
    kv_rating: float = 0.0  # RPM/V for electric motors
    rpm_max: float = 0.0  # Maximum RPM
    
    def __post_init__(self):
        self.component_type = ComponentType.MOTOR

@dataclass
class ControlSurfaceBlueprint(ComponentBlueprint):
    """Control surface specifications"""
    surface_type: str  # elevator, aileron, rudder, flaperon
    area: float  # m^2
    chord: float  # meters
    deflection_range: Tuple[float, float] = (-30.0, 30.0)  # degrees
    effectiveness: float = 1.0  # scaling factor
    hinge_moment: float = 0.0  # N*m
    
    def __post_init__(self):
        self.component_type = ComponentType.CONTROL_SURFACE

@dataclass
class BatteryBlueprint(ComponentBlueprint):
    """Battery specifications"""
    capacity: float  # Ah
    voltage: float  # Volts
    discharge_rate: float = 1.0  # C-rating
    chemistry: str = "LiPo"
    cells: int = 0
    
    def __post_init__(self):
        self.component_type = ComponentType.BATTERY
        self.cells = int(self.voltage / 3.7) if self.cells == 0 else self.cells
    
    @property
    def energy(self) -> float:
        """Total energy in Watt-hours"""
        return self.capacity * self.voltage

# ============================================================================
#  Vehicle Blueprint
# ============================================================================

@dataclass
class VehicleBlueprint:
    """Complete vehicle design blueprint"""
    id: str
    name: str
    version: str = "1.0"
    description: str = ""
    author: str = ""
    created: str = field(default_factory=lambda: datetime.now().isoformat())
    modified: str = field(default_factory=lambda: datetime.now().isoformat())
    
    # Components
    components: Dict[str, ComponentBlueprint] = field(default_factory=dict)
    
    # Configuration
    mass_empty: float = 0.0  # Computed from components if zero
    max_payload_mass: float = 0.0
    fuel_capacity: float = 0.0  # liters
    wing_config: Optional[Dict] = None
    propulsion_config: Optional[Dict] = None
    control_config: Optional[Dict] = None
    
    # Aerodynamic parameters
    cd0: float = 0.03  # Zero-lift drag coefficient
    cl_max: float = 1.5
    cl_alpha: float = 5.0  # per radian
    cm_alpha: float = -0.5
    cm_q: float = -5.0
    
    # Inertia (body frame)
    inertia: Optional[List[List[float]]] = None
    
    # Performance limits
    max_speed: float = 100.0  # m/s
    max_altitude: float = 5000.0  # meters
    max_g_load: float = 4.0  # G's
    
    # Metadata
    tags: List[str] = field(default_factory=list)
    notes: str = ""
    signature: str = ""
    
    def __post_init__(self):
        if self.inertia is None:
            self.inertia = [[10.0, 0, 0], [0, 50.0, 0], [0, 0, 50.0]]
        self.update_mass()
        self.generate_signature()
    
    def update_mass(self):
        """Compute mass from components and update empty mass"""
        total_mass = 0.0
        for comp in self.components.values():
            total_mass += comp.mass
        self.mass_empty = total_mass
    
    def add_component(self, component: ComponentBlueprint):
        """Add a component to the blueprint"""
        self.components[component.id] = component
        self.update_mass()
        self.modified = datetime.now().isoformat()
    
    def remove_component(self, component_id: str):
        """Remove a component by ID"""
        if component_id in self.components:
            del self.components[component_id]
            self.update_mass()
            self.modified = datetime.now().isoformat()
    
    def get_component_by_type(self, comp_type: ComponentType) -> List[ComponentBlueprint]:
        """Get all components of a specific type"""
        return [c for c in self.components.values() if c.component_type == comp_type]
    
    def get_wing(self) -> Optional[WingBlueprint]:
        """Get the first wing component"""
        wings = self.get_component_by_type(ComponentType.WING)
        return wings[0] if wings else None
    
    def get_motor(self) -> Optional[MotorBlueprint]:
        """Get the first motor component"""
        motors = self.get_component_by_type(ComponentType.MOTOR)
        return motors[0] if motors else None
    
    def generate_signature(self):
        """Generate a unique signature for this blueprint version"""
        data = f"{self.id}:{self.name}:{self.version}:{self.mass_empty}"
        for comp_id, comp in sorted(self.components.items()):
            data += f":{comp_id}:{comp.mass}"
        self.signature = hashlib.md5(data.encode()).hexdigest()[:8]
    
    def validate(self) -> Tuple[bool, List[str]]:
        """Validate the blueprint for physical consistency"""
        errors = []
        
        # Check for wings
        wings = self.get_component_by_type(ComponentType.WING)
        if len(wings) == 0:
            errors.append("No wing component found")
        
        # Check for motors
        motors = self.get_component_by_type(ComponentType.MOTOR)
        if len(motors) == 0:
            errors.append("No motor component found")
        
        # Check mass consistency
        computed_mass = sum(c.mass for c in self.components.values())
        if abs(computed_mass - self.mass_empty) > 1e-6:
            errors.append(f"Mass mismatch: components sum = {computed_mass}, empty mass = {self.mass_empty}")
        
        # Check wing loading
        wing = self.get_wing()
        if wing:
            wing_loading = self.mass_empty / wing.wing_area * 9.81  # N/m^2
            if wing_loading > 1000:  # Very high
                errors.append(f"High wing loading: {wing_loading:.1f} N/m²")
        
        # Check thrust-to-weight ratio
        motor = self.get_motor()
        if motor:
            tw_ratio = motor.thrust_max / (self.mass_empty * 9.81)
            if tw_ratio < 0.5:
                errors.append(f"Low thrust-to-weight ratio: {tw_ratio:.2f}")
            if tw_ratio > 3.0:
                errors.append(f"Very high thrust-to-weight ratio: {tw_ratio:.2f}")
        
        return len(errors) == 0, errors
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert blueprint to dictionary for serialization"""
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
            
            # Add type-specific fields
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
            
            data["components"][comp_id] = comp_data
        
        return data
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'VehicleBlueprint':
        """Create blueprint from dictionary"""
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
        
        # Reconstruct components
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
            else:
                # Generic component
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
        """Save blueprint to JSON file"""
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
    
    @classmethod
    def load_json(cls, filepath: str) -> 'VehicleBlueprint':
        """Load blueprint from JSON file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_dict(data)

# ============================================================================
#  Blueprint Registry
# ============================================================================

class BlueprintRegistry:
    """Central registry for managing vehicle blueprints"""
    
    def __init__(self):
        self._blueprints: Dict[str, VehicleBlueprint] = {}
        self._categories: Dict[str, List[str]] = {}
        
    def register(self, blueprint: VehicleBlueprint, category: str = "general"):
        """Register a blueprint in the registry"""
        self._blueprints[blueprint.id] = blueprint
        if category not in self._categories:
            self._categories[category] = []
        if blueprint.id not in self._categories[category]:
            self._categories[category].append(blueprint.id)
    
    def unregister(self, blueprint_id: str):
        """Remove a blueprint from the registry"""
        if blueprint_id in self._blueprints:
            del self._blueprints[blueprint_id]
            for category, ids in self._categories.items():
                if blueprint_id in ids:
                    ids.remove(blueprint_id)
    
    def get(self, blueprint_id: str) -> Optional[VehicleBlueprint]:
        """Get a blueprint by ID"""
        return self._blueprints.get(blueprint_id)
    
    def list_all(self) -> List[VehicleBlueprint]:
        """List all registered blueprints"""
        return list(self._blueprints.values())
    
    def list_by_category(self, category: str) -> List[VehicleBlueprint]:
        """List blueprints in a category"""
        if category not in self._categories:
            return []
        return [self._blueprints[id] for id in self._categories[category] if id in self._blueprints]
    
    def search(self, query: str) -> List[VehicleBlueprint]:
        """Search blueprints by name or tags"""
        results = []
        query_lower = query.lower()
        for bp in self._blueprints.values():
            if query_lower in bp.name.lower():
                results.append(bp)
            elif any(query_lower in tag.lower() for tag in bp.tags):
                results.append(bp)
            elif query_lower in bp.description.lower():
                results.append(bp)
        return results
    
    def import_from_directory(self, directory: str, pattern: str = "*.json"):
        """Import all JSON blueprints from a directory"""
        import glob
        import os
        
        for filepath in glob.glob(os.path.join(directory, pattern)):
            try:
                bp = VehicleBlueprint.load_json(filepath)
                self.register(bp, "imported")
            except Exception as e:
                print(f"Failed to load {filepath}: {e}")

# ============================================================================
#  Blueprint Factory - Creates flying objects from blueprints
# ============================================================================

class BlueprintFactory:
    """Creates flying object instances from blueprints"""
    
    @staticmethod
    def create_from_blueprint(blueprint: VehicleBlueprint, 
                             env: 'Environment' = None) -> 'FlyingObject':
        """Create a flying object from a blueprint"""
        from flying_object import FlyingObject, Environment, Airplane, AirplaneParams
        
        if env is None:
            env = Environment()
        
        # Extract wing parameters
        wing = blueprint.get_wing()
        motor = blueprint.get_motor()
        
        if wing is None:
            raise ValueError("Blueprint must contain a wing component")
        
        # Create parameters
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
        
        # Create the aircraft
        aircraft = Airplane(params, env)
        
        # Add blueprint reference for later use
        aircraft.blueprint_id = blueprint.id
        aircraft.blueprint = blueprint
        
        return aircraft
    
    @staticmethod
    def create_initial_state(blueprint: VehicleBlueprint, 
                            altitude: float = 100.0, 
                            speed: float = 30.0,
                            heading: float = 0.0) -> np.ndarray:
        """Create initial state for a blueprint"""
        state = np.zeros(12)
        state[2] = -altitude  # z negative = up
        state[3] = speed * np.cos(heading)
        state[4] = speed * np.sin(heading)
        state[5] = 0.0
        # Euler angles (phi, theta, psi)
        state[6] = 0.0
        state[7] = 0.0
        state[8] = heading
        return state

# ============================================================================
#  Blueprint Library - Pre-defined vehicle designs
# ============================================================================

class BlueprintLibrary:
    """Library of pre-defined vehicle blueprints"""
    
    @staticmethod
    def create_trainer_airplane() -> VehicleBlueprint:
        """Create a basic trainer airplane blueprint"""
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
        
        # Add wing
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
        
        # Add motor
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
        
        # Add battery
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
        
        # Add control surfaces
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
        """Create a racing drone blueprint"""
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
        
        # Add 4 motors
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
        
        # Add battery
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
        """Create a large passenger jet blueprint"""
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
        
        # Add multiple engines
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
#  Blueprint Manager - High-level interface
# ============================================================================

class BlueprintManager:
    """High-level manager for the blueprint system"""
    
    def __init__(self):
        self.registry = BlueprintRegistry()
        self.factory = BlueprintFactory()
        
        # Load built-in designs
        self._load_builtins()
    
    def _load_builtins(self):
        """Load built-in blueprints into the registry"""
        library = BlueprintLibrary()
        
        self.registry.register(library.create_trainer_airplane(), "trainer")
        self.registry.register(library.create_racing_drone(), "drone")
        self.registry.register(library.create_jumbo_jet(), "commercial")
    
    def create_blueprint(self, name: str, **kwargs) -> VehicleBlueprint:
        """Create a new blueprint programmatically"""
        bp = VehicleBlueprint(
            id=f"{name.lower().replace(' ', '_')}_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            name=name,
            **kwargs
        )
        return bp
    
    def save_blueprint(self, blueprint: VehicleBlueprint, filepath: str):
        """Save blueprint to file"""
        blueprint.save_json(filepath)
    
    def load_blueprint(self, filepath: str, register: bool = True) -> VehicleBlueprint:
        """Load blueprint from file"""
        bp = VehicleBlueprint.load_json(filepath)
        if register:
            self.registry.register(bp, "loaded")
        return bp
    
    def create_vehicle(self, blueprint_id: str, env=None) -> 'FlyingObject':
        """Create a flying object from a registered blueprint"""
        blueprint = self.registry.get(blueprint_id)
        if blueprint is None:
            raise ValueError(f"Blueprint '{blueprint_id}' not found")
        return self.factory.create_from_blueprint(blueprint, env)
    
    def list_blueprints(self) -> Dict[str, List[VehicleBlueprint]]:
        """List all blueprints organized by category"""
        result = {}
        for category in self.registry._categories:
            result[category] = self.registry.list_by_category(category)
        return result
    
    def export_catalog(self, filepath: str):
        """Export all blueprints to a catalog file"""
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
#  Example Usage
# ============================================================================

def example_usage():
    """Demonstrate the blueprint system"""
    
    print("=" * 60)
    print("FLYING OBJECT BLUEPRINT SYSTEM")
    print("=" * 60)
    
    # 1. Create a blueprint manager
    manager = BlueprintManager()
    
    # 2. List available blueprints
    print("\n📋 Available Blueprints:")
    for category, blueprints in manager.list_blueprints().items():
        print(f"\n  Category: {category.upper()}")
        for bp in blueprints:
            print(f"    - {bp.name} (v{bp.version}) [{bp.id}]")
            print(f"      Mass: {bp.mass_empty:.1f} kg, Components: {len(bp.components)}")
    
    # 3. Create a custom blueprint
    print("\n✏️ Creating custom blueprint...")
    custom_bp = manager.create_blueprint(
        name="Custom Glider",
        description="A lightweight glider for thermal soaring",
        tags=["glider", "sailplane", "beginner"]
    )
    
    # Add components
    wing = WingBlueprint(
        id="wing_glider",
        name="Glider Wing",
        mass=2.0,
        span=5.0,
        root_chord=0.3,
        tip_chord=0.15,
        aspect_ratio=22.0,  # High aspect ratio for glider
        airfoil=AirfoilType.LAMINAR,
        material="composite"
    )
    custom_bp.add_component(wing)
    
    motor = MotorBlueprint(
        id="motor_sustain",
        name="Sustainer Motor",
        mass=0.5,
        thrust_max=15.0,
        power_max=200.0,
        efficiency=0.8,
        motor_type="electric"
    )
    custom_bp.add_component(motor)
    
    # 4. Validate the blueprint
    print("\n🔍 Validating blueprint...")
    valid, errors = custom_bp.validate()
    if valid:
        print("  ✅ Blueprint is valid!")
    else:
        print("  ❌ Validation errors:")
        for error in errors:
            print(f"    - {error}")
    
    # 5. Register the custom blueprint
    manager.registry.register(custom_bp, "custom")
    print(f"\n✅ Registered: {custom_bp.name} (ID: {custom_bp.id})")
    
    # 6. Save blueprint to file
    import tempfile
    import os
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        custom_bp.save_json(f.name)
        print(f"\n💾 Saved blueprint to: {f.name}")
    
    # 7. Load blueprint from file
    loaded_bp = manager.load_blueprint(f.name, register=False)
    print(f"\n📂 Loaded blueprint: {loaded_bp.name}")
    os.unlink(f.name)  # Clean up
    
    # 8. Create a flying object from blueprint
    print("\n✈️ Creating flying object from trainer blueprint...")
    try:
        vehicle = manager.create_vehicle("trainer_v1")
        print(f"  ✅ Created: {type(vehicle).__name__}")
        print(f"    Mass: {vehicle.mass:.1f} kg")
        print(f"    State: {vehicle.position}")
    except Exception as e:
        print(f"  ❌ Error: {e}")
    
    # 9. Export catalog
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        manager.export_catalog(f.name)
        print(f"\n📊 Exported catalog to: {f.name}")
        os.unlink(f.name)  # Clean up
    
    print("\n" + "=" * 60)
    print("✅ Blueprint system demonstration complete!")

# ============================================================================
#  Main execution
# ============================================================================

if __name__ == "__main__":
    example_usage()