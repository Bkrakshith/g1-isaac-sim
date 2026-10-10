"""Extra DDS types read directly by run_sim.py (kept free of `from __future__ import annotations`: cyclonedds
resolves the dataclass annotations at runtime)."""
from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import uint8


@dataclass
class Mode_(IdlStruct, typename="g1_interfaces::msg::dds_::Mode_"):
    """g1_interfaces/msg/Mode on the wire (ROS 2 topic /g1/mode = DDS topic rt/g1/mode)."""
    mode: uint8
    source: str
    detail: str
