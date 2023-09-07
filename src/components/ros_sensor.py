import asyncio
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence

from typing_extensions import Self

from viam.components.sensor import Sensor
from viam.module.module import Module
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from viam.logging import getLogger

#from ros2.viam_ros_node import ViamRosNode
from rclpy.node import Node
from rclpy.subscription import Subscription

# Import ROS2 message type
# e.g. from sensor_msgs.msg import Imu

LOGGER = getLogger(__name__) 
#LOGGER.debug(f"Task {task.get_name()} returned with result {result}")

class ROSSensor(Sensor):
    # Subclass the Viam Sensor component and implement the required functions
    MODEL: ClassVar[Model] = Model(ModelFamily("viamlabs", "ros2"), "rossensor")
    multiplier: float

    ros_topic: str
    ros_node: Node


    def __init__(self, name: str):
        super().__init__(name)

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        sensor = cls(config.name)
        return sensor

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        if "multiplier" in config.attributes.fields:
            if not isinstance(config.attributes.fields["multiplier"], float):
                raise Exception("Multiplier must be a float.")
            cls.multiplier = config.attributes.fields["multiplier"].number_value
            if cls.multiplier == 0:
                raise Exception("Multiplier cannot be 0.")
        else:
            cls.multiplier = 1.0
        return [""]

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        self.multiplier = config.attributes.fields["multiplier"].number_value

    """
    Viam standard Sensor class methods to be implemented
    """

    async def get_readings(
        self, extra: Optional[Dict[str, Any]] = None, **kwargs
    ) -> Mapping[str, Any]:
        return {"signal": 1 * self.multiplier}

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        return command
