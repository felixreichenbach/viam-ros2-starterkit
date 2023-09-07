from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence
from typing_extensions import Self
# ROS 2 Imports
from ros2.viam_ros_node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
# Viam Imports
from viam.components.sensor import Sensor
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from viam.logging import getLogger
# Custom Code Imports
from ros2.viam_ros_node import ViamRosNode

# Import ROS2 message type https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html
from sensor_msgs.msg import Temperature

LOGGER = getLogger(__name__)

class ROSSensor(Sensor, Reconfigurable):
    # Subclass the Viam Sensor component and implement the required functions
    MODEL: ClassVar[Model] = Model(
        ModelFamily("viamlabs", "ros2"), "rossensor")

    # Sensor attributes
    ros_topic: str  # The ROS topic to subscribe to provided through component configuration
    ros_node: Node
    subscription: Subscription
    temperature: float  # A temperature in our example

    def __init__(self, name: str):
        super().__init__(name)

    # Creates a new sensor instance
    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        sensor = cls(config.name)
        cls.temperature = 999
        cls.ros_node = None
        return sensor

    # Validates component configuration parameters
    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        if "ros_topic" in config.attributes.fields:
            cls.ros_topic = config.attributes.fields["ros_topic"].string_value
        else:
            raise Exception('Attribute "ros_topic" is mandatory!')
        return []

    # Applies configuration changes to a component instance
    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        self.ros_topic = config.attributes.fields["ros_topic"].string_value
        if self.ros_node is not None:
            if self.subscription is not None:
                self.ros_node.destroy_subscription(self.subscription)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        self.subscription = self.ros_node.create_subscription(
            Temperature, self.ros_topic, self.subscriber_callback, qos_profile_sensor_data)
        LOGGER.info(f"Reconfigure executed {self.subscription}!")

    # Processes ROS 2 Temperature messages
    def subscriber_callback(self, temperature_msg: Temperature) -> None:
        self.temperature = temperature_msg.temperature

    """
    Viam standard Sensor class methods to be implemented
    """

    # Mandatory: Viam standard sensor API to return sensor reading
    async def get_readings(
        self, extra: Optional[Dict[str, Any]] = None, **kwargs
    ) -> Mapping[str, Any]:
        return {"temperature": self.temperature}

    # Optional: An easy way to add additonal functionality to a sensor component through the Viam standard API
    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        return command


# Register the model with the viam server
Registry.register_resource_creator(Sensor.SUBTYPE, ROSSensor.MODEL, ResourceCreatorRegistration(
    ROSSensor.new, ROSSensor.validate_config))
