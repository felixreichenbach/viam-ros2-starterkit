import asyncio
from viam.module.module import Module
from viam.components.sensor import Sensor
from components.ros_sensor import ROSSensor
from viam.logging import getLogger

from ros2.rclpy_utils import RclpyNodeManager
from ros2.viam_ros_node import ViamRosNode

LOGGER = getLogger(__name__)


async def main():
    """This function creates and starts a new module, after adding all desired resource models.
    Resource creators must be registered to the resource registry before the module adds the resource model.
    """

    try:
        global rclpy_mgr
        global viam_node
        LOGGER.info("Starting ros2 module server")

        # setup viam ros node & do we need to do work in finally
        rclpy_mgr = RclpyNodeManager.get_instance()
        viam_node = ViamRosNode.get_viam_ros_node()
        rclpy_mgr.spin_and_add_node(viam_node)

        m = Module.from_args()
        m.add_model_from_registry(Sensor.SUBTYPE, ROSSensor.MODEL)

        await m.start()
    finally:
        rclpy_mgr.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
