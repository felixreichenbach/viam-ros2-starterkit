import asyncio
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.module.module import Module
from viam.components.sensor import Sensor
from components.ros_sensor import ROSSensor

async def main():
    """This function creates and starts a new module, after adding all desired resource models.
    Resource creators must be registered to the resource registry before the module adds the resource model.
    """
    Registry.register_resource_creator(Sensor.SUBTYPE, ROSSensor.MODEL, ResourceCreatorRegistration(ROSSensor.new, ROSSensor.validate_config))

    module = Module.from_args()
    module.add_model_from_registry(Sensor.SUBTYPE, ROSSensor.MODEL)
    await module.start()


if __name__ == "__main__":
    asyncio.run(main())

