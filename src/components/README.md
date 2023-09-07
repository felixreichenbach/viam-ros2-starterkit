# Viam ROS2 Sensor Component Template


## Sensor Component Configuration
```
  "components": [
    {
      "model": "viamlabs:ros2:rossensor",
      "attributes": {
        "ros_topic": "/temperatures"
      },
      "depends_on": [],
      "name": "my-sensor",
      "type": "sensor"
    }
  ]
  ```