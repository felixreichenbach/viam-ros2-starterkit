# Viam ROS2 Component Templates

```
  "modules": [
    {
      "name": "starter-kit",
      "executable_path": "/home/ubuntu/viam-ros2-starterkit/run.sh",
      "type": "local"
    }
  ],
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