# Create a Viam Module and Upload it to the Viam Module Registry

## Create / Upload Module

1. Create public namespace under your org’s settings
2. Create meta.json: ```viam module create --name viamlabs-ros2-integration --public-namespace your-registry```
3. Update module after changing the meta.json file ```viam module update```
4. Create tarball: ```tar -czf module.tar.gz run.sh requirements.txt src```
5. Upload module: ```viam module upload --version 0.0.1 --platform linux/arm64 module.tar.gz```


Sample meta.json:
```
{
  "module_id": "your-registry:viamlabs-ros2-starterkit",
  "visibility": "private",
  "url": "https://github.com/felixreichenbach/viam-ros2-starterkit",
  "description": "A simple ROS2 integration framework to help you build ROS2 integrations faster.",
  "models": [
    {
      "api": "rdk:component:sensor",
      "model": "your-registry:ros2:rossensor"
    }
  ],
  "entrypoint": "run.sh"
}
```

## Install on Robot

1. Use the create new component menu and search for the uploaded module
2. Add the module and create the component

```
    {
      "type": "registry",
      "name": "viam-ros2-starter-kit",
      "module_id": "felix-home-registry:viamlabs-ros2-starterkit",
      "version": "0.0.1"
    }
```