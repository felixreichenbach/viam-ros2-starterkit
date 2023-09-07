# viam-ros2-starterkit


## Setup Environment

Setup Python virtual environment

```python3 -m venv --system-site-packages venv```

```source .venv/bin/activate```

```python3 -m pip install -r requirements.txt```


```source /opt/ros/humble/setup.bash```



## VS Code Configuration

In .vscode/settings.json

```
{
    "[python]": {
        "editor.defaultFormatter": "ms-python.python"
    },
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages/",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages/",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ]
}
```

## Ubuntu 22.04 Setup

To be able to install viam-server the following package is required:
```
sudo apt install libfuse2
```

## Viam ROS2 Module Configuration

```
  "modules": [
    {
      "name": "starter-kit",
      "executable_path": "/home/ubuntu/viam-ros2-starterkit/run.sh",
      "type": "local"
    }
  ]
  ```
