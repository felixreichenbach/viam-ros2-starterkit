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