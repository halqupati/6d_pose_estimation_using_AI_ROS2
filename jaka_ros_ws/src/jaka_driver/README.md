# jaka_driver

ROS 2 JAKA hardware driver package.

## What it provides

- `jaka_driver` (C++ executable): SDK-backed robot driver node
- `launch/driver_bringup.launch.py`: standard driver bring-up entry
- `config/driver_params.yaml`: robot IP parameter (`ip`)

## Command

```bash
ros2 launch jaka_driver driver_bringup.launch.py
```

## Integration note

Main driver implementation is in `src/jaka_driver.cpp` and links `lib/libjakaAPI.so`.
