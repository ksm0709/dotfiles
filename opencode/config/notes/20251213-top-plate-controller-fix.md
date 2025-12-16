# Top Plate Controller Fix (Gazebo)

## Problem

Top plate controller failed to load in `amr_rect_300L_0.1.0` simulation.

- Error 1: `joint_state_controller` type mismatch.
- Error 2: `top_plate_controller` missing PID gains.
- Error 3: Controller expected `EffortJointInterface` but URDF provided `PositionJointInterface`.

## Fixes

1. **Launch File (`ROS/pennybot/launch/gazebo.launch`)**:
   - Updated `joint_state_controller` type to `joint_state_controller/JointStateController`.
   - Moved PID gains to `/top_plate_controller/pid`.
   - Tuned `d` gain from 8000.0 to 100.0.
2. **URDF (`ROS/pennybot_description/urdf/amr_rect_300L_0.1.0.urdf.xacro`)**:
   - Changed `top_plate_joint` to use `EffortJointInterface`.
   - Increased limits: `upper="0.3"` (was 0.06), `effort="5000"`. (Later reverted)

## Useful Commands

Run ROS commands inside `bearenv` container:

```bash
docker exec bear1130 bash -c "source /opt/pennybot/install/setup.bash && rostopic echo -n 1 /top_plate_controller/state"
```

#gazebo #simulation #ros #controller #troubleshooting
