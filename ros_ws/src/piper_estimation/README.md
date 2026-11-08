# piper_estimation

Contact and grasp estimates from the Piper's own feedback, with no force sensor: the external
wrench at the tool from the joint torques the motors report, and whether the gripper holds
something.

## Run

Beside a running stack (it reads `/robot_description` and `/joint_states`):

```bash
ros2 launch piper_estimation state_estimator.launch.py
ros2 launch piper_estimation state_estimator.launch.py use_sim_time:=true
```

## How it estimates

- The wrench: the joint torques minus the arm's gravity torques (a KDL model built from the URDF's
  masses) are taken as external, and mapped to a wrench at `tip_link` through the pseudo-inverse of
  the Jacobian's transpose, expressed in `base_link`. Friction and the motors' current-to-torque
  error stay in the estimate, so it suits contact detection and coarse force feedback, not force
  control. Near singular poses (the folded rest pose) it means little.
- The grasp: the gripper holds something while its torque stays above
  `grasp_effort_threshold_nm` with the fingers more than `grasp_min_opening_m` apart, for
  `grasp_samples` joint states in a row.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/robot_description` | `std_msgs/String` | In |
| `/joint_states` | `sensor_msgs/JointState` | In |
| `~/tcp_wrench` | `geometry_msgs/WrenchStamped` | Out, one per joint state |
| `~/grasped` | `std_msgs/Bool` | Out on a change (transient local) |

## Parameters

`config/state_estimator.yaml`: the chain (`base_link`, `tip_link`, `joint_names`), the gripper
joint, and the grasp thresholds (0.3 N m, 4 mm, 20 samples at 200 Hz). The thresholds are for the
real gripper; to be tuned at the arm.

## Tests

```bash
./scripts/manage.sh test piper_estimation
```

`test_wrench_estimator` checks the gravity model against a closed form, recovers a known push at
the tool, refuses a URDF without the chain, and the grasp detector's thresholds and persistence.
