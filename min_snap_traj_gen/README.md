# trajectory generator

`trajectory_generator_base` defines basic structure (including objective function, common parameters, topics, etc).

Specific trajectories (e.g. `ellipse.launch`) function by inheriting from the base and override the constructor to load the relevant parameters and `updateWaypoints` to create the waypoints for the optimization.

All trajectories share the following parameters

```YAML
frame_id: "map"
...
optimization:
  rho_t: 250.0
  rho_v: 200.0
  rho_a: 200.0
  vmax: 1.0
  amax: 1.0
  max_iter: 500
  M: 5
...
start_with_pose: false
pose_topic: "/mavros/local_position/pose"
```

where `M` relates to the L-BFGS optimization, and the `rho_*` variables are penalties for duration, velocity, and acceleration considering soft constraints for max velocity and acceleration. The rest are self-explanatory.

`rviz.launch` contains the visualization setup including `takeoff` and `start` service buttons.

> [!CAUTION]
> Trajectories are defined w.r.t. coordinate frame at start, therefore long trajectories can be negatively impacted by small yaw misalignment.