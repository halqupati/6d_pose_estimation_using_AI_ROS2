# hexnut_6d_planner

Planner pipeline for hex-nut 6D pose to grasp and compliance command generation.

## Node graph

- `pose_fusion_node`
  - Inputs: `/yolov12_seg/hexnut_pose`, `/aae/hexnut_pose`
  - Output: `/hexnut/pose_6d`
- `grasp_planner_node`
  - Input: `/hexnut/pose_6d`
  - Output: `/graspnet/target_pose`
- `adaptive_compliance_node`
  - Inputs: `/graspnet/target_pose`, `/force_torque/wrench`
  - Output: `/jaka_driver/compliance_twist_cmd`

## Command

```bash
ros2 launch hexnut_6d_planner planner_pipeline.launch.py
```

## Integration note

Current behavior is intentionally minimal and deterministic; replace internal logic with your production YOLOv12-seg + AAE + grasp-net and compliance control policies.
