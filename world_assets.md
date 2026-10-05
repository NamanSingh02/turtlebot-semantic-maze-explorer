# Part 2: Semantic maze assets

My semantic world is `tb_worlds/worlds/sim_house_semantic.sdf.xacro`. The placement manifest is `semantic_benches.json`; `tools/build_semantic_world.py` reproduces the world from the original maze. Poses use the Gazebo world frame, metres, and radians. The original maze model translation is included in these poses.

| Bench ID | Centre x, y | Yaw | Wall | Object set |
| --- | --- | --- | --- | --- |
| bench_south | 2.434466, -1.778810 | 0 | Wall_4 | cup, bottle, book |
| bench_east | 5.230030, 1.552894 | 1.570796 | Wall_6 | bowl, laptop, mouse |
| bench_north | 4.471170, 3.222890 | 0 | Wall_7 | cup, bowl, vase |

Each bench measures 1.20 × 0.36 m, with its top at 0.13 m. Each wall is 0.15 m thick. Bench centres are 0.255 m from the wall centreline, placing the bench edge against the inner wall face. Three objects are spaced 0.36 m apart along each bench. Static geometry prevents objects from falling during simulation.

My objects use local composite SDF geometry: cups have hollow walls and handles; bottles have bodies, shoulders, necks and caps; books have pages and covers; the laptop has a keyboard base and raised screen. The bowl, mouse and vase also use procedural geometry. Their intended classes belong to COCO. These models do not establish that pretrained YOLO will recognize them reliably; realistic textures or replacement meshes may be needed after camera evaluation.

I removed the saved world-state snapshot from the generated world so it cannot override the model poses. Automated checks verified the bench count, distinct object sets, object presence and wall contact. Xacro expanded successfully.

Camera visibility and detector performance have not been measured. Camera screenshots were excluded from the implementation scope, so no screenshots are included or claimed. The low bench height is a design choice for the robot camera and remains subject to runtime verification.
