# dynamic_global_planner

## Mesh based planner with dynamic planning of paths with minimum crowd

## Algorithm:

- Mesh is generated using the PRM(Probabilistic Roadmap) and published in a form of graph
- A* based planning by [death_star](https://github.com/KrishnaBhatu/death_star) package using the weights of the node as heuristic
- Weights of the node determine the crowding near the nodes
- Continuous checking of the total path weight and if the path weight is increased then the path is replanned with the updated mesh

## Instructiosn to use this package:
- Launch stageros and mesh generator using `smart_planner.launch` script
- Wait untill the mesh is generated whcih will be instructed on the terminal
- Launch the `death_star.launch` as a global plannere for every robot
- Modify your local planning script to call for global path on `gen_path` service
- Launch your local planner and warehouse manager

