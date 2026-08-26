# Offline Route Generation

`topology_core/` is a focused source snapshot used to identify the exact
TopoGraph, Bubble A*, and iKD-tree dependencies of `epic_route_labeler`.

The snapshot deliberately excludes the online `epic_graph_node.cpp`. It is not
a license to fork route-search behavior: the implementation step is to expose
the production sources as a reusable `scalenav_topology` target and link the
offline labeler against that target.
