=== datasetsorter.sh ===

This is a very simple script for sorting the pose SLAM datasets.
It cannot handle comments, vertex initializations or hyperedges,
it expects that the second and third fields of each line are
vertex ids. It would have to be modified to handle all those.

This sorts the order of the lines in the file without changing
the ids of the vertices. Note thet doing this may still produce
invalid graphs in case the ids are not contiguous zero-based
integers. The other script called graph_linearize_ids.sh needs
to be applied in such case.
