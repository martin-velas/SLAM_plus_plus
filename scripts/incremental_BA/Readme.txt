This is an incremental BA dataset modification script. The ordinary BA datasets
are not suitable for incremental processing because all the cameras are usually
at the beginning of the graph and the points are then added in undefined order.

To process incrementally, we need to reorder the graph file so that the cameras
go in order and the vertices are added once they are seen by at least a pair of
cameras, as it would be in a real vision pipeline (the points position can't be
triangulated from a single view.

To generate incremental datasets of venice871.g2o and cathedral_2014-10-02_.txt
run mkorders.sh. To erase the outputs, run clean.sh. For demo purposes there is
a matlab script called BA_camorder_demo.m. To process the output graph files, a
recent SLAM++ is needed, run with -us -dm -nsp 100000000 -s.

There are three algorithms that generate camera orders from camera connectivity
information: DFS, BFS and BFSpq (priority queue). So far BFS seemed to give the
most reasonable results. DFS gives slightly lower chi2 but seems unsuitable for
GPU processing (keep getting not pos def errors).

This requires GNU AWK (gawk) to run. Matlab scripts worked with 7.6.0 (R2008a).
