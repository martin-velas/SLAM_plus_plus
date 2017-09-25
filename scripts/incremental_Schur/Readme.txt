These scripts are used in analysis of incremental Schur feasibility.

=== ba_graph_incremental_size.sh ===

This script expects incremental BA graph file as an argument and prints
the cummulative numbers of edges and vertices after each CONSISTENCY_MARKER.
This can be used to roughly estimate the cost of Schur complement at each
incremental step.


=== dx_to_numiters.sh ===

This script expects files named "dx_iter_<step>_<linstep>.txt", e.g.:

dx_iter_00001_01.txt
dx_iter_00002_02.txt
dx_iter_00003_03.txt
dx_iter_00004_04.txt
dx_iter_00005_05.txt
dx_iter_00006_01.txt
dx_iter_00007_02.txt
dx_iter_00008_03.txt
dx_iter_00009_04.txt
...

To be present in the current working directory, where each file contains
elements of the dx vector as space separated floating-point numbers on a
single line. It prints the numbers of linear solver iterations taken at
each nonlinear solver step, as space separated numbers on a single line.
This can determine when the nonlinear solver decided to not take more steps.


=== dx_update_stats.sh ===

This script, similarly as dx_to_numiters.sh expects the "dx_iter_<step>_<linstep>.txt"
files in the current working directory. It prints step and linstep of each file,
followed by number of elements in the state, norm of dx and a histogram of elements
of dx with absolute value over 1e-6, 1e-5, 1e-4 and 1e-3.
