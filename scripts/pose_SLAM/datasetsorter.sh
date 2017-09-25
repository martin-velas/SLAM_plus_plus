#!/bin/sh

# this is a very simple script for sorting the pose SLAM datasets. it cannot handle
# comments, vertex initializations or hyperedges, it expects that the second and
# third fields of each line are vertex ids.

# it would have to be modified to handle all those.

# this sorts the order of the lines in the file without changing the ids of the vertices.
# this may still produce invalid graphs in case the ids are not contiguous zero-based integers.
# the other script called graph_linearize_ids.sh needs to be applied in such case.

if [ $# -ne 1 ]; then
	>&2 echo 'error: use ./datasetsorter.sh <graph file>'
	exit
fi

cat $1 | gawk ' { x = $2; y = $3; if(x > y) { x = $3; y = $2; } print x + y * 100000000 " " $0; } ' | sort -n | gawk ' { s = $2; for(i = 3; i <= NF; ++ i) s = s " " $i; print s; } '
