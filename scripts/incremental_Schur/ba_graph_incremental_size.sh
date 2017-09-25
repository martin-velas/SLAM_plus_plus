#!/bin/sh

if [[ $# != 1 ]]; then
	>&2 echo "error: use ba_graph_incremental_size.sh <graph file>"
	exit
fi

gawk 'BEGIN { print "vertices;edges" e = 0; v = 0; lv = 0; le = 0; } { if(index($1, "VERTEX") == 1) ++ v; else if(index($1, "EDGE") == 1) ++ e; else if($1 == "CONSISTENCY_MARKER") { print v ";" e; lv = v; le = e; } } END { if(lv != v || le != e) print v ";" e; }' $1
