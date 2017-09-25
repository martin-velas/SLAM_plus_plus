#!/bin/bash

if [[ $# -ne 1 ]]; then
	>&2 echo "use: $0 <input file>"
	exit 1
fi

grep EDGE_P $0 | awk 'BEGIN { num = 0; } { if(!num) { min_x = min_y = $4; min_y = max_y = $5; num = 1; } else { if($4 < min_x) min_x = $4; else if($4 > max_x) max_x = $4; if($5 < min_y) min_y = $5; else if($5 > max_y) max_y = $5; } } END { printf("min_x %f, min_y %f, max_x %f, max_y %f, w %f, h %f\n", min_x, min_y, max_x, max_y, max_x - min_x, max_y - min_y); }'
# this matches EDGE_PROJ_P2MC or EDGE_PROJECT_P2MC or EDGE_P2MC and a bunch of others; no error checking here
