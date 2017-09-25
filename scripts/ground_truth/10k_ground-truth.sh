#!/bin/sh
cat $1 | grep "EDGE_SE2" | gawk 'BEGIN { FS = " "; pi = 3.1415926535897932; OFMT = "%.15f"; } function round(x) { if(x >= 0) return int(x + .5); return -int(-x + .5); } { if($3 == $2 + 1) { $4 = round($4); $5 = round($5); $6 = sprintf(OFMT, (pi / 2) * round($6 / (pi / 2))); print $0; } }'
