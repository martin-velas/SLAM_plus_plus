#!/bin/bash

if [[ $# -ne 1 ]]; then
	>&2 echo "error: need one argument (the path to the .bla file)"
	exit 1
fi

#echo "`head -n 1 $1` points (`head -n 1 $1 | sed s/[x\(\)]//g | awk '{ print $3 / ($1 * $2) * 100 "% dense"; }'`)"
#echo "`head -n 2 $1 | tail -n 1` blocks (`head -n 2 $1 | tail -n 1 | sed s/[x\(\)]//g | awk '{ print $3 / ($1 * $2) * 100 "% dense"; }'`)"
#tail $1 -n 3 | tail -n 1 | awk '{ for(i = 1; i <= NF; ++ i) { if(i > 1) { n = $i - prev; counts[n] ++; } prev = $i; } for(i in counts) print "block-row-count[" i "] = " counts[i]; }'
#tail $1 -n 4 | tail -n 1 | awk '{ for(i = 1; i <= NF; ++ i) { if(i > 1) { n = $i - prev; counts[n] ++; } prev = $i; } for(i in counts) print "block-col-count[" i "] = " counts[i]; }'
# this could be done in a single head -n 4 and a single awk run.

head -n 2 $1 | sed 's/\r//' | sed s/[x\(\)]//g | awk 'BEGIN { ln = 0; } { if(++ ln == 1) { m = $1; n = $2; nnz = $3; print $1 " x " $2 " elements (" $3 " nnz, " ($3 / ($1 * $2) * 100) "% dense)"; } else { mb = $1; nb = $2; bnnz = $3; print $1 " x " $2 " blocks (" $3 " nnz, " ($3 / ($1 * $2) * 100) "% dense)"; } } END { f = 8; i = 8; print "VBR would take " (6 * i + nnz * f + (mb + 1) * i + (nb + 1) * i + (mb + 1) * i + (bnnz + 1) * i + bnnz * i) / 1048576.0 " MB (double precision, 64-bit indices)"; }'
#                                                                                                                                                                                                                                                                                                                                             m,n,bm,bn,nnz,bnnz   val       rpntr          cpntr          bpntr          indx            bindx
head -n 4 $1 | sed 's/\r//' | tail -n 2 | awk 'BEGIN { ln = 0; } { delete counts; for(i = 1; i <= NF; ++ i) { if(i > 1) { n = $i - prev; counts[n] ++; } prev = $i; } for(i in counts) print ln " " i " |" "block-" ((ln == 0)? "row" : "col") "-count[" i "] = " counts[i]; ++ ln; }' | sort -n | cut -d "|" -f 2
# better - more information
