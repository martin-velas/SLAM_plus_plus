#!/bin/sh
dos2unix cityTrees10k.txt
cat cityTrees10k.txt | gawk 'BEGIN { OFS = " "; } { getline line < "cityTrees10000_groundtruth.txt"; print $0 " " line; }' | \
	gawk 'BEGIN { FS = " "; } { if(NF == 24) print $1 " " $2 " " $3 " " $16 " " $17 " " $18 " " $7 " " $8 " " $9 " " $10 " " $11 " " $12; else print $1 " " $2 " " $3 " " $12 " " $13 " " $6 " " $7 " " $8; }' > cityTrees10k_groundtruth.txt
