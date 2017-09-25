#!/bin/sh

ICS=-1
if [[ $# -ge 1 ]]; then
	ICS=$1
fi

find sparse_flops -name log.txt -print0 |
	while IFS= read -r -d $'\0' f; do
		if [[ -f "$f" ]]; then
			cat "$f"
			if [[ $ICS -gt 0 && `cat "$f" | grep "cliques of size $ICS" | wc -l` -ne 0 ]]; then
				>&2 echo "$f" # output the file to stderr so that it avoids gawk below
			fi
		fi
	done | grep -e "cliques of size" | gawk '{
		histogram[$4] = histogram[$4] + $6;
	}
	END {
		n = asorti(histogram, idx);
		for(i = 1; i <= n; ++ i)
			idx[i] = sprintf("%07d", idx[i]);
		asort(idx);
		for(i = 1; i <= n; ++ i)
			idx[i] = idx[i] + 0;
		for(i = 1; i <= n; ++ i)
			print "cliques of size " idx[i] ": " histogram[idx[i]];
	}'
