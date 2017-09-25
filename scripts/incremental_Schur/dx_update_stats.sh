#!/bin/sh

echo 'solver step;linear step;state size elems;norm(dx);elems in dx over 1e-6;elems in dx over 1e-5;elems in dx over 1e-4;elems in dx over 1e-3'

for f in dx*.txt
do
	cat $f | gawk -v fn="$f" \
	'BEGIN {
		split(substr(fn, 1, length(fn) - 4), idx, "_");
		n = 0;
		nlg6 = 0;
		nlg5 = 0;
		nlg4 = 0;
		nlg3 = 0;
		norm = 0;
	}
	{
		n = n + NF;
		for(i = 1; i < NF; ++ i) {
			norm = norm + $(i) * $(i);
			absval = $(i);
			if(absval < 0)
				absval = -absval;
			if(absval > 1e-3)
				nlg3 ++;
			if(absval > 1e-4)
				nlg4 ++;
			if(absval > 1e-5)
				nlg5 ++;
			if(absval > 1e-6)
				nlg6 ++;
		}
	}
	END {
		print idx[3] ";" idx[4] ";" n ";" sqrt(norm) ";" nlg6 ";" nlg5 ";" nlg4 ";" nlg3;
	}'
done
