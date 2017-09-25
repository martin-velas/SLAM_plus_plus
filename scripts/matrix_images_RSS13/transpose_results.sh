F=*.txt
for f in $F; do echo -n "$f"; cat "$f" | awk 'BEGIN {NL = 0;} { gsub(/\r/, "", $1); a[++ NL] = $1; } END { for(i = 1; i <= NL; ++ i) printf(" %s", a[i]); printf("\n"); }'; done | sort -n
