#!/bin/sh

ls -1 dx*.txt | gawk 'BEGIN { li = 0; } { split(substr($0, 1, index($0, ".") - 1), a, "_"); idx = a[4] + 0; if(li >= idx) printf("%s ", li); li = idx; } END { printf("\n"); } '
