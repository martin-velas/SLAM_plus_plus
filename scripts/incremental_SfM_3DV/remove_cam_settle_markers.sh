#!/bin/sh

if [[ $# -ne 1 ]]; then
	>&2 echo "use: $0 <input file>"
	exit 1
fi

sed $'s/\r//' "$1" | gawk 'BEGIN { num_cams = 0; num_markers = 0; markers_kept = 0; had_verts = 0; had_other = 0; }
	{
		if($1 == "VERTEX_CAM" || $1 == "VERTEX_CAM:SIM3")
			++ num_cams;
		else if($1 == "VERTEX_XYZ" || $1 == "VERTEX:INVD")
			had_verts = 1;
		if($1 != "CONSISTENCY_MARKER" || had_verts != 0 || had_other == 0) {
			print $0;
			if($1 == "CONSISTENCY_MARKER")
				++ markers_kept;
		}
		if($1 == "CONSISTENCY_MARKER") {
			++ num_markers;
			had_verts = 0;
			had_other = 0;
		} else
			had_other = 1;
	}
	END {
		print "had " num_cams " cameras, " num_markers " markers, kept " markers_kept | "cat >& 2";
	}'
# this drops markers if there are no new landmark vertices introduced in between. keeps runs of several consecutive markers.
