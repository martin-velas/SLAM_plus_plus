#!/bin/sh

INFILE="$1"
HIGHEST_VERTEX_ID=`sed $'s/\r$//' ${INFILE} | grep VERTEX | awk '{ if(max_idx < $2) max_idx = $2; } END { print max_idx; }'`
#NUM_MARKERS=`sed $'s/\r$//' ${INFILE} | grep CONSISTENCY_MARKER | wc -l`
NUM_CAMERAS=`sed $'s/\r$//' ${INFILE} | grep VERTEX_CAM | wc -l`

# I want to sort by:
#	num_cams * num_verts * 3 * segment denoted by CONSISTENCY_MARKER
#	num_cams * num_verts * (0-2) type of primitive in that segment (vertices first (order unchanged), all edges, then the marker)
#	num_cams * (0-num_verts) landmark id?
#	(0-num_cams) camera id?

# really only need much fewer ids
# num_cams * num_verts * segment + 3
#   vertices get (num_cams * num_verts + 3) * segment + 0
#   edges get    (num_cams * num_verts + 3) * segment + 1 + camera and vertex id
#   markers get  (num_cams * num_verts + 3) * segment + 2 + num_cams * num_verts = (num_cams * num_verts + 3) * (segment + 1) - 1

sed $'s/\r$//' ${INFILE} | awk -v mvert=$HIGHEST_VERTEX_ID -v num_cams=$NUM_CAMERAS 'BEGIN {
		segment_id = 0;
		max_camera_id = 0;
		num_verts = mvert + 1;
	} {
		if($1 == "CONSISTENCY_MARKER") {
			#sort_index = segment_id * 3 * num_verts * num_cams + 2 * num_verts * num_cams; # old indexing
			sort_index = segment_id * (3 + num_verts * num_cams) + 2 + num_verts * num_cams; # CONSISTENCY_MARKER is at the end of the segment
			++ segment_id;
		} else if(index($1, "VERTEX")) {
			#sort_index = segment_id * 3 * num_verts * num_cams + 0 * num_verts * num_cams; # old indexing
			sort_index = segment_id * (3 + num_verts * num_cams) + 0; # VERTEX* is at the beginning of the segment
			if(index($1, "VERTEX_CAM")) {
				camera_ids[$2] = max_camera_id; # keep a table mapping vertex ids to linear camera ids (there are just a few)
				++ max_camera_id;
			}
        } else if($1 == "EDGE_PROJ_SELF") {
			vertex_id = $2;
			camera_id = camera_ids[$3];
			#sort_index = segment_id * 3 * num_verts * num_cams + 1 * num_verts * num_cams + num_cams * vertex_id + camera_id; # old indexing
			sort_index = segment_id * (3 + num_verts * num_cams) + 1 + num_cams * vertex_id + camera_id; # landmark major order
			#sort_index = segment_id * (3 + num_verts * num_cams) + 1 + vertex_id + num_verts * camera_id; # camera major order (seems to yield appalling performance)
		} else if($1 == "EDGE_PROJ_OTHER") {
			vertex_id = $2;
			chain_len = $3;
			camera_id = camera_ids[$4];
			owner_camera_id = camera_ids[$(3 + chain_len)];
			#sort_index = segment_id * 3 * num_verts * num_cams + 1 * num_verts * num_cams + num_cams * vertex_id + camera_id; # old indexing
			sort_index = segment_id * (3 + num_verts * num_cams) + 1 + num_cams * vertex_id + camera_id; # landmark major order
			#sort_index = segment_id * (3 + num_verts * num_cams) + 1 + vertex_id + num_verts * camera_id; # camera major order (seems to yield appalling performance)
		} else {
			print "warning: unknown token \"" $1 "\" (ignored)" | "cat >&2";
		}
		print sort_index " " $0;
	}' | sort -n -s | cut -d " " -f 2-
