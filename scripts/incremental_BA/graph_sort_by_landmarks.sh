# use this to sort a BA dataset edges by landmark ids. all the
# ertices are at the beginning and the edges are sorted by landmark id in stable fashion

# this works for both incremental and "normal" BA datasets

# vertex order or ids are not affected in any way

# this allows data with VERTEX_INTRINSICS but does not perform any ordering on intrinsics

if [ $# -ne 1 ]; then
	>&2 echo 'error: use ./graph_sort_by_landmarks.sh <graph file>'
	exit
fi

# the structure is:
# verrices (preserve order)
# edges (sort)
# consistency marker
# verrices (preserve order)
# edges (sort)
# consistency marker
# ...
# verrices (preserve order)
# edges (sort)
# consistency marker

# to assign ids for sorting, one would need to sort by 3 keys (descending importance):
# (segment id) | (type in segment vert / edge / marker) | (landmark id of an edge or line inside the file to make the sort stable)
# this transforms to a linear key as:
# (segment id) * (number of types) * max(highest landmark id, number of lines) +
# (type in segment vert / edge / marker) * max(highest landmark id, number of lines) +
# (landmark id of an edge or line inside the file to make the sort stable)

# could also use sort -n -s which is stable (-s)

#num_segments=`sed $'s/\r$//' $1 | grep CONSISTENCY_MARKER | wc -l`
num_lines=`sed $'s/\r$//' $1 | wc -l`
max_lmid=`sed $'s/\r$//' $1 | grep VERTEX_XYZ | cut -d " " -f 2 | gawk 'BEGIN { max = 0; } { if(max < $1) max = $1 } END { print max; }'`

sed $'s/\r$//' $1 | gawk -v "num_lines=$num_lines" -v "max_lmid=$max_lmid" '
	BEGIN {
		if(max_lmid < num_lines)
			max_lmid = num_lines;
		segment_id = 0;
		line = 0;
	}
	{
		if($1 == "CONSISTENCY_MARKER") {
			sort_val = segment_id * 3 * max_lmid + 2 * max_lmid + line; # CONSISTENCY_MARKER is at the end of the segment
			print sort_val " " $0;
			++ segment_id;
		} else if(index($1, "VERTEX_")) {
			sort_val = segment_id * 3 * max_lmid + 0 * max_lmid + line; # VERTEX_* is at the beginning of the segment
			print sort_val " " $0;
		} else if(index($1, "EDGE_")) {
			sort_val = segment_id * 3 * max_lmid + 1 * max_lmid + $2; # EDGE_* is in the middle of the segment (well, before CONSISTENCY_MARKER anyway), and are relatively ordered by landmark id, which is in $2
			print sort_val " " $0;
		}
		++ line;
	}' | sort -n | cut -d " " -f 2-

exit # failed attempt below

# the code below is suppseod to be a preprocessing pass for graph_reorder_cameras.sh
# to sort the edges by landmark ids so that the landmarks are inserted by the edges
# in the same order as they come. unfortunately, this is inconsequential and in fact
# the outputs on cathedral with and without using the script are the same because
# the linearize_ids script changes the ids. probably the only way to sort it is after,
# not before.

cat $1 | gawk -v "num_lines=`cat $1 | wc -l`" '
	function MkEdge(edge_name,verts) # verts contains vertex tokens separated by spaces; fills edge_degrees[edge_name] with edge degree and edge_types[edge_name][i] with vertex types
	{
		split(verts, edge, " ");
		degree = 1;
		for(v in edge) {
			v = edge[v];
			if(v in vertex_types) {
				edge_types[edge_name, degree] = vertex_types[v];
				++ degree;
			} else
				print "error: unknown vertex " v > "/dev/stderr";
		}
		edge_degrees[edge_name] = degree - 1;
	}

	function StdParseTypes()
	{
		special_tokens["EQUIV"] = "";
		# define special tokens that are supposed to go through unmodified

		vertex_types["VERTEX2"] = 0;
		vertex_types["VERTEX_SE2"] = 0;
		vertex_types["VERTEX"] = 0;

		vertex_types["VERTEX_LANDPARK2"] = 1; // does not really exist as a token but we need it semantically

		vertex_types["VERTEX3"] = 2;
		vertex_types["VERTEX_SE3"] = 2;

		vertex_types["VERTEX_XYZ"] = 3;

		vertex_types["VERTEX_CAM"] = 4;

		vertex_types["VERTEX_INTRINSICS"] = 5;

		vertex_types["VERTEX_SCAM"] = 6;

		vertex_types["VERTEX_SPHERON:QUAT"] = 7;
		# define vertex group types

		MkEdge("EDGE2", "VERTEX2 VERTEX2");
		MkEdge("EDGE_SE2", "VERTEX2 VERTEX2");
		MkEdge("EDGE", "VERTEX2 VERTEX2");
		MkEdge("ODOMETRY", "VERTEX2 VERTEX2");

		MkEdge("LANDMARK2:XY", "VERTEX2 VERTEX_LANDPARK2");
		MkEdge("EDGE_SE2_XY", "VERTEX2 VERTEX_LANDPARK2");
		MkEdge("EDGE_BEARING_SE2_XY", "VERTEX2 VERTEX_LANDPARK2");
		MkEdge("LANDMARK2:RB", "VERTEX2 VERTEX_LANDPARK2");
		MkEdge("EDGE_BEARING_SE2_RB", "VERTEX2 VERTEX_LANDPARK2"); # note that xy and rb landmarks do not have the same measurement but are between the same vertices

		MkEdge("EDGE3", "VERTEX3 VERTEX3");
		MkEdge("EDGE3:AXISANGLE", "VERTEX3 VERTEX3");
		MkEdge("EDGE_SE3", "VERTEX3 VERTEX3");
		MkEdge("EDGE_SE3:AXISANGLE", "VERTEX3 VERTEX3"); # note that RPY and axis-angle edges do not have the same measurement but are between the same vertices

		MkEdge("LANDMARK3:XYZ", "VERTEX3 VERTEX_XYZ");
		MkEdge("EDGE_SE3_XYZ", "VERTEX3 VERTEX_XYZ");

		MkEdge("EDGE_SPHERON_XYZ", "VERTEX_SPHERON:QUAT VERTEX_XYZ");

		MkEdge("EDGE_PROJECT_P2MC", "VERTEX_XYZ VERTEX_CAM");
		MkEdge("EDGE_P2MC", "VERTEX_XYZ VERTEX_CAM");
		MkEdge("EDGE_P2C", "VERTEX_XYZ VERTEX_CAM"); # note that the landmark goes first

		MkEdge("EDGE_PROJECT_P2MCI", "VERTEX_XYZ VERTEX_CAM VERTEX_INTRINSICS");
		MkEdge("EDGE_P2MCI", "VERTEX_XYZ VERTEX_CAM VERTEX_INTRINSICS");
		MkEdge("EDGE_P2CI", "VERTEX_XYZ VERTEX_CAM VERTEX_INTRINSICS"); # note that the landmark goes first

		MkEdge("EDGE_PROJECT_P2SC", "VERTEX_XYZ VERTEX_SCAM");
		MkEdge("EDGE_P2SC", "VERTEX_XYZ VERTEX_SCAM"); # note that the landmark goes first
		# define edge types
	}

	BEGIN {
		StdParseTypes();
		# initialize the parser

		camera_types[vertex_types["VERTEX_CAM"]];
		camera_types[vertex_types["VERTEX_SCAM"]];
		camera_types[vertex_types["VERTEX_SPHERON:QUAT"]];
		# select vertex types which correspond to the cameras

		line = 0;
	}

	{
		if(NF > 1 && $1 in edge_degrees) {
			print (num_lines + $2) " " $0; # prefix the line with landmark id + line number to allow for stable sorting
		} else
			print line " " $0; # prefix the line with line number to allow for stable sorting
		++ line;
	}' | sort -n | cut -d " " -f 2-
