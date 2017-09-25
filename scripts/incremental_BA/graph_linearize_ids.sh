#!/bin/sh

# this script renumbers the ids in such a way that the ids are laid
# out in a global zero-based contiguous space of integers.

# this can fix datasets where the landmarks are in a separate id space
# from the poses (so e.g. id 0 can be both a pose and a landmark,
# depending on the context; certain MIT datasets use this convention).

# when applied to a graph where the ids are not rising, it reassigns the ids.
# it does not change the order in which the vertex values / measurements
# appear in the file (it does not sort).

if [ $# -ne 1 ]; then
	>&2 echo 'error: use ./graph_linearize_ids.sh <graph file>'
	exit
fi

gawk 'function MkEdge(edge_name,verts) # verts contains vertex tokens separated by spaces; fills edge_degrees[edge_name] with edge degree and edge_types[edge_name][i] with vertex types
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
		special_tokens["CONSISTENCY_MARKER"] = "";
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

	function VertexId(vertex_type, old_id) # reassigns vertex ids; vertex_type is vertex type number (obtained from e.g. vertex_types(token)), old_id is id of the vertex in the input graph. returns the new linearized it.
	{
		if((vertex_type, old_id) in vertex_ids)
			return vertex_ids[vertex_type, old_id];
		new_id = next_free_id;
		++ next_free_id;
		vertex_ids[vertex_type, old_id] = new_id;
		return new_id;
	}

	BEGIN {
		next_free_id = 0;
		vertex_ids["foo", "bar"] = -1; # not sure if this is needed

		StdParseTypes(); # initialize the parser
	}
	{
		str = $0;
		sub(/[ \t]*[%#].*/, "", str); # erase matlab or hash tag comments
		if(length(str)) { # only non-empty lines
			if($1 in vertex_types) {
				# a vertex line

				$2 = VertexId(vertex_types[$1], $2);
				# reassign the id

				print $0;
				# print the modified vertex
			} else if($1 in edge_degrees) {
				# an edge line

				degree = edge_degrees[$1];
				if(NF < degree + 1)
					print "error: edge truncated: \"" $0 "\" (expected at least " degree + 1 " fields, got " NF ")" > "/dev/stderr";
				# not enough parameters for all the vertex ids

				for(i = 1; i <= degree; ++ i)
					$(i + 1) = VertexId(edge_types[$1, i], $(i + 1));
				# reassign the ids

				print $0;
				# print the modified edge
			} else if($1 in special_tokens) {
				print $0;
				# print special tokens unmodified
			} else
				print "error: unkown token: " $1 > "/dev/stderr";
		} else
			print $0; # print comments and the like
	}' $1
