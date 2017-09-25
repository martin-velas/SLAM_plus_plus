#!/bin/sh

# 2015-06-16 - modified so that a CONSISTENCY_MARKER is printed at the end of the graph as well
#              so that it optimizes correctly even when running with -nsp 100000000 (means
#              incremental each never).
#            - fixed a condition of consistency so that edges are required to be present after
#              each camera.

if [ $# -ne 2 ]; then
	>&2 echo 'error: use ./graph_reorder_cameras.sh <graph file> <spanning tree>'
	exit
fi

CAM_ORDER=`cat $2 | grep ORDER | gawk 'BEGIN { printf(""); first = 1; } { if(first) { printf($2); first = 0; } else printf(" " $2); } END { printf(""); }'`
# extract camera order as a one-liner

# this works in several passes; the first pass only rearranges the cameras so that they appear
# in the order as they are supposed to in the final graph

# REMARKS: the parser code is duplicated among several calls to gawk,
# when changing something, you need to change it everywhere!

TEMPFILE=$(mktemp)
# or TEMPFILE=$(tempfile) or TEMPFILE=/tmp/grc_temp.graph if this does not work on your system

gawk -v "cam_order=$CAM_ORDER" '
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
		split(cam_order, cameras, " ");
		#print "expecting " length(cameras) " cameras"; # debug

		n = length(cameras);
		for(i = 1; i <= n; ++ i)
			camera_order[cameras[i]] = i;
		# reverse camera order lookup

		StdParseTypes();
		# initialize the parser

		camera_types[vertex_types["VERTEX_CAM"]];
		camera_types[vertex_types["VERTEX_SCAM"]];
		camera_types[vertex_types["VERTEX_SPHERON:QUAT"]];
		# select vertex types which correspond to the cameras

		num_other_edges = 0;
		num_landmarks = 0;
		warned_hypercams = 0;
	}

	{
		str = $0;
		sub(/[ \t]*[%#].*/, "", str); # erase matlab or hash tag comments
		if(length(str)) { # only non-empty lines
			if($1 in vertex_types) {
				# a vertex line

				vid = $2;
				vtype = vertex_types[$1];

				if(vtype in camera_types) {
					if(!(vid in camera_order))
						print "error: camera " vid " not found in the spanning tree" > "/dev/stderr";
					camera_lines[vid] = $0;
					# store the line for printing later
				} else {
					landmark_lines[++ num_landmarks] = $0;
					# store the line for printing later
				}

				# ignore the vertices, only care about the edges in here
			} else if($1 in edge_degrees) {
				# an edge line

				degree = edge_degrees[$1];
				if(NF < degree + 1)
					print "error: edge truncated: \"" $0 "\" (expected at least " degree + 1 " fields, got " NF ")" > "/dev/stderr";
				# not enough parameters for all the vertex ids

				split("", edge_cameras); # empty array
				max_camid = -1;
				for(i = 1; i <= degree; ++ i) {
					camid = $(i + 1); # (supposed) camera id
					vtype = edge_types[$1, i];
					if(vtype in camera_types) { # this edge connects a camera with some landmarks
						edge_cameras[camid];
						if(max_camid == -1 || camera_order[max_camid] < camera_order[camid])
							max_camid = camid;
					}
				}
				# find the camera id which is to be ordered last (this way we can permit odometry edges)

				if(max_camid == -1) {
					other_edge_lines[++ num_other_edges] = $0;
					# remember all the edge lines
				} else {
					cdeg = camera_introdegree[max_camid] = camera_introdegree[max_camid] + 1;
					# calculate degree for the camera which inserts the edge

					camera_introedges[max_camid, cdeg] = $0;
					# remember this edge to be introduced by this camera

					if(length(edge_cameras) >= 2 && degree > 2 && !warned_hypercams) {
						warned_hypercams = 1;
						print "warning: edge with >2 cameras: landmark introduction points may be inaccurate" > "/dev/stderr";
					}
					# if there are multiple cameras in an edge and also landmarks, it is not clear after which camera the landmarks should be introduced
				}
			} else if($1 in special_tokens) {
				print "warning: ignoring line: " $0 > "/dev/stderr";
			} else
				print "error: unkown token: " $1 > "/dev/stderr";
		}
	}

	END {
		for(i = 1; i <= num_landmarks; ++ i)
			print landmark_lines[i];
		# print all the landmark lines without changing the order, we will do that later

		for(i = 1; i <= n; ++ i) {
			camid = cameras[i];
			if(!(camid in camera_lines))
				print "error: camera: " cameras[i] " not found in the graph" > "/dev/stderr";
			print camera_lines[camid];

			if(camid in camera_introdegree) {
				cdeg = camera_introdegree[camid];
				for(j = 1; j <= cdeg; ++ j) {
					print camera_introedges[camid, j];
					delete camera_introedges[camid, j];
				}
			}
		}
		# print the camera lines in the order in which the cameras are supposed to go

		if(length(camera_introedges) > 0)
			print "error: not all the camera edges printed" > "/dev/stderr";
		# make sure we printed all the camera-related edges

		for(i = 1; i <= num_other_edges; ++ i)
			print other_edge_lines[i];
		# print all the other edge lines in the order they came
	}
	' $1 | gawk -v "cam_order=$CAM_ORDER" '
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
		split(cam_order, cameras, " ");
		#print "expecting " length(cameras) " cameras"; # debug

		n = length(cameras);
		for(i = 1; i <= n; ++ i)
			camera_order[cameras[i]] = i;
		# reverse camera order lookup

		StdParseTypes();
		# initialize the parser

		camera_types[vertex_types["VERTEX_CAM"]];
		camera_types[vertex_types["VERTEX_SCAM"]];
		camera_types[vertex_types["VERTEX_SPHERON:QUAT"]];
		# select vertex types which correspond to the cameras

		edge_num = 0;
		other_edge_num = 0;
		warned_hypercams = 0;
	}

	{
		str = $0;
		sub(/[ \t]*[%#].*/, "", str); # erase matlab or hash tag comments
		if(length(str)) { # only non-empty lines
			if($1 in vertex_types) {
				# a vertex line

				vid = $2;
				vtype = vertex_types[$1];

				if(vtype in camera_types) {
					if(!(vid in camera_order))
						print "error: camera " vid " not found in the spanning tree" > "/dev/stderr";
					camera_lines[vid] = $0;
					# store the line for printing later
				} else {
					landmark_lines[vid] = $0;
					# store the line for printing later
				}

				# ignore the vertices, only care about the edges in here
			} else if($1 in edge_degrees) {
				# an edge line

				degree = edge_degrees[$1];
				if(NF < degree + 1)
					print "error: edge truncated: \"" $0 "\" (expected at least " degree + 1 " fields, got " NF ")" > "/dev/stderr";
				# not enough parameters for all the vertex ids

				edge_lines[++ edge_num] = $0;
				# remember the line

				split("", edge_cameras); # empty array
				max_camid = -1;
				for(i = 1; i <= degree; ++ i) {
					camid = $(i + 1); # (supposed) camera id
					vtype = edge_types[$1, i];
					if(vtype in camera_types) { # this edge connects a camera with some landmarks
						edge_cameras[camid];
						if(max_camid == -1 || camera_order[max_camid] < camera_order[camid])
							max_camid = camid;
					} else {
						lmnum = edge_lmnum[edge_num] = edge_lmnum[edge_num] + 1;
						edge_landmarks[edge_num, lmnum] = camid;
						# store the landmarks inside this edge in an array for a later lookup
					}
				}
				# find the camera id which is to be ordered last (this way we can permit odometry edges)

				if(max_camid == -1) {
					other_edge_lines[++ other_edge_num] = $0;
					if(edge_num in edge_lmnum)
						delete edge_lmnum[edge_num];
					delete edge_lines[edge_num]; # this edge is to be printed differently
					-- edge_num; # this id is now unused
				} else {
					for(c in edge_cameras)
						camera_introdegree[c] = camera_introdegree[c] + 1;
					# calculate degree for each camera (will determine how much space needs to be left for the line ids)

					if(length(edge_cameras) >= 2 && degree > 2 && !warned_hypercams) {
						warned_hypercams = 1;
						print "warning: edge with >2 cameras: landmark introduction points may be inaccurate" > "/dev/stderr";
					}
					# if there are multiple cameras in an edge and also landmarks, it is not clear after which camera the landmarks should be introduced

					last_cam_edge[max_camid] = edge_num;
					# remember the number of edges up to this camera
					# this can be done because we know that the edges come in the same order as the cameras

					for(i = 1; i <= degree; ++ i) {
						lmid = $(i + 1); # (supposed) camera id
						vtype = edge_types[$1, i];
						if(!(vtype in camera_types)) { # landmark vertices
							lm_degree[lmid] = lm_degree[lmid] + 1;
							if(lm_degree[lmid] >= 2 && !(lmid in introduced_landmarks)) {
								introduced_landmarks[lmid];
								# keep a set of landmarks which were already introduced

								lmnum = camera_intro_lmnum[max_camid] = camera_intro_lmnum[max_camid] + 1;
								camera_intro_landmarks[max_camid, lmnum] = lmid;
								# count how many landmarks a camera introduces, make a list
							}
						}
					}
					# accumulate landmark degrees
					# and determine points of insertion of the landmarks; now we can do that because the
					# edges that observe the landmarks appear in the same order as the (ordered) cameras

					# at this point we may or may not know where the edge should be sorted. if all the
					# landmarks are introduced then we can just see which of them is introduced the last
					# and that will be the camera where this edge needs to appear. otherwise we need to
					# wait for more observations. it would be faster to have two branches but this is
					# error prone enough as it is, lets go slow'n'steady.
				}
			} else if($1 in special_tokens) {
				print "warning: ignoring line: " $0 > "/dev/stderr";
			} else
				print "error: unkown token: " $1 > "/dev/stderr";
		}
	}

	END {
		for(i = 1; i <= n; ++ i) {
			printed_edges = 0;
			# reset the counter at each camera to make sure that
			# there are new edges before each CONSISTENCY_MARKER
			# note that this test is rudimentary, we should test
			# whether each vertex in the system is covered by an
			# edge instead. seems to work so far, though.

			camid = cameras[i];
			if(!(camid in camera_lines))
				print "error: camera: " cameras[i] " not found in the graph" > "/dev/stderr";
			print camera_lines[camid];
			# print camera line

			if(camid in camera_intro_lmnum) {
				lmnum = camera_intro_lmnum[camid];
				for(j = 1; j <= lmnum; ++ j) {
					lmid = camera_intro_landmarks[camid, j];
					visible_landmarks[lmid];
					print landmark_lines[lmid];
				}
			}
			# print all the landmarks that are introduced by this camera

			if(camid in last_cam_edge) {
				printable_edge_num = last_cam_edge[camid];
				for(j = 1; j <= printable_edge_num; ++ j) {
					if(!(j in edge_lines))
						continue;
					lmnum = edge_lmnum[j];
					can_insert = 0;
					for(k = 1; k <= lmnum; ++ k) {
						if(edge_landmarks[j, k] in visible_landmarks) {
							print edge_lines[j];
							delete edge_lines[j];
							++ printed_edges;
						}
					}
				}
			}
			# for each edge which wasnt seen yet

			if(printed_edges && (1 || i != n)) # permit after last camera as well, when running with -nsp 10000000 then the final optimization is not implicit
				print "CONSISTENCY_MARKER"; # only if this camera is not the last one
			# in case there were edges, print a consistency marker - we can optimize the system now

			# note that there will be one marker potentially missing after the last camera
			# if there are errand vertices introduced after it, but the semantics are unclear
			# so we may as well leave it as is.
		}

		printed_more = 0;

		warned_about_errand_landmarks = 0;
		for(lmid in landmark_lines) {
			if(lmid in introduced_landmarks)
				continue;
			if(!warned_about_errand_landmarks) {
				warned_about_errand_landmarks = 1;
				print "warning: there are errand landmarks in the graph" > "/dev/stderr";
			}
			print landmark_lines[lmid];
			++ printed_more;
		}
		# print all the vertices which were not introduced by the cameras

		if(length(edge_lines) > 0) {
			print "warning: there are errand edges in the graph" > "/dev/stderr";
			for(j = 1; j <= edge_num; ++ j) {
				if(!(j in edge_lines))
					continue;
				print edge_lines[j];
				++ printed_more;
			}
		}
		# print all the edges which failed to print

		for(i = 1; i <= other_edge_num; ++ i)
			print other_edge_lines[i];
		printed_more += other_edge_num;
		# print all the edges that do not have cameras in them

		if(printed_more) # do that. when running with -nsp 10000000 then the final optimization is not implicit
			print "CONSISTENCY_MARKER"; # dont, there will be the default batch running at the end, this would make it run twice
		# print one more marker at the end
	}' > $TEMPFILE

./graph_linearize_ids.sh $TEMPFILE
# the above only reorders the graph, we need to linearize ids

rm -f $TEMPFILE
# remove the temp file
