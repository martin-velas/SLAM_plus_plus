#!/bin/sh

# 2015-06-16 - modified so that a CONSISTENCY_MARKER is printed at the end of the graph as well
#              so that it optimizes correctly even when running with -nsp 100000000 (means
#              incremental each never).
#            - fixed a condition of consistency so that edges are required to be present after
#              each camera.
# 2015-06-19 - optimized the lookup of the edges, this is now 8x faster than the older version
#              of the script (retained as graph_reorder_cameras_slow-reference-impl.sh) while
#              giving identical outputs.
# 2015-06-29 - added a final sorting pass which orders the edges by the landmark, so that the
#              blocks are added into the matrix in nicely ordered fashion. otherwise it was
#              horribly inefficient.

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
TEMPFILE2=$(mktemp)
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

		camera_num = length(cameras);
		for(i = 1; i <= camera_num; ++ i)
			camera_order[cameras[i]] = i;
		# reverse camera order lookup

		StdParseTypes();
		# initialize the parser

		camera_types[vertex_types["VERTEX_CAM"]];
		camera_types[vertex_types["VERTEX_SCAM"]];
		camera_types[vertex_types["VERTEX_SPHERON:QUAT"]];
		# select vertex types which correspond to the cameras

		num_other_edges = 0;
		#num_landmarks = 0; # unused
		warned_hypercams = 0;

		#time_1stpass = systime(); # debug
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
					print $0; # print right away, will not reorder them and is supposed to be the first in the file
					#landmark_lines[++ num_landmarks] = $0;
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
		#for(i = 1; i <= num_landmarks; ++ i)
		#	print landmark_lines[i];
		# print all the landmark lines without changing the order, we will do that later

		camera_introedges_to_remove = 0;
		if(1) { # see if it is faster to remove from the array, or to just count removed
			# this branch makes the whole pass take 31 seconds
			for(i = 1; i <= camera_num; ++ i) {
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
		} else {
			# this branch makes the whole pass take 32 seconds (i guess it is slower because then we have to look up a larger associative array)
			for(i = 1; i <= camera_num; ++ i) {
				camid = cameras[i];
				if(!(camid in camera_lines))
					print "error: camera: " cameras[i] " not found in the graph" > "/dev/stderr";
				print camera_lines[camid];

				if(camid in camera_introdegree) {
					cdeg = camera_introdegree[camid];
					for(j = 1; j <= cdeg; ++ j) {
						print camera_introedges[camid, j];
						#delete camera_introedges[camid, j];
						++ camera_introedges_to_remove;
					}
				}
			}
		}
		# print the camera lines in the order in which the cameras are supposed to go

		if(length(camera_introedges) > camera_introedges_to_remove)
			print "error: not all the camera edges printed" > "/dev/stderr";
		# make sure we printed all the camera-related edges

		for(i = 1; i <= num_other_edges; ++ i)
			print other_edge_lines[i];
		# print all the other edge lines in the order they came

		#print "debug: the 1st pass finished (" (systime() - time_1stpass) " sec elapsed)" > "/dev/stderr"; # debug
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

		camera_num = length(cameras);
		for(i = 1; i <= camera_num; ++ i)
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

		edge_insertion_doublecheck = 1;
		# be safe

		#print "debug: the 2nd pass starting" > "/dev/stderr";
		#time_reading = systime(); # debug
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

				# file reading alone takes about 11 seconds on venice

				edge_lines[++ edge_num] = $0; # on venice, this takes ~7 seconds (large volume of data)
				# remember the line

				#split("", edge_cameras); # empty array # unused
				max_camid = -1;
				for(i = 1; i <= degree; ++ i) {
					camid = $(i + 1); # (supposed) camera id
					vtype = edge_types[$1, i];
					if(vtype in camera_types) { # this edge connects a camera with some landmarks
						#edge_cameras[camid]; # unused
						if(max_camid == -1 || camera_order[max_camid] < camera_order[camid])
							max_camid = camid;
					} else {
						lmnum = edge_lmnum[edge_num] = edge_lmnum[edge_num] + 1;
						edge_landmarks[edge_num, lmnum] = camid;
						# store the landmarks inside this edge in an array for a later lookup
					}
				}
				# find the camera id which is to be ordered last (this way we can permit odometry edges)
				# this takes about 12 seconds on venice

				if(max_camid == -1 && !(edge_num in edge_lmnum)) { # no camera and no landmarks to speak of
					other_edge_lines[++ other_edge_num] = $0;
					delete edge_lines[edge_num]; # this edge is to be printed differently
					-- edge_num; # this id is now unused
				} else {
					#for(c in edge_cameras)
					#	camera_introdegree[c] = camera_introdegree[c] + 1;
					## calculate degree for each camera (will determine how much space needs to be left for the line ids)
					# not used by this second pass script

					#if(length(edge_cameras) >= 2 && degree > 2 && !warned_hypercams) {
					#	warned_hypercams = 1;
					#	print "warning: edge with >2 cameras: landmark introduction points may be inaccurate" > "/dev/stderr";
					#}
					## if there are multiple cameras in an edge and also landmarks, it is not clear after which camera the landmarks should be introduced
					# not used here, already warned about this in the 1st pass above

					edge_last_cam[edge_num] = max_camid; # note that the camera id is guaranteed to be valid (see condition of this branch). edges on landmarky only are sorted at the end of the file now.
					if(edge_insertion_doublecheck) # optimization: this reverse array is not needed unless there is something wrong with the graph. not sure if at all, here it serves a double-check.
						last_cam_edge[max_camid] = edge_num; # this does not really take any time
					# remember the number of edges up to this camera
					# this can be done because we know that the edges come in the same order as the cameras

					for(i = 1; i <= degree; ++ i) {
						lmid = $(i + 1); # (supposed) camera id
						vtype = edge_types[$1, i];
						if(!(vtype in camera_types)) { # landmark vertices
							lm_degree[lmid] = lm_degree[lmid] + 1;
							if(lm_degree[lmid] >= 2 && !(lmid in introduced_landmarks)) {
								introduced_landmarks[lmid] = max_camid; # remember the camera where the landmark is introduced (reverse lookup of camera_intro_landmarks)
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
					# this takes about 12 seconds on venice

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
		#time_reading = systime() - time_reading;
		#print "debug: the 2nd pass in the final phase (" time_reading " sec elapsed)" > "/dev/stderr";
		#time_edges = systime(); # debug

		for(i = 1; i <= edge_num; ++ i) {
			eintro_cam_id = -1;
			# we may know nothing

			if(i in edge_last_cam)
				eintro_cam_id = edge_last_cam[i]; # guaranteed to be a valid camera
			# if there are cameras in the edge, then the edge cannot come before those cameras

			lmnum = edge_lmnum[edge_num];
			for(j = 1; j <= lmnum; ++ j) {
				lmid = edge_landmarks[i, j];
				if(lmid in introduced_landmarks) {
					lmintro_cam_id = introduced_landmarks[lmid];
					# id of the camera that introduces the landmarks

					if(eintro_cam_id == -1 || camera_order[eintro_cam_id] < camera_order[lmintro_cam_id])
						eintro_cam_id = lmintro_cam_id;
					# find the camera that introduces the last landmark
				} else
					print "error: landmark " lmid " never introduced, edge " i " may be affected" > "/dev/stderr"; # will also later print a warning about errand landmarks
			}
			# go through edge landmarks and see what landmark is introduced at the latest
			# this is essentialy a constant cost (O(2) or O(3) for most BA or SfM datasets)

			if(eintro_cam_id == -1) { # this edge did not have landmarks related to a camera, nor did it have cameras
				other_edge_lines[++ other_edge_num] = edge_line[i];
				edge_line[i] = "";
				print "warning: edge " i " never introduced, moving to the end of the file" > "/dev/stderr";
				continue;
			}
			# in case the landmarks did not introduce the edge, see if a camera could introduce it
			# another constant cost, only runs if the edge does not have landmarks (odometry / unary factors on cameras)

			if(edge_insertion_doublecheck) { # optimization: assume all is well and we dont need this check
				printable_edge_num = last_cam_edge[eintro_cam_id];
				if(i > printable_edge_num) {
					# cameras say that the edge should be introduced later. why? should not really happen unless i forgot something

					for(j = 1; j <= camera_num; ++ j) {
						if(cameras[j] == eintro_cam_id + 1) # cameras not indexed linearly - there is the ordering
							break; # found the next camera after eintro_cam_id
					}
					for(; j <= camera_num; ++ j) {
						camid = cameras[j];
						if(last_cam_edge[camid] >= i) {
							print "warning: postponing edge " i " from camera " eintro_cam_id " till camera " camid " (edge_last_cam[i] = " edge_last_cam[i] ")" > "/dev/stderr";
							eintro_cam_id = camid;
							printable_edge_num = last_cam_edge[eintro_cam_id];
							break;
						}
					}
					# see if there is a camera later that would allow this edge
	
					if(i > printable_edge_num) {
						other_edge_lines[++ other_edge_num] = edge_line[i];
						edge_line[i] = "";
						print "warning: edge " i " never introduced, moving to the end of the file" > "/dev/stderr";
						continue;
					}
					# if not, postpone it
				} else { #if(i <= printable_edge_num)
					# all is good, the edge can be printed
				}
				# see when an edge can be introduced
			}

			# the edge can be printed after camera eintro_cam_id

			e_num = camera_intro_enum[eintro_cam_id] = camera_intro_enum[eintro_cam_id] + 1;
			camera_intro_edges[eintro_cam_id, e_num] = i;
			# build a list of edges that each camera introduces
		}
		# an O(n) algorithm for sorting edges between cameras (theoretical, not counting the associative array access complexity)

		#time_edges = systime() - time_edges;
		#print "debug: edge insertion order determined (" time_edges " sec elapsed)" > "/dev/stderr";
		#time_writing = systime(); # debug

		for(i = 1; i <= camera_num; ++ i) {
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
					#visible_landmarks[lmid]; # unused now
					print landmark_lines[lmid];
				}
			}
			# print all the landmarks that are introduced by this camera

			#if(camid in last_cam_edge) {
			#	printable_edge_num = last_cam_edge[camid];
			#	for(j = 1; j <= printable_edge_num; ++ j) {
			#		if(!(j in edge_lines))
			#			continue;
			#		lmnum = edge_lmnum[j];
			#		can_insert = 0;
			#		for(k = 1; k <= lmnum; ++ k) {
			#			if(edge_landmarks[j, k] in visible_landmarks) {
			#				print edge_lines[j];
			#				delete edge_lines[j];
			#				++ printed_edges;
			#			}
			#		}
			#	}
			#}
			## for each edge which wasnt seen yet
			# the "slow" way of printing the edges, requires O(n) at each camera

			if(camid in camera_intro_enum) {
				e_num = camera_intro_enum[camid];
				for(j = 1; j <= e_num; ++ j) {
					edgeid = camera_intro_edges[camid, j];
					if(!(edgeid in edge_lines))
						print "error: edge: " edgeid " wanted to be printed twice (but was not)" > "/dev/stderr";
					else {
						print edge_lines[edgeid];
						delete edge_lines[edgeid];
						++ printed_edges;
					}
				}
			}
			# simply print all the edges that this camera was to introduce, O(n) for all cameras

			if(printed_edges)# && i != camera_num) # permit after last camera as well, when running with -nsp 10000000 then the final optimization is not implicit
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

		#time_writing = systime() - time_writing;
		#print "debug: linearizing the ids (" time_writing " sec elapsed)" > "/dev/stderr"; # debug
	}' > $TEMPFILE

./graph_linearize_ids.sh $TEMPFILE > $TEMPFILE2
# the above only reorders the graph, we need to linearize ids
# note that the above cannot redirect to the same file, as it
# does stream processing - it prints the lines right away ...

rm -f $TEMPFILE
# remove the temp file

./graph_sort_by_landmarks.sh $TEMPFILE2
# after linearizing the vertex ids, the order in which edges
# reference landmarks tends to be rather randoml. sort the order of the edges, so that the
# landmarks come in ascending order

rm -f $TEMPFILE
# remove the temp file
