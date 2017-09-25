#!/bin/sh

# this takes a graph file and creates a smaller graph file with only cameras
# with the edges between the camera pairs weighted by the number of common points
# that the given camera pairs see. this can be used to calculate optimal camera
# order for incremental processing as a maximum weight spanning tree.

if [ $# -ne 1 ]; then
	>&2 echo 'error: use ./graph_get_cam_connectivity.sh <graph file>'
	exit
fi

gawk '	# verts contains vertex tokens separated by spaces; fills edge_degrees[edge_name] with edge degree and edge_types[edge_name][i] with vertex types
	function MkEdge(edge_name, verts,		degree, v)
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
	}

	function CamConnect(camid0, camid1,		degree, tmp)
	{
		if(camid0 > camid1) {
			tmp = camid0;
			camid0 = camid1;
			camid1 = tmp;
		}
		# make sure the cameras are ordered

		#print "edge " camid0 " -> " camid1;
		# debug

		camera_ids[camid0] = "";
		camera_ids[camid1] = "";
		# keep a set of camera ids

		degree = 0;
		if((camid0, camid1) in camera_links)
			degree = camera_links[camid0, camid1];
		++ degree;
		camera_links[camid0, camid1] = degree;
	}

	function numsort_asc(arr1, arr2,		cmd)
	{
 		cmd = "sort -n"
		return __asort(arr1, arr2, cmd)
	}

	function numsort_desc(arr1, arr2,		cmd)
	{
		cmd = "sort -nr"
		return __asort(arr1, arr2, cmd)
	}

	function __asort(arr1, arr2, cmd,		i, n, m)
	{
		for(i in arr1) { print i |& cmd; }
		close(cmd, "to");
		while((cmd |& getline m) > 0)
			arr2[++ n] = m;
		close(cmd, "from");
		return n;
	}

	# awk integer divide stackoverflow.com/questions/14858182/integer-division-in-awk
	function idiv(n, d)
	{
		return (n - n % d) / d + (n < 0);
	}

	function PrintGraph(		n, sorted_cam_ids, max_camid, c, camera_numkeys, sorted_edge_keys, camid01, camid)
	{
		#n = asorti(camera_ids, sorted_cam_ids);
		n = numsort_asc(camera_ids, sorted_cam_ids); # meh, awk uses string comparison for integers
		for(i = 1; i <= n; ++ i)
			print "CAM " sorted_cam_ids[i];
		# sort and print cameras

		max_camid = (n)? sorted_cam_ids[n] + 1 : 1;
		n = 0;
		for(camid01 in camera_links) {
			split(camid01, camid, SUBSEP);
			n = camid[1] * max_camid + camid[2];
			camera_numkeys[n];# = camera_links[camid[1], camid[2]];
		}
		# convert camera id pairs from strings with integer pairs to simple integers

		#n = asorti(camera_numkeys, sorted_edge_keys);
		n = numsort_asc(camera_numkeys, sorted_edge_keys);
		for(i = 1; i <= n; ++ i) {
			camid01 = sorted_edge_keys[i];
			#split(camid01, camid, SUBSEP);
			camid[1] = idiv(camid01, max_camid);
			camid[2] = camid01 % max_camid;
			print "EDGE " camid[1] " " camid[2] " " camera_links[camid[1], camid[2]];
		}
	}

	{
		str = $0;
		sub(/[ \t]*[%#].*/, "", str); # erase matlab or hash tag comments
		if(length(str)) { # only non-empty lines
			if($1 in vertex_types) {
				# a vertex line

				# ignore the vertices, only care about the edges in here
			} else if($1 in edge_degrees) {
				# an edge line

				degree = edge_degrees[$1];
				if(NF < degree + 1)
					print "error: edge truncated: \"" $0 "\" (expected at least " degree + 1 " fields, got " NF ")" > "/dev/stderr";
				# not enough parameters for all the vertex ids

				for(i = 1; i <= degree; ++ i) {
					camid = $(i + 1); # (supposed) camera id
					vtype = edge_types[$1, i];
					if(vtype in camera_types) { # this edge connects a camera with some landmarks
						for(j = 1; j <= degree; ++ j) {
							if(j == i)
								continue;
							lmid = $(j + 1); # ladndmark id

							if(edge_types[$1, j] in camera_types) {
								CamConnect(camid, lmid);
								# a direct camera-camera edge (e.g. odometry)
							} else {
								# a camera-landmark edge

								lmdegree = 0;
								if(lmid in landmark_degree) {
									lmdegree = landmark_degree[lmid]; # how many cameras observed this so far
									for(k = 1; k <= lmdegree; ++ k)
										CamConnect(camid, landmark_observations[lmid, k]);
								}
								++ lmdegree;
								# connect to the already existing cameras

								landmark_degree[lmid] = lmdegree;
								landmark_observations[lmid, lmdegree] = camid;
								# add the current camera to the list
							}
						}
					}
				}
				# count the camera interactions
			} else if($1 in special_tokens) {
				# quiet
			} else
				print "error: unkown token: " $1 > "/dev/stderr";
		}
	}

	END {
		PrintGraph();
	}' $1
