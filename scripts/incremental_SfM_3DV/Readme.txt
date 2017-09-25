These are assorted scripts we used for the 3DV paper.

parse_margscosts.sh parses SLAM++ logs and outputs .cvs with costs of calculating the marginals

parse_incsccosts.sh parses SLAM++ logs and outputs .cvs with costs of calculating the incremental
	Schur complement

remove_cam_settle_markers.sh parses a graph file and removes CONSISTENCY_MARKER markes
	that follow a camera vertex in case no new landmark vertices were inserted since the last
	camera (or since the beginning)

get_observation_xy_ranges.sh parses a (SE3) graph and calculates min / max of the observations

make_frankengraph.sh was used for debugging by splicing two graph files together (one with
	absolute and one with relative landmarks); this helped to diagnose source of problems
	in the frontend
