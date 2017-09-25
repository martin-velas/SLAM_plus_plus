/*
								+-----------------------------------+
								|                                   |
								| *** Compact pose SLAM example *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|             Main.cpp              |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam_compact_pose_example/Main.cpp
 *	@brief data association / compact pose SLAM example
 *	@date 2015
 *	@author -tHE SWINe-
 */

#include <stdio.h> // printf
#include "slam/LinearSolver_UberBlock.h" // linear solver
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/SE2_Types.h" // SE(2) types
#include "slam/Marginals.h" // covariance calculation
#include "slam/Distances.h" // distance calculation
#include "eigen/Eigen/SVD" // JacobiSVD

/**
 *	@brief parsed edge data
 */
struct TEdgeData {
	size_t p_vertex[2]; /**< @brief indices of the vertices */
	Eigen::Vector3d v_measurement; /**< @brief measurement vector */
	Eigen::Matrix3d t_information; /**< @brief information matrix */

	/**
	 *	@brief default constructor; has no effect
	 */
	TEdgeData()
	{}

	/**
	 *	@brief constructor; initializes the edge data
	 *
	 *	@param[in] n_vertex_0 is index of the first vertex
	 *	@param[in] n_vertex_1 is index of the second vertex
	 *	@param[in] r_v_measurement is measurement vector
	 *	@param[in] r_t_information is information matrix
	 */
	TEdgeData(size_t n_vertex_0, size_t n_vertex_1,
		const Eigen::Vector3d &r_v_measurement, const Eigen::Matrix3d &r_t_information)
		:v_measurement(r_v_measurement), t_information(r_t_information)
	{
		p_vertex[0] = n_vertex_0;
		p_vertex[1] = n_vertex_1;
	}

	/**
	 *	@brief comparison operator
	 *	@param[in] r_edge is an edge
	 *	@return Returns true if this edge connects the same two vertices
	 *		as r_edge does (in any order), otherwise returns false.
	 */
	bool operator ==(const std::pair<size_t, size_t> &r_edge) const
	{
		return (p_vertex[0] == r_edge.first && p_vertex[1] == r_edge.second) ||
			   (p_vertex[1] == r_edge.first && p_vertex[0] == r_edge.second);
	}
};

/**
 *	@brief a simple predicate that sorts the odometry edges first
 */
class COdometryFirst {
public:
	/**
	 *	@brief less-than comparison for edges
	 *
	 *	@param[in] r_edge_a is an edge
	 *	@param[in] r_edge_b is an edge
	 *
	 *	@return Returns true if r_edge_a is ordered before r_edge_b, otherwise returns false.
	 */
	bool operator ()(const TEdgeData &r_edge_a, const TEdgeData &r_edge_b) const
	{
		bool b_a_is_odometry = r_edge_a.p_vertex[0] + 1 == r_edge_a.p_vertex[1];
		bool b_b_is_odometry = r_edge_b.p_vertex[0] + 1 == r_edge_b.p_vertex[1];
		return b_a_is_odometry > b_b_is_odometry; // if a is odometry edge and b is not, then a is ordered before b
	}
};

/**
 *	@brief a helper function that adds a bunch of edges
 *	@param[out] edges is vector of edges to add the edges to
 *	@note The data are taken from Olson's Manhattan dataset.
 */
static void Add_ManhattanEdges(std::vector<TEdgeData> &edges);

/**
 *	@brief a helper function that loads a 2D dataset (2D to make the data structures involved in this example simple)
 *
 *	@param[in] p_s_filename is null-terminated string, containing file name of a dataset
 *	@param[out] edges is vector of edges to add the edges to
 */
static void Load_Dataset(const char *p_s_filename, std::vector<TEdgeData> &edges);

/**
 *	@brief information gain and distance for a porential loop closure
 */
struct TInfoGain_Distance {
	double f_gain; /**< @brief information gain of the loop */
	double f_distance; /**< @brief distance probability of the vertices */
	//Eigen::Matrix3d t_sigma_d; /**< @brief distance covariance */
	size_t n_vertex; /**< @brief id of the vertex we are potentially closing loop with */

	/**
	 *	@brief default constructor
	 *
	 *	@param[in] _f_gain is information gain of the loop
	 *	@param[in] _f_distance is distance probability of the vertices
	 *	@param[in] _n_vertex is id of the vertex we are potentially closing loop with
	 */
	// *	@param[in] r_t_sigma_d is distance covariance
	TInfoGain_Distance(double _f_gain = 0, double _f_distance = 0,
		/*const Eigen::Matrix3d &r_t_sigma_d = Eigen::Matrix3d(),*/ size_t _n_vertex = size_t(-1))
		:f_gain(_f_gain), f_distance(_f_distance), /*t_sigma_d(r_t_sigma_d),*/ n_vertex(_n_vertex)
	{}

	/**
	 *	@brief less-than comparison operator for sorting
	 *	@param[in] r_other is the other information gain to compare with
	 *	@return Returns true if this should precede r_other in the sequence, otherwise returns false.
	 */
	inline bool operator <(const TInfoGain_Distance &r_other) const
	{
		if(_isnan(f_gain) || _isnan(r_other.f_gain))
			throw std::runtime_error("NaN gains occured"); // can't sort NaNs, they do not compare, which breaks most sotting algorithms
		return f_gain > r_other.f_gain; // sort by information gain in descending order
	}
};

/**
 *	@brief main
 *
 *	@param[in] n_arg_num is number of commandline arguments
 *	@param[in] p_arg_list is the list of commandline arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 */
int main(int n_arg_num, const char **p_arg_list)
{
	typedef MakeTypelist(CVertexPose2D) TVertexTypelist;
	typedef MakeTypelist(CEdgePose2D) TEdgeTypelist;

	typedef CFlatSystem<CVertexPose2D, TVertexTypelist, CEdgePose2D,
		TEdgeTypelist, CProportionalUnaryFactorFactory> CSystemType;

	typedef CLinearSolver_UberBlock<CSystemType::_TyHessianMatrixBlockList> CLinearSolverType;

	//const Eigen::Vector2d v_min(-12, -9), v_max(8.5, 9); // accomodates the first 100 poses of molson
	//const Eigen::Vector2d v_min(-30, -20), v_max(8.5, 9); // accomodates slightly more of molson
	const Eigen::Vector2d v_min(-12, -24), v_max(18, 6); // accomodates intel
	const int n_resolution = 720;
	// image parameters

	TBmp *p_image;
	Eigen::Vector2d v_off, v_scale;
	{
		Eigen::Vector2d v_size = v_max - v_min;
		double f_short_side = std::min(v_size(0), v_size(1));
		double f_scale = n_resolution / f_short_side;
		int n_width = std::max(1, int(v_size(0) * f_scale));
		int n_height = std::max(1, int(v_size(1) * f_scale));
		// calculate image size

		v_off = -v_min;
		v_scale.setConstant(f_scale);
		// set offset and scale

		v_scale(1) = -f_scale;
		v_off(1) += n_height / v_scale(1);
		// the y axis is flipped, need to modify offset and scale

		if(!(p_image = TBmp::p_Alloc(n_width, n_height)))
			return -1;
		// alloc image
	}
	// calculate image resolution and transformation to raster coordinates

	Eigen::Vector3d v_sensor_range;
	//v_sensor_range << 3, 3, 2 * M_PI; // manhattan (has some long loop closures)
	v_sensor_range << 3, 2, 2 * M_PI; // intel
	// sensor range (a tolerance, affects which poses will be matched)

	Eigen::Matrix3d t_Sigma_bar = Eigen::Matrix3d::Identity();
	// sigma bar (sample from the dataset or from the first edge or take a guess)

	double f_loop_probability_dist_threshold = 0.15;//0.1578;//0.1; // as in TRO
	// poses with higher loop probability will be candidates for sensor matching

	double f_keep_pose_gain_threshold = 0.1;//0.1;//0.25 // experimentally set
	double f_loop_closure_gain_threshold = f_keep_pose_gain_threshold; // initially the same but can be set differently if needed
	// minimal information gain to not extend an edge

	std::vector<TEdgeData> edges;
	if(n_arg_num == 2)
		Load_Dataset(p_arg_list[1], edges);
	else
		Add_ManhattanEdges(edges); // to avoid making this too long
	std::stable_sort(edges.begin(), edges.end(), COdometryFirst()); // sort odometry edges first
	// put all dataset edges in a list

	std::vector<TEdgeData> loop_closures;
	// keep a list of loop closures, so we can add them later,
	// simulating loop closing using a real sensor

	for(size_t i = 0, n = edges.size(), n_vertex_num = 0; i < n; ++ i) {
		const TEdgeData &r_edge = edges[i];
		if(r_edge.p_vertex[0] < n_vertex_num &&
		   r_edge.p_vertex[1] < n_vertex_num) {
			// both edges are already in the system, this is a loop closure

			loop_closures.push_back(r_edge);
			// put it to the list of loop closures

			edges.erase(edges.begin() + i);
			-- i;
			-- n;
			continue;
			// remove it from the list of odometry edges
		}
		// check loop closures

		n_vertex_num = std::max(n_vertex_num, std::max(r_edge.p_vertex[0], r_edge.p_vertex[1]) + 1);
		// simulate adding vertices to the system
	}
	// filter our the loop closures from the entire dataset (forming a greedy spanning tree)
	// note that the loop closures could also be filtered out in the processing loop
	// but the fake "sensor matching" code would have to be at the beginning of the loop,
	// which seemed counterintuitive for a code example

	const size_t n_loop_closure_num = loop_closures.size(),
		n_total_edge_num = loop_closures.size() + edges.size();
	// total for the whole dataset

	const size_t n_pose_num = (edges.empty())? 0 :
		std::max(edges.back().p_vertex[0], edges.back().p_vertex[1]) + 1;
	_ASSERTE(n_pose_num == edges.size() + 1); // should match
	// the total number of poses in the graph (edges contain the odometry edges now)

	size_t n_closed_loop_num = 0, n_distance_probability_match_num = 0,
		n_association_num = 0, n_low_gain_loop_num = 0;;
	// keep a counter of how many times sensor matching was attempted

	double f_min_probability = 1e100, f_min_loop_gain = 1e100, f_min_pose_gain = 1e100;
	// what was the minimum probability that closed the loop (a statistic, not a threshold)

	size_t n_discarded_loop_num = 0;

	CTimer t;
	CTimerSampler timer(t);

	double f_solver_time = 0; // NLS solving
	double f_distances_time = 0; // calculating the candidates
	double f_drawing_time = 0; // saving images and other outputs
	// timing stats

	// the compact pose SLAM algorithm starts here

	CSystemType system;
	EBlockMatrixPart n_marginal_selection = EBlockMatrixPart(mpart_LastColumn | mpart_Diagonal);
	CNonlinearSolver_FastL<CSystemType, CLinearSolverType> solver(system,
		solve::Nonlinear(frequency::Every(1), 5, .01),
		TMarginalsComputationPolicy(marginals::do_calculate,
		frequency::Every(1), n_marginal_selection, n_marginal_selection));
	// set solver to calculate marginals, which are needed to calculate the distances

	bool b_keep_pose = true;
	// compact pose SLAM edge replace flag

	Eigen::Vector3d v_cum_measurement;
	Eigen::Matrix3d t_cum_information; // value does not matter, will be overwritten
	// keep a cumulative edge

	for(size_t i = 0, n = edges.size(); i < n; ++ i) {
		printf("\rstep " PRIsize " ", i);
		timer.Accum_DiffSample(f_drawing_time); // in windows, the console output actually takes a lot of time

		const TEdgeData &r_edge = edges[i];
		// get an edge

		if(b_keep_pose) {
			t_cum_information = r_edge.t_information;
			v_cum_measurement = r_edge.v_measurement;
			// the first step is simple
		} else {
			Eigen::Matrix3d J0, J1;
			C2DJacobians::Relative_to_Absolute(v_cum_measurement, r_edge.v_measurement, v_cum_measurement, J0, J1);
			// accumulate measurements and get jacobians

			Eigen::Matrix3d cov = J0 * t_cum_information.inverse() * J0.transpose() +
				J1 * r_edge.t_information.inverse() * J1.transpose();
			t_cum_information = cov.inverse();
		}
		// accumulate edge information

		if(b_keep_pose) {
			solver.Incremental_Step(system.r_Add_Edge(
				CEdgePose2D(r_edge.p_vertex[0], r_edge.p_vertex[1],
				v_cum_measurement, t_cum_information, system)));
		} else {
			_ASSERTE(!system.r_Edge_Pool().b_Empty() && !system.r_Vertex_Pool().b_Empty());
			CEdgePose2D &r_last_edge = system.r_Edge_Pool().r_At<CEdgePose2D>(system.r_Edge_Pool().n_Size() - 1);
			// get the last edge

			Eigen::Map<Eigen::VectorXd> v_vertex0 = system.r_Vertex_Pool()[r_last_edge.n_Vertex_Id(0)].v_State();
			// get the last vertex

			Eigen::Vector3d v_vertex1;
			C2DJacobians::Relative_to_Absolute(v_vertex0, v_cum_measurement, v_vertex1);
			// calculate where the second vertex ends up, given the accumulated measurement

			solver.Change_LastEdge(r_last_edge, v_cum_measurement, t_cum_information, v_vertex1);
			// let the solver update the system
		}
		// this is a spanning tree edge, add it to the system and update

		const CUberBlockMatrix &r_marginals = solver.r_MarginalCovariance().r_SparseMatrix();
		// get the marginals

		_ASSERTE(r_marginals.n_BlockColumn_Num() == system.r_Vertex_Pool().n_Size());
		Eigen::MatrixXd last_sigma = r_marginals.t_GetBlock_Log(
			r_marginals.n_BlockRow_Num() - 1, r_marginals.n_BlockColumn_Num() - 1);
		// get the last block of the marginals, corresponding to the current pose

		timer.Accum_DiffSample(f_solver_time);

		bool b_low_gain_edge = true;
		// see if the edge presents sufficient information gain

		std::vector<size_t> probable_matches, true_matches; // list of probable matches with the last vertex
		{
			Eigen::VectorXd v_threshold(3); // corresponds to CEdgePose2D
			v_threshold = v_sensor_range; // sensor precision (field of view like)
			const Eigen::VectorXd *p_thresh_list[] = {&v_threshold};
			const size_t n_thresh_num = sizeof(p_thresh_list) / sizeof(p_thresh_list[0]);
			// need a list of distance thresholds, one per each edge in TEdgeTypelist

			CDistances<CSystemType/*, TEdgeTypelist*/> distances(system, // the second template argument can be inferred from CSystemType
				solver.r_MarginalCovariance().r_SparseMatrix(), p_thresh_list, n_thresh_num);
			// initialize an object to calculate distances

			const CSystemType::_TyVertexMultiPool &r_vertex_pool = system.r_Vertex_Pool();
			size_t n_last_vertex = r_vertex_pool.n_Size() - 1;
				// id of the last vertex

			std::vector<TInfoGain_Distance> gain_distance_list(r_vertex_pool.n_Size() - 1);
			// allocate a list of information gains and distances

			_ASSERTE(n_last_vertex <= INT_MAX);
			int _n_last_vertex = int(n_last_vertex);
			#pragma omp parallel for schedule(dynamic, 32) if(n_last_vertex > 64)
			for(int j = 0; j < _n_last_vertex; ++ j) {
				std::pair<Eigen::VectorXd, Eigen::MatrixXd> dist_sig;
				distances.Calculate_Distance_Sigma_d(dist_sig.first, dist_sig.second, j, n_last_vertex); // compact pose needs sigma d as well
				Eigen::VectorXd v_probability_distance = dist_sig.first;
				double f_min_probability_distance = v_probability_distance.minCoeff(); // minimal of all probabilities
				// calculate probability that the vertices are closer than the specified distance threshold
				// calculates distance between vertices j and n_last_vertex

				double f_info_gain = distances.f_Information_Gain(t_Sigma_bar, dist_sig.second);
				if(_isnan(f_info_gain))
					f_info_gain = -1e100;
				// calculate information gain of the edge

				gain_distance_list[j] = TInfoGain_Distance(f_info_gain,
					f_min_probability_distance, j);
			}
			// calculate a list of information gains and distances

			std::sort(gain_distance_list.begin(), gain_distance_list.end());
			// sort the list by information gain

			for(size_t j = 0; j < n_last_vertex; ++ j) {
				size_t n_loop_vertex = gain_distance_list[j].n_vertex;
				double f_min_probability_distance = gain_distance_list[j].f_distance;
				double f_gain_prior = gain_distance_list[j].f_gain;
				// read the id / distance pair

				if(f_min_probability_distance >= f_loop_probability_dist_threshold)
					++ n_distance_probability_match_num;
				// count those as well

				if(f_min_probability_distance > f_loop_probability_dist_threshold &&
				   f_gain_prior >= f_loop_closure_gain_threshold) {
					++ n_association_num;
					// will attempt sensor matching on poses (n_loop_vertex, n_last_vertex)

					Eigen::Matrix3d t_Sigma_d;
					distances.Calculate_Sigma_d(t_Sigma_d, n_loop_vertex, n_last_vertex); // compact pose needs sigma d as well
					// the second calculation of distance to get the 6x6 covariance matrix (the distance itself not needed)

					bool b_matched = false;
					std::vector<TEdgeData>::iterator p_edge_it;
					while((p_edge_it = std::find(loop_closures.begin(), loop_closures.end(),
					   std::make_pair(n_loop_vertex, n_last_vertex))) != loop_closures.end()) {
						const TEdgeData &r_edge = *p_edge_it;
						// found an edge between vertices n_loop_vertex and n_last_vertex

						Eigen::Matrix3d t_Sigma_e = r_edge.t_information.inverse();
						double f_info_gain = distances.f_Information_Gain(t_Sigma_e, t_Sigma_d);
						if(_isnan(f_info_gain))
							f_info_gain = -1e100;
						// calculate the real information gain, based on the sensor matching

						if(f_info_gain > f_loop_closure_gain_threshold) {
							timer.Accum_DiffSample(f_distances_time);

							if(f_min_loop_gain > f_info_gain && f_info_gain != -1e100)
								f_min_loop_gain = f_info_gain;
							// remember which gain the loops had

							if(!b_matched)
								solver.Delay_Optimization();
							// if adding multiple loops (but between the same two vertices - e.g. from
							// multiple sensors) we dont need to optimize after each. just optimize
							// quite at the end

							++ n_closed_loop_num;
							solver.Incremental_Step(system.r_Add_Edge(
								CEdgePose2D(r_edge.p_vertex[0], r_edge.p_vertex[1],
								r_edge.v_measurement, r_edge.t_information, system)));
							// add it to the system, recalculate marginal covariances

							timer.Accum_DiffSample(f_solver_time);

							b_matched = true;
						} else
							++ n_low_gain_loop_num;
						// in case the loop closure is informative enough, add it to the system

						loop_closures.erase(p_edge_it);
						// remove it from the loop closure list (if matched or if not - this pose will not come back)
					}
					// perform fake sensor matching on poses (n_loop_vertex, n_last_vertex)

					if(b_matched) {
						if(f_min_probability > f_min_probability_distance)
							f_min_probability = f_min_probability_distance;
						// see what the threshold needs to be

						true_matches.push_back(n_loop_vertex);
						// remember matches

						timer.Accum_DiffSample(f_distances_time);

						solver.Enable_Optimization();
						// if we added any edges, optimize now

						solver.Optimize(0, 0);
						// force update of the marginals (no new vertices added so the solver
						// wouldnt update them by itself)

						timer.Accum_DiffSample(f_solver_time);

						#pragma omp parallel for schedule(dynamic, 32) if(n_last_vertex - (j + 1) > 64)
						for(int k = int(j + 1); k < _n_last_vertex; ++ k) {
							size_t n_vertex_id = gain_distance_list[k].n_vertex;
							// see which vertex is remaining in the list

							Eigen::Vector3d v_distance;
							Eigen::Matrix3d t_Sigma_d;
							distances.Calculate_Distance_Sigma_d(v_distance, t_Sigma_d, n_vertex_id, n_last_vertex); // compact pose needs sigma d as well
							double f_min_probability_distance = v_distance.minCoeff(); // minimal of all probabilities
							// calculate probability that the vertices are closer than the specified distance threshold
							// calculates distance between vertices k and n_last_vertex

							double f_info_gain = distances.f_Information_Gain(t_Sigma_bar, t_Sigma_d);
							if(_isnan(f_info_gain)) // sometimes the sigma_d has slightly negative determinant
								f_info_gain = -1e100;
							// calculate information gain of the edge

							gain_distance_list[k] = TInfoGain_Distance(f_info_gain,
								f_min_probability_distance, n_vertex_id);
							// update the information gain and distance of that vertex
						}
						// calculate a list of information gains and distances for the remaining vertices

						std::sort(gain_distance_list.begin() + (j + 1), gain_distance_list.end());
						// sort the suffix of the list by information gain
					} else {
						probable_matches.push_back(n_loop_vertex);
						// remember the probable matches (for drawing the animation)
					}
					// handle successful loop closures
				} else
					++ n_low_gain_loop_num;
			}
			// close the loops

			if(!gain_distance_list.empty()) {
				double f_min_gain = gain_distance_list.back().f_gain;

				if(f_min_pose_gain > f_min_gain && f_min_gain != -1e100)
					f_min_pose_gain = f_min_gain;

				if(f_min_gain >= f_keep_pose_gain_threshold) {
					b_low_gain_edge = false;
					// if any of the loop closures is under low gain then this edge is low gain
					// (the algorithm in the paper actually takes a minimum of the gains of all
					// the proposed loop closures and compares that to the threshold, which is
					// equivalent to doing this)
				}
				// compare the information gain
			}
			// see if the smallest information gain was too small
		}
		// find probable matches using the covariances

		bool b_closed_loops = !true_matches.empty();
		// see if we closed any loops

		b_keep_pose = b_closed_loops || !b_low_gain_edge;
		// decide whether to keep the pose

		timer.Accum_DiffSample(f_distances_time);

		if(!b_keep_pose) {
			// this pose did not close any loops and does not have enough information
			// gain. we will not keep it and instead we will extend the edge a bit more.

			n_discarded_loop_num += loop_closures.size();
			size_t n_remove_vertex = system.r_Vertex_Pool().n_Size() - 1; // this one will be left out
			{
				size_t n_last_vertex = system.r_Vertex_Pool().n_Size() - 1; // this is in the system
				for(size_t j = i + 1, m = edges.size(); j < m; ++ j) { // dont bother with edges we already processed
					TEdgeData &r_edge = edges[j];
					if(r_edge.p_vertex[0] >= n_remove_vertex)
						-- r_edge.p_vertex[0];
					if(r_edge.p_vertex[1] >= n_remove_vertex)
						-- r_edge.p_vertex[1];
					if(r_edge.p_vertex[0] == r_edge.p_vertex[1]) {
						edges.erase(edges.begin() + j);
						-- m;
						-- j; // !!
						// the edge somehow degenerated, remove it
					}
				}
				n = edges.size(); // adjust the outer loop limit, if needed
				// update odometry vertex indices

				if(i + 1 < n) {
					const TEdgeData &r_next_edge = edges[i + 1];
					size_t n_vertex0 = r_next_edge.p_vertex[0];
					size_t n_vertex1 = r_next_edge.p_vertex[1];
					if(n_vertex0 > n_vertex1)
						std::swap(n_vertex0, n_vertex1);
					// get vertices of the next edge

					if(n_vertex0 + 1 != n_vertex1 || n_vertex1 != n_last_vertex) {
						fprintf(stderr, "error: the graph does not contain good odometry edges, can't continue\n");
						return -1;
					}
				}
				// make sure that the next edge is on the last two vertices, so that we can
				// do edge replace (if the graph contains simple 0->1, 1->2, ... (n-1)->n
				// odometry then this will work, otherwise problems may arise)

				for(size_t j = 0, m = loop_closures.size(); j < m; ++ j) {
					TEdgeData &r_edge = loop_closures[j];
					if(r_edge.p_vertex[0] == n_remove_vertex ||
					   r_edge.p_vertex[1] == n_remove_vertex) {
						++ n_discarded_loop_num;
						loop_closures.erase(loop_closures.begin() + j);
						-- m;
						-- j; // !!
						continue;
					}
					// if the edge closed a loop with the vertex we are about to remove, remove it - we lost this loop closure

					if(r_edge.p_vertex[0] > n_remove_vertex)
						-- r_edge.p_vertex[0];
					if(r_edge.p_vertex[1] > n_remove_vertex)
						-- r_edge.p_vertex[1];
					// modify the indices

					if(r_edge.p_vertex[0] == r_edge.p_vertex[1]) {
						++ n_discarded_loop_num;
						loop_closures.erase(loop_closures.begin() + j);
						-- m;
						-- j; // !!
						// the edge somehow degenerated, remove it
					}
				}
				// update the loop closure vertex indices
			}
			n_discarded_loop_num -= loop_closures.size(); // track the number of removed loop closures
			n = edges.size(); // adjust the outer loop limit, it may have changed!
			// this actually affects indexing of the graph: the next vertex is not added
			// and so we need to modify the vertex indices. if this was an online scenario
			// we wouldnt have to worry about this bit, since we would calculate the
			// indices on the fly.
		}
		// do compact pose SLAM

		timer.Accum_DiffSample(f_distances_time);

		if(1) {
			p_image->Clear(-1);
			// clear to white

			const CSystemType::_TyEdgeMultiPool &r_edge_pool = system.r_Edge_Pool();
			const CSystemType::_TyVertexMultiPool &r_vertex_pool = system.r_Vertex_Pool();
			for(size_t j = 0, m = r_vertex_pool.n_Size(); j < m; ++ j) {
				Eigen::Map<const Eigen::VectorXd> t_vert = r_vertex_pool[j].v_State();
				// get a vertex

				float f_x = float((t_vert(0) + v_off(0)) * v_scale(0));
				float f_y = float((t_vert(1) + v_off(1)) * v_scale(1));
				// transform vertex position to raster coordinates

				const uint32_t n_red = 0xffff0000U;
				p_image->DrawLine_SP(f_x - 10, f_y, f_x + 10, f_y, n_red, 2);
				p_image->DrawLine_SP(f_x, f_y - 10, f_x, f_y + 10, n_red, 2);
				// draw a red cross under the vertex
			}
			for(size_t j = 0, m = r_edge_pool.n_Size(); j < m; ++ j) {
				CSystemType::_TyEdgeMultiPool::_TyConstBaseRef r_edge = r_edge_pool[j];
				Eigen::Map<const Eigen::VectorXd> t_vert0 = r_vertex_pool[r_edge.n_Vertex_Id(0)].v_State();
				Eigen::Map<const Eigen::VectorXd> t_vert1 = r_vertex_pool[r_edge.n_Vertex_Id(1)].v_State();
				// get the edge and the vertices

				float f_x0 = float((t_vert0(0) + v_off(0)) * v_scale(0));
				float f_y0 = float((t_vert0(1) + v_off(1)) * v_scale(1));
				float f_x1 = float((t_vert1(0) + v_off(0)) * v_scale(0));
				float f_y1 = float((t_vert1(1) + v_off(1)) * v_scale(1));
				// transform vertex positions to raster coordinates

				const uint32_t n_blue = 0xff0000ffU;
				p_image->DrawLine_AA(f_x0, f_y0, f_x1, f_y1, n_blue, 5);
				// draw edge
			}
			{
				Eigen::Map<const Eigen::VectorXd> t_vert0 = r_vertex_pool[r_vertex_pool.n_Size() - 1].v_State();
				// get the last vertex

				float f_x0 = float((t_vert0(0) + v_off(0)) * v_scale(0));
				float f_y0 = float((t_vert0(1) + v_off(1)) * v_scale(1));
				// transform vertex positions to raster coordinates

				for(size_t j = 0, m = probable_matches.size(); j < m; ++ j) {
					Eigen::Map<const Eigen::VectorXd> t_vert1 = r_vertex_pool[probable_matches[j]].v_State();
					float f_x1 = float((t_vert1(0) + v_off(0)) * v_scale(0));
					float f_y1 = float((t_vert1(1) + v_off(1)) * v_scale(1));

					const uint32_t n_light_gray = 0xffddddddU;
					p_image->DrawLine_AA(f_x0, f_y0, f_x1, f_y1, n_light_gray, 1);
					// draw edge
				}
				// draw the probable matches

				for(size_t j = 0, m = true_matches.size(); j < m; ++ j) {
					Eigen::Map<const Eigen::VectorXd> t_vert1 = r_vertex_pool[true_matches[j]].v_State();
					float f_x1 = float((t_vert1(0) + v_off(0)) * v_scale(0));
					float f_y1 = float((t_vert1(1) + v_off(1)) * v_scale(1));

					const uint32_t n_orange = 0xffffa500u;
					p_image->DrawLine_AA(f_x0, f_y0, f_x1, f_y1, n_orange, 5);
					// draw edge
				}
				// draw the true matches
			}
			{
				Eigen::Map<const Eigen::VectorXd> t_vert = r_vertex_pool[r_vertex_pool.n_Size() - 1].v_State();
				// get the last vertex

				Eigen::Vector2d v_right, v_up;
				{
					//Eigen::LLT<Eigen::Matrix2d, Eigen::Upper> chol_sigma(last_sigma.topLeftCorner<2, 2>());
					//Eigen::Matrix2d R = chol_sigma.matrixU(); // does not quite work, but gets close
					Eigen::JacobiSVD<Eigen::Matrix2d, Eigen::NoQRPreconditioner>
						svd_sigma(last_sigma.topLeftCorner<2, 2>(), Eigen::ComputeFullU/*| Eigen::ComputeFullV*/);
					Eigen::Matrix2d R = svd_sigma.matrixU() * svd_sigma.singularValues().array().sqrt().matrix().asDiagonal();
					// calculate a slightly different RRT decomposition using SVD

					//double f_decomp_err = (R * R.transpose() - last_sigma.topLeftCorner<2, 2>()).norm();
					//printf("%g\n", f_decomp_err); // debug - should be zero

					v_right = R.col(0) * 2.1459;
					v_up = R.col(1) * 2.1459;
					// (90% confidence level is obtained at 2.1459)
				}
				// calculate basis for an ellipse

				Eigen::Vector2d v_prev;
				for(int i = 0; i < 101; ++ i) {
					double f_angle = (i % 100) * .01 * 2 * M_PI;
					Eigen::Vector2d v_pos = t_vert.head<2>() + cos(f_angle) * v_right + sin(f_angle) * v_up;
					// calculate point on an ellipse

					v_pos = ((v_pos + v_off).array() * v_scale.array()).matrix();
					// transform to raster coordinates

					if(i) {
						const uint32_t n_gray = 0xff808080U;
						p_image->DrawLine_AA(float(v_prev(0)), float(v_prev(1)),
							float(v_pos(0)), float(v_pos(1)), n_gray, 1);
					}
					v_prev = v_pos;
				}
				// draw confidence interval around the current robot pose
			}

			char p_s_filename[256];
			sprintf(p_s_filename, "anim/frame_%04" _PRIsize ".tga", i);
			CTgaCodec::Save_TGA(p_s_filename, *p_image, false);
			// save as a bitmap
		}

		timer.Accum_DiffSample(f_drawing_time);
	}
	// put edges in the system, the vertices are created and initialized

	size_t n_discarded_pose_num = n_pose_num - system.r_Vertex_Pool().n_Size();
	// keep a counter of how many poses the compact pose slam discarded and how many loops we lost to it (statistics)

	printf("\ndone. the dataset has " PRIsize " vertices and " PRIsize " edges, out of which "
		PRIsize " are loop closures\n", n_pose_num, n_total_edge_num, n_loop_closure_num);
	printf("correctly matched " PRIsize " / " PRIsize " loop closures (tried "
		PRIsize " times)\n", n_closed_loop_num, n_loop_closure_num, n_association_num);
	printf("the minimal successful probability that closed a loop was %g\n", f_min_probability);
	printf("the minimal loop closure information gain was %g\n", f_min_loop_gain);
	printf("the minimal pose information gain was %g\n", f_min_pose_gain);
	printf("the compact pose SLAM algorithm discarded " PRIsize " / " PRIsize " poses (lost " PRIsize
		" loop closures to that, lost " PRIsize " loops to gain test)\n", n_discarded_pose_num,
		n_pose_num, n_discarded_loop_num, n_low_gain_loop_num);
	printf("solver took %f sec\n", f_solver_time);
	printf("distance calculation and pose matching took %f sec\n", f_distances_time);
	printf("animation rasterization %f sec\n", f_drawing_time);
	// print how many loop closures were missed (should be 0)

	p_image->Delete();
	// delete the bitmap

	system.Plot2D("result.tga", plot_quality::plot_Printing); // plot in print quality
	//solver.Dump(f_solver_time); // show some stats
	//solver.Dump_SystemMatrix("lambda.tga"); // debug

	return 0;
}

/**
 *	@brief adds edges from Olson's Manhattan 3500 dataset
 */
void Add_ManhattanEdges(std::vector<TEdgeData> &edges)
{
	Eigen::Matrix3d information;
	information <<
		45,  0,  0,
		 0, 45,  0,
		 0,  0, 45; // manhattan
	/*information <<
		22.36,	   0,	  0,
			0, 22.36,	  0,
			0,	   0, 70.71;*/ // intel
	// prepare the information matrix (all edges have the same)

	edges.push_back(TEdgeData(0, 1, Eigen::Vector3d(1.03039, 0.0113498, -0.0129577), information));
	edges.push_back(TEdgeData(1, 2, Eigen::Vector3d(1.0139, -0.0586393, -0.013225), information));
	edges.push_back(TEdgeData(2, 3, Eigen::Vector3d(1.02765, -0.00745597, 0.00483283), information));
	edges.push_back(TEdgeData(3, 4, Eigen::Vector3d(-0.0120155, 1.00436, 1.56679), information));
	edges.push_back(TEdgeData(4, 5, Eigen::Vector3d(1.01603, 0.0145648, -0.0163041), information));
	edges.push_back(TEdgeData(5, 6, Eigen::Vector3d(1.02389, 0.00680757, 0.0109813), information));
	edges.push_back(TEdgeData(6, 7, Eigen::Vector3d(0.957734, 0.00315932, 0.0109005), information));
	edges.push_back(TEdgeData(7, 8, Eigen::Vector3d(-1.02382, -0.0136683, -3.09324), information));
	edges.push_back(TEdgeData(5, 9, Eigen::Vector3d(0.0339432, 0.0324387, -3.1274), information));
	edges.push_back(TEdgeData(8, 9, Eigen::Vector3d(1.02344, 0.0139844, -0.00780158), information));
	edges.push_back(TEdgeData(3, 10, Eigen::Vector3d(0.0440195, 0.988477, -1.56353), information));
	edges.push_back(TEdgeData(9, 10, Eigen::Vector3d(1.00335, 0.0222496, 0.0234909), information));
	edges.push_back(TEdgeData(10, 11, Eigen::Vector3d(0.977245, 0.0190423, -0.0286232), information));
	edges.push_back(TEdgeData(11, 12, Eigen::Vector3d(-0.99688, -0.0255117, 3.15627), information));
	edges.push_back(TEdgeData(12, 13, Eigen::Vector3d(0.990646, 0.0183964, -0.0165195), information));
	edges.push_back(TEdgeData(8, 14, Eigen::Vector3d(0.0158085, 0.0210588, 3.12831), information));
	edges.push_back(TEdgeData(13, 14, Eigen::Vector3d(0.945873, 0.00889308, -0.00260169), information));
	edges.push_back(TEdgeData(7, 15, Eigen::Vector3d(-0.0147277, -0.00159495, -0.0195786), information));
	edges.push_back(TEdgeData(14, 15, Eigen::Vector3d(1.00001, 0.00642824, 0.0282342), information));
	edges.push_back(TEdgeData(15, 16, Eigen::Vector3d(0.0378719, -1.02609, -1.5353), information));
	edges.push_back(TEdgeData(16, 17, Eigen::Vector3d(0.98379, 0.019891, 0.0240848), information));
	edges.push_back(TEdgeData(17, 18, Eigen::Vector3d(0.957199, 0.0295867, -0.0115004), information));
	edges.push_back(TEdgeData(18, 19, Eigen::Vector3d(0.99214, 0.0192015, -0.00729783), information));
	edges.push_back(TEdgeData(19, 20, Eigen::Vector3d(-0.0459215, -1.01632, -1.53912), information));
	edges.push_back(TEdgeData(20, 21, Eigen::Vector3d(0.99845, -0.00523202, -0.0340973), information));
	edges.push_back(TEdgeData(21, 22, Eigen::Vector3d(0.988728, 0.00903381, -0.0129141), information));
	edges.push_back(TEdgeData(22, 23, Eigen::Vector3d(0.989422, 0.00698231, -0.0242835), information));
	edges.push_back(TEdgeData(23, 24, Eigen::Vector3d(-1.00201, -0.00626341, 3.13974), information));
	edges.push_back(TEdgeData(24, 25, Eigen::Vector3d(1.01535, 0.00491314, 3.02279e-05), information));
	edges.push_back(TEdgeData(21, 26, Eigen::Vector3d(-0.95214, -0.0418463, 3.13475), information));
	edges.push_back(TEdgeData(25, 26, Eigen::Vector3d(1.03299, -0.00172652, 0.0224073), information));
	edges.push_back(TEdgeData(19, 27, Eigen::Vector3d(-0.0176158, -0.0052181, 1.56791), information));
	edges.push_back(TEdgeData(26, 27, Eigen::Vector3d(0.989137, -0.00857052, -0.0209045), information));
	edges.push_back(TEdgeData(27, 28, Eigen::Vector3d(-0.0483998, 0.981715, 1.56408), information));
	edges.push_back(TEdgeData(28, 29, Eigen::Vector3d(1.03082, -0.021271, -0.06069), information));
	edges.push_back(TEdgeData(29, 30, Eigen::Vector3d(1.01192, 0.0164477, -0.0352014), information));
	edges.push_back(TEdgeData(30, 31, Eigen::Vector3d(0.991338, 0.00781231, 0.0305919), information));
	edges.push_back(TEdgeData(31, 32, Eigen::Vector3d(0.00861057, -0.974025, -1.56961), information));
	edges.push_back(TEdgeData(32, 33, Eigen::Vector3d(1.04256, 0.0106692, 0.0220136), information));
	edges.push_back(TEdgeData(33, 34, Eigen::Vector3d(0.990826, 0.0166949, -0.0427845), information));
	edges.push_back(TEdgeData(34, 35, Eigen::Vector3d(0.995988, 0.029526, -0.0144112), information));
	edges.push_back(TEdgeData(35, 36, Eigen::Vector3d(-0.0107743, 0.996051, 1.59416), information));
	edges.push_back(TEdgeData(36, 37, Eigen::Vector3d(1.00499, 0.0110863, -0.00316511), information));
	edges.push_back(TEdgeData(37, 38, Eigen::Vector3d(1.03843, 0.0146778, -0.0323211), information));
	edges.push_back(TEdgeData(38, 39, Eigen::Vector3d(1.00625, 0.00674436, -0.0280641), information));
	edges.push_back(TEdgeData(39, 40, Eigen::Vector3d(0.0561635, 0.984988, -4.70377), information));
	edges.push_back(TEdgeData(40, 41, Eigen::Vector3d(0.984656, -0.0319246, 0.0110837), information));
	edges.push_back(TEdgeData(41, 42, Eigen::Vector3d(1.00266, 0.030635, 0.0300476), information));
	edges.push_back(TEdgeData(42, 43, Eigen::Vector3d(0.986417, -0.0130982, -0.0241183), information));
	edges.push_back(TEdgeData(43, 44, Eigen::Vector3d(0.97872, 0.0120778, -0.0117429), information));
	edges.push_back(TEdgeData(44, 45, Eigen::Vector3d(0.996113, -0.0407306, -0.0152182), information));
	edges.push_back(TEdgeData(45, 46, Eigen::Vector3d(1.00255, -0.00216301, -0.0105021), information));
	edges.push_back(TEdgeData(46, 47, Eigen::Vector3d(0.999641, -0.0336501, 0.0188071), information));
	edges.push_back(TEdgeData(47, 48, Eigen::Vector3d(-0.949748, 0.0117583, 3.11376), information));
	edges.push_back(TEdgeData(48, 49, Eigen::Vector3d(1.01739, 0.0123797, -0.00893411), information));
	edges.push_back(TEdgeData(49, 50, Eigen::Vector3d(1.01548, 0.0274024, -0.019191), information));
	edges.push_back(TEdgeData(40, 51, Eigen::Vector3d(2.97743, 0.0326539, 3.1211), information));
	edges.push_back(TEdgeData(50, 51, Eigen::Vector3d(1.05227, 0.0147383, -0.00136236), information));
	edges.push_back(TEdgeData(51, 52, Eigen::Vector3d(-0.0108141, -0.98436, -1.56099), information));
	edges.push_back(TEdgeData(52, 53, Eigen::Vector3d(1.03071, 0.00895876, -0.00840075), information));
	edges.push_back(TEdgeData(53, 54, Eigen::Vector3d(0.98342, 0.00979391, -0.0306844), information));
	edges.push_back(TEdgeData(7, 55, Eigen::Vector3d(-0.0335046, -0.00680906, -1.58407), information));
	edges.push_back(TEdgeData(54, 55, Eigen::Vector3d(1.01204, -0.015331, 0.00584842), information));
	edges.push_back(TEdgeData(14, 56, Eigen::Vector3d(0.00385417, 0.000186059, -3.14717), information));
	edges.push_back(TEdgeData(8, 56, Eigen::Vector3d(-0.0196555, 0.00673762, 0.0127182), information));
	edges.push_back(TEdgeData(55, 56, Eigen::Vector3d(-0.00365754, -0.984986, -1.57285), information));
	edges.push_back(TEdgeData(5, 57, Eigen::Vector3d(-0.048046, -0.00753482, -3.16285), information));
	edges.push_back(TEdgeData(56, 57, Eigen::Vector3d(1.031, -0.0163252, -0.0169613), information));
	edges.push_back(TEdgeData(57, 58, Eigen::Vector3d(0.983393, -0.0113447, -0.0148402), information));
	edges.push_back(TEdgeData(58, 59, Eigen::Vector3d(1.01024, 0.011576, 0.00432891), information));
	edges.push_back(TEdgeData(59, 60, Eigen::Vector3d(0.0201084, -1.00859, 4.73677), information));
	edges.push_back(TEdgeData(60, 61, Eigen::Vector3d(0.992544, -0.00406336, 0.00906878), information));
	edges.push_back(TEdgeData(61, 62, Eigen::Vector3d(0.980911, -0.0126781, 0.0247609), information));
	edges.push_back(TEdgeData(62, 63, Eigen::Vector3d(1.00765, -0.0370944, -0.00745089), information));
	edges.push_back(TEdgeData(47, 64, Eigen::Vector3d(-0.992098, -0.0164591, 3.12281), information));
	edges.push_back(TEdgeData(63, 64, Eigen::Vector3d(-0.0145417, -0.998609, -1.54739), information));
	edges.push_back(TEdgeData(64, 65, Eigen::Vector3d(1.03794, -0.0168313, -0.0130817), information));
	edges.push_back(TEdgeData(65, 66, Eigen::Vector3d(0.9912, 0.0115711, -0.0249519), information));
	edges.push_back(TEdgeData(66, 67, Eigen::Vector3d(0.949443, -0.0154924, -0.0091255), information));
	edges.push_back(TEdgeData(43, 68, Eigen::Vector3d(0.993623, 0.0391936, -0.00106149), information));
	edges.push_back(TEdgeData(67, 68, Eigen::Vector3d(-0.978361, -0.00927414, -3.13791), information));
	edges.push_back(TEdgeData(45, 69, Eigen::Vector3d(-0.006758, -0.00679624, -0.00213649), information));
	edges.push_back(TEdgeData(68, 69, Eigen::Vector3d(1.00367, -0.0352973, 0.0340684), information));
	edges.push_back(TEdgeData(69, 70, Eigen::Vector3d(1.02981, 0.00255454, 0.0150012), information));
	edges.push_back(TEdgeData(70, 71, Eigen::Vector3d(1.03652, 0.0118072, -0.00163612), information));
	edges.push_back(TEdgeData(71, 72, Eigen::Vector3d(0.00398168, -0.993979, 4.69836), information));
	edges.push_back(TEdgeData(72, 73, Eigen::Vector3d(0.969371, -0.0306017, -0.0326511), information));
	edges.push_back(TEdgeData(73, 74, Eigen::Vector3d(0.985691, 0.0111442, -0.00166414), information));
	edges.push_back(TEdgeData(74, 75, Eigen::Vector3d(0.981205, -0.00596464, 0.0226695), information));
	edges.push_back(TEdgeData(75, 76, Eigen::Vector3d(-0.00825988, 0.981841, -4.71863), information));
	edges.push_back(TEdgeData(76, 77, Eigen::Vector3d(1.01399, 0.0332094, -0.0649213), information));
	edges.push_back(TEdgeData(77, 78, Eigen::Vector3d(1.02795, 0.00984078, 0.0340066), information));
	edges.push_back(TEdgeData(78, 79, Eigen::Vector3d(1.00265, -0.00774271, 0.00950595), information));
	edges.push_back(TEdgeData(79, 80, Eigen::Vector3d(-0.0102099, -0.978673, 4.74423), information));
	edges.push_back(TEdgeData(80, 81, Eigen::Vector3d(1.01265, 0.0192011, -0.00199527), information));
	edges.push_back(TEdgeData(81, 82, Eigen::Vector3d(0.994241, -0.0319085, -0.0197558), information));
	edges.push_back(TEdgeData(82, 83, Eigen::Vector3d(1.00925, 0.00590969, -0.0214812), information));
	edges.push_back(TEdgeData(83, 84, Eigen::Vector3d(-0.0184826, 1.03307, -4.72819), information));
	edges.push_back(TEdgeData(84, 85, Eigen::Vector3d(0.984696, 0.0196236, 0.00877518), information));
	edges.push_back(TEdgeData(85, 86, Eigen::Vector3d(0.993027, 0.010799, 0.0107298), information));
	edges.push_back(TEdgeData(86, 87, Eigen::Vector3d(0.992905, 0.0213607, 0.0110665), information));
	edges.push_back(TEdgeData(87, 88, Eigen::Vector3d(0.00121839, 1.04031, 1.53711), information));
	edges.push_back(TEdgeData(88, 89, Eigen::Vector3d(1.00767, -0.0150986, 0.0147958), information));
	edges.push_back(TEdgeData(89, 90, Eigen::Vector3d(1.01226, -0.00539061, 0.030011), information));
	edges.push_back(TEdgeData(90, 91, Eigen::Vector3d(1.03457, 0.00297329, -0.00901519), information));
	edges.push_back(TEdgeData(91, 92, Eigen::Vector3d(-0.0159521, 0.972423, 1.55259), information));
	edges.push_back(TEdgeData(92, 93, Eigen::Vector3d(0.990753, 0.0620248, -0.0146912), information));
	edges.push_back(TEdgeData(93, 94, Eigen::Vector3d(0.971423, 0.0142496, 0.000217408), information));
	edges.push_back(TEdgeData(94, 95, Eigen::Vector3d(1.02272, -0.0278824, 0.000365479), information));
	edges.push_back(TEdgeData(95, 96, Eigen::Vector3d(-0.0193242, 1.04934, 1.57253), information));
	edges.push_back(TEdgeData(96, 97, Eigen::Vector3d(1.03931, -0.0130887, 0.0113687), information));
	edges.push_back(TEdgeData(97, 98, Eigen::Vector3d(0.993004, 0.0393656, -0.0105709), information));
	edges.push_back(TEdgeData(98, 99, Eigen::Vector3d(1.03897, -0.0238964, -0.0173253), information));
	edges.push_back(TEdgeData(99, 100, Eigen::Vector3d(-0.985853, -0.00979836, -3.15198), information));
	edges.push_back(TEdgeData(83, 101, Eigen::Vector3d(-1.97704, -0.00703316, -3.16668), information));
	edges.push_back(TEdgeData(100, 101, Eigen::Vector3d(1.02465, 0.0317282, 0.024041), information));
	edges.push_back(TEdgeData(96, 102, Eigen::Vector3d(0.007666, 0.00487275, -3.16637), information));
	edges.push_back(TEdgeData(96, 102, Eigen::Vector3d(0.0105401, -0.00932236, -3.11433), information));
	edges.push_back(TEdgeData(101, 102, Eigen::Vector3d(0.99396, 0.0202571, -0.00240724), information));
}

#include "slam_app/ParsePrimitives.h"
#include "slam/Parser.h"

/**
 *	@brief a simple parse loop
 */
class CMyParseLoop {
protected:
	std::vector<TEdgeData> &m_r_edges; /**< @brief reference to the output vector of edges */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_edges is reference to the output vector of edges
	 */
	CMyParseLoop(std::vector<TEdgeData> &r_edges)
		:m_r_edges(r_edges)
	{}

	/**
	 *	@brief appends the system with an odometry measurement
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	void AppendSystem(const CParserBase::TEdge2D &r_t_edge) // throw(std::bad_alloc)
	{
		m_r_edges.push_back(TEdgeData(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
			r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma));
	}
};

void Load_Dataset(const char *p_s_filename, std::vector<TEdgeData> &r_edges)
{
	r_edges.clear(); // !!

	typedef MakeTypelist(CEdge2DParsePrimitive) AllowedEdges;
	typedef CParserTemplate<CMyParseLoop, AllowedEdges> CParser;

	CMyParseLoop ploop(r_edges);
	CParser().Parse(p_s_filename, ploop);

	printf("loaded \'%s\' : " PRIsize " edges\n", p_s_filename, r_edges.size());
}

/**
 *	@page compactposeslamexample Compact pose sLAM Example
 *
 *	@todo At this point there is no description. But you can browse the source code.
 */

/*
 *	end-of-file
 */
