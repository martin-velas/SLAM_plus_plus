/*
								+-----------------------------------+
								|                                   |
								| *** IJRR'15 Compact pose SLAM *** |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2015  |
								|                                   |
								|             Main.cpp              |
								|                                   |
								+-----------------------------------+
*/

/**
 *	@file src/slam_compact_pose_ijrr/Main.cpp
 *	@brief data association / compact pose SLAM implementation for the IJRR 2015 journal paper
 *	@date 2015
 *	@author -tHE SWINe-
 */

#include <stdio.h> // printf
#include <algorithm>
#include <numeric>
#include <queue>
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP
#include "slam/LinearSolver_UberBlock.h" // linear solver
#include "slam/ConfigSolvers.h" // nonlinear graph solvers
#include "slam/SE2_Types.h" // SE(2) types
#include "slam/SE3_Types.h" // SE(3) types
#include "slam/Marginals.h" // covariance calculation
#include "slam/Distances.h" // distance calculation
#include "slam_app/ParsePrimitives.h"
#include "slam/Parser.h"
#include "eigen/Eigen/SVD" // JacobiSVD
#include "slam/MemUsage.h"

/**
 *	@brief parsed edge data
 *	@tparam n_dimension is measurement dimension (either 3 for 2D or 6 for 3D)
 */
template <const int n_dimension = 3>
struct TEdgeData {
	typedef Eigen::Matrix<double, n_dimension, n_dimension, Eigen::DontAlign> _TyMatrix; /**< @brief matrix type */
	typedef Eigen::Matrix<double, n_dimension, 1, Eigen::DontAlign> _TyVector; /**< @brief vector type */
	size_t p_vertex[2]; /**< @brief indices of the vertices */
	_TyVector v_measurement; /**< @brief measurement vector */
	_TyMatrix t_information; /**< @brief information matrix */

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
		const _TyVector &r_v_measurement, const _TyMatrix &r_t_information)
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

typedef TEdgeData<3> TEdgeData2D; /**< @brief 2D edge data type */
typedef TEdgeData<6> TEdgeData3D; /**< @brief 3D edge data type */

/**
 *	@brief a simple predicate that sorts the odometry edges first
 */
class COdometryFirst {
public:
	/**
	 *	@brief less-than comparison for edges
	 *
	 *	@tparam CEdgeData is edge data type (may be TEdgeData2D or TEdgeData3D)
	 *
	 *	@param[in] r_edge_a is an edge
	 *	@param[in] r_edge_b is an edge
	 *
	 *	@return Returns true if r_edge_a is ordered before r_edge_b, otherwise returns false.
	 */
	template <class CEdgeData>
	bool operator ()(const CEdgeData &r_edge_a, const CEdgeData &r_edge_b) const
	{
		bool b_a_is_odometry = r_edge_a.p_vertex[0] + 1 == r_edge_a.p_vertex[1];
		bool b_b_is_odometry = r_edge_b.p_vertex[0] + 1 == r_edge_b.p_vertex[1];
		return b_a_is_odometry > b_b_is_odometry; // if a is odometry edge and b is not, then a is ordered before b
	}
};

/**
 *	@brief a helper function that detects dataset type
 *
 *	@param[in] p_s_filename is null-terminated string, containing file name of a dataset
 *	@param[out] r_b_is_2D is dataset type flag (filled upon successful return, true means a 2D dataset, false means 3D)
 *
 *	@return Returns true on success, false on failure (failure both to parse the file or to determine dataset type).
 */
static bool Detect_DatasetType(const char *p_s_filename, bool &r_b_is_2D);

/**
 *	@brief a helper function that loads a 2D dataset
 *
 *	@param[in] p_s_filename is null-terminated string, containing file name of a dataset
 *	@param[out] edges is vector of edges to add the edges to
 *
 *	@return Returns true on success, false on failure.
 */
static bool Load_Dataset(const char *p_s_filename, std::vector<TEdgeData2D> &edges);

/**
 *	@brief a helper function that loads a 3D dataset
 *
 *	@param[in] p_s_filename is null-terminated string, containing file name of a dataset
 *	@param[out] edges is vector of edges to add the edges to
 *
 *	@return Returns true on success, false on failure.
 */
static bool Load_Dataset(const char *p_s_filename, std::vector<TEdgeData3D> &edges);

/**
 *	@brief information gain and distance for a porential loop closure
 */
struct TInfoGain_Distance {
	double f_gain; /**< @brief information gain of the loop */
	double f_distance; /**< @brief distance probability of the vertices */
	//Eigen::MatrixXd t_sigma_d; /**< @brief distance covariance */
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
		/*const Eigen::MatrixXd &r_t_sigma_d = Eigen::MatrixXd(),*/ size_t _n_vertex = size_t(-1))
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
 *	@brief displays help
 */
static void PrintHelp();

/**
 *	@brief compact pose SLAM parameters
 */
struct TParams {
	const char *p_s_input_file; /**< @brief input file */
	const char *p_s_config_file; /**< @brief configuration file or null */
	bool b_write_bitmaps; /**< @brief bitmap write flag */
	bool b_disable_incremental_bitmaps; /**< @brief incremental solution bitmap write override flag */
	bool b_bitmaps_xz_plane; /**< @brief bitmap projection type (if set, xz; if cleared, xy) */
	bool b_bitmaps_highlight_loops; /**< @brief loop closure highlingting flag */
	bool b_write_systems; /**< @brief state vector write flag */
	bool b_write_animdata; /**< @brief ICRA 2015 demo animation data write flag */
	bool b_write_covmats; /**< @brief covariance matrices write flag */
	bool b_write_cov_norms; /**< @brief full covariance norm write flag */
	bool b_write_cumtime; /**< @brief cummulative timing write flag */
	bool b_write_mem_usage; /**< @brief writes memory usage */
	bool b_write_compact_edge_lengths; /**< @brief compact edge lengths write flag (testing only) */
	bool b_do_thresh_analysis; /**< @brief threshold analysis flag (testing only) */
	bool b_do_thresh_sensitivity_analysis; /**< @brief threshold sensitivity analysis flag (testing only) */

	Eigen::Vector2d v_min; /**< @brief coordinates of the top left corner of the visualized area */
	Eigen::Vector2d v_max;  /**< @brief coordinates of the bottom right corner of the visualized area */
	int n_resolution; /**< @brief output image resolution, in pixels */

	Eigen::VectorXd v_sensor_range; /**< @brief sensor range (a tolerance, affects which poses will be matched) */

	double f_loop_probability_dist_threshold; /**< @brief poses with higher loop probability will be candidates for sensor matching */

	double f_keep_pose_gain_threshold; /**< @brief minimal information gain to keep a pose (not extend an edge) */
	double f_loop_closure_gain_threshold; /**< @brief minimal information gain to keep a loop closure @note Initially the same as keep pose gain but can be set differently if needed. */

	Eigen::MatrixXd t_Sigma_bar; /**< @brief value of \$f\bar\Sigma\$f */

	size_t n_max_pose_num; /**< @brief maximal number of poses in the dataset */
	size_t n_max_loop_num; /**< @brief maximal number of loops in the dataset */
	int n_random_seed; /**< @brief random seed for dataset pruning */

	const char *p_s_gt_file; /**< @brief name of the file containing ground truth or null for no ground truth */
	const char *p_s_ref_file; /**< @brief name of the file containing reference trajectory (for visual comparison) or null */
	const char *p_s_compact_edge_lengths_file; /**< @brief name of the file containing compact edge lengths for covariance norm marginaliation or null for no marginalization (testing only) */

	int n_omp_threads; /**< @brief number of OpenMP threads or 0 for automatic */
	bool b_omp_dynamic; /**< @brief OpenMP dynamic threads flag */

	/**
	 *	@brief sets defaults
	 *	@param[in] b_2D_SLAM is dimensionality flag (if set, 2D, if cleared, 3D)
	 */
	void Defaults(bool b_2D_SLAM);

	/**
	 *	@brief parses command line arguments
	 *
	 *	@param[in] n_arg_num is number of commandline arguments
	 *	@param[in] p_arg_list is the list of commandline arguments
	 *
	 *	@return Returns 1 on success, 0 on success with no continuation or -1 on failure.
	 */
	int n_ParseCommandline(int n_arg_num, const char **p_arg_list);

	/**
	 *	@brief loads additional parameters from a configuration file
	 *
	 *	@param[in] p_s_config_file is name of the configuration file
	 *	@param[in] b_2D_SLAM is dimensionality flag (if set, 2D, if cleared, 3D)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Load_Config(const char *p_s_config_file, bool b_2D_SLAM);
};

/**
 *	@brief creates a directory
 *	@param[in] p_s_path is null-terminated string containing the name of the directory to create
 *	@return Returns zero on success, nonzero on failure.
 */
static int MakeDir(const char *p_s_path);

/**
 *	@brief compact pose SLAM implementation
 *	@tparam b_2D_SLAM is 2D / 3D selection flag
 */
template <const bool b_2D_SLAM = true>
class CCompactPoseSLAM {
public:
	/**
	 *	@brief parameters stored as enum
	 */
	enum {
		n_pose_dimension = (b_2D_SLAM)? 3 : 6, /**< @brief vertex dimension */
		n_pose_pos_components = (b_2D_SLAM)? 2 : 3, /**< @brief number of position components in a vertex */
		n_pose_rot_components = n_pose_dimension - n_pose_pos_components, /**< @brief number of rotation components in a vertex */
		n_distance_dimension = (b_2D_SLAM)? 3 : 4 /**< @brief vertex dimension */
	};

	typedef TEdgeData<n_pose_dimension> _TyEdgeData; /**< @brief parsed edge type */
	typedef Eigen::Matrix<double, n_pose_dimension, n_pose_dimension> _TyMatrix; /**< @copydoc _TyEdgeData::_TyMatrix */
	typedef Eigen::Matrix<double, n_distance_dimension, n_distance_dimension> _TyDistanceMatrix; /**< @brief matrix the size of reduced dimension distance */
	typedef Eigen::Matrix<double, n_pose_dimension, 1> _TyVector; /**< @copydoc _TyEdgeData::_TyVector */
	typedef Eigen::Matrix<double, n_distance_dimension, 1> _TyDistanceVector; /**< @brief vector the size of reduced dimension distance */
	typedef typename CChooseType<CVertexPose2D, CVertexPose3D, b_2D_SLAM>::_TyResult _TyVertexPose; /**< @brief pose vertex type */
	typedef typename CChooseType<CEdgePose2D, CEdgePose3D, b_2D_SLAM>::_TyResult _TyEdgePose; /**< @brief pose-pose edge type */

	typedef Eigen::Matrix<double, n_pose_dimension, 1, Eigen::DontAlign> _TyVectorUnalign; /**< @brief unaligned vector type */
	typedef Eigen::Matrix<double, n_pose_dimension, n_pose_dimension, Eigen::DontAlign> _TyMatrixUnalign; /**< @brief unaligned matrix type */
	typedef Eigen::Matrix<double, n_distance_dimension, 1, Eigen::DontAlign> _TyDistanceVectorUnalign; /**< @brief unaligned vector the size of reduced dimension distance */
	typedef Eigen::Matrix<double, n_distance_dimension, n_distance_dimension, Eigen::DontAlign> _TyDistanceMatrixUnalign; /**< @brief unaligned matrix the size of reduced dimension distance */
	// unaligned matrix types required for anything to be stored inside an std::vector in x86

	typedef typename MakeTypelist(_TyVertexPose) TVertexTypelist; /**< @brief list of vertex types */
	typedef typename MakeTypelist(_TyEdgePose) TEdgeTypelist; /**< @brief list of edge types */
	typedef CFlatSystem<_TyVertexPose, TVertexTypelist, _TyEdgePose,
		TEdgeTypelist, CProportionalUnaryFactorFactory> CSystemType; /**< @brief optimized system type */
	typedef CLinearSolver_UberBlock<typename CSystemType::_TyHessianMatrixBlockList> CLinearSolverType; /**< @brief linear solver type */
	typedef CNonlinearSolver_FastL<CSystemType, CLinearSolverType> CNonlinearSolverType; /**< @brief nonlinear solver type */

	typedef CDistances<CSystemType> CDistanceType_Bare; /**< @brief distance calculation object */
	typedef CDistances<CSystemType, TEdgeTypelist,	CSE3_XYZ_ViewDirection_DistanceTransform> CDistanceType_ReducedDim; /**< @brief reduced dimension distance calculation object */
	typedef typename CChooseType<CDistanceType_Bare, CDistanceType_ReducedDim, b_2D_SLAM>::_TyResult CDistanceType; /**< @brief default distance calculation object */

	/**
	 *	@brief loop edge threshold diagnostic information
	 */
	struct TLoopInfo {
		size_t p_vertex[2]; // vertices of the loop edge
		_TyMatrixUnalign t_Sigma_e; // sigma of the edge
		_TyDistanceVectorUnalign v_mean_d; // mean d of the variables (APAL), possibly after applying distance transform
		_TyDistanceMatrixUnalign t_Sigma_d; // Sigma_d of the variables (APAL), possibly after applying distance transform
		double f_information_gain_prior; // information gain using a prior on the edge sigma
		double f_information_gain_matched; // information gain using calculated edge sigma, after sensor matching

		/**
		 *	@brief default constructor; has no effect
		 */
		TLoopInfo()
		{}

		/**
		 *	@brief constructor; initializes edge information
		 */
		TLoopInfo(size_t n_vertex_0, size_t n_vertex_1, const _TyMatrix &r_t_Sigma_e,
			const _TyDistanceVector &r_v_mean_d, const _TyDistanceMatrix &r_t_Sigma_d,
			const _TyMatrix &r_t_Sigma_d_raw)
			:t_Sigma_e(r_t_Sigma_e), v_mean_d(r_v_mean_d), t_Sigma_d(r_t_Sigma_d),
			f_information_gain_prior(CDistancesUtils::f_Information_Gain(_TyDistanceMatrix::Identity(), r_t_Sigma_d)),
			f_information_gain_matched(CDistancesUtils::f_Information_Gain(r_t_Sigma_e, r_t_Sigma_d_raw))
		{
			p_vertex[0] = n_vertex_0;
			p_vertex[1] = n_vertex_1;

			if(_isnan(f_information_gain_prior)) {
				//CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, r_t_Sigma_d, "Sigma_d");
				f_information_gain_prior = -1e100;
			}
			if(_isnan(f_information_gain_matched)) {
				//CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, r_t_Sigma_d_raw, "Sigma_d");
				f_information_gain_matched = -1e100;
			}
		}

		/**
		 *	@brief calculates distance probability of this loop closure, given the threshold
		 *	@param[in] r_v_thresh is sensor range threshold
		 *	@return Returns the minimum component of the probability distance of this edge.
		 */
		double f_DistanceProba(const _TyDistanceVector &r_v_thresh) const
		{
			return (CDistancesUtils::v_GaussianCPD(v_mean_d, t_Sigma_d, r_v_thresh) -
				CDistancesUtils::v_GaussianCPD(v_mean_d, t_Sigma_d, -r_v_thresh)).minCoeff();
		}

		/**
		 *	@brief gets information gain prior (before sensor matching)
		 *	@return Returns the information gain prior.
		 *	@note This does not depend on the sensor range threshold.
		 */
		double f_InfoGain_Prior() const
		{
			return f_information_gain_prior;
		}

		/**
		 *	@brief gets information gain prior (after sensor matching, using the estimated edge information)
		 *	@return Returns the information gain.
		 *	@note This does not depend on the sensor range threshold.
		 */
		double f_InfoGain() const
		{
			return f_information_gain_matched;
		}
	};

public:
	/**
	 *	@brief GetNeighbours
	 *
	 *	@param[in] i is the index of the variable for which we want neighbours
	 *	@param{in} adj is the adjacency matrix (this is full matrix)
	 *
	 *	@return returns a list of neighbours of the variable i
	 */
	std::vector<size_t> GetNeighbours(size_t i, const cs* adj)
	{
		size_t first = adj->p[i], last = adj->p[i + 1];
		std::vector<size_t> neigh(adj->i + first, adj->i + last);
		return neigh;
	}

	CUberBlockMatrix MarkovBlanket(const CUberBlockMatrix& lambda)
	{
		cs *upper_adj = lambda.p_BlockStructure_to_Sparse();
		cs *lower_adj = cs_transpose(upper_adj, 0);
		cs *adj = cs_add(upper_adj, lower_adj, 1, 1);
		size_t n = adj->n;
		cs_spfree(upper_adj);
		cs_spfree(lower_adj);

		CUberBlockMatrix approx_inverse;
		for(size_t i = 0; i < n; ++ i){
			std::vector<size_t> neigh = GetNeighbours(i, adj);
			CUberBlockMatrix blanket;
			lambda.PermuteTo(blanket, &neigh[0], neigh.size());
			blanket.SliceTo(blanket, neigh.size(), neigh.size());
			blanket.InverseOf_Symmteric(blanket);
			size_t current_variable = lambda.n_BlockColumn_Block_Num(i);
			approx_inverse.Append_Block_Log(blanket.t_GetBlock_Log(current_variable, current_variable), i, i);
		}
		cs_spfree(adj);

		return approx_inverse;
	}

	/**
	 *	@brief main function
	 *
	 *	@param[in] n_arg_num is number of commandline arguments
	 *	@param[in] p_arg_list is the list of commandline arguments
	 *	@param[in] n_memory_use_after_init is process memory use, in bytes, before loading anything
	 *
	 *	@return Returns 0 on success, -1 on failure.
	 */
	static int main(int n_arg_num, const char **p_arg_list, uint64_t n_memory_use_after_init)
	{
		CCompactPoseSLAM<b_2D_SLAM>().MarkovBlanket(CUberBlockMatrix());
		// test the markov blankets algorithm

		TParams p;
		p.Defaults(b_2D_SLAM);
		switch(p.n_ParseCommandline(n_arg_num, p_arg_list)) {
		case -1:
			return -1; // exit after a failure
		case 0:
			return 0; // exit after printing help
		}
		if(b_2D_SLAM)
			p.b_bitmaps_xz_plane = false; // no z in 2D
		// parse commandline

#ifdef _OPENMP
		if(p.n_omp_threads)
			omp_set_num_threads(p.n_omp_threads); // can use this to set no. of threads
		if(p.b_omp_dynamic)
			omp_set_dynamic(true); // dynamically allocate threads
		#pragma omp parallel
		#pragma omp master
		{
			printf("%s (%d threads)\n", "_OPENMP", omp_get_num_threads());
		}
#endif // _OPENMP
		// configure OpenMP, where available

		if(p.p_s_config_file && !p.Load_Config(p.p_s_config_file, b_2D_SLAM)) {
			fprintf(stderr, "error: failed to open configuration file \'%s\'\n", p.p_s_config_file);
			return -1;
		}
		const _TyMatrix t_Sigma_bar = p.t_Sigma_bar; // convert to fixed-size to speed up the computation later
		// load the cnfiguration file, if one was specified

		if(p.b_write_bitmaps || p.b_write_systems || p.b_write_covmats)
			MakeDir("anim");
		// make an output folder for the animation data, ignore if the directory already exists

		TBmp *p_image = 0;
		Eigen::Vector2d v_off, v_scale;
		if(/*p.b_write_bitmaps &&*/ !Alloc_Bitmap(p.v_min, p.v_max, p.n_resolution, p_image, v_off, v_scale)) // alloc always, will use this bitmap for drawing the solution
			return -1;
		// calculate image resolution and transformation to raster coordinates

		std::vector<_TyEdgeData> edges, loop_closures;
		if(!Load_SeparatedDataset(p.p_s_input_file, edges, loop_closures)) {
			fprintf(stderr, "error: failed to load \'%s\'\n", p.p_s_input_file);
			return -1;
		}
		// keep a list of loop closures, so we can add them later,
		// simulating loop closing using a real sensor

		const std::vector<_TyEdgeData> measurements = (p.p_s_gt_file ||
			p.p_s_ref_file)? edges : std::vector<_TyEdgeData>();
		// only needed for error evaluation (edges gets modified as poses are removed
		// from the graph - for error evaluation, we need the original ones)

		std::vector<size_t> pruned_edge_lengths;
		if(p.n_max_loop_num != size_t(-1) || p.n_max_pose_num != size_t(-1)) {
			srand(p.n_random_seed); // make it reproducible
			if(p.n_max_pose_num != size_t(-1)) {
				size_t n = edges.size() + 1;
				pruned_edge_lengths.resize(edges.size(), 1);
				if(n > p.n_max_pose_num) {
					fprintf(stderr, "debug: pruning the number of poses from "
						PRIsize " down to " PRIsize "\n", n, p.n_max_pose_num);
					for(size_t i = 0, n_remove_edges = n - p.n_max_pose_num; i < n_remove_edges; ++ i) {
						size_t n_vertex_to_remove = rand() % n; // not quite good uniform rand, but ...
						for(size_t j = 0, m = edges.size(); j < m; ++ j) {
							if(edges[j].p_vertex[0] == n_vertex_to_remove ||
							   edges[j].p_vertex[1] == n_vertex_to_remove) {
								// edge j is going to be extended

								size_t k;
								if((k = j + 1) != m && (edges[k].p_vertex[0] == n_vertex_to_remove ||
								   edges[k].p_vertex[1] == n_vertex_to_remove)) {
									// k already set
								} else if(j > 0 && (edges[k = j - 1].p_vertex[0] == n_vertex_to_remove ||
								   edges[k].p_vertex[1] == n_vertex_to_remove)) {
									// k already set
								} else {
									if(!j && m > 1)
										k = j + 1;
									else if(j + 1 == m && j)
										k = j - 1;
									else {
										fprintf(stderr, "warning: unable to extend edge with a removed vertex\n");
										// will probably crash error evaluation later
										break;
									}
								}
								// find the adjacent edge that shares the selected vertex to be removed

								if(j > k)
									std::swap(j, k);
								// make sure that the removed edge comes after the current edge

								int vj = (edges[j].p_vertex[1] == n_vertex_to_remove)? 1 : 0;
								int vk = (edges[k].p_vertex[1] == n_vertex_to_remove)? 1 : 0;
								if(edges[k].p_vertex[vk] == n_vertex_to_remove) { // if we took the last branch then it does not
									edges[j].p_vertex[1 - vj] = edges[k].p_vertex[1 - vk];
									if(edges[j].p_vertex[0] > edges[j].p_vertex[1])
										std::swap(edges[j].p_vertex[0], edges[j].p_vertex[1]); // keep it sorted
								}
								// use "the other" vertex in edge k to set index of "the other" vertex in edge j

								size_t n_append_to = (edges[k].p_vertex[vk] == n_vertex_to_remove)? j : k;

								_TyMatrix J0, J1;
								_TyVector v_cum_measurement;
								Relative_to_Absolute(edges[j].v_measurement, edges[k].v_measurement,
									v_cum_measurement, J0, J1); // the first and the third argument cannot be the same
								edges[n_append_to].v_measurement = v_cum_measurement;
								// accumulate measurements and get jacobians

								_TyMatrix t_cov = J0 * edges[j].t_information.inverse() * J0.transpose() +
									J1 * edges[k].t_information.inverse() * J1.transpose();
								edges[n_append_to].t_information = t_cov.inverse();
								// accumulate information

								pruned_edge_lengths[n_append_to] +=
									pruned_edge_lengths[(n_append_to == j)? k : j];
								// accumulate length

								if(n_append_to == j) {
									pruned_edge_lengths.erase(pruned_edge_lengths.begin() + k);
									edges.erase(edges.begin() + k);
								} else {
									pruned_edge_lengths.erase(pruned_edge_lengths.begin() + j);
									edges.erase(edges.begin() + j);
								}
								// erase the k-th edge but only if we could extend the j-th edge
								break;
							}
						}
						_ASSERTE(pruned_edge_lengths.size() == edges.size());
						// find the edge with the removed vertex and extend the neighbor edge

						if(!Dataset_RemovePose(n_vertex_to_remove, edges, loop_closures, size_t(-1))) {
							fprintf(stderr, "error: failed to prune dataset poses\n");
							return -1;
						}
						-- n; // !!
						// still need to update indexing of the vertices and erase any affected loops

						_ASSERTE(pruned_edge_lengths.size() == edges.size()); // well ...
					}
					// remove random poses
				} else {
					fprintf(stderr, "debug: cannot prune the poses to " PRIsize ", there are only "
						PRIsize " poses in the dataset\n", p.n_max_pose_num, n);
				}
			}
			if(p.n_max_loop_num != size_t(-1)) {
				size_t n = loop_closures.size();
				if(n > p.n_max_loop_num) {
					fprintf(stderr, "debug: pruning the number of loops from "
						PRIsize " down to " PRIsize "\n", n, p.n_max_loop_num);
					for(size_t i = 0, n_remove_loops = n - p.n_max_loop_num; i < n_remove_loops; ++ i) {
						size_t n_loop_to_remove = rand() % n; // not quite good, but ...
						loop_closures.erase(loop_closures.begin() + n_loop_to_remove);
						-- n; // !!
					}
					// remove random loops
				} else {
					fprintf(stderr, "debug: cannot prune the loops to " PRIsize ", there are only "
						PRIsize " loops left in the dataset\n", p.n_max_loop_num, n);
				}
			}
		}
		// delete random poses and loops to get RFPFL

		const size_t n_loop_closure_num = loop_closures.size(),
			n_total_edge_num = loop_closures.size() + edges.size();
		// total for the whole dataset

		const size_t n_pose_num = (edges.empty())? 0 :
			std::max(edges.back().p_vertex[0], edges.back().p_vertex[1]) + 1;
		_ASSERTE(n_pose_num == edges.size() + 1); // should match
		// the total number of poses in the graph (edges contain the odometry edges now)

		size_t n_closed_loop_num = 0, n_distance_probability_match_num = 0,
			n_association_num = 0, n_low_gain_loop_num = 0;
		// keep a counter of how many times sensor matching was attempted (statistics)

		std::vector<double> loop_gains, pose_gains;
		double f_min_probability = 1e100, f_min_loop_gain = 1e100, f_min_pose_gain = 1e100;
		// what was the minimum probability that closed the loop (a statistic, not a threshold)

		size_t n_discarded_loop_num = 0;

		std::vector<size_t> compact_edge_lengths; // stores number of edges each compact edge is composed of
		std::vector<TLoopInfo> true_loop_distance_info; // stores distance and info gain information for ground truth loop closures (only calculated if p.b_do_thresh_analysis)
		std::vector<double> all_to_all_distance_info; // stores distances for all the possible loop closures in the dataset (only calculated if p.b_do_thresh_sensitivity_analysis, note that this takes O(n^2/2) space in the number of poses, for 100k that's roughly 37 GB)
		// data required for evaluation

		std::vector<size_t> ref_compact_edge_lengths;
		if(p.p_s_compact_edge_lengths_file) {
			FILE *p_fr;
			if(!(p_fr = fopen(p.p_s_compact_edge_lengths_file, "rb")))
				fprintf(stderr, "error: failed to open \'%s\' for reading\n", p.p_s_compact_edge_lengths_file);
			else {
				if(!Read_Vector32(p_fr, ref_compact_edge_lengths)) {
					fprintf(stderr, "error: i/o error while reading \'%s\'\n", p.p_s_compact_edge_lengths_file);
					ref_compact_edge_lengths.clear(); // don't use it !!
				}
				fclose(p_fr);
			}
		}
		// read the compact edge lengths data (for comparisons employing marginalization)

		FILE *p_fw_cov_norms = 0;
		if(p.b_write_cov_norms && !(p_fw_cov_norms = fopen((ref_compact_edge_lengths.empty())? "cov_norms.csv" : "cov_norms_marginalized.csv", "w")))
			fprintf(stderr, "error: failed to open \'%s\' for writing\n", (ref_compact_edge_lengths.empty())? "cov_norms.csv" : "cov_norms_marginalized.csv");
		// start writing covariance norms

		FILE *p_fw_animdata = 0;
		if(p.b_write_animdata && !(p_fw_animdata = fopen("covanim_data.dat", "wb")))
			fprintf(stderr, "error: failed to open \'%s\' for writing\n", "covanim_data.dat");
		// start writing the animation data

		FILE *p_cumtime_fw = 0;
		if(p.b_write_cumtime) {
			if(!(p_cumtime_fw = fopen("cumtime.csv", "w")))
				fprintf(stderr, "error: failed to open \'%s\' for writing\n", "cumtime.csv");
			else {
				fprintf(p_cumtime_fw, "solver time,compact pose time (incl. candidate times),"
					"drawing time,graph mod time,cov norm time,num verts,num all edges,num loop"
					" edges,num dist candidates,num gain candidates,loop candidate time,loop "
					"candidate sort time,loop candidate update time,loop candidate sort"
					" update time,marginals time\n");
			}
		}
		// start writing the timing info

		FILE *p_musage_fw = 0;
		if(p.b_write_mem_usage) {
			if(!(p_musage_fw = fopen("musage.csv", "w")))
				fprintf(stderr, "error: failed to open \'%s\' for writing\n", "musage.csv");
			else
				fprintf(p_musage_fw, "proces memory,information NNZ,R factor NNZ\n");
		}
		// start writing the timing info

		CTimer t;
		CTimerSampler timer(t);

		double f_solver_time = 0; // NLS solving
		double f_loop_candidates_time = 0; // calculating the candidates
		double f_candidates_sort_time = 0; // sorting the loop candidates
		double f_loop_candidates_update_time = 0; // updating the candidates
		double f_candidates_sort_update_time = 0; // sorting the updated loop candidates
		double f_compact_pose_time = 0; // compact pose SLAM algorithm
		double f_drawing_time = 0; // saving images and other outputs
		double f_graph_mod_time = 0; // graph modification upon pose removal
		double f_cov_norm_time = 0; // calculation of full covariance norms
		// timing stats

		// the compact pose SLAM algorithm starts here

		CSystemType system;
		EBlockMatrixPart n_marginal_selection = EBlockMatrixPart(mpart_LastColumn | mpart_Diagonal);
		CNonlinearSolverType solver(system, solve::Nonlinear(frequency::Every(1), 5, 0.1),
			TMarginalsComputationPolicy(marginals::do_calculate,
			frequency::Every(1), n_marginal_selection, n_marginal_selection));
		// set solver to calculate marginals, which are needed to calculate the distances

		bool b_keep_pose = true;
		// compact pose SLAM edge replace flag

		_TyVector v_cum_measurement;
		_TyMatrix t_cum_information; // value does not matter, will be overwritten
		// keep a cumulative edge

		for(size_t i = 0, n = edges.size(); i < n; ++ i) {
			printf("\rstep " PRIsize " ", i);
			timer.Accum_DiffSample(f_drawing_time); // in windows, the console output actually takes a lot of time

			const _TyEdgeData &r_edge = edges[i];
			// get an edge

			if(b_keep_pose) {
				v_cum_measurement = r_edge.v_measurement;
				t_cum_information = r_edge.t_information;
				// the first step is simple

				compact_edge_lengths.push_back((pruned_edge_lengths.empty())? 1 : pruned_edge_lengths[i]);	// insert new edge counter
				// store last estimated edge
			} else {
				_TyMatrix J0, J1;
				_TyVector temp;
				Relative_to_Absolute(v_cum_measurement, r_edge.v_measurement, temp, J0, J1); // the first and the third argument cannot be the same
				v_cum_measurement = temp;
				// accumulate measurements and get jacobians

				_TyMatrix t_cov = J0 * t_cum_information.inverse() * J0.transpose() +
					J1 * r_edge.t_information.inverse() * J1.transpose();
				t_cum_information = t_cov.inverse();
				// accumulate information

				compact_edge_lengths.back() += (pruned_edge_lengths.empty())? 1 : pruned_edge_lengths[i];
				// increment the edge counter
			}
			// accumulate edge information

			if(b_keep_pose) {
				solver.Incremental_Step(system.r_Add_Edge(
					_TyEdgePose(r_edge.p_vertex[0], r_edge.p_vertex[1],
					v_cum_measurement, t_cum_information, system)));

#ifdef _DEBUG
				if(!solver.Check_Marginals())
					fprintf(stderr, "error: marginals incorrect\n");
#endif // _DEBUG
			} else {
#ifdef _DEBUG
				if(!solver.Check_Marginals())
					fprintf(stderr, "error: marginals incorrect\n");
#endif // _DEBUG

				_ASSERTE(!system.r_Edge_Pool().b_Empty() && !system.r_Vertex_Pool().b_Empty());
				_TyEdgePose &r_last_edge = system.r_Edge_Pool().template r_At<_TyEdgePose>(system.r_Edge_Pool().n_Size() - 1);
				// get the last edge

				Eigen::Map<Eigen::VectorXd> v_vertex0 = system.r_Vertex_Pool()[r_last_edge.n_Vertex_Id(0)].v_State();
				// get the last vertex

				_TyVector v_vertex1;
				Relative_to_Absolute(v_vertex0, v_cum_measurement, v_vertex1);
				// calculate where the second vertex ends up, given the accumulated measurement

				solver.Change_LastEdge(r_last_edge, v_cum_measurement, t_cum_information, v_vertex1, false);
				// let the solver update the system

#ifdef _DEBUG
				if(!solver.Check_Marginals())
					fprintf(stderr, "error: marginals incorrect\n");
#endif // _DEBUG
			}
			// this is a spanning tree edge, add it to the system and update

			const CUberBlockMatrix &r_marginals = solver.r_MarginalCovariance().r_SparseMatrix();
			// get the marginals

			_ASSERTE(r_marginals.n_BlockColumn_Num() == system.r_Vertex_Pool().n_Size());
			Eigen::MatrixXd last_sigma = r_marginals.t_GetBlock_Log(
				r_marginals.n_BlockRow_Num() - 1, r_marginals.n_BlockColumn_Num() - 1);
			// get the last block of the marginals, corresponding to the current pose

			timer.Accum_DiffSample(f_solver_time);

			if(p.b_write_systems) {
				char p_s_filename[256];
				sprintf(p_s_filename, "anim/sys%05" _PRIsize ".txt", i + 2); // 2 vertices at the first edge
				system.Dump(p_s_filename);
				// write the system
			}

			timer.Accum_DiffSample(f_drawing_time);

			bool b_low_gain_edge = true;
			// see if the edge presents sufficient information gain

			std::vector<size_t> probable_matches, true_matches; // list of probable matches with the last vertex

			{
				Eigen::VectorXd v_threshold; // corresponds to CEdgePose2D
				v_threshold = p.v_sensor_range; // sensor precision (field of view like)
				const Eigen::VectorXd *p_thresh_list[] = {&v_threshold};
				const size_t n_thresh_num = sizeof(p_thresh_list) / sizeof(p_thresh_list[0]);
				// need a list of distance thresholds, one per each edge in TEdgeTypelist

				CDistanceType distances(system, solver.r_MarginalCovariance().r_SparseMatrix(), p_thresh_list, n_thresh_num);
				// initialize an object to calculate distances, in 3D apply view direction angle equation to reduce 6D distance to 4D

				Eigen::VectorXd v_threshold2(Eigen::VectorXd::Ones(n_pose_dimension)); // value does not matter, we just want sigma d and it does not depend on this
				const Eigen::VectorXd *p_thresh_list2[] = {&v_threshold2};
				const size_t n_thresh_num2 = sizeof(p_thresh_list2) / sizeof(p_thresh_list2[0]);
				// need a list of distance thresholds, one per each edge in TEdgeTypelist

				CDistanceType_Bare distances_bare(system, solver.r_MarginalCovariance().r_SparseMatrix(), p_thresh_list2, n_thresh_num2);
				// initialize an object to calculate Sigma_d, in the original dimension (in 3D, that's 6D)

				const typename CSystemType::_TyVertexMultiPool &r_vertex_pool = system.r_Vertex_Pool();
				const size_t n_last_vertex = r_vertex_pool.n_Size() - 1;
				// id of the last vertex

				if(!p.b_do_thresh_analysis && !p.b_do_thresh_sensitivity_analysis) { // compact pose SLAM
					std::vector<TInfoGain_Distance> gain_distance_list(r_vertex_pool.n_Size() - 1);
					// allocate a list of information gains and distances

					_ASSERTE(n_last_vertex <= INT_MAX);
					int _n_last_vertex = int(n_last_vertex);
					#pragma omp parallel for schedule(dynamic, 32) if(n_last_vertex > 64)
					for(int j = 0; j < _n_last_vertex; ++ j) {
						_TyDistanceVector v_distance;
#if 0
						_TyDistanceMatrix t_Sigma_d;
						distances.Calculate_Distance_Sigma_d(v_distance, t_Sigma_d, j, n_last_vertex); // compact pose needs sigma d as well
						double f_info_gain = distances.f_Information_Gain(t_Sigma_bar/*_TyDistanceMatrix::Identity()*/, t_Sigma_d); // won't always work, the dimension might be wrong
#else
						distances.Calculate_Distance(v_distance, j, n_last_vertex);
						_TyMatrix t_Sigma_d;
						distances_bare.Calculate_Sigma_d(t_Sigma_d, j, n_last_vertex); // compact pose needs sigma d as well
						double f_info_gain = distances_bare.f_Information_Gain(t_Sigma_bar, t_Sigma_d);
						// a different way of calculating the information gain
#endif
						double f_min_probability_distance = v_distance.minCoeff(); // minimal of all probabilities
						// calculate probability that the vertices are closer than the specified distance threshold
						// calculates distance between vertices j and n_last_vertex

						if(_isnan(f_info_gain)) { // todo - remove this, does not happen anymore
							//CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, dist_sig.second, "Sigma_d");
							f_info_gain = -1e100;
						}
						// calculate the assumed information gain of the edge

						gain_distance_list[j] = TInfoGain_Distance(f_info_gain,
							f_min_probability_distance, j);
					}
					// calculate a list of information gains and distances

					timer.Accum_DiffSample(f_loop_candidates_time);

					std::sort(gain_distance_list.begin(), gain_distance_list.end());
					// sort the list by information gain

					timer.Accum_DiffSample(f_candidates_sort_time);

					for(size_t j = 0; j < n_last_vertex; ++ j) {
						size_t n_loop_vertex = gain_distance_list[j].n_vertex;
						double f_min_probability_distance = gain_distance_list[j].f_distance;
						double f_gain_prior = gain_distance_list[j].f_gain;
						// read the id and distance

						if(f_min_probability_distance >= p.f_loop_probability_dist_threshold)
							++ n_distance_probability_match_num;
						// count those as well

						if(f_min_probability_distance >= p.f_loop_probability_dist_threshold &&
						   f_gain_prior >= p.f_loop_closure_gain_threshold) { // greater or equal so that the threshold can be zero
							++ n_association_num;
							// will attempt sensor matching on poses (n_loop_vertex, n_last_vertex)

							_TyMatrix t_Sigma_d;
							distances_bare.Calculate_Sigma_d(t_Sigma_d, n_loop_vertex, n_last_vertex); // compact pose needs sigma d as well
							// the second calculation of distance to get the 6x6 covariance matrix (the distance itself not needed)

							bool b_matched = false;
							typename std::vector<_TyEdgeData>::iterator p_edge_it;
							while((p_edge_it = std::find(loop_closures.begin(), loop_closures.end(),
							   std::make_pair(n_loop_vertex, n_last_vertex))) != loop_closures.end()) {
								const _TyEdgeData &r_edge = *p_edge_it;
								// found an edge between vertices n_loop_vertex and n_last_vertex

								_TyMatrix t_Sigma_e = r_edge.t_information.inverse();
								double f_info_gain = distances_bare.f_Information_Gain(t_Sigma_e, t_Sigma_d);
								// calculate the real information gain, based on the sensor matching

								if(_isnan(f_info_gain)) {
									//CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, dist_sig.second, "Sigma_d"); // debug
									f_info_gain = -1e100;
								}

								if(f_info_gain >= p.f_loop_closure_gain_threshold) { // greater or equal
									if(f_gain_prior < p.f_loop_closure_gain_threshold)
										fprintf(stderr, "error: would reject a loop closure\n"); // see if this is a problem
									timer.Accum_DiffSample(f_compact_pose_time);

									loop_gains.push_back(f_info_gain);
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
										_TyEdgePose(r_edge.p_vertex[0], r_edge.p_vertex[1],
										r_edge.v_measurement, r_edge.t_information, system)));
									// add it to the system

									timer.Accum_DiffSample(f_solver_time);

									b_matched = true;
								} else
									++ n_low_gain_loop_num;
								// in case the loop closure is informative enough, add it to the system

								loop_closures.erase(p_edge_it);
								// remove it from the loop closure list (if matched or if not - this pose will not come back)
							}
							// perform fake sensor matching on poses (n_loop_vertex, n_last_vertex)
							// could close multiple loops from multiple sensors in this step

							if(b_matched) {
								if(f_min_probability > f_min_probability_distance)
									f_min_probability = f_min_probability_distance;
								// see what the threshold needs to be

								true_matches.push_back(n_loop_vertex);
								// remember matches

								timer.Accum_DiffSample(f_compact_pose_time);

								solver.Enable_Optimization();
								// if we added any edges, optimize now

								solver.Optimize(0, 0);
								// force update of the marginals (no new vertices added so the solver
								// wouldnt update them by itself)

#ifdef _DEBUG
								if(!solver.Check_Marginals())
									fprintf(stderr, "error: marginals incorrect\n");
#endif // _DEBUG

								timer.Accum_DiffSample(f_solver_time);

								#pragma omp parallel for schedule(dynamic, 32) if(n_last_vertex - (j + 1) > 64)
								for(int k = int(j + 1); k < _n_last_vertex; ++ k) {
									size_t n_vertex_id = gain_distance_list[k].n_vertex;
									// see which vertex is remaining in the list

									_TyDistanceVector v_distance;
									_TyDistanceMatrix t_Sigma_d;
									distances.Calculate_Distance_Sigma_d(v_distance, t_Sigma_d, n_vertex_id, n_last_vertex); // compact pose needs sigma d as well
									double f_min_probability_distance = v_distance.minCoeff(); // minimal of all probabilities
									// calculate probability that the vertices are closer than the specified distance threshold
									// calculates distance between vertices k and n_last_vertex

									double f_info_gain = distances.f_Information_Gain(_TyDistanceMatrix::Identity(), t_Sigma_d);
									// calculate information gain of the edge

									if(_isnan(f_info_gain)) {
										//CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, dist_sig.second, "Sigma_d"); // debug
										f_info_gain = -1e100;
									}

									gain_distance_list[k] = TInfoGain_Distance(f_info_gain, f_min_probability_distance, n_vertex_id);
									// update the information gain and distance of that vertex
								}
								// calculate a list of information gains and distances for the remaining vertices

								timer.Accum_DiffSample(f_loop_candidates_update_time);

								std::sort(gain_distance_list.begin() + (j + 1), gain_distance_list.end());
								// sort the suffix of the list by information gain

								timer.Accum_DiffSample(f_candidates_sort_update_time);
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

						if(f_min_gain != -1e100)
							pose_gains.push_back(f_min_gain);
						if(f_min_pose_gain > f_min_gain && f_min_gain != -1e100)
							f_min_pose_gain = f_min_gain;

						if(f_min_gain >= p.f_keep_pose_gain_threshold) {
							b_low_gain_edge = false;
							// if any of the loop closures is under low gain then this edge is low gain
							// (the algorithm in the paper actually takes a minimum of the gains of all
							// the proposed loop closures and compares that to the threshold, which is
							// equivalent to doing this)
						}
						// compare the information gain
					}
					// see if the smallest information gain was too small
				} else {
					// do threshold analysis

					b_low_gain_edge = false; // want to keep all poses

					if(p.b_do_thresh_sensitivity_analysis) {
						for(size_t j = 0, m = std::max(size_t(1), n_last_vertex) - 1; j < m; ++ j) { // all possible edges except odometry
							_TyDistanceVector v_distance;
							distances.Calculate_Distance(v_distance, j, n_last_vertex);
							all_to_all_distance_info.push_back(v_distance.minCoeff());
						}
					}
					// generate distances to all possible loop closure candidates (includes the loop closures)

					bool b_matched = false;
					for(size_t j = 0, m = loop_closures.size(); j < m; ++ j) {
						const _TyEdgeData &r_edge = loop_closures[j];
						size_t n_loop_vertex;
						if(r_edge.p_vertex[0] == n_last_vertex && r_edge.p_vertex[1] < r_edge.p_vertex[0])
							n_loop_vertex = r_edge.p_vertex[1];
						else if(r_edge.p_vertex[1] == n_last_vertex && r_edge.p_vertex[0] < r_edge.p_vertex[1])
							n_loop_vertex = r_edge.p_vertex[0];
						else
							continue;
						// found an edge between vertices n_loop_vertex and n_last_vertex
						// where n_loop_vertex is already in the system, so that this loop can be closed now

						_TyDistanceVector v_mean_d;
						_TyDistanceMatrix t_Sigma_d; // transformed
						_TyMatrix t_Sigma_d_raw; // not transformed
						distances.Calculate_Mean_d_Sigma_d(v_mean_d, t_Sigma_d, n_loop_vertex, n_last_vertex);
						distances_bare.Calculate_Sigma_d(t_Sigma_d_raw, n_loop_vertex, n_last_vertex);
						// get mean_d and Sigma_d

						_TyMatrix t_Sigma_e = r_edge.t_information.inverse();
						// get Sigma_e

						TLoopInfo t_loop_info(r_edge.p_vertex[0], r_edge.p_vertex[1], t_Sigma_e, v_mean_d, t_Sigma_d, t_Sigma_d_raw);
						true_loop_distance_info.push_back(t_loop_info);
						// get loop info, store it

						/*if(f_min_loop_gain > t_loop_info.f_InfoGain())
							f_min_loop_gain = t_loop_info.f_InfoGain();*/ // this is only approximate, would have to close the loops in the order of decreasing information gain and then updating the gain. lets run quickly here.
						// remember which gain the loops had

						timer.Accum_DiffSample(f_compact_pose_time);

						if(!b_matched)
							solver.Delay_Optimization();
						// if adding multiple loops (but between the same two vertices - e.g. from
						// multiple sensors) we dont need to optimize after each. just optimize
						// quite at the end

						++ n_closed_loop_num;
						solver.Incremental_Step(system.r_Add_Edge(
							_TyEdgePose(r_edge.p_vertex[0], r_edge.p_vertex[1],
							r_edge.v_measurement, r_edge.t_information, system)));
						// add it to the system

						timer.Accum_DiffSample(f_solver_time);

						b_matched = true;

						loop_closures.erase(loop_closures.begin() + j);
						-- j;
						-- m;
						// remove it from the loop closure list (if matched or if not - this pose will not come back)
					}
					// close all the loop closures with this vertex

					if(b_matched) {
						timer.Accum_DiffSample(f_compact_pose_time);

						solver.Enable_Optimization();

						solver.Optimize(0, 0);
						// force update of the marginals (no new vertices added so the solver
						// wouldnt update them by itself)

#ifdef _DEBUG
						if(!solver.Check_Marginals())
							fprintf(stderr, "error: marginals incorrect\n");
#endif // _DEBUG

						timer.Accum_DiffSample(f_solver_time);
					}
					// if we added any edges, optimize now

					/*double f_min_gain;
					size_t n_min_gain_vertex = n_last_vertex - 1;
					for(size_t j = 0; j < n_last_vertex; ++ j) {
						_TyDistanceMatrix t_Sigma_d;
						distances.Calculate_Sigma_d(t_Sigma_d, j, n_last_vertex); // compact pose needs sigma d as well
						double f_info_gain = distances.f_Information_Gain(_TyDistanceMatrix::Identity(), t_Sigma_d);
						if(_isnan(f_info_gain)) {
							//CDebug::Print_DenseMatrix_in_MatlabFormat(stdout, dist_sig.second, "Sigma_d");
							f_info_gain = -1e100;
						}
						// calculate the assumed information gain of the edge

						if(!j || f_min_gain > f_info_gain) {
							f_min_gain = f_info_gain;
							n_min_gain_vertex = j;
						}
					}
					// calculate the minimum information gain of al the possible loops, after the optimization

					//if(n_min_gain_vertex != n_last_vertex - 1)
					//	fprintf(stderr, "warning: notable: the second last vertex was not the minimum gain vertex\n");
					// if calculated before closing the loops, the second last vertex is the minimum gain one. otherwise,
					// the vertex with which a loop was closed may occasionaly have a lower information gain

					if(f_min_pose_gain > f_min_gain)
						f_min_pose_gain = f_min_gain;*/ // this is only approximate
					// keep track of this
				}
			}
			// find probable matches using the covariances

			bool b_closed_loops = !true_matches.empty();
			// see if we closed any loops

			b_keep_pose = b_closed_loops || !b_low_gain_edge;
			// decide whether to keep the pose

			timer.Accum_DiffSample(f_compact_pose_time);

			if(!b_keep_pose) {
				// this pose did not close any loops and does not have enough information
				// gain. we will not keep it and instead we will extend the edge a bit more.

				n_discarded_loop_num += loop_closures.size();
				size_t n_remove_vertex = system.r_Vertex_Pool().n_Size() - 1; // this one will be left out
				if(!Dataset_RemovePose(n_remove_vertex, edges, loop_closures, i)) {
					fprintf(stderr, "error: the graph does not contain good odometry edges, can't continue\n");
					return -1;
				}
				n_discarded_loop_num -= loop_closures.size(); // track the number of removed loop closures
				n = edges.size(); // adjust the outer loop limit, it may have changed!
				// this actually affects indexing of the graph: the next vertex is not added
				// and so we need to modify the vertex indices. if this was an online scenario
				// we wouldnt have to worry about this bit, since we would calculate the
				// indices on the fly.
			}
			// do compact pose SLAM

			timer.Accum_DiffSample(f_graph_mod_time);

			double f_covariance_norm = -1, f_marginals_norm = -1;
			if(p.b_write_cov_norms) {
				Calculate_CovarianceNorms(f_covariance_norm, f_marginals_norm,
					solver, compact_edge_lengths, ref_compact_edge_lengths);
				timer.Accum_DiffSample(f_cov_norm_time);
			}
			// keep it separate for timing

			if(p.b_write_animdata && p_fw_animdata)
				Write_AnimationData(p_fw_animdata, system, last_sigma, probable_matches, true_matches);
			if(p.b_write_covmats) {
				char p_s_filename[256];
				sprintf(p_s_filename, "anim/covmat_%04" _PRIsize ".tga", i);
				solver.r_MarginalCovariance().r_SparseMatrix().Rasterize(p_s_filename);
			}
			if(p.b_write_bitmaps && !p.b_disable_incremental_bitmaps) {
				Draw_SolutionFrame(system, i, last_sigma, probable_matches, true_matches, p_image, v_off,
					v_scale, p.v_min, p.v_max, p.b_bitmaps_xz_plane, p.b_bitmaps_highlight_loops);
			}
			if(p.b_write_cov_norms && p_fw_cov_norms)
				fprintf(p_fw_cov_norms, "%.10g,%.10g\n", f_covariance_norm, f_marginals_norm);
			if(p.b_write_cumtime && p_cumtime_fw) {
				timer.Accum_DiffSample(f_drawing_time);
				fprintf(p_cumtime_fw, "%.10f,%.10f,%.10f,%.10f,%.10f," PRIsize
					"," PRIsize "," PRIsize "," PRIsize "," PRIsize ",%.10f,%.10f,%.10f,%.10f,%.10f\n",
					f_solver_time, f_compact_pose_time + (f_loop_candidates_time +
					f_candidates_sort_time + f_loop_candidates_update_time +
					f_candidates_sort_update_time), f_drawing_time, f_graph_mod_time,
					f_cov_norm_time, system.r_Vertex_Pool().n_Size(), system.r_Edge_Pool().n_Size(),
					n_closed_loop_num, n_distance_probability_match_num, n_association_num,
					f_loop_candidates_time, f_candidates_sort_time, f_loop_candidates_update_time,
					f_candidates_sort_update_time, solver.f_Marginals_CumTime());
			}
			if(p.b_write_mem_usage && p_musage_fw) {
				fprintf(p_musage_fw, PRIsize "," PRIsize "," PRIsize "\n",
					size_t(CProcessMemInfo::n_MemoryUsage()), // clang on OS X has problem with %llu now
					solver.r_Lambda().n_NonZero_Num(), solver.r_R().n_NonZero_Num());
			}
			// write any requested debug data

			timer.Accum_DiffSample(f_drawing_time);
		}
		// put edges in the system, the vertices are created and initialized

		if(p.b_write_cov_norms && p_fw_cov_norms) {
			if(ferror(p_fw_cov_norms))
				fprintf(stderr, "error: i/o error while writing \'%s\'\n", (ref_compact_edge_lengths.empty())? "cov_norms.csv" : "cov_norms_marginalized.csv");
			fclose(p_fw_cov_norms);
		}
		// close the covariance norm info

		if(p.b_write_cumtime && p_cumtime_fw) {
			if(ferror(p_cumtime_fw))
				fprintf(stderr, "error: i/o error while writing \'%s\'\n", "cumtime.csv");
			fclose(p_cumtime_fw);
		}
		// close the file with timing info

		if(p.b_write_mem_usage && p_musage_fw) {
			if(ferror(p_musage_fw))
				fprintf(stderr, "error: i/o error while writing \'%s\'\n", "musage.csv");
			fclose(p_musage_fw);
		}
		// close the file with memory use info

		if(p.b_write_animdata && p_fw_animdata)
			Finalize_AnimData(p_fw_animdata);
		// finalize the covariance data stream

		if(p.b_write_compact_edge_lengths) {
			FILE *p_fw;
			if(!(p_fw = fopen("compact_edges.dat", "wb")))
				fprintf(stderr, "error: failed to open \'%s\' for writing\n", "compact_edges.dat");
			else {
				if(!Write_Vector32(p_fw, compact_edge_lengths))
					fprintf(stderr, "error: i/o error while writing \'%s\'\n", "compact_edges.dat");
				fclose(p_fw);
			}
		}
		// write the compact edge lengths data

		size_t n_discarded_pose_num = n_pose_num - system.r_Vertex_Pool().n_Size();
		// keep a counter of how many poses the compact pose slam discarded and how many loops we lost to it (statistics)

		printf("\ndone. the dataset has " PRIsize " vertices and " PRIsize " edges, out of which "
			PRIsize " are loop closures\n", n_pose_num, n_total_edge_num, n_loop_closure_num);
		printf("correctly matched " PRIsize " / " PRIsize " loop closures (tried "
			PRIsize " times)\n", n_closed_loop_num, n_loop_closure_num, n_association_num);
		printf("the minimal successful probability that closed a loop was %g\n", f_min_probability);
		std::sort(pose_gains.begin(), pose_gains.end());
		std::sort(loop_gains.begin(), loop_gains.end());
		printf("the minimal loop closure information gain was %g (max %g, med %g)\n", f_min_loop_gain,
			(loop_gains.empty())? -1 : loop_gains.back(), (loop_gains.empty())? -1 : loop_gains[loop_gains.size() / 2]);
		printf("the minimal pose information gain was %g (max %g, med %g)\n", f_min_pose_gain,
 			(pose_gains.empty())? -1 : pose_gains.back(), (pose_gains.empty())? -1 : pose_gains[pose_gains.size() / 2]);
		for(int n_pass = 0; n_pass < 2; ++ n_pass) {
			const char *p_s_filename = (!n_pass)? "pose_gains.csv" : "loop_gains.csv";
			FILE *p_fw;
			if((p_fw = fopen(p_s_filename, "w"))) {
				fprintf(p_fw, "percent,info gain\n");
				for(size_t i = 0, n = pose_gains.size(); i < n; ++ i)
					fprintf(p_fw, "%f,%g\n", float(i) / (n - 1) * 100, pose_gains[i]);
				if(ferror(p_fw))
					fprintf(stderr, "error: failed to write \'%s\'\n", p_s_filename);
				fclose(p_fw);
			}
			std::swap(pose_gains, loop_gains);
		}
		//printf("the minimal loop closure information gain was %g\n", f_min_loop_gain);
		//printf("the minimal odometry edge information gain was %g\n", f_min_pose_gain);
		printf("the compact pose SLAM algorithm discarded " PRIsize " / " PRIsize " poses (lost " PRIsize
			" loop closures to that, lost " PRIsize " loops to gain test)\n", n_discarded_pose_num,
			n_pose_num, n_discarded_loop_num, n_low_gain_loop_num);
		printf("solver took %f sec\n", f_solver_time);
		printf("compact pose SLAM algorithm took %f sec\n", f_compact_pose_time +
			(f_loop_candidates_time + f_candidates_sort_time +
			f_loop_candidates_update_time + f_candidates_sort_update_time));
		printf("out of that:\n    loop candidate computation took %f sec\n", f_loop_candidates_time);
		printf("    loop candidate sorting took %f sec\n", f_candidates_sort_time);
		printf("    loop candidate update computation took %f sec\n", f_loop_candidates_update_time);
		printf("    loop candidate sorting update took %f sec\n", f_candidates_sort_update_time);
		printf("graph filtering took %f sec\n", f_graph_mod_time);
		if(p.b_write_cov_norms)
			printf("covariance norm calculation took %f sec\n", f_cov_norm_time);
		/*if(p.b_bitmaps_xz_plane)
			system.Plot3D("result.tga", plot_quality::plot_Printing); // plot in print quality
		else
			system.Plot2D("result.tga", plot_quality::plot_Printing);*/ // plot in print quality
		Draw_FinalFrame(system, p_image, v_off, v_scale, p.v_min, p.v_max,
			p.b_bitmaps_xz_plane, p.b_bitmaps_highlight_loops, p.b_disable_incremental_bitmaps); // draw the final frame *before* writing bbox info otherwise it will not update when -nib is specified!!
		if(p.b_write_bitmaps) {
			printf("animation rasterization took %f sec\n", f_drawing_time);
			{
				Eigen::Vector2d &r_v_min = p.v_min, &r_v_max = p.v_max;
				int n_resolution = p.n_resolution;

				Eigen::Vector2d v_size = r_v_max - r_v_min;
				double f_short_side = std::min(v_size(0), v_size(1));
				double f_scale = n_resolution / f_short_side; // pixels = logical * f_scale
				double f_10px = 10 / f_scale; // 10 = logical * f_scale, 10 / f_scale = logical
				r_v_min.array() -= f_10px;
				r_v_max.array() += f_10px;
			}
			printf("bounding rectangle is [%g, %g] - [%g, %g]\n",
				p.v_min(0), p.v_min(1), p.v_max(0), p.v_max(1));
			printf("use -bb \"[%g %g] [%g %g]\"\n",
				p.v_min(0), p.v_min(1), p.v_max(0), p.v_max(1));
		}
		// print stats

		if(p.p_s_gt_file || p.p_s_ref_file) {
			printf("\n");
			Eigen::Vector6d error;
			if(!ComputeError(compact_edge_lengths, measurements, system.r_Vertex_Pool(),
			   p.p_s_gt_file, p.p_s_ref_file, error, system, p_image, v_off, v_scale, p.b_bitmaps_xz_plane))
				fprintf(stderr, "error: failed to compute solution error\n");
		}
		// evaluate error

		{
			solver.Optimize(0, 0); // make sure that R is up to date
			printf("\n");
			double f_covariance_norm = -1, f_marginals_norm = -1;
			Calculate_CovarianceNorms(f_covariance_norm, f_marginals_norm,
				solver, compact_edge_lengths, std::vector<size_t>()); // run with an empty vector
			printf("norm of covariance is %.15g, norm of marginals is %.15g\n", f_covariance_norm, f_marginals_norm);
			if(!ref_compact_edge_lengths.empty()) {
				Calculate_CovarianceNorms(f_covariance_norm, f_marginals_norm,
					solver, compact_edge_lengths, ref_compact_edge_lengths);
				printf("norm of marginalized covariance is %.15g, norm of "
					"marginalized marginals is %.15g\n", f_covariance_norm, f_marginals_norm);
			}
			// print
		}
		// calculate the norm of covariances at the end

		system.Dump("vertices.txt");
		// dump vertices

		printf("\n");
		//solver.Dump(f_solver_time); // show some stats
		//solver.Dump_SystemMatrix("lambda.tga"); // debug

		if(p_image)
			p_image->Delete();
		// delete the bitmap

		if(!true_loop_distance_info.empty()) {
			if(p.b_do_thresh_sensitivity_analysis) {
				FILE *p_fw = fopen("thresh_sensitivity.csv", "w");
				fprintf(p_fw, "Threshold,Closed Loops,False Positives\n");
				for(double f_thresh = 0; f_thresh <= 1; f_thresh += 1.0 / 256) { // use power of two so that it hits 1.0 exactly
					size_t n_closed_loop_num = 0;
					for(size_t i = 0, n = true_loop_distance_info.size(); i < n; ++ i) {
						if(true_loop_distance_info[i].f_DistanceProba(p.v_sensor_range) >= f_thresh)
							++ n_closed_loop_num;
					}
					// count the closed loops

					size_t n_false_positive_num = 0;
					for(size_t i = 0, n = all_to_all_distance_info.size(); i < n; ++ i) {
						if(all_to_all_distance_info[i] >= f_thresh)
							++ n_false_positive_num;
					}
					_ASSERTE(n_false_positive_num >= n_closed_loop_num);
					n_false_positive_num -= n_closed_loop_num;
					// count all the loop candidates that would pass the distance test (ignoring the information gain test)

					fprintf(p_fw, "%g," PRIsize "," PRIsize "\n", f_thresh, n_closed_loop_num, n_false_positive_num);
				}
				if(ferror(p_fw))
					fprintf(stderr, "error: i/o error while writing \'%s\'\n", "thresh_sensitivity.csv");
				fclose(p_fw);
				// generate a threshold sensitivity plot
			} else {
				printf("enter sensor range threshold or press ctrl+c to exit\n");
				printf("sensor range threshold in the config is");
				for(size_t i = 0, n = p.v_sensor_range.rows(); i < n; ++ i)
					printf(" %g", p.v_sensor_range(i));
				printf("\n");
				for(;;) {
					printf("sensor range threshold: ");
					fflush(stdout); // if running in terminal, want to see what it says
					_TyDistanceVector v_thresh;
					bool b_fail = false;
					for(size_t i = 0, n = v_thresh.rows(); i < n; ++ i) {
						if(scanf("%lf", &v_thresh(i)) != 1)
							b_fail = true;
					}
					if(b_fail) {
						fprintf(stderr, "error: invalid input. try again ...\n");
						for(int c; (c = getchar()) != '\n' && c != EOF;); // throw away the rest of the line
						//fflush(stdin); // throw away the bad digits (not portable)
						fflush(stderr); // if running in terminal, want to see what it says
						continue;
					}
					// get a sensor range threshold

					std::vector<double> loop_probabilities(true_loop_distance_info.size());
					for(size_t i = 0, n = true_loop_distance_info.size(); i < n; ++ i)
						loop_probabilities[i] = true_loop_distance_info[i].f_DistanceProba(v_thresh);
					std::sort(loop_probabilities.begin(), loop_probabilities.end());
					double f_min_probability = loop_probabilities.front();
					double f_95p_probability = loop_probabilities[size_t(loop_probabilities.size() * .01)];
					// calculate minimum probability

					for(size_t i = 0, n = v_thresh.rows(); i < n; ++ i)
						printf(" %g" + !i, v_thresh(i));
					printf("\nthe min distance probability to close all the loops is %g, 99-percentile is %g\n", f_min_probability, f_95p_probability);
					fflush(stdout); // if running in terminal, want to see what it says
					// report
				}
			}
		}
		// interactive mode

		return 0;
	}

	/**
	 *	@brief converts a pose from relative measurement to absolute measurement (overload for 2D)
	 *
	 *	@param[in] r_absolute is the first pose, in absolute coordinates
	 *	@param[in] r_relative is the second pose, relative to the first one
	 *	@param[out] r_dest is filled with absolute coordinates of the second pose
	 */
	static inline void Relative_to_Absolute(const Eigen::Vector3d &r_absolute,
		const Eigen::Vector3d &r_relative, Eigen::Vector3d &r_dest)
	{
		C2DJacobians::Relative_to_Absolute(r_absolute, r_relative, r_dest);
	}

	/**
	 *	@brief converts a pose from relative measurement to absolute measurement (overload for 2D)
	 *
	 *	@param[in] r_absolute is the first pose, in absolute coordinates
	 *	@param[in] r_relative is the second pose, relative to the first one
	 *	@param[out] r_dest is filled with absolute coordinates of the second pose
	 *	@param[out] r_J0 is filled with Jacobian of the result with respect to the first pose
	 *	@param[out] r_J1 is filled with Jacobian of the result with respect to the second pose
	 */
	static inline void Relative_to_Absolute(const Eigen::Vector3d &r_absolute,
		const Eigen::Vector3d &r_relative, Eigen::Vector3d &r_dest,
		Eigen::Matrix3d &r_J0, Eigen::Matrix3d &r_J1)
	{
		C2DJacobians::Relative_to_Absolute(r_absolute, r_relative, r_dest, r_J0, r_J1);
	}

	/**
	 *	@brief converts a pose from absolute measurement to relative measurement (overload for 2D)
	 *
	 *	@param[in] r_absolute0 is the first pose, in absolute coordinates
	 *	@param[in] r_absolute1 is the second pose, in absolute coordinates
	 *	@param[out] r_dest is filled with relative coordinates of the second pose
	 *	@param[out] r_J0 is filled with Jacobian of the result with respect to the first pose
	 *	@param[out] r_J1 is filled with Jacobian of the result with respect to the second pose
	 */
	static inline void Absolute_to_Relative(const Eigen::Vector3d &r_absolute,
		const Eigen::Vector3d &r_relative, Eigen::Vector3d &r_dest,
		Eigen::Matrix3d &r_J0, Eigen::Matrix3d &r_J1)
	{
		C2DJacobians::Absolute_to_Relative(r_absolute, r_relative, r_dest, r_J0, r_J1);
	}

	/**
	 *	@brief converts a pose from relative measurement to absolute measurement (overload for 3D)
	 *
	 *	@param[in] r_absolute is the first pose, in absolute coordinates
	 *	@param[in] r_relative is the second pose, relative to the first one
	 *	@param[out] r_dest is filled with absolute coordinates of the second pose
	 */
	static inline void Relative_to_Absolute(const Eigen::Vector6d &r_absolute,
		const Eigen::Vector6d &r_relative, Eigen::Vector6d &r_dest)
	{
		C3DJacobians::Relative_to_Absolute(r_absolute, r_relative, r_dest);
	}

	/**
	 *	@brief converts a pose from relative measurement to absolute measurement (overload for 3D)
	 *
	 *	@param[in] r_absolute is the first pose, in absolute coordinates
	 *	@param[in] r_relative is the second pose, relative to the first one
	 *	@param[out] r_dest is filled with absolute coordinates of the second pose
	 *	@param[out] r_J0 is filled with Jacobian of the result with respect to the first pose
	 *	@param[out] r_J1 is filled with Jacobian of the result with respect to the second pose
	 */
	static inline void Relative_to_Absolute(const Eigen::Vector6d &r_absolute,
		const Eigen::Vector6d &r_relative, Eigen::Vector6d &r_dest,
		Eigen::Matrix6d &r_J0, Eigen::Matrix6d &r_J1)
	{
		C3DJacobians::Relative_to_Absolute(r_absolute, r_relative, r_dest, r_J0, r_J1);
	}

	/**
	 *	@brief converts a pose from absolute measurement to relative measurement (overload for 3D)
	 *
	 *	@param[in] r_absolute0 is the first pose, in absolute coordinates
	 *	@param[in] r_absolute1 is the second pose, in absolute coordinates
	 *	@param[out] r_dest is filled with relative coordinates of the second pose
	 *	@param[out] r_J0 is filled with Jacobian of the result with respect to the first pose
	 *	@param[out] r_J1 is filled with Jacobian of the result with respect to the second pose
	 */
	static inline void Absolute_to_Relative(const Eigen::Vector6d &r_absolute,
		const Eigen::Vector6d &r_relative, Eigen::Vector6d &r_dest,
		Eigen::Matrix6d &r_J0, Eigen::Matrix6d &r_J1)
	{
		C3DJacobians::Absolute_to_Relative(r_absolute, r_relative, r_dest, r_J0, r_J1);
	}

	/**
	 *	@brief converts a pose from absolute measurement to relative measurement (overload for 2D)
	 *
	 *	@param[in] r_absolute0 is the first pose, in absolute coordinates
	 *	@param[in] r_absolute1 is the second pose, in absolute coordinates
	 *	@param[out] r_dest is filled with relative coordinates of the second pose
	 */
	static inline void Absolute_to_Relative(const Eigen::Vector3d &r_absolute0,
		const Eigen::Vector3d &r_absolute1, Eigen::Vector3d &r_dest)
	{
		C2DJacobians::Absolute_to_Relative(r_absolute0, r_absolute1, r_dest);
	}

	/**
	 *	@brief converts a pose from absolute measurement to relative measurement (overload for 3D)
	 *
	 *	@param[in] r_absolute0 is the first pose, in absolute coordinates
	 *	@param[in] r_absolute1 is the second pose, in absolute coordinates
	 *	@param[out] r_dest is filled with relative coordinates of the second pose
	 */
	static inline void Absolute_to_Relative(const Eigen::Vector6d &r_absolute0,
		const Eigen::Vector6d &r_absolute1, Eigen::Vector6d &r_dest)
	{
		C3DJacobians::Absolute_to_Relative(r_absolute0, r_absolute1, r_dest);
	}

	/**
	 *	@brief calculates norms of covariances
	 *
	 *	@param[out] r_f_covariance_norm is filled with the norm of covariances
	 *	@param[out] r_f_marginals_norm is filled with the norm of marginal covariances
	 *	@param[in] r_solver is reference to the current state of the nonlinear solver
	 *	@param[in] r_compact_edge_lengths is list of compact edge lengths at the current step
	 *	@param[in] r_ref_compact_edge_lengths is list of compact edge lengths at the end of
	 *		a reference solution for marginalization, or an empty vector for full covariances
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Calculate_CovarianceNorms(double &r_f_covariance_norm,
		double &r_f_marginals_norm, CNonlinearSolverType &r_solver, // solver not const because of Check_Marginals()
		const std::vector<size_t> &r_compact_edge_lengths,
		const std::vector<size_t> &r_ref_compact_edge_lengths) // throw(std::bad_alloc)
	{
#ifdef _DEBUG
		if(!r_solver.Check_Marginals())
			fprintf(stderr, "error: marginals incorrect\n");
#endif // _DEBUG
		// make sure everything is up to date (all the loop closures are in place, so it should be)

		CUberBlockMatrix schur_complement;
		if(!r_ref_compact_edge_lengths.empty()) {
#ifdef _DEBUG
			std::vector<size_t> schur_ordering_dbg;
			schur_ordering_dbg.push_back(0); // zero always
			{
				std::vector<size_t> estimated_variables; // ids in the original graph
				estimated_variables.push_back(0);
				for(size_t i = 0, n = r_compact_edge_lengths.size(); i < n; ++ i)
					estimated_variables.push_back(estimated_variables.back() + r_compact_edge_lengths[i]);
				size_t n_last_estimated_variable = estimated_variables.back();
				// reconstruct list of estimated variables

				bool b_all_ones = !estimated_variables.empty();
				for(size_t i = 0, n = r_compact_edge_lengths.size(); i < n; ++ i) {
					if(r_compact_edge_lengths[i] != 1) {
						b_all_ones = false;
						break;
					}
				}

				std::vector<size_t> compared_variables; // ids in the original graph
				compared_variables.push_back(0);
				for(size_t i = 0, n = r_ref_compact_edge_lengths.size(); i < n; ++ i) {
					size_t n_next_variable = compared_variables.back() + r_ref_compact_edge_lengths[i];
					if(n_next_variable > n_last_estimated_variable)
						break; // not interested in more, for now
					compared_variables.push_back(n_next_variable);

					std::vector<size_t>::iterator p_var_it = std::lower_bound(estimated_variables.begin(),
						estimated_variables.end(), n_next_variable);
					if(p_var_it == estimated_variables.end() || *p_var_it != n_next_variable) {
						fprintf(stderr, "warning: some variable(s) to be compared were not estimated in"
							" this run; marginalized covariance norms will be imprecise");
						// this system is missing a variable which the other system had
					} else {
						size_t n_variable_index = p_var_it - estimated_variables.begin();
						schur_ordering_dbg.push_back(n_variable_index);
						// this is a variable that we'll keep
					}
				}
				// reconstruct list of variables that were estimated in the system we are comparing to

				_ASSERTE(!b_all_ones || schur_ordering_dbg == compared_variables);
				// in case all the variables are estimated, then the ordering should contain all the variables from the compared graph
			}
			// calculate reference ordering, only used for double checking
#endif // _DEBUG

			std::vector<size_t> schur_ordering;
			schur_ordering.push_back(0); // vertex zero is always estimated in both systems
			if(!r_compact_edge_lengths.empty() && !r_ref_compact_edge_lengths.empty()) {
				size_t n_estimated_var = r_compact_edge_lengths.front(), n_ref_var = r_ref_compact_edge_lengths.front(), i = 0;
				for(size_t j = 0, n = r_compact_edge_lengths.size(), m = r_ref_compact_edge_lengths.size();;) {
					if(n_estimated_var > n_ref_var) {
						if(++ j == m)
							break;
						n_ref_var += r_ref_compact_edge_lengths[j];
						fprintf(stderr, "warning: some variable(s) to be compared were not estimated in"
							" this run; marginalized covariance norms will be imprecise");
					} else if(n_estimated_var < n_ref_var) {
						if(++ i == n)
							break;
						n_estimated_var += r_compact_edge_lengths[i];
						// we could insert this variable to the end of the ordering (to be marginalized out) and avoid the postprocessing loop
					} else {
						schur_ordering.push_back(i + 1); // insert index (i) in this system instead of the index in the graph (n_estimated_var)
						if(++ i == n || ++ j == m)
							break;
						n_estimated_var += r_compact_edge_lengths[i];
						n_ref_var += r_ref_compact_edge_lengths[j];
					}
				}
			}
			// calculate the same ordering while using less memory (ordered merge on cumsums, could be
			// implemented using std::set_intersection() and some custom iterator trickery)

#ifdef _DEBUG
			_ASSERTE(schur_ordering == schur_ordering_dbg);
			if(schur_ordering != schur_ordering_dbg) // make sure this is the same on all the datasets when we run in release
				fprintf(stderr, "warning: schur ordering is dubious; marginalized covariance norms will be imprecise");
			// should end up with the same ordering
#endif // _DEBUG

			const CUberBlockMatrix &r_lambda = r_solver.r_Lambda();

			const size_t n_schur_variable_num = schur_ordering.size();
			for(size_t i = 1; i < n_schur_variable_num; ++ i) {
				for(size_t j = schur_ordering[i - 1] + 1, m = schur_ordering[i]; j < m; ++ j)
					schur_ordering.push_back(j);
			}
			for(size_t j = schur_ordering[n_schur_variable_num - 1] + 1, m = r_lambda.n_BlockColumn_Num(); j < m; ++ j)
				schur_ordering.push_back(j);
			// complete the ordering using all the variables that were not selected

			_ASSERTE(CMatrixOrdering::b_IsValidOrdering(&schur_ordering[0], schur_ordering.size()));
			// make sure we came up with a valid ordering

			if(n_schur_variable_num != r_lambda.n_BlockColumn_Num()) { // unless we want all the variables, do the SC
				_ASSERTE(n_schur_variable_num < r_lambda.n_BlockColumn_Num()); // not more of them, are there? :)

				CUberBlockMatrix lambda_perm;
				CMatrixOrdering mord;
				r_lambda.Permute_UpperTriangular_To(lambda_perm, mord.p_InvertOrdering(&schur_ordering[0],
					schur_ordering.size()), schur_ordering.size(), true);

				CUberBlockMatrix A, U, C, V;
				const size_t n = r_lambda.n_BlockColumn_Num();
				lambda_perm.SliceTo(A, 0, n_schur_variable_num, 0, n_schur_variable_num, true);
				lambda_perm.SliceTo(U, 0, n_schur_variable_num, n_schur_variable_num, n, true);
				lambda_perm.SliceTo(C, n_schur_variable_num, n, n_schur_variable_num, n, true);
				// cut Lambda matrix into pieces
				// \lambda = | A U |
				//           | V C |

				std::vector<size_t> group_ordering;
				std::vector<size_t> vertex_groups(C.n_BlockColumn_Num(), size_t(-1));
				{
					cs *p_upper = C.p_BlockStructure_to_Sparse();
					cs *p_lower = cs_transpose(p_upper, 0);
					cs *p_adj = cs_add(p_upper, p_lower, 1, 1);
					cs_spfree(p_upper);
					cs_spfree(p_lower);

					size_t n_group_num = 0;
					for(size_t i = 0, n = p_adj->n; i < n; ++ i) {
						if(vertex_groups[i] != size_t(-1))
							continue; // vertex already assigned a group

						std::vector<size_t> vertices_to_mark;
						vertex_groups[i] = n_group_num;
						group_ordering.push_back(i);
						for(size_t n_vertex = i;;) {
							_ASSERTE(vertex_groups[n_vertex] == n_group_num); // make sure the vertex was already marked (avoids adding too many vertices to the stack)
							for(size_t j = p_adj->p[n_vertex], m = p_adj->p[n_vertex + 1]; j < m; ++ j) {
								size_t n_neighbor = p_adj->i[j];
								_ASSERTE(vertex_groups[n_neighbor] == size_t(-1) ||
									vertex_groups[n_neighbor] == n_group_num); // make sure that the vertex is either not visited or already a part of this group (otherwise it means the graph traversal is broken)
								if(vertex_groups[n_neighbor] == size_t(-1)) {
									vertex_groups[n_neighbor] = n_group_num; // mark as part of this group
									group_ordering.push_back(n_neighbor);
									vertices_to_mark.push_back(n_neighbor);
								}
							}
							if(vertices_to_mark.empty())
								break;
							n_vertex = vertices_to_mark.back();
							vertices_to_mark.erase(vertices_to_mark.end() - 1);
						}

						++ n_group_num; // !!
					}

					_ASSERTE(group_ordering.size() == p_adj->n); // make sure that all the groups are represented
					_ASSERTE(CMatrixOrdering::b_IsValidOrdering(&group_ordering[0], group_ordering.size())); // make sure it is a valid ordering
					for(size_t i = 0, n = group_ordering.size(); i < n; ++ i) {
						_ASSERTE(group_ordering[i] < n); // make sure it is a valid vertex index
						_ASSERTE(i || vertex_groups[group_ordering[i]] == 0); // make sure the first vertex has group id zero
						_ASSERTE(!i || vertex_groups[group_ordering[i]] >= vertex_groups[group_ordering[i - 1]]); // make sure the group ids are nondecreasing (the groups are not mixed then)
					}
					// run some simple debug checks on group ordering

					cs_spfree(p_adj);
				}
				// discover vertex groups and build a group ordering

				CUberBlockMatrix SC_unperm; // debug
				CUberBlockMatrix C_perm; // needs to exist past the below block, as C can reference the data in it
				if(vertex_groups[group_ordering.back()] + 1 > 1 &&
				   vertex_groups[group_ordering.back()] + 1 < vertex_groups.size()) { // more than 1 group, but not each ertex in its own group
					CUberBlockMatrix C_inv_unperm;

#ifdef _DEBUG
					if(C.n_Row_Num() <= 4096) { // only if we can affort potentially completely dense inverse of C
						C_inv_unperm.InverseOf_Symmteric(C, true);
						C_inv_unperm.Scale(-1.0);
						CUberBlockMatrix minus_U_Cinv_unperm, V_;
						minus_U_Cinv_unperm.ProductOf(U, C_inv_unperm);
						U.TransposeTo(V_);
						SC_unperm.ProductOf(minus_U_Cinv_unperm, V_, true);
						A.AddTo(SC_unperm); // U * (-C^-1) * V + A
					}
					// calculate the schur complement without any ordering
#endif // _DEBUG

					CMatrixOrdering mkinv;
					const size_t *p_inv_ordering = mkinv.p_InvertOrdering(&group_ordering[0], group_ordering.size());
					CUberBlockMatrix U_perm;
					U.PermuteTo(U_perm, p_inv_ordering, group_ordering.size(), false, true, true); // reorder columns of U
					U.Swap(U_perm); // swap matrices (they both only store references to lambda_perm, it is ok to free the former U)

#ifdef _DEBUG
					C.Rasterize("sc_00_D_orig.tga");
					C_inv_unperm.Rasterize("sc_03_D_inv_orig.tga");
#endif // _DEBUG

					C.Permute_UpperTriangular_To(C_perm, p_inv_ordering, group_ordering.size(), true);
					C_perm.Swap(C); // swap matrices (now it gets messy, C stores references both to lamdda perm and the former C)

#ifdef _DEBUG
					C.Rasterize("sc_01_D_perm.tga");
#endif // _DEBUG
				}
				// in case there is more than one vertex group, reorder the matrices to avoid fill-in in inversion of C

				U.TransposeTo(V); // because lower-triangular of lambda is not calculated
				// calculate this after permuting U

				CUberBlockMatrix C_inv;
				C_inv.InverseOf_Symmteric(C, true); // todo - debug the block grouping
				// calculate inverse of C (might get dense)

#ifdef _DEBUG
				if(C_inv.n_Row_Num() <= 4096) { // will want to invert the whole thing
					Eigen::MatrixXd c_dense;
					C.Convert_to_Dense(c_dense);
					c_dense.triangularView<Eigen::StrictlyLower>() =
						c_dense.triangularView<Eigen::StrictlyUpper>().transpose();
					Eigen::MatrixXd c_inv_dense;
					C_inv.Convert_to_Dense(c_inv_dense);
					//c_inv_dense.triangularView<Eigen::StrictlyLower>() =
					//	c_inv_dense.triangularView<Eigen::StrictlyUpper>().transpose(); // no.
					double f_error = (c_inv_dense - c_dense.inverse()).norm(); // todo - debug this on large data
					fprintf(stderr, "debug: block matrix inverse error: %g\n", f_error);
				}

				if(!C_perm.b_Empty())
					C_inv.Rasterize("sc_02_D_inv.tga");
#endif // _DEBUG

				C_inv.Scale(-1.0);
				CUberBlockMatrix minus_U_Cinv;
				minus_U_Cinv.ProductOf(U, C_inv);
				schur_complement.ProductOf(minus_U_Cinv, V, true);
				A.AddTo(schur_complement); // U * (-C^-1) * V + A
				// calculate the schur complement

#ifdef _DEBUG
				if(!SC_unperm.b_Empty()) {
					schur_complement.AddTo(SC_unperm, -1);
					double f_sc_error = SC_unperm.f_Norm();
					fprintf(stderr, "debug: ordered - unordered SC difference: %g\n", f_sc_error);
				}
				// debug - check SC error
#endif // _DEBUG
			}
		}
		// in case we have a reference list of compact lengths, select the variables to marginalize

		const CUberBlockMatrix &r_lambda = (schur_complement.b_Empty())? r_solver.r_Lambda() : schur_complement;
		// note that we could use r_solver.r_R() but that would potentially affect
		// the timing by preparing the R matrix for the next iteration

		CUberBlockMatrix R, lambda_perm;
		CMatrixOrdering mord;
		mord.p_BlockOrdering(r_lambda, true);
		r_lambda.Permute_UpperTriangular_To(lambda_perm,
			mord.p_Get_InverseOrdering(), mord.n_Ordering_Size(), true);
		if(!R.CholeskyOf_FBS<typename CNonlinearSolverType::_TyLambdaMatrixBlockSizes>(lambda_perm))
			fprintf(stderr, "warning: R.CholeskyOf_FBS<_TyLambdaMatrixBlockSizes>(lambda_perm) failed\n");
		else {
			double f_squared_norm = 0;
			double f_diag_squared_norm = 0;
			const size_t n_stripe = n_pose_dimension, n_columns = r_lambda.n_Column_Num();
			#pragma omp parallel
			{
/*#ifdef _OPENMP
				if(!omp_get_thread_num())
					printf("debug: calculating norm of covariances using %d threads\n", omp_get_num_threads());
#endif // _OPENMP*/
				// debug
				double f_partial_sum = 0;
				double f_diag_partial_sum = 0;
				_ASSERTE(n_columns <= INT_MAX);
				int _n_columns = int(n_columns);
				#pragma omp for
				for(int i = 0; i < _n_columns; i += n_stripe) {
					Eigen::MatrixXd t_my_stripe(n_columns, std::min(i + n_stripe, n_columns) - i);
					CMarginals::Calculate_SubblockMarginals_Fast_ColumnBand_FBS<typename
						CNonlinearSolverType::_TyLambdaMatrixBlockSizes>(t_my_stripe, R,
						i, mord.p_Get_InverseOrdering(), mord.n_Ordering_Size(), mord.n_Ordering_Size());
					// calculate a stripe of the marginals at a time

					f_partial_sum += t_my_stripe.squaredNorm(); // can't sum norms, only squared norms
					_ASSERTE(n_stripe == n_pose_dimension); // if this is changed then the line below won't work!
					f_diag_partial_sum += t_my_stripe.template block<n_pose_dimension, n_pose_dimension>(i, 0).eval().squaredNorm(); // todo - eigen bug
					// calculate norm
				}
				#pragma omp atomic
				f_squared_norm += f_partial_sum;
				f_diag_squared_norm += f_diag_partial_sum;
			}
			//printf("debug: finished\n"); // debug
			r_f_covariance_norm = sqrt(f_squared_norm);
			r_f_marginals_norm = sqrt(f_diag_squared_norm);
		}
		// calculate the norm
	}

	/**
	 *	@brief in SE(2) the angular errors may be outside the \f$[-2\pi, 2\pi]\f$ range and need to be clamped
	 *	@param[in] f_error_radians is angular error
	 *	@return Returns angular error modulo \f$2\pi\f$.
	 */
	static inline double f_ClampAngularError_2Pi_Abs(double f_error_radians)
	{
		if(b_2D_SLAM) {
			f_error_radians = fmod(f_error_radians, 2 * M_PI);
			f_error_radians = std::min(fabs(f_error_radians),
				std::min(fabs(f_error_radians - 2 * M_PI), fabs(f_error_radians + 2 * M_PI)));
		} else {
			_ASSERTE(f_error_radians >= 0 && f_error_radians <= 2 * M_PI);
			// in SE(3) this should not generally happen as the error is calculated
			// using quaternion / matrix algebra and should clamp itself
		}
		return f_error_radians;
	}

	/**
	 *	@brief computes poses corresponding to all the vertices in the original graph
	 *
	 *	@param[out] vertices_v0 is filled with interpolated vertices (equal weights)
	 *	@param[out] vertices_v1 is filled with interpolated vertices (weighted by measurements)
	 *	@param[out] vertices_v2 is filled with error space interpolated vertices
	 *	@param[in] r_compact_edge_lengths is list of collapsed edge counts for each compact edge
	 *	@param[in] r_measurements is list of original (unabridged) measurements
	 *	@param[in] r_compact_poses is structure containing estimated vertices
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static void Reconstruct_FullTrajectory(std::vector<_TyVectorUnalign> &vertices_v0,
		std::vector<_TyVectorUnalign> &vertices_v1, std::vector<_TyVectorUnalign> &vertices_v2,
		const std::vector<size_t> &r_compact_edge_lengths, const std::vector<_TyEdgeData> &r_measurements,
		const typename CSystemType::_TyVertexMultiPool &r_compact_poses) // throw(std::bad_alloc)
	{
		vertices_v0.clear();
		vertices_v1.clear();
		vertices_v2.clear();
		// !!

		size_t i = 0;
		// index of the current measurement

		const double f_rot_regularize = 1; // regularize rotation in calculation of edge weights (1 = norm of measurement, lower values remove the effect of rotation, 0 = norm of translation components only)
		for(size_t j_plus_1 = 1, n = r_compact_poses.n_Size(); j_plus_1 < n; ++ j_plus_1) { // loop with j + 1 instead of j to avoid unsigned underflow in n when r_compact_poses is empty
			const size_t q = r_compact_edge_lengths[j_plus_1 - 1];
			// number of segments of the edge

			const _TyVector compact_vertex_j = r_compact_poses[j_plus_1 - 1].v_State(); // get the first vertex
			const _TyVector compact_vertex_j1 = r_compact_poses[j_plus_1].v_State(); // get the last vertex
			// ^C\Theta_j, ^C\Theta_{j + 1}, corresponds to ^I\Theta_i, ^I\Theta_{i + q}

			_TyVector v_est_edge;
			Absolute_to_Relative(compact_vertex_j, compact_vertex_j1, v_est_edge);
			if(b_2D_SLAM)
				v_est_edge(2) = C2DJacobians::f_ClampAngularError_2Pi(v_est_edge(2)); // make sure we don't rotate the long way in 2D
			// calculate an estimated odometry edge

			double f_weight_denom = 0;
			_TyVector initial_cumulative = _TyVector::Zero(); // sum of z_{i,i + 1} \oplus ... z_{i + q - 1,i + q}
			for(size_t l = i; l < i + q; ++ l) {
				const typename _TyEdgeData::_TyVector &z_l = r_measurements[l].v_measurement;
				f_weight_denom += sqrt(z_l.template head<n_pose_pos_components>().squaredNorm() +
					z_l.template tail<n_pose_rot_components>().squaredNorm() * f_rot_regularize);
				Relative_to_Absolute(initial_cumulative, z_l, initial_cumulative);
			}
			// compute length of eq 22 denominator (sum of edge lengths)
			// also compute cumulative initial vector needed for eq 24

			_TyVector d_correction;
			Absolute_to_Relative(initial_cumulative, v_est_edge, d_correction);
			if(b_2D_SLAM)
				d_correction(2) = C2DJacobians::f_ClampAngularError_2Pi(d_correction(2)); // make sure we don't rotate the long way in 2D
			// compute d_{j,j + 1} value eq 24

			if(j_plus_1 == 1) {
				vertices_v0.push_back(compact_vertex_j);
				vertices_v1.push_back(compact_vertex_j);
				vertices_v2.push_back(compact_vertex_j);
			}
			// in the first iteration, push the first vertex into all the sequences
			// (in all the next iterations, this vertex is calculated in the below loop)

			_TyVector lerp_vertex_v2_uncorrected = compact_vertex_j;
			// running value of ^C\Theta_j \oplus (z_{i,i + 1} \oplus ... z_{i + m - 1,i + m})

			double f_weight_cumsum = 0; // eq 22 value
			for(size_t m = 1; m <= q; ++ m, ++ i) {
				const typename _TyEdgeData::_TyVector &z_i = r_measurements[i].v_measurement;
				// get measurement z_{i,i + 1}

				{
					double f_equal_weight_cumsum = double(m) / q;
					_TyVector lerp_vertex_v0;
					Relative_to_Absolute(compact_vertex_j, f_equal_weight_cumsum * v_est_edge, lerp_vertex_v0);
					vertices_v0.push_back(lerp_vertex_v0);
				}
				// simple interpolation, equal edge lenghts

				f_weight_cumsum += sqrt(z_i.template head<n_pose_pos_components>().squaredNorm() +
					z_i.template tail<n_pose_rot_components>().squaredNorm() * f_rot_regularize) / f_weight_denom;
				// compute weight from eq 22

				{
					_TyVector v_error;
					_TyVector lerp_vertex_v1;
					Relative_to_Absolute(compact_vertex_j, f_weight_cumsum * v_est_edge, lerp_vertex_v1);
					vertices_v1.push_back(lerp_vertex_v1);
				}
				// weighted interpolation, edge lengths proportional to the measurements

				{
					Relative_to_Absolute(lerp_vertex_v2_uncorrected, z_i, lerp_vertex_v2_uncorrected);
					_TyVector lerp_vertex_v2;
					Relative_to_Absolute(lerp_vertex_v2_uncorrected, f_weight_cumsum * d_correction, lerp_vertex_v2);
					vertices_v2.push_back(lerp_vertex_v2);
				}
				// weighted interpolation in error space (measurements superimposed over the optimized
				// structure to provide cheap high-resolution estimate)
			}
			// re-create the in-between vertices, and the last vertex (^I\Theta_{i + 1} through ^I\Theta_{i + q})

			const double f_epsilon = 1e-6;
			{
				_TyVector v_error;
				Absolute_to_Relative(vertices_v0.back(), compact_vertex_j1, v_error);
				_ASSERTE(v_error.template head<n_pose_pos_components>().norm() < f_epsilon);
				_ASSERTE(f_ClampAngularError_2Pi_Abs(v_error.template tail<n_pose_rot_components>().norm()) < f_epsilon);
			}
			{
				_TyVector v_error;
				Absolute_to_Relative(vertices_v1.back(), compact_vertex_j1, v_error);
				_ASSERTE(v_error.template head<n_pose_pos_components>().norm() < f_epsilon);
				_ASSERTE(f_ClampAngularError_2Pi_Abs(v_error.template tail<n_pose_rot_components>().norm()) < f_epsilon);
			}
			{
				_TyVector v_error;
				Absolute_to_Relative(vertices_v2.back(), compact_vertex_j1, v_error);
				_ASSERTE(v_error.template head<n_pose_pos_components>().norm() < f_epsilon);
				_ASSERTE(f_ClampAngularError_2Pi_Abs(v_error.template tail<n_pose_rot_components>().norm()) < f_epsilon);
			}
			// now the interpolated vertex should match the second vertex in the compact edge - see if it does
		}

		_ASSERTE(i == r_measurements.size()); // if this triggers, not all edges in the graph file were used for comparison
		_ASSERTE(vertices_v0.size() == vertices_v2.size() && vertices_v1.size() == vertices_v2.size()); // should be the same size
	}

	static void Compute_RelativePoseError_AllToAll(const std::vector<_TyVectorUnalign> &est_vertices,
		const std::vector<_TyVectorUnalign> &gt_vertices, const _TyVector &v_rel_pose,
		double &r_f_translation_error, double &r_f_rotation_error,
		double &r_f_squared_translation_error, double &r_f_squared_rotation_error)
	{
		r_f_translation_error = 0;
		r_f_rotation_error = 0;
		r_f_squared_translation_error = 0;
		r_f_squared_rotation_error = 0;

		_ASSERTE(est_vertices.size() == gt_vertices.size());
		for(size_t n_delta = 1, n = est_vertices.size(); n_delta < n; ++ n_delta) {
			double f_translation_error, f_rotation_error, f_squared_translation_error, f_squared_rotation_error;
			Compute_RelativePoseError(est_vertices, gt_vertices, v_rel_pose, n_delta,
				f_translation_error, f_rotation_error, f_squared_translation_error, f_squared_rotation_error);
			r_f_translation_error += f_translation_error;
			r_f_rotation_error += f_rotation_error;
			r_f_squared_translation_error += f_squared_translation_error;
			r_f_squared_rotation_error += f_squared_rotation_error;
		}
		// calculate RPE for all the possible deltas

		r_f_translation_error /= double(est_vertices.size());
		r_f_rotation_error /= double(est_vertices.size());
		r_f_squared_translation_error /= double(est_vertices.size());
		r_f_squared_rotation_error /= double(est_vertices.size());
		// divide by n
	}

	static void Compute_RelativePoseError(const std::vector<_TyVectorUnalign> &est_vertices,
		const std::vector<_TyVectorUnalign> &gt_vertices, const _TyVector &v_rel_pose,
		size_t n_delta, double &r_f_translation_error, double &r_f_rotation_error,
		double &r_f_squared_translation_error, double &r_f_squared_rotation_error)
	{
		_ASSERTE(n_delta > 0); // ...

		r_f_translation_error = 0;
		r_f_rotation_error = 0;
		r_f_squared_translation_error = 0;
		r_f_squared_rotation_error = 0;

		const double f_deg_per_rad = 180 / M_PI;
		// number of degrees per one radian (for angular unit conversion)

		std::queue<_TyVectorUnalign> gt_vertices_transformed;
		_ASSERTE(est_vertices.size() == gt_vertices.size());
		for(size_t i = 0, n = est_vertices.size(); i < n; ++ i) {
			_TyVector v_gt_pose_cur;
			Relative_to_Absolute(v_rel_pose, gt_vertices[i], v_gt_pose_cur); // arg order! relative pose is on the left!

			if(i >= n_delta) {
				_TyVector v_gt_pose_prev = gt_vertices_transformed.front(); // one from n_delta vertices ago
				gt_vertices_transformed.pop(); // !!
				_TyVector v_est_pose_prev = est_vertices[i - n_delta];
				_TyVector v_est_pose_cur = est_vertices[i];
				// get the previous and current vertices

				_TyVector v_edge_est, v_edge_gt;
				Absolute_to_Relative(v_est_pose_cur, v_est_pose_prev, v_edge_est);
				Absolute_to_Relative(v_gt_pose_cur, v_gt_pose_prev, v_edge_gt);
				// calculate edges between those vertices

				_TyVector v_error;
				Absolute_to_Relative(v_edge_est, v_edge_gt, v_error);
				double f_trans_error2 = v_error.template head<n_pose_pos_components>().squaredNorm();
				r_f_translation_error += sqrt(f_trans_error2);
				r_f_squared_translation_error += f_trans_error2;
				double f_rot_error_degrees = f_ClampAngularError_2Pi_Abs(v_error.template tail<n_pose_rot_components>().norm()) * f_deg_per_rad;
				r_f_rotation_error += f_rot_error_degrees;
				r_f_squared_rotation_error += f_rot_error_degrees * f_rot_error_degrees;
				// calculate the errors
			}

			gt_vertices_transformed.push(v_gt_pose_cur);
		}
		// calculate RPE errors
	}

	static void Compute_AbsoluteTrajectoryError(const std::vector<_TyVectorUnalign> &est_vertices,
		const std::vector<_TyVectorUnalign> &gt_vertices, const _TyVector &v_rel_pose,
		double &r_f_translation_error, double &r_f_rotation_error,
		double &r_f_squared_translation_error, double &r_f_squared_rotation_error)
	{
		r_f_translation_error = 0;
		r_f_rotation_error = 0;
		r_f_squared_translation_error = 0;
		r_f_squared_rotation_error = 0;

		const double f_deg_per_rad = 180 / M_PI;
		// number of degrees per one radian (for angular unit conversion)

		_ASSERTE(est_vertices.size() == gt_vertices.size());
		for(size_t i = 0, n = est_vertices.size(); i < n; ++ i) {
			_TyVector v_gt_pose;
			Relative_to_Absolute(v_rel_pose, gt_vertices[i], v_gt_pose); // arg order! relative pose is on the left!
			// put the ground truth vertex in the solution coordinate frame

			_TyVector v_error;
			Absolute_to_Relative(est_vertices[i], v_gt_pose, v_error);
			double f_trans_error2 = v_error.template head<n_pose_pos_components>().squaredNorm();
			r_f_translation_error += sqrt(f_trans_error2);
			r_f_squared_translation_error += f_trans_error2;
			double f_rot_error_degrees = f_ClampAngularError_2Pi_Abs(v_error.template tail<n_pose_rot_components>().norm()) * f_deg_per_rad;
			r_f_rotation_error += f_rot_error_degrees;
			r_f_squared_rotation_error += f_rot_error_degrees * f_rot_error_degrees;
			// calculate the errors
		}
		// calculate ATE errors
	}

	/**
	 *	@brief computes error comparing to the ground truth
	 *
	 *	@param[in] r_compact_edge_lengths is list of collapsed edge counts for each compact edge
	 *	@param[in] r_measurements is list of original (unabridged) measurements
	 *	@param[in] r_compact_poses is structure containing estimated vertices
	 *	@param[in] p_s_gt_filename is name of file with ground truth (or null)
	 *	@param[in] p_s_ref_filename is name of file with reference trajectory (or null)
	 *	@param[out] r_v_error is output vector, contains translation error and rotation error
	 *	@param[in] r_system is reference to the optimized system (required for debug image rendering)
	 *	@param[in,out] p_image is a temporary storage for the rasterized image (contains the image upon return)
	 *	@param[in] r_v_off is offset of the state coordinates in the raster coordinates
	 *	@param[in] r_v_scale is scale relating the state coordinates to the raster coordinates
	 *	@param[in] b_xz_projection is x-z projection flag (if set, the projection is x-z, otherwise it is x-y)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool ComputeError(const std::vector<size_t> &r_compact_edge_lengths,
		const std::vector<_TyEdgeData> &r_measurements,
		const typename CSystemType::_TyVertexMultiPool &r_compact_poses,
		const char *p_s_gt_filename, const char *p_s_ref_filename, Eigen::Vector6d &r_v_error,
		const CSystemType &r_system, TBmp *p_image, const Eigen::Vector2d &r_v_off,
		const Eigen::Vector2d &r_v_scale, bool b_xz_projection)
	{
		_ASSERTE(r_compact_edge_lengths.size() + 1 == r_compact_poses.n_Size());
		// there should be one more vertex than there are odometry edges

		_ASSERTE(std::accumulate(r_compact_edge_lengths.begin(), r_compact_edge_lengths.end(), size_t(0)) == r_measurements.size());
		// the numbers of original and compacted edges should match

		if(r_measurements.size() != std::accumulate(r_compact_edge_lengths.begin(), r_compact_edge_lengths.end(),
		   size_t(0)) || r_compact_edge_lengths.size() + 1 != r_compact_poses.n_Size()) {
			fprintf(stderr, "error: the number of edges and vertices in the compact graph does not correspond to the original graph\n");
			return false;
		}
		// the sizes must match

		std::vector<_TyVectorUnalign> gt_vertices;
		if(p_s_gt_filename) {
			FILE *p_gt_file;
			if(!(p_gt_file = fopen(p_s_gt_filename, "r"))) {
				fprintf(stderr, "error: can not open ground truth file \'%s\'\n", p_s_gt_filename);
				return false;
			}
			for(;;) {
				_TyVector v_gt_vetex;
				bool keep_reading = true;
				for(size_t a = 0; a < n_pose_dimension; ++ a) {
					int n_result;
					if((n_result = fscanf(p_gt_file, "%lf", &v_gt_vetex(a))) != 1) {
						if(n_result != EOF) {
							fprintf(stderr, "error: can not read ground truth file\n");
							fclose(p_gt_file);
							return false;
						}
						keep_reading = false;
						break;
					}
				}
				if(!keep_reading) { // end at the end of file
					if(feof(p_gt_file)) // fscanf() also returns EOF on failure
						break;
					fclose(p_gt_file);
					fprintf(stderr, "error: can not read ground truth file\n");
					return false;
				}
				// if fscanf() returns EOF, it means that it did not read the value and the vertex is incomplete

				gt_vertices.push_back(v_gt_vetex); // add it to the list
			}
			fclose(p_gt_file);

			if(r_measurements.size() + 1 != gt_vertices.size()) { // if it is the same graph, then this should match
				fprintf(stderr, "warning: the number of edges in the graph file does"
					" not correspond to the number of vertices in the ground truth file\n");
			}
		}
		// load all ground truth vertices

		std::vector<_TyVectorUnalign> ref_vertices;
		if(p_s_ref_filename) {
			FILE *p_ref_file;
			if(!(p_ref_file = fopen(p_s_ref_filename, "r"))) {
				fprintf(stderr, "error: can not open ground truth file \'%s\'\n", p_s_ref_filename);
				return false;
			}
			for(;;) {
				_TyVector v_ref_vetex;
				bool keep_reading = true;
				for(size_t a = 0; a < n_pose_dimension; ++ a) {
					int n_result;
					if((n_result = fscanf(p_ref_file, "%lf", &v_ref_vetex(a))) != 1) {
						if(n_result != EOF) {
							fprintf(stderr, "error: can not read reference trajectory file\n");
							fclose(p_ref_file);
							return false;
						}
						keep_reading = false;
						break;
					}
				}
				if(!keep_reading) { // end at the end of file
					if(feof(p_ref_file)) // fscanf() also returns EOF on failure
						break;
					fclose(p_ref_file);
					fprintf(stderr, "error: can not read reference trajectory file\n");
					return false;
				}
				// if fscanf() returns EOF, it means that it did not read the value and the vertex is incomplete

				ref_vertices.push_back(v_ref_vetex); // add it to the list
			}
			fclose(p_ref_file);

			if(r_measurements.size() + 1 != ref_vertices.size()) { // if it is the same graph, then this should match
				fprintf(stderr, "warning: the number of edges in the graph file does"
					" not correspond to the number of vertices in the reference trajectory file\n");
			}
		}
		// load all reference trajectory vertices

		std::vector<_TyVectorUnalign> vertices_v0, vertices_v1, vertices_v2;
		Reconstruct_FullTrajectory(vertices_v0, vertices_v1,
			vertices_v2, r_compact_edge_lengths, r_measurements, r_compact_poses);
		_ASSERTE(vertices_v0.size() == vertices_v1.size() &&
			vertices_v0.size() == vertices_v2.size());
		// reconstruct the full trajectory by interpolating the compact poses

		const _TyVector v_rel_pose_v0 = (gt_vertices.empty())? _TyVector::Zero() :
			v_Align_PoseSets(gt_vertices, vertices_v0);
		const _TyVector v_rel_pose_v1 = (gt_vertices.empty())? _TyVector::Zero() :
			v_Align_PoseSets(gt_vertices, vertices_v1);
		const _TyVector v_rel_pose_v2 = (gt_vertices.empty())? _TyVector::Zero() :
			v_Align_PoseSets(gt_vertices, vertices_v2);
		// compute a rigid transformation that puts ground truth vertices close to the solution vertices

		const _TyVector v_rel_pose_v2_ref = (ref_vertices.empty())? _TyVector::Zero() :
			v_Align_PoseSets(ref_vertices, vertices_v2);
		// compute a rigid transformation that puts the reference vertices close to the solution vertices

		if(p_image) {
			p_image->Clear(-1);
			// clear to white

			Draw_Edges(r_system, p_image, r_v_off, r_v_scale, b_xz_projection, 0xff0000ff, 3); // compact pose edges in the bottom
			Draw_VertexTicks(r_system, p_image, r_v_off, r_v_scale, b_xz_projection); // compact pose vertices

			for(size_t i = 0, n = vertices_v2.size(); i < n; ++ i) {
				{
					float f_x = float((vertices_v1[i](0) + r_v_off(0)) * r_v_scale(0));
					float f_y = float((vertices_v1[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
					// transform vertex position to raster coordinates

					const uint32_t n_dkyellow = 0xff8800ffU;
					float f_bug_size = 4, w = 2;
#if 0
					float f_x0 = f_x - f_bug_size, f_x1 = f_x - f_bug_size * .5f,
						f_x3 = f_x + f_bug_size * .5f, f_x4 = f_x + f_bug_size;
					float f_y0 = f_y - f_bug_size, f_y1 = f_y - f_bug_size * .5f,
						f_y3 = f_y + f_bug_size * .5f, f_y4 = f_y + f_bug_size;
					p_image->DrawLine_AA(f_x0, f_y1, f_x0, f_y3, n_dkyellow, w);
					p_image->DrawLine_AA(f_x1, f_y4, f_x0, f_y3, n_dkyellow, w);
					p_image->DrawLine_AA(f_x1, f_y4, f_x3, f_y4, n_dkyellow, w);
					p_image->DrawLine_AA(f_x4, f_y3, f_x3, f_y4, n_dkyellow, w);
					p_image->DrawLine_AA(f_x4, f_y3, f_x4, f_y1, n_dkyellow, w);
					p_image->DrawLine_AA(f_x3, f_y0, f_x4, f_y1, n_dkyellow, w);
					p_image->DrawLine_AA(f_x3, f_y0, f_x1, f_y0, n_dkyellow, w);
					p_image->DrawLine_AA(f_x1, f_y0, f_x0, f_y1, n_dkyellow, w);
					// draw a octagon under the vertex
#else // 0
					p_image->DrawCircle_AA(f_x, f_y, float(f_bug_size), n_dkyellow, w);
					// or just ...
#endif // 0
				}

				{
					float f_x = float((vertices_v2[i](0) + r_v_off(0)) * r_v_scale(0));
					float f_y = float((vertices_v2[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
					// transform vertex position to raster coordinates

					int n_rect_size = 4;
					const uint32_t n_green = 0xff00ff00U;
					p_image->DrawRect(int(f_x - n_rect_size + .5f), int(f_y - n_rect_size + .5f), int(f_x + n_rect_size + .5f), int(f_y + n_rect_size + .5f), n_green, 2);
					// draw a green rectangle under the vertex
				}
			}
			// draw crosses where the poses are (only v1, v2)

			for(size_t i = 0, n = gt_vertices.size(); i < n; ++ i) {
				_TyVector v_pose_gt;
				Relative_to_Absolute(v_rel_pose_v2, gt_vertices[i], v_pose_gt); // arg order! relative pose is on the left!
				// put the ground truth vertex in the solution coordinate frame

				float f_x = float((v_pose_gt(0) + r_v_off(0)) * r_v_scale(0));
				float f_y = float((v_pose_gt((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
				// transform vertex position to raster coordinates

				const uint32_t n_black = 0xff000000U;
				p_image->DrawRect(int(f_x + .5f) - 1, int(f_y + .5f) - 1, int(f_x + .5f), int(f_y + .5f), n_black);
				// draw a black dot under the ground truth vertex
			}
			// draw dots where ground truth poses are

			CTgaCodec::Save_TGA("compact-pose-slam_error-eval.tga", *p_image, false);

			if(b_2D_SLAM) { // debug - heading markers
				p_image->Clear(-1);
				// clear to white

				Draw_VertexTicks(r_system, p_image, r_v_off, r_v_scale, b_xz_projection); // compact pose vertices

				for(size_t i = 0, n = vertices_v2.size(); i < n; ++ i) {
					{
						float f_x = float((vertices_v2[i](0) + r_v_off(0)) * r_v_scale(0)) + .5f;
						float f_y = float((vertices_v2[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1)) + .5f;
						// transform vertex position to raster coordinates

						int n_rect_size = 4;
						const uint32_t n_green = 0xff00ff00U;
						double f_heading = vertices_v2[i](2);
						p_image->DrawLine_AA(f_x + float(cos(f_heading)) * 5,
							f_y + float(sin(f_heading)) * 5, f_x + float(cos(f_heading)) * 10,
							f_y + float(sin(f_heading)) * 10, n_green, 1.5f);
						p_image->DrawRect(int(f_x - n_rect_size), int(f_y - n_rect_size),
							int(f_x + n_rect_size), int(f_y + n_rect_size), n_green, 2);
						// draw a green rectangle under the vertex
					}
				}
				// draw crosses where the poses are (only v2)

				for(size_t i = 0, n = gt_vertices.size(); i < n; ++ i) {
					_TyVector v_pose_gt;
					Relative_to_Absolute(v_rel_pose_v2, gt_vertices[i], v_pose_gt); // arg order! relative pose is on the left!
					// put the ground truth vertex in the solution coordinate frame

					float f_x = float((v_pose_gt(0) + r_v_off(0)) * r_v_scale(0)) + .5f;
					float f_y = float((v_pose_gt((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1)) + .5f;
					// transform vertex position to raster coordinates

					const uint32_t n_black = 0xff000000U;
					p_image->DrawLine_AA(f_x, f_y, f_x + float(cos(v_pose_gt(2))) * 5,
						f_y + float(sin(v_pose_gt(2))) * 5, n_black, 1.5f);
					p_image->DrawRect(int(f_x) - 1, int(f_y) - 1, int(f_x), int(f_y), n_black);
					// draw a black dot under the ground truth vertex
				}
				// draw dots where ground truth poses are

				CTgaCodec::Save_TGA("compact-pose-slam_error-eval-2D-headings.tga", *p_image, false);
			}

			{
				p_image->Clear(-1);
				// clear to white

				for(size_t i = 0, n = vertices_v0.size(); i < n; ++ i) {
					float f_x = float((vertices_v0[i](0) + r_v_off(0)) * r_v_scale(0));
					float f_y = float((vertices_v0[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
					// transform vertex position to raster coordinates

					const uint32_t n_red = 0xff880000U;
					p_image->DrawLine_SP(f_x - 5, f_y, f_x + 5, f_y, n_red, 1);
					p_image->DrawLine_SP(f_x, f_y - 5, f_x, f_y + 5, n_red, 1);
					// draw a red cross under the vertex
				}
				// draw crosses where the poses are (only v0)

				for(size_t i = 0, n = gt_vertices.size(); i < n; ++ i) {
					_TyVector v_pose_gt;
					Relative_to_Absolute(v_rel_pose_v2, gt_vertices[i], v_pose_gt); // arg order! relative pose is on the left!
					// put the ground truth vertex in the solution coordinate frame

					float f_x = float((v_pose_gt(0) + r_v_off(0)) * r_v_scale(0));
					float f_y = float((v_pose_gt((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
					// transform vertex position to raster coordinates

					const uint32_t n_black = 0xff000000U;
					p_image->DrawRect(int(f_x + .5f) - 1, int(f_y + .5f) - 1, int(f_x + .5f), int(f_y + .5f), n_black);
					// draw a black dot under the ground truth vertex
				}
				// draw dots where ground truth poses are

				CTgaCodec::Save_TGA("compact-pose-slam_error-eval-v0.tga", *p_image, false);
			}

			{
				p_image->Clear(-1);

				for(int n_ver = 0; n_ver < 3; ++ n_ver) {
					float f_prev_x, f_prev_y;
					for(size_t i = 0, n = vertices_v2.size(); i < n; ++ i) {
						float f_x, f_y;
						uint32_t n_color;

						if(n_ver == 0) {
							f_x = float((vertices_v0[i](0) + r_v_off(0)) * r_v_scale(0));
							f_y = float((vertices_v0[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
							// transform vertex position to raster coordinates

							n_color = 0xff880000U;
						} else if(n_ver == 1) {
							f_x = float((vertices_v1[i](0) + r_v_off(0)) * r_v_scale(0));
							f_y = float((vertices_v1[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
							// transform vertex position to raster coordinates

							n_color = 0xff8800ffU;
						} else {
							f_x = float((vertices_v2[i](0) + r_v_off(0)) * r_v_scale(0));
							f_y = float((vertices_v2[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
							// transform vertex position to raster coordinates

							n_color = 0xff00ff00U;
						}

						if(i) // except for the first pass ...
							p_image->DrawLine_AA(f_x, f_y, f_prev_x, f_prev_y, n_color, 5); // increase the width a bit

						f_prev_x = f_x;
						f_prev_y = f_y;
					}
				}

				{
					float f_prev_x, f_prev_y;
					for(size_t i = 0, n = gt_vertices.size(); i < n; ++ i) {
						_TyVector v_pose_gt;
						Relative_to_Absolute(v_rel_pose_v2, gt_vertices[i], v_pose_gt); // arg order! relative pose is on the left!
						// put the ground truth vertex in the solution coordinate frame

						float f_x = float((v_pose_gt(0) + r_v_off(0)) * r_v_scale(0));
						float f_y = float((v_pose_gt((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
						// transform vertex position to raster coordinates

						const uint32_t n_black = 0xff000000U;
						if(i)
							p_image->DrawLine_AA(f_x, f_y, f_prev_x, f_prev_y, n_black, 1);

						f_prev_x = f_x;
						f_prev_y = f_y;
					}
				}
				// draw dots where ground truth poses are

				CTgaCodec::Save_TGA("compact-pose-slam_error-eval-lines.tga", *p_image, false);
			}

			{
				p_image->Clear(-1);

				if(!ref_vertices.empty()) {
					float f_prev_x, f_prev_y;
					for(size_t i = 0, n = ref_vertices.size(); i < n; ++ i) {
						_TyVector v_pose_ref;
						Relative_to_Absolute(v_rel_pose_v2_ref, ref_vertices[i], v_pose_ref); // arg order! relative pose is on the left!
						// put the ground truth vertex in the solution coordinate frame

						float f_x = float((v_pose_ref(0) + r_v_off(0)) * r_v_scale(0));
						float f_y = float((v_pose_ref((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
						// transform vertex position to raster coordinates

						const uint32_t n_lt_blue = 0xff8888ffU;
						if(i)
							p_image->DrawLine_AA(f_x, f_y, f_prev_x, f_prev_y, n_lt_blue, 5);

						f_prev_x = f_x;
						f_prev_y = f_y;
					}
				}

				//for(int n_ver = 0; n_ver < 3; ++ n_ver) {
					float f_prev_x, f_prev_y;
					for(size_t i = 0, n = vertices_v2.size(); i < n; ++ i) {
						float f_x, f_y;
						uint32_t n_color;

						/*if(n_ver == 0) {
							f_x = float((vertices_v0[i](0) + r_v_off(0)) * r_v_scale(0));
							f_y = float((vertices_v0[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
							// transform vertex position to raster coordinates

							n_color = 0xff880000U;
						} else if(n_ver == 1) {
							f_x = float((vertices_v1[i](0) + r_v_off(0)) * r_v_scale(0));
							f_y = float((vertices_v1[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
							// transform vertex position to raster coordinates

							n_color = 0xff8800ffU;
						} else {*/
							f_x = float((vertices_v2[i](0) + r_v_off(0)) * r_v_scale(0));
							f_y = float((vertices_v2[i]((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
							// transform vertex position to raster coordinates

							n_color = 0xff00ff00U;
						//}

						if(i) // except for the first pass ...
							p_image->DrawLine_AA(f_x, f_y, f_prev_x, f_prev_y, n_color, 5); // increase the width a bit

						f_prev_x = f_x;
						f_prev_y = f_y;
					}
				//}

				{
					float f_prev_x, f_prev_y;
					for(size_t i = 0, n = gt_vertices.size(); i < n; ++ i) {
						_TyVector v_pose_gt;
						Relative_to_Absolute(v_rel_pose_v2, gt_vertices[i], v_pose_gt); // arg order! relative pose is on the left!
						// put the ground truth vertex in the solution coordinate frame

						float f_x = float((v_pose_gt(0) + r_v_off(0)) * r_v_scale(0));
						float f_y = float((v_pose_gt((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
						// transform vertex position to raster coordinates

						const uint32_t n_black = 0xff000000U;
						if(i)
							p_image->DrawLine_AA(f_x, f_y, f_prev_x, f_prev_y, n_black, 1.5f);

						f_prev_x = f_x;
						f_prev_y = f_y;
					}
				}
				// draw ground truth trajectory

				CTgaCodec::Save_TGA("compact-pose-slam_error-eval-lines-v2.tga", *p_image, false);
			}
		}
		// draw error evaluation debug image

		if(!gt_vertices.empty()) {
			double f_pos_ate_v0, f_rot_ate_v0, f_pos2_ate_v0, f_rot2_ate_v0;
			double f_pos_ate_v1, f_rot_ate_v1, f_pos2_ate_v1, f_rot2_ate_v1;
			double f_pos_ate_v2, f_rot_ate_v2, f_pos2_ate_v2, f_rot2_ate_v2;
			double f_pos_rpe_v0, f_rot_rpe_v0, f_pos2_rpe_v0, f_rot2_rpe_v0;
			double f_pos_rpe_v1, f_rot_rpe_v1, f_pos2_rpe_v1, f_rot2_rpe_v1;
			double f_pos_rpe_v2, f_rot_rpe_v2, f_pos2_rpe_v2, f_rot2_rpe_v2;
			double f_pos_rpe_aa_v0, f_rot_rpe_aa_v0, f_pos2_rpe_aa_v0, f_rot2_rpe_aa_v0;
			double f_pos_rpe_aa_v1, f_rot_rpe_aa_v1, f_pos2_rpe_aa_v1, f_rot2_rpe_aa_v1;
			double f_pos_rpe_aa_v2, f_rot_rpe_aa_v2, f_pos2_rpe_aa_v2, f_rot2_rpe_aa_v2;
			// results of different error metrics, direct and squared

			Compute_AbsoluteTrajectoryError(vertices_v0, gt_vertices, v_rel_pose_v0,
				f_pos_ate_v0, f_rot_ate_v0, f_pos2_ate_v0, f_rot2_ate_v0);
			Compute_AbsoluteTrajectoryError(vertices_v1, gt_vertices, v_rel_pose_v1,
				f_pos_ate_v1, f_rot_ate_v1, f_pos2_ate_v1, f_rot2_ate_v1);
			Compute_AbsoluteTrajectoryError(vertices_v2, gt_vertices, v_rel_pose_v2,
				f_pos_ate_v2, f_rot_ate_v2, f_pos2_ate_v2, f_rot2_ate_v2);
			// calculate the ATEs

			Compute_RelativePoseError(vertices_v0, gt_vertices, v_rel_pose_v0, 1,
				f_pos_rpe_v0, f_rot_rpe_v0, f_pos2_rpe_v0, f_rot2_rpe_v0);
			Compute_RelativePoseError(vertices_v1, gt_vertices, v_rel_pose_v1, 1,
				f_pos_rpe_v1, f_rot_rpe_v1, f_pos2_rpe_v1, f_rot2_rpe_v1);
			Compute_RelativePoseError(vertices_v2, gt_vertices, v_rel_pose_v2, 1,
				f_pos_rpe_v2, f_rot_rpe_v2, f_pos2_rpe_v2, f_rot2_rpe_v2);
			// calculate the RPEs

			Compute_RelativePoseError_AllToAll(vertices_v0, gt_vertices, v_rel_pose_v0,
				f_pos_rpe_aa_v0, f_rot_rpe_aa_v0, f_pos2_rpe_aa_v0, f_rot2_rpe_aa_v0);
			Compute_RelativePoseError_AllToAll(vertices_v1, gt_vertices, v_rel_pose_v1,
				f_pos_rpe_aa_v1, f_rot_rpe_aa_v1, f_pos2_rpe_aa_v1, f_rot2_rpe_aa_v1);
			Compute_RelativePoseError_AllToAll(vertices_v2, gt_vertices, v_rel_pose_v2,
				f_pos_rpe_aa_v2, f_rot_rpe_aa_v2, f_pos2_rpe_aa_v2, f_rot2_rpe_aa_v2);
			// calculate the RPE-AAs

			printf("cumulative v0 ATE of the aligned poses is %f, %f\n", f_pos_ate_v0, f_rot_ate_v0);
			printf("cumulative v1 ATE of the aligned poses is %f, %f\n", f_pos_ate_v1, f_rot_ate_v1);
			printf("cumulative v2 ATE of the aligned poses is %f, %f\n", f_pos_ate_v2, f_rot_ate_v2);

			f_pos_ate_v0 /= gt_vertices.size();
			f_pos_ate_v1 /= gt_vertices.size();
			f_pos_ate_v2 /= gt_vertices.size();
			f_rot_ate_v0 /= gt_vertices.size();
			f_rot_ate_v1 /= gt_vertices.size();
			f_rot_ate_v2 /= gt_vertices.size();
			f_pos2_ate_v0 /= gt_vertices.size();
			f_pos2_ate_v1 /= gt_vertices.size();
			f_pos2_ate_v2 /= gt_vertices.size();
			f_rot2_ate_v0 /= gt_vertices.size();
			f_rot2_ate_v1 /= gt_vertices.size();
			f_rot2_ate_v2 /= gt_vertices.size();

			printf("per vert. v0 ATE of the aligned poses is %f, %f\n", f_pos_ate_v0, f_rot_ate_v0);
			printf("per vert. v1 ATE of the aligned poses is %f, %f\n", f_pos_ate_v1, f_rot_ate_v1);
			printf("per vert. v2 ATE of the aligned poses is %f, %f\n", f_pos_ate_v2, f_rot_ate_v2);
			printf("RMSE v0 ATE of the aligned poses is %f, %f\n", sqrt(f_pos2_ate_v0), sqrt(f_rot2_ate_v0));
			printf("RMSE v1 ATE of the aligned poses is %f, %f\n", sqrt(f_pos2_ate_v1), sqrt(f_rot2_ate_v1));
			printf("RMSE v2 ATE of the aligned poses is %f, %f\n", sqrt(f_pos2_ate_v2), sqrt(f_rot2_ate_v2));

			printf("\ncumulative v0 RPE of the aligned poses is %f, %f\n", f_pos_rpe_v0, f_rot_rpe_v0);
			printf("cumulative v1 RPE of the aligned poses is %f, %f\n", f_pos_rpe_v1, f_rot_rpe_v1);
			printf("cumulative v2 RPE of the aligned poses is %f, %f\n", f_pos_rpe_v2, f_rot_rpe_v2);

			f_pos_rpe_v0 /= gt_vertices.size();
			f_pos_rpe_v1 /= gt_vertices.size();
			f_pos_rpe_v2 /= gt_vertices.size();
			f_rot_rpe_v0 /= gt_vertices.size();
			f_rot_rpe_v1 /= gt_vertices.size();
			f_rot_rpe_v2 /= gt_vertices.size();
			f_pos2_rpe_v0 /= gt_vertices.size();
			f_pos2_rpe_v1 /= gt_vertices.size();
			f_pos2_rpe_v2 /= gt_vertices.size();
			f_rot2_rpe_v0 /= gt_vertices.size();
			f_rot2_rpe_v1 /= gt_vertices.size();
			f_rot2_rpe_v2 /= gt_vertices.size();

			printf("per vert. v0 RPE of the aligned poses is %f, %f\n", f_pos_rpe_v0, f_rot_rpe_v0);
			printf("per vert. v1 RPE of the aligned poses is %f, %f\n", f_pos_rpe_v1, f_rot_rpe_v1);
			printf("per vert. v2 RPE of the aligned poses is %f, %f\n", f_pos_rpe_v2, f_rot_rpe_v2);
			printf("RMSE v0 RPE of the aligned poses is %f, %f\n", sqrt(f_pos2_rpe_v0), sqrt(f_rot2_rpe_v0));
			printf("RMSE v1 RPE of the aligned poses is %f, %f\n", sqrt(f_pos2_rpe_v1), sqrt(f_rot2_rpe_v1));
			printf("RMSE v2 RPE of the aligned poses is %f, %f\n", sqrt(f_pos2_rpe_v2), sqrt(f_rot2_rpe_v2));

			printf("\ncumulative v0 RPE all-to-all of the aligned poses is %f, %f\n", f_pos_rpe_aa_v0, f_rot_rpe_aa_v0);
			printf("cumulative v1 RPE all-to-all of the aligned poses is %f, %f\n", f_pos_rpe_aa_v1, f_rot_rpe_aa_v1);
			printf("cumulative v2 RPE all-to-all of the aligned poses is %f, %f\n", f_pos_rpe_aa_v2, f_rot_rpe_aa_v2);

			f_pos_rpe_aa_v0 /= gt_vertices.size();
			f_pos_rpe_aa_v1 /= gt_vertices.size();
			f_pos_rpe_aa_v2 /= gt_vertices.size();
			f_rot_rpe_aa_v0 /= gt_vertices.size();
			f_rot_rpe_aa_v1 /= gt_vertices.size();
			f_rot_rpe_aa_v2 /= gt_vertices.size();
			f_pos2_rpe_aa_v0 /= gt_vertices.size();
			f_pos2_rpe_aa_v1 /= gt_vertices.size();
			f_pos2_rpe_aa_v2 /= gt_vertices.size();
			f_rot2_rpe_aa_v0 /= gt_vertices.size();
			f_rot2_rpe_aa_v1 /= gt_vertices.size();
			f_rot2_rpe_aa_v2 /= gt_vertices.size();

			printf("per vert. v0 RPE all-to-all of the aligned poses is %f, %f\n", f_pos_rpe_aa_v0, f_rot_rpe_aa_v0);
			printf("per vert. v1 RPE all-to-all of the aligned poses is %f, %f\n", f_pos_rpe_aa_v1, f_rot_rpe_aa_v1);
			printf("per vert. v2 RPE all-to-all of the aligned poses is %f, %f\n", f_pos_rpe_aa_v2, f_rot_rpe_aa_v2);
			printf("RMSE v0 RPE all-to-all of the aligned poses is %f, %f\n", sqrt(f_pos2_rpe_aa_v0), sqrt(f_rot2_rpe_aa_v0));
			printf("RMSE v1 RPE all-to-all of the aligned poses is %f, %f\n", sqrt(f_pos2_rpe_aa_v1), sqrt(f_rot2_rpe_aa_v1));
			printf("RMSE v2 RPE all-to-all of the aligned poses is %f, %f\n", sqrt(f_pos2_rpe_aa_v2), sqrt(f_rot2_rpe_aa_v2));

			printf("note that all the rotation errors are in degrees\n");

			r_v_error << f_pos2_ate_v0, f_rot2_ate_v0, f_pos2_ate_v1, f_rot2_ate_v1, f_pos2_ate_v2, f_rot2_ate_v2;
			// output the RMSE errors
		} else {
			r_v_error.setZero();
			r_v_error.array() -= 1;
			// set all errors to -1
		}

		return true;
	}

	/**
	 *	@brief rigidly aligns two sets of poses
	 *
	 *	This calculates such a relative pose <tt>P</tt>, such that:
	 *
	 *	@code
	 *	_TyVector pose;
	 *	Relative_to_Absolute(P, r_vertices[i], pose); // P is on the left!
	 *	double error = (r_tar_vertices[i].head<3>() - pose.head<3>()).norm();
	 *	@endcode
	 *
	 *	The error in <tt>error</tt> is minimized.
	 *
	 *	@param[in] r_vertices is a set of vertices to be aligned
	 *	@param[in] r_tar_vertices is a set of vertices to align to
	 *
	 *	@return Returns a relative pose that rigidly aligns the two given sets of poses.
	 *
	 *	@note This requires the two sets of poses to have the corresponding vertices stored under the same index.
	 */
	static _TyVector v_Align_PoseSets(const std::vector<_TyVectorUnalign> &r_vertices,
		const std::vector<_TyVectorUnalign> &r_tar_vertices)
	{
		/*r_tar_vertices = r_vertices;
		// the error should be zero

		Eigen::Matrix3d Rfake;
		Rfake << sin(M_PI / 4), -cos(M_PI / 4), 0,
			     cos(M_PI / 4),  sin(M_PI / 4), 0,
				             0,              0, 1;
		for(size_t i = 0, n = r_vertices.size(); i < n; ++ i) {
			r_vertices[i].head<3>() = Rfake * r_vertices[i].head<3>() + Eigen::Vector3d(10.0, 20.0, 30.0);
			Eigen::Quaterniond rot;
			C3DJacobians::AxisAngle_to_Quat(r_vertices[i].tail<3>(), rot);
			C3DJacobians::Quat_to_AxisAngle(Eigen::Quaterniond(Rfake) * rot, r_vertices[i].tail<3>()); // rotate the attitude too!
		}*/
		// debug - create a known Rt to see what the code below calculates

		Eigen::Vector3d v_center3 = Eigen::Vector3d::Zero(), v_center_gt3 = Eigen::Vector3d::Zero();
		for(size_t i = 0, n = r_tar_vertices.size(); i < n; ++ i)
			v_center3.template head<n_pose_pos_components>() += r_tar_vertices[i].template head<n_pose_pos_components>();
		for(size_t i = 0, n = r_vertices.size(); i < n; ++ i)
			v_center_gt3.template head<n_pose_pos_components>() += r_vertices[i].template head<n_pose_pos_components>();
		v_center3.template head<n_pose_pos_components>() /= double(r_tar_vertices.size());
		v_center_gt3.template head<n_pose_pos_components>() /= double(r_vertices.size());
		// calculate centers of positions, potentially extend to 3D

		Eigen::Matrix3d t_cov = Eigen::Matrix3d::Zero();
		for(size_t i = 0, n = std::min(r_tar_vertices.size(), r_vertices.size()); i < n; ++ i) {
			Eigen::Vector3d vert_i = Eigen::Vector3d::Zero(), gt_vert_i = Eigen::Vector3d::Zero();
			vert_i.template head<n_pose_pos_components>() = r_tar_vertices[i].template head<n_pose_pos_components>();
			gt_vert_i.template head<n_pose_pos_components>() = r_vertices[i].template head<n_pose_pos_components>();
			// get both vertices, potentially extend them to 3D

			t_cov.noalias() += (gt_vert_i - v_center_gt3) * (vert_i - v_center3).transpose();
			// accumulate
		}
		// calculate the covariance matrix (always performed in double precision to avoid roundoff)

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(t_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
		// calculate the SVD

		double f_det = (svd.matrixV() * svd.matrixU().transpose()).determinant();
		Eigen::Vector3d e(1, 1, (f_det < 0)? -1 : 1);
		// calculate determinant of V*U^T to disambiguate rotation sign

		Eigen::Matrix3d R;
		R.noalias() = svd.matrixV() * e.asDiagonal() * svd.matrixU().transpose();
		// compute the rotation part

		R = Eigen::Quaterniond(R).normalized().toRotationMatrix();
		// renormalize the rotation

		Eigen::Vector3d v_translation3 = v_center3 - R * v_center_gt3;
		// want to align center with ground truth

		Eigen::Vector6d v_rel_pose6;
		v_rel_pose6.head<3>() = v_translation3;
		Eigen::VectorBlock<Eigen::Vector6d, 3> v_rot_part = v_rel_pose6.template tail<3>(); // g++ requires a temporary
		C3DJacobians::Quat_to_AxisAngle(Eigen::Quaterniond(R), v_rot_part);
		// make a pose that relates the two sets of poses

		_TyVector v_rel_pose;
		if(b_2D_SLAM) {
			Eigen::Vector3d v_rel_pose3;
			v_rel_pose3.template head<n_pose_pos_components>() = v_rel_pose6.template head<n_pose_pos_components>(); // compiles for both SE(2) and SE(3)
			v_rel_pose3.template tail<n_pose_rot_components>()(0) = v_rel_pose6(5); // want in-plane rotation // compiles for both SE(2) and SE(3), only makes sense in SE(2)
			v_rel_pose.template head<3>() = v_rel_pose3; // compiles for both SE(2) and SE(3), only makes sense in SE(2)
		} else
			v_rel_pose.template head<n_pose_dimension>() = v_rel_pose6.template head<n_pose_dimension>(); // compiles for both SE(2) and SE(3), only makes sense in SE(3)
		// convert to the current coordinate system, pay mind to both branches actually having
		// to compile in both 2D and 3D (but only the respective branch is taken at runtime)

		return v_rel_pose;
	}

	/**
	 *	@brief a helper function that loads a 2D dataset and separates it to odometry edges and loop closures
	 *
	 *	@param[in] p_s_input_file is null-terminated string, containing file name of a dataset
	 *	@param[out] r_edges is vector of edges to add the odometry edges to
	 *	@param[out] r_loop_closures is vector of edges to add the loop closing edges to
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Load_SeparatedDataset(const char *p_s_input_file,
		std::vector<_TyEdgeData> &r_edges, std::vector<_TyEdgeData> &r_loop_closures)
	{
		if(!Load_Dataset(p_s_input_file, r_edges))
			return false;
		std::stable_sort(r_edges.begin(), r_edges.end(), COdometryFirst()); // sort odometry edges first
		// put all dataset edges in a list

		for(size_t i = 0, n = r_edges.size(), n_vertex_num = 0; i < n; ++ i) {
			const _TyEdgeData &r_edge = r_edges[i];
			if(r_edge.p_vertex[0] < n_vertex_num &&
			   r_edge.p_vertex[1] < n_vertex_num) {
				// both edges are already in the system, this is a loop closure

				r_loop_closures.push_back(r_edge);
				// put it to the list of loop closures

				r_edges.erase(r_edges.begin() + i);
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
		// but then the fake "sensor matching" code would have to be at the beginning of the loop,
		// which seemed counterintuitive for a code example

		return true;
	}

	/**
	 *	@brief removes a single pose from the graph
	 *
	 *	@param[in] n_remove_vertex is zero-based index of the vertex to be removed
	 *	@param[in,out] r_edges is vector of odometry edges
	 *	@param[in,out] r_loop_closures is vector of loop closing edges
	 *	@param[in] n_cur_odometry_edge is index of the odometry edge being currently processed
	 *		(it and the preceding edges were already processed and do not require modification)
	 *
	 *	@return Returns true on success, false on failure (odometry chain brokend, unable to continue).
	 */
	static bool Dataset_RemovePose(size_t n_remove_vertex, std::vector<_TyEdgeData> &r_edges,
		std::vector<_TyEdgeData> &r_loop_closures, size_t n_cur_odometry_edge)
	{
		{
			for(size_t j = n_cur_odometry_edge + 1, m = r_edges.size(); j < m; ++ j) { // dont bother with edges we already processed
				_TyEdgeData &r_edge = r_edges[j];
				if(r_edge.p_vertex[0] >= n_remove_vertex)
					-- r_edge.p_vertex[0];
				if(r_edge.p_vertex[1] >= n_remove_vertex)
					-- r_edge.p_vertex[1];
				if(r_edge.p_vertex[0] == r_edge.p_vertex[1] ||
				   r_edge.p_vertex[0] == size_t(-1) || r_edge.p_vertex[1] == size_t(-1)) {
					r_edges.erase(r_edges.begin() + j);
					-- m;
					-- j; // !!
					// the edge somehow degenerated, remove it
				}
			}
			size_t n = r_edges.size(); // adjust the outer loop limit, if needed
			// update odometry vertex indices

			if(n_cur_odometry_edge != size_t(-1) && n_cur_odometry_edge + 1 < n) {
				const _TyEdgeData &r_next_edge = r_edges[n_cur_odometry_edge + 1];
				size_t n_vertex0 = r_next_edge.p_vertex[0];
				size_t n_vertex1 = r_next_edge.p_vertex[1];
				if(n_vertex0 > n_vertex1)
					std::swap(n_vertex0, n_vertex1);
				// get vertices of the next edge

				size_t n_last_vertex = n_remove_vertex; // the same index, since we removed the one which we did not want to keep
				if(n_vertex0 + 1 != n_vertex1 || n_vertex1 != n_last_vertex)
					return false;
			}
			// make sure that the next edge is on the last two vertices, so that we can
			// do edge replace (if the graph contains simple 0->1, 1->2, ... (n - 1)->n
			// odometry then this will work, otherwise problems may arise)

			for(size_t j = 0, m = r_loop_closures.size(); j < m; ++ j) {
				_TyEdgeData &r_edge = r_loop_closures[j];
				if(r_edge.p_vertex[0] == n_remove_vertex ||
				   r_edge.p_vertex[1] == n_remove_vertex) {
					r_loop_closures.erase(r_loop_closures.begin() + j);
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
					r_loop_closures.erase(r_loop_closures.begin() + j);
					-- m;
					-- j; // !!
					// the edge somehow degenerated, remove it
				}
			}
			// update the loop closure vertex indices
		}
		// remove the next vertex from the graph

		return true;
	}

	/**
	 *	@brief reads a vector of size_t, stored as binary 32-bit numbers
	 *
	 *	@param[in] p_fr is pointer to the file to read from
	 *	@param[out] r_vector is vector filled with values from the file
	 *
	 *	@return Returns true on success, false on failure (the output vector may contain some of the values on output).
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static bool Read_Vector32(FILE *p_fr, std::vector<size_t> &r_vector) // throw(std::bad_alloc)
	{
		uint32_t n_size;
		if(fread(&n_size, sizeof(uint32_t), 1, p_fr) != 1)
			return false;
		r_vector.resize(n_size);
		if(n_size && sizeof(uint32_t) == sizeof(size_t)) // if empty, avoid calling r_vector[0]!
			return fread(&r_vector[0], sizeof(uint32_t), n_size, p_fr) == n_size;
		else {
			for(uint32_t i = 0; i < n_size; ++ i) {
				uint32_t n_value;
				if(fread(&n_value, sizeof(uint32_t), 1, p_fr) != 1)
					return false;
				r_vector[i] = n_value;
			}
			return true;
		}
	}

	/**
	 *	@brief writes a vector of size_t as binary 32-bit numbers
	 *
	 *	@param[in] p_fw is pointer to the file to write to
	 *	@param[in] r_vector is vector to be written
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Write_Vector32(FILE *p_fw, const std::vector<size_t> &r_vector)
	{
		if(r_vector.size() > UINT32_MAX)
			return false;
		const uint32_t n_size = uint32_t(r_vector.size());
		if(fwrite(&n_size, sizeof(uint32_t), 1, p_fw) != 1)
			return false;
		if(n_size && sizeof(size_t) == sizeof(uint32_t)) // if empty, dont bother calling fwrite()
			return fwrite(&r_vector[0], sizeof(uint32_t), n_size, p_fw) == n_size;
		else {
			for(uint32_t i = 0; i < n_size; ++ i) {
				_ASSERTE(r_vector[i] <= UINT32_MAX); // this is also an issue
				uint32_t n_value = uint32_t(r_vector[i]);
				if(fwrite(&n_value, sizeof(uint32_t), 1, p_fw) != 1)
					return false;
			}
			return true;
		}
	}

	/**
	 *	@brief writes binary data for the ICRA 2015 demo
	 *
	 *	@param[in] p_fw_animdata is pointer to the output file, opened for binary writing
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in] r_t_last_sigma is covariance of the last pose
	 *	@param[in] r_probable_matches is a list of probable matching vertex indices
	 *	@param[in] r_true_matches is a list of indices of vertices where matches were established
	 *
	 *	@note This function does not depend on any of the template parameters and could
	 *		be implemented outside this class. For simplicity, it is kept inline.
	 */
	static void Write_AnimationData(FILE *p_fw_animdata, const CSystemType &r_system,
		const Eigen::MatrixXd &r_t_last_sigma, const std::vector<size_t> &r_probable_matches,
		const std::vector<size_t> &r_true_matches) // todo - template params independent
	{
		_ASSERTE(r_system.r_Vertex_Pool().n_Size() - 1 <= UINT32_MAX);
		uint32_t n_last_vertex = uint32_t(r_system.r_Vertex_Pool().n_Size() - 1);
		_ASSERTE(r_true_matches.size() <= UINT32_MAX && r_probable_matches.size() <= UINT32_MAX);
		uint32_t n_match_num = uint32_t(r_true_matches.size()), n_test_num = uint32_t(r_probable_matches.size());

		fwrite(&n_last_vertex, sizeof(uint32_t), 1, p_fw_animdata);
		// write the vertex id

		{
			Eigen::Vector2d v_right, v_up;
			{
				//Eigen::LLT<_TyMatrix, Eigen::Upper> chol_sigma(r_t_last_sigma.template topLeftCorner<2, 2>());
				//_TyMatrix R = chol_sigma.matrixU(); // does not quite work, but gets close
				Eigen::JacobiSVD<_TyMatrix, Eigen::NoQRPreconditioner>
					svd_sigma(r_t_last_sigma.template topLeftCorner<n_pose_dimension, n_pose_dimension>(), Eigen::ComputeFullU/*| Eigen::ComputeFullV*/);
				_TyMatrix R = svd_sigma.matrixU() * svd_sigma.singularValues().array().sqrt().matrix().asDiagonal();
				// calculate a slightly different RRT decomposition using SVD

				//double f_decomp_err = (R * R.transpose() - r_t_last_sigma.template topLeftCorner<n_pose_dimension, n_pose_dimension>()).norm();
				//printf("%g\n", f_decomp_err); // debug - should be zero

				v_right = (R.col(0) * 2.1459).template head<2>();
				v_up = (R.col(1) * 2.1459).template head<2>();
				// (90% confidence level is obtained at 2.1459)
			}
			// calculate basis for an ellipse (project to 2D)

			fwrite(v_right.data(), sizeof(double), 2, p_fw_animdata);
			fwrite(v_up.data(), sizeof(double), 2, p_fw_animdata);
		}
		// write the ellipse parameters for this vertex

		Write_Vector32(p_fw_animdata, r_probable_matches);
		Write_Vector32(p_fw_animdata, r_true_matches);
		// write the matches
	}

	/**
	 *	@brief finalizes the binary stream for the ICRA 2015 demo
	 *	@param[in] p_fw_animdata is pointer to the output file, opened for binary writing (will be closed)
	 *	@note This function does not depend on any of the template parameters and could
	 *		be implemented outside this class. For simplicity, it is kept inline.
	 */
	static void Finalize_AnimData(FILE *p_fw_animdata) // todo - template params independent
	{
		uint32_t n_null_vertex = 0;
		fwrite(&n_null_vertex, sizeof(uint32_t), 1, p_fw_animdata);
		if(ferror(p_fw_animdata))
			fprintf(stderr, "error: write error(s) occured while writing \'%s\'\n", "covanim_data.dat");
		fclose(p_fw_animdata);
	}

	/**
	 *	@brief rasterizes an image of the current state (projected on the x-y plane)
	 *
	 *	@param[in] r_v_min is coordinates of the top left corner of the visualized area
	 *	@param[in] r_v_max is coordinates of the bottom right corner of the visualized area
	 *	@param[in] n_resolution is output image resolution, in pixels
	 *	@param[out] r_p_image is a storage for the rasterized image (contains a newly allocated image upon successful return)
	 *	@param[out] r_v_off is filled with the offset of the state coordinates in the raster coordinates
	 *	@param[out] r_v_scale is filled with the scale relating the state coordinates to the raster coordinates
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This function does not depend on any of the template parameters and could
	 *		be implemented outside this class. For simplicity, it is kept inline.
	 */
	static bool Alloc_Bitmap(const Eigen::Vector2d &r_v_min, const Eigen::Vector2d &r_v_max,
		int n_resolution, TBmp *&r_p_image, Eigen::Vector2d &r_v_off, Eigen::Vector2d &r_v_scale) // todo - template params independent, bool b_2D_SLAM is enough // todo - naming conventions, mark functions that do not depend on template params
	{
		Eigen::Vector2d v_size = r_v_max - r_v_min;
		double f_short_side = std::min(v_size(0), v_size(1));
		double f_scale = n_resolution / f_short_side;
		int n_width = std::max(1, int(v_size(0) * f_scale));
		int n_height = std::max(1, int(v_size(1) * f_scale));
		// calculate image size

		r_v_off = -r_v_min;
		r_v_scale.setConstant(f_scale);
		// set offset and scale

		r_v_scale(1) = -f_scale;
		r_v_off(1) += n_height / r_v_scale(1);
		// the y axis is flipped, need to modify offset and scale

		return (r_p_image = TBmp::p_Alloc(n_width, n_height)) != 0;
		// alloc image
	}

	/**
	 *	@brief updates the bounding box of the vertices
	 *
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in] b_bb_reset is bounding box reset flag (if set, the bounding box is calculated only for the given
	 *		vertices; if clered, the bounding box is extended to fit the given vertices)
	 *	@param[in,out] r_v_min is minimum coordinate of the system bounding box (updated to fit the rasterized images)
	 *	@param[in,out] r_v_max is maximum coordinate of the system bounding box (updated to fit the rasterized images)
	 *	@param[in] b_xz_projection is x-z projection flag (if set, the projection is x-z, otherwise it is x-y)
	 */
	static void Draw_BoundingBoxUpdate(const CSystemType &r_system, bool b_bb_reset,
		Eigen::Vector2d &r_v_min, Eigen::Vector2d &r_v_max, bool b_xz_projection)
	{
		const typename CSystemType::_TyVertexMultiPool &r_vertex_pool = r_system.r_Vertex_Pool();
		for(size_t j = 0, m = r_vertex_pool.n_Size(); j < m; ++ j) {
			Eigen::Map<const Eigen::VectorXd> t_vert = r_vertex_pool[j].v_State();
			// get a vertex

			if(b_xz_projection) {
				if(b_bb_reset && !j)
					r_v_min = r_v_max = Eigen::Vector2d(t_vert(0), t_vert(2));
				else {
					r_v_min = r_v_min.array().min(Eigen::Vector2d(t_vert(0), t_vert(2)).array()).matrix();
					r_v_max = r_v_max.array().max(Eigen::Vector2d(t_vert(0), t_vert(2)).array()).matrix();
				}
			} else {
				if(b_bb_reset && !j)
					r_v_min = r_v_max = t_vert.head<2>();
				else {
					r_v_min = r_v_min.array().min(t_vert.head<2>().array()).matrix();
					r_v_max = r_v_max.array().max(t_vert.head<2>().array()).matrix();
				}
			}
			// update the bounding box
		}
	}

	/**
	 *	@brief draws ticks under the vertices
	 *
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in,out] p_image is a temporary storage for the rasterized image (contains the image upon return)
	 *	@param[in] r_v_off is offset of the state coordinates in the raster coordinates
	 *	@param[in] r_v_scale is scale relating the state coordinates to the raster coordinates
	 *	@param[in] b_xz_projection is x-z projection flag (if set, the projection is x-z, otherwise it is x-y)
	 */
	static void Draw_VertexTicks(const CSystemType &r_system, TBmp *p_image,
		const Eigen::Vector2d &r_v_off, const Eigen::Vector2d &r_v_scale, bool b_xz_projection)
	{
		const typename CSystemType::_TyVertexMultiPool &r_vertex_pool = r_system.r_Vertex_Pool();
		for(size_t j = 0, m = r_vertex_pool.n_Size(); j < m; ++ j) {
			Eigen::Map<const Eigen::VectorXd> t_vert = r_vertex_pool[j].v_State();
			// get a vertex

			float f_x = float((t_vert(0) + r_v_off(0)) * r_v_scale(0));
			float f_y = float((t_vert((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
			// transform vertex position to raster coordinates

			const uint32_t n_red = 0xffff0000U;
			p_image->DrawLine_AA(f_x - 10, f_y, f_x + 10, f_y, n_red, 2);
			p_image->DrawLine_AA(f_x, f_y - 10, f_x, f_y + 10, n_red, 2);
			// draw a red cross under the vertex
		}
	}

	/**
	 *	@brief draws edge lines
	 *
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in,out] p_image is a temporary storage for the rasterized image (contains the image upon return)
	 *	@param[in] r_v_off is offset of the state coordinates in the raster coordinates
	 *	@param[in] r_v_scale is scale relating the state coordinates to the raster coordinates
	 *	@param[in] b_xz_projection is x-z projection flag (if set, the projection is x-z, otherwise it is x-y)
	 *	@param[in] n_color is color to draw with
	 *	@param[in] n_thickness is line thickness in pixels
	 *	@param[in] n_loop_odo_draw_flag is edge draw flag (2 = loops only, 1 = odometry only, 3 = draw all)
	 */
	static void Draw_Edges(const CSystemType &r_system, TBmp *p_image,
		const Eigen::Vector2d &r_v_off, const Eigen::Vector2d &r_v_scale, bool b_xz_projection,
		const uint32_t n_color = 0xff0000ffU, float f_thickness = 5, int n_loop_odo_draw_flag = 3)
	{
		const typename CSystemType::_TyEdgeMultiPool &r_edge_pool = r_system.r_Edge_Pool();
		const typename CSystemType::_TyVertexMultiPool &r_vertex_pool = r_system.r_Vertex_Pool();
		std::pair<size_t, size_t> t_prev(-1, -1);
		for(size_t j = 0, m = r_edge_pool.n_Size(); j < m; ++ j) {
			typename CSystemType::_TyEdgeMultiPool::_TyConstBaseRef r_edge = r_edge_pool[j];
			std::pair<size_t, size_t> t_edge(r_edge.n_Vertex_Id(0), r_edge.n_Vertex_Id(1));
			if(t_edge.first > t_edge.second)
				std::swap(t_edge.first, t_edge.second);
			Eigen::Map<const Eigen::VectorXd> t_vert0 = r_vertex_pool[t_edge.first].v_State();
			Eigen::Map<const Eigen::VectorXd> t_vert1 = r_vertex_pool[t_edge.second].v_State();
			// get the edge and the vertices

			float f_x0 = float((t_vert0(0) + r_v_off(0)) * r_v_scale(0));
			float f_y0 = float((t_vert0((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
			float f_x1 = float((t_vert1(0) + r_v_off(0)) * r_v_scale(0));
			float f_y1 = float((t_vert1((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
			// transform vertex positions to raster coordinates

			bool b_loop = t_edge.second != t_edge.first + 1 || t_edge == t_prev;
			// edge is a loop if it is not between consecutive vertices, or if it is not the
			// first edge between those two vertices

			bool b_draw = ((n_loop_odo_draw_flag >> ((b_loop)? 1 : 0)) & 1) != 0;
			// if the edge is a loop, the second least significant bit must be set
			// if it is odometry, the least significant bit must be set

			if(b_draw) {
				p_image->DrawLine_AA(f_x0, f_y0, f_x1, f_y1, n_color,
					f_thickness, (n_loop_odo_draw_flag == 2)? 2 : 1);
			}
			// draw edge, use round ends if only drawing loop closures

			t_prev = t_edge;
		}
	}

	/**
	 *	@brief draws match lines
	 *
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in,out] p_image is a temporary storage for the rasterized image (contains the image upon return)
	 *	@param[in] r_v_off is offset of the state coordinates in the raster coordinates
	 *	@param[in] r_v_scale is scale relating the state coordinates to the raster coordinates
	 *	@param[in] b_xz_projection is x-z projection flag (if set, the projection is x-z, otherwise it is x-y)
	 *	@param[in] n_color is color to draw with
	 *	@param[in] n_thickness is line thickness in pixels
	 */
	static void Draw_Matches(const CSystemType &r_system,
		const std::vector<size_t> &r_match_list, size_t n_match_vertex, TBmp *p_image,
		const Eigen::Vector2d &r_v_off, const Eigen::Vector2d &r_v_scale, bool b_xz_projection,
		const uint32_t n_color = 0xff000000U, float f_thickness = 1)
	{
		const typename CSystemType::_TyEdgeMultiPool &r_edge_pool = r_system.r_Edge_Pool();
		const typename CSystemType::_TyVertexMultiPool &r_vertex_pool = r_system.r_Vertex_Pool();
		{
			Eigen::Map<const Eigen::VectorXd> t_vert0 = r_vertex_pool[n_match_vertex].v_State();
			// get the last vertex

			float f_x0 = float((t_vert0(0) + r_v_off(0)) * r_v_scale(0));
			float f_y0 = float((t_vert0((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));
			// transform vertex positions to raster coordinates

			for(size_t j = 0, m = r_match_list.size(); j < m; ++ j) {
				Eigen::Map<const Eigen::VectorXd> t_vert1 = r_vertex_pool[r_match_list[j]].v_State();
				float f_x1 = float((t_vert1(0) + r_v_off(0)) * r_v_scale(0));
				float f_y1 = float((t_vert1((b_xz_projection)? 2 : 1) + r_v_off(1)) * r_v_scale(1));

				p_image->DrawLine_AA(f_x0, f_y0, f_x1, f_y1, n_color, f_thickness, 2); // use round ends here
				// draw edge
			}
			// draw the probable matches
		}
	}

	struct TGreenTint { // g++ requires this to be not inside the function
		inline void operator ()(uint32_t &r_pixel)
		{
			TBmp::AlphaBlend(r_pixel, 0xffccffccU, 0x59); // alpha blend green color in
		}
	};

	/**
	 *	@brief draws a confidence ellipse around a selected vertex
	 *
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in] n_vertex is zero-based index of the vertex to draw the ellipse around
	 *	@param[in] r_t_sigma is covariance of the last pose
	 *	@param[in,out] p_image is a temporary storage for the rasterized image (contains the image upon return)
	 *	@param[in] r_v_off is offset of the state coordinates in the raster coordinates
	 *	@param[in] r_v_scale is scale relating the state coordinates to the raster coordinates
	 *	@param[in] b_xz_projection is x-z projection flag (if set, the projection is x-z, otherwise it is x-y)
	 */
	static void Draw_ConfidenceInterval(const CSystemType &r_system, size_t n_vertex, const Eigen::MatrixXd &r_t_sigma,
		TBmp *p_image, const Eigen::Vector2d &r_v_off, const Eigen::Vector2d &r_v_scale, bool b_xz_projection)
	{
		const typename CSystemType::_TyVertexMultiPool &r_vertex_pool = r_system.r_Vertex_Pool();
		Eigen::Map<const Eigen::VectorXd> t_vert = r_vertex_pool[n_vertex].v_State();
		// get the vertex

		Eigen::Vector2d v_right, v_up;
		if(b_xz_projection) {
			Eigen::Matrix3d t_sigma_2D = r_t_sigma.template topLeftCorner<3, 3>();
			t_sigma_2D.col(1) = t_sigma_2D.col(2);
			t_sigma_2D.row(1) = t_sigma_2D.row(2);
			// get the 3x3 piece of sigma and push column 2 to column 1 and row 2 to row 1 to get x, z in the first two places

			Eigen::JacobiSVD<Eigen::Matrix2d, Eigen::NoQRPreconditioner>
				svd_sigma(t_sigma_2D.template topLeftCorner<2, 2>(), Eigen::ComputeFullU/*| Eigen::ComputeFullV*/);
			Eigen::Matrix2d R = svd_sigma.matrixU() * svd_sigma.singularValues().array().sqrt().matrix().asDiagonal();
			// calculate a slightly different RRT decomposition using SVD

			v_right = R.col(0) * 2.1459;
			v_up = R.col(1) * 2.1459;
			// (90% confidence level is obtained at 2.1459)
		} else {
			Eigen::JacobiSVD<Eigen::Matrix2d, Eigen::NoQRPreconditioner>
				svd_sigma(r_t_sigma.template topLeftCorner<2, 2>(), Eigen::ComputeFullU/*| Eigen::ComputeFullV*/);
			Eigen::Matrix2d R = svd_sigma.matrixU() * svd_sigma.singularValues().array().sqrt().matrix().asDiagonal();
			// calculate a slightly different RRT decomposition using SVD

			v_right = R.col(0) * 2.1459;
			v_up = R.col(1) * 2.1459;
			// (90% confidence level is obtained at 2.1459)
		}
		// calculate basis for an ellipse

		Eigen::Vector2d v_prev, v_cener = Eigen::Vector2d(t_vert(0), t_vert((b_xz_projection)? 2 : 1));
		Eigen::Vector2d v_cener_rast = ((v_cener + r_v_off).array() * r_v_scale.array()).matrix();
		for(int i = 0; i < 101; ++ i) {
			double f_angle = (i % 100) * .01 * 2 * M_PI;
			Eigen::Vector2d v_pos = v_cener + cos(f_angle) * v_right + sin(f_angle) * v_up;
			// calculate point on an ellipse

			v_pos = ((v_pos + r_v_off).array() * r_v_scale.array()).matrix();
			// transform to raster coordinates

			if(i) {
				const float p_vertex[3][3] = {
					float(v_prev(0)), float(v_prev(1)), 1,
					float(v_pos(0)), float(v_pos(1)), 1,
					float(v_cener_rast(0)), float(v_cener_rast(1)), 1
				};
				p_image->DrawTriangle_Shader(p_vertex, TGreenTint());
			}
			v_prev = v_pos;
		}
		// fill confidence interval around the current robot pose

		for(int i = 0; i < 101; ++ i) {
			double f_angle = (i % 100) * .01 * 2 * M_PI;
			Eigen::Vector2d v_pos = v_cener + cos(f_angle) * v_right + sin(f_angle) * v_up;
			// calculate point on an ellipse

			v_pos = ((v_pos + r_v_off).array() * r_v_scale.array()).matrix();
			// transform to raster coordinates

			if(i) {
				const uint32_t n_gray = 0xff667f66U;//0xff808080U; // make it green but darker than the inside
				p_image->DrawLine_AA(float(v_prev(0)), float(v_prev(1)),
					float(v_pos(0)), float(v_pos(1)), n_gray, 1);
			}
			v_prev = v_pos;
		}
		// draw confidence interval around the current robot pose
	}

	/**
	 *	@brief rasterizes an image of the current state (projected on the x-y plane)
	 *
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in] i is zero-based index of the current edge (used to create output file names)
	 *	@param[in] r_t_last_sigma is covariance of the last pose
	 *	@param[in] r_probable_matches is a list of probable matching vertex indices
	 *	@param[in] r_true_matches is a list of indices of vertices where matches were established
	 *	@param[in,out] p_image is a temporary storage for the rasterized image (contains the image upon return)
	 *	@param[in] r_v_off is offset of the state coordinates in the raster coordinates
	 *	@param[in] r_v_scale is scale relating the state coordinates to the raster coordinates
	 *	@param[in,out] r_v_min is minimum coordinate of the system bounding box (updated to fit the rasterized images)
	 *	@param[in,out] r_v_max is maximum coordinate of the system bounding box (updated to fit the rasterized images)
	 *	@param[in] b_xz_projection is x-z projection flag (if set, the projection is x-z, otherwise it is x-y)
	 *	@param[in] b_highlight_loops is loop highlight flag
	 */
	static void Draw_SolutionFrame(const CSystemType &r_system, size_t i, const Eigen::MatrixXd &r_t_last_sigma,
		const std::vector<size_t> &r_probable_matches, const std::vector<size_t> &r_true_matches,
		TBmp *p_image, const Eigen::Vector2d &r_v_off, const Eigen::Vector2d &r_v_scale,
		Eigen::Vector2d &r_v_min, Eigen::Vector2d &r_v_max, bool b_xz_projection, bool b_highlight_loops)
	{
		p_image->Clear(-1);
		// clear to white

		Draw_BoundingBoxUpdate(r_system, !i, r_v_min, r_v_max, b_xz_projection);

		Draw_VertexTicks(r_system, p_image, r_v_off, r_v_scale, b_xz_projection);
		if(b_highlight_loops) {
			Draw_Edges(r_system, p_image, r_v_off, r_v_scale, b_xz_projection, 0xff0000ffU, 5, 1);
			// draw odometry

			Draw_Edges(r_system, p_image, r_v_off, r_v_scale, b_xz_projection, 0xff00ff00U, /*3*/5, 2); // thick for the video
			// draw loop closures, slightly thinner
		} else
			Draw_Edges(r_system, p_image, r_v_off, r_v_scale, b_xz_projection);

		size_t n_last_vertex = r_system.r_Vertex_Pool().n_Size() - 1;
		// get the last vertex

		{
			//const uint32_t n_light_gray = 0xffddddddU;
			//Draw_Matches(r_system, r_probable_matches, n_last_vertex, p_image, r_v_off, r_v_scale, b_xz_projection, n_light_gray, 1);
			// not enough

			const uint32_t n_black = 0xff000000U;
			Draw_Matches(r_system, r_probable_matches, n_last_vertex, p_image, r_v_off, r_v_scale, b_xz_projection, n_black, 1);
			// more visible

			//const uint32_t n_orange = 0xffffa500u;
			const uint32_t n_green = 0xff00ff00u; // as per popular request
			Draw_Matches(r_system, r_true_matches, n_last_vertex, p_image, r_v_off, r_v_scale, b_xz_projection, n_green, 5);
			// draw the matches
		}

		Draw_ConfidenceInterval(r_system, n_last_vertex, r_t_last_sigma, p_image, r_v_off, r_v_scale, b_xz_projection);
		// draw the confidence interval

		char p_s_filename[256];
		sprintf(p_s_filename, "anim/frame_%04" _PRIsize ".tga", i);
		CTgaCodec::Save_TGA(p_s_filename, *p_image, false);
		// save as a bitmap
	}

	/**
	 *	@brief rasterizes an image of the current state (projected on the x-y plane)
	 *
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in,out] p_image is a temporary storage for the rasterized image (contains the image upon return)
	 *	@param[in] r_v_off is offset of the state coordinates in the raster coordinates
	 *	@param[in] r_v_scale is scale relating the state coordinates to the raster coordinates
	 *	@param[in] r_v_min is minimum coordinate of the system bounding box (updated to fit the rasterized images)
	 *	@param[in] r_v_max is maximum coordinate of the system bounding box (updated to fit the rasterized images)
	 *	@param[in] b_xz_projection is x-z projection flag (if set, the projection is x-z, otherwise it is x-y)
	 *	@param[in] b_highlight_loops is loop highlight flag
	 *	@param[in] b_bbox_reset is bounding box reset flag (set if the incremental bitmaps are disabled)
	 */
	static void Draw_FinalFrame(const CSystemType &r_system, TBmp *p_image,
		const Eigen::Vector2d &r_v_off, const Eigen::Vector2d &r_v_scale,
		Eigen::Vector2d &r_v_min, Eigen::Vector2d &r_v_max, bool b_xz_projection,
		bool b_highlight_loops, bool b_bbox_reset)
	{
		p_image->Clear(-1);
		// clear to white

		Draw_BoundingBoxUpdate(r_system, b_bbox_reset, r_v_min, r_v_max, b_xz_projection);

		Draw_VertexTicks(r_system, p_image, r_v_off, r_v_scale, b_xz_projection);
		if(b_highlight_loops) {
			Draw_Edges(r_system, p_image, r_v_off, r_v_scale, b_xz_projection, 0xff0000ffU, 5, 1);
			// draw odometry

			Draw_Edges(r_system, p_image, r_v_off, r_v_scale, b_xz_projection, 0xff00ff00U, 3, 2);
			// draw loop closures, slightly thinner
		} else
			Draw_Edges(r_system, p_image, r_v_off, r_v_scale, b_xz_projection);

		size_t n_last_vertex = r_system.r_Vertex_Pool().n_Size() - 1;
		// get the last vertex

		CTgaCodec::Save_TGA("result.tga", *p_image, false);
		// save as a bitmap
	}

	/**
	 *	@brief some code for edge composition which i dont want to delete just yet
	 *		(unused in the calculation though, note that this method is not static)
	 *
	 *	@param[in,out] v_cum_measurement is cumulative measurement
	 *	@param[in,out] t_cum_information is cumulative information matrix
	 *	@param[in] r_edge is edge to be appended to the cummulative measurement / information
	 */
	void EdgeComposition_CodeDump(_TyVector &v_cum_measurement, _TyMatrix &t_cum_information, const _TyEdgeData &r_edge)
	{
#if 0 // method of preserving last pose covariance. very conservative, some numerical issues
		_TyMatrix t_sigma_0 = _TyMatrix::Identity();
		_TyVector v_origin = _TyVector::Zero();

		if(0) { // doesn't work particularly well
			size_t n_last_vertex = system.r_Vertex_Pool().n_Size() - 2; // the next is going to be replaced
			v_origin = system.r_Vertex_Pool()[n_last_vertex].v_State();
			t_sigma_0 = solver.r_MarginalCovariance().r_SparseMatrix().t_GetBlock_Log(n_last_vertex, n_last_vertex);
		}
		// can use the last vertex as origin

		_TyMatrix t_sigma_inv1 = t_cum_information; // the first edge
		_TyMatrix t_sigma_inv0 = r_edge.t_information; // the second edge

		Eigen::Matrix<double, 3 * n_pose_dimension, 3 * n_pose_dimension> inc_sys; // system where we increment
		Eigen::Matrix<double, 2 * n_pose_dimension, 2 * n_pose_dimension> comp_sys; // a system where we compose
		_TyMatrix J0, J1, K0, K1, L0, L1;
		{
			_TyVector v_moving_pose, v_endpoint; // temp, the first and the third argument cannot be the same
			Relative_to_Absolute(v_origin, v_cum_measurement, v_moving_pose, J0, J1);
			Relative_to_Absolute(v_moving_pose, r_edge.v_measurement, v_endpoint, K0, K1);
			// get jacobians

			inc_sys.setZero();

			inc_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				topLeftCorner<n_pose_dimension, n_pose_dimension>() = J0.transpose() * t_sigma_inv0 * J0 + t_sigma_0;
			inc_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				topRightCorner<n_pose_dimension, n_pose_dimension>() = J0.transpose() * t_sigma_inv0 * J1;
			inc_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				bottomLeftCorner<n_pose_dimension, n_pose_dimension>() =
				inc_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				topRightCorner<n_pose_dimension, n_pose_dimension>().transpose();
			inc_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				bottomRightCorner<n_pose_dimension, n_pose_dimension>() = J1.transpose() * t_sigma_inv0 * J1;

			inc_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				topLeftCorner<n_pose_dimension, n_pose_dimension>() += K0.transpose() * t_sigma_inv1 * K0;
			inc_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				topRightCorner<n_pose_dimension, n_pose_dimension>() += K0.transpose() * t_sigma_inv1 * K1;
			inc_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				bottomLeftCorner<n_pose_dimension, n_pose_dimension>() +=
				inc_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				topRightCorner<n_pose_dimension, n_pose_dimension>().transpose();
			inc_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
				bottomRightCorner<n_pose_dimension, n_pose_dimension>() = K1.transpose() * t_sigma_inv1 * K1;
		}
		_TyMatrix t_cov_endpoint = inc_sys.inverse().template bottomRightCorner<n_pose_dimension, n_pose_dimension>();

		_TyVector v_composite_measurement;
		_TyMatrix t_sigma_compose;
		{
			Relative_to_Absolute(v_cum_measurement, r_edge.v_measurement, v_composite_measurement);
			_TyVector v_endpoint; // temp, the first and the third argument cannot be the same
			Relative_to_Absolute(v_origin, v_composite_measurement, v_endpoint, L0, L1);
			// get jacobians

			t_sigma_compose = (L1 * t_cov_endpoint * L1.transpose() - L0 * t_sigma_0.inverse() * L0.transpose()).inverse();

			comp_sys.template topLeftCorner<n_pose_dimension, n_pose_dimension>() = L0.transpose() * t_sigma_compose * L0 + t_sigma_0;
			comp_sys.template topRightCorner<n_pose_dimension, n_pose_dimension>() = L0.transpose() * t_sigma_compose * L1;
			comp_sys.template bottomLeftCorner<n_pose_dimension, n_pose_dimension>() =
				comp_sys.template topRightCorner<n_pose_dimension, n_pose_dimension>().transpose();
			comp_sys.template bottomRightCorner<n_pose_dimension, n_pose_dimension>() = L1.transpose() * t_sigma_compose * L1;
		}
		_TyMatrix t_cov_endpoint_comp = comp_sys.inverse().template bottomRightCorner<n_pose_dimension, n_pose_dimension>();

		double f_symmetry_error = (t_sigma_compose - t_sigma_compose.transpose()).norm(); // quite ok
		double f_compose_error = (t_cov_endpoint_comp - t_cov_endpoint).norm(); // ok
		double f_norm_error = comp_sys.inverse().norm() - inc_sys.inverse().norm(); // not ok, evidently impossible at the same time

		v_cum_measurement = v_composite_measurement;
		t_cum_information = t_sigma_compose; // as is, this will often cause not pos def lambda, have to fix it

		/*t_cum_information.triangularView<Eigen::StrictlyUpper>() =
			t_cum_information.triangularView<Eigen::StrictlyUpper>() +
			t_cum_information.triangularView<Eigen::StrictlyLower>().transpose();
		t_cum_information.triangularView<Eigen::StrictlyUpper>() *= .5;
		t_cum_information.triangularView<Eigen::StrictlyLower>() = t_cum_information.triangularView<Eigen::StrictlyUpper>().transpose();*/
		// symmetry is not the problem. it is not positive semidefinite?

		/*Eigen::LDLT<_TyMatrix> ldlt_sigma(t_sigma_compose);
		{
			_TyMatrix PT = _TyMatrix::Identity() * ldlt_sigma.transpositionsP().transpose();
			_TyMatrix L = ldlt_sigma.matrixL();
			_TyVector d = ldlt_sigma.vectorD();
			for(int i = 0; i < n_pose_dimension; ++ i) {
				if(d(i) < 0)
					d(i) = 0;
			}
			_TyMatrix L = PT * (L * d.array().sqrt().matrix().asDiagonal());
			double f_error_0_ = (L * L.transpose() - t_sigma_compose).norm();
		}*/
		// LDLT does not work well here

		Eigen::JacobiSVD<_TyMatrix, Eigen::NoQRPreconditioner> svd_sigma(t_sigma_compose, Eigen::ComputeFullU| Eigen::ComputeFullV);
		_TyMatrix R = svd_sigma.matrixU() * svd_sigma.singularValues().array().sqrt().matrix().asDiagonal();
		// SVD trick works much better

		double f_error_0 = (R * R.transpose() - t_sigma_compose).norm(); // this is usually quite small

		t_cum_information = R * R.transpose();
		// force positive (semi)definite
#elif 0 // Schur complement approach, conservative in the norm of marginals
		{
			//const CUberBlockMatrix &r_marginals = solver.r_MarginalCovariance().r_SparseMatrix();
			// get the marginals

			_TyMatrix J0, J1, K0, K1, L0, L1;
			_TyVector v_origin, v_moving_pose, v_composite_measurement, v_neg_composite;
			v_origin = _TyVector::Zero();//system.r_Vertex_Pool()[system.r_Vertex_Pool().n_Size() - 1].v_State();
			Relative_to_Absolute(v_origin, v_cum_measurement, v_moving_pose, J0, J1); // the first and the third argument cannot be the same
			Relative_to_Absolute(v_moving_pose, r_edge.v_measurement, v_composite_measurement, K0, K1); // the first and the third argument cannot be the same
			Absolute_to_Relative(v_composite_measurement, v_origin, v_neg_composite, L0, L1); // the first and the third argument cannot be the same
			// calculate jacobians of a system with three poses

			_TyMatrix t_sigma_0 = _TyMatrix::Identity();//r_marginals.t_GetBlock_Log(r_marginals.n_BlockColumn_Num() - 2, r_marginals.n_BlockColumn_Num() - 2);//_TyMatrix::Identity(); // unary factor
			_TyMatrix t_sigma_inv1 = t_cum_information; // the first edge
			_TyMatrix t_sigma_inv0 = r_edge.t_information; // the second edge
			Eigen::Matrix<double, 3 * n_pose_dimension, 3 * n_pose_dimension> small_sys; // information matrix
			small_sys.setZero();
			{
				small_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					topLeftCorner<n_pose_dimension, n_pose_dimension>() = J0.transpose() * t_sigma_inv0 * J0;
				small_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					topRightCorner<n_pose_dimension, n_pose_dimension>() = J0.transpose() * t_sigma_inv0 * J1;
				small_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					bottomLeftCorner<n_pose_dimension, n_pose_dimension>() =
					small_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					topRightCorner<n_pose_dimension, n_pose_dimension>().transpose();
				small_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					bottomRightCorner<n_pose_dimension, n_pose_dimension>() = J1.transpose() * t_sigma_inv0 * J1;

				small_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					topLeftCorner<n_pose_dimension, n_pose_dimension>() += K0.transpose() * t_sigma_inv1 * K0;
				small_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					topRightCorner<n_pose_dimension, n_pose_dimension>() += K0.transpose() * t_sigma_inv1 * K1;
				small_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					bottomLeftCorner<n_pose_dimension, n_pose_dimension>() +=
					small_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					topRightCorner<n_pose_dimension, n_pose_dimension>().transpose();
				small_sys.template bottomRightCorner<2 * n_pose_dimension, 2 * n_pose_dimension>().template
					bottomRightCorner<n_pose_dimension, n_pose_dimension>() = K1.transpose() * t_sigma_inv1 * K1;

				small_sys.template topLeftCorner<n_pose_dimension, n_pose_dimension>() += t_sigma_0;
			}
			// form the 3x3 information matrix

			double small_sys_symmetry_err = (small_sys - small_sys.transpose()).norm();

			{
				Eigen::Matrix<double, 3 * n_pose_dimension, 3 * n_pose_dimension> temp;
				/*temp.template block<n_pose_dimension, 3 * n_pose_dimension>(0, 0) = small_sys.template block<n_pose_dimension, 3 * n_pose_dimension>(0, 0);
				temp.template block<n_pose_dimension, 3 * n_pose_dimension>(n_pose_dimension, 0) = small_sys.template block<n_pose_dimension, 3 * n_pose_dimension>(2 * n_pose_dimension, 0);
				temp.template block<n_pose_dimension, 3 * n_pose_dimension>(2 * n_pose_dimension, 0) = small_sys.template block<n_pose_dimension, 3 * n_pose_dimension>(n_pose_dimension, 0);
				small_sys.template block<3 * n_pose_dimension, n_pose_dimension>(0, 0) = temp.template block<3 * n_pose_dimension, n_pose_dimension>(0, 0);
				small_sys.template block<3 * n_pose_dimension, n_pose_dimension>(0, n_pose_dimension) = temp.template block<3 * n_pose_dimension, n_pose_dimension>(0, 2 * n_pose_dimension);
				small_sys.template block<3 * n_pose_dimension, n_pose_dimension>(0, 2 * n_pose_dimension) = temp.template block<3 * n_pose_dimension, n_pose_dimension>(0, n_pose_dimension);*/
				temp.setZero();
				temp.template block<n_pose_dimension, n_pose_dimension>(0, 0) = _TyMatrix::Identity();
				temp.template block<n_pose_dimension, n_pose_dimension>(n_pose_dimension, 2 * n_pose_dimension) = _TyMatrix::Identity();
				temp.template block<n_pose_dimension, n_pose_dimension>(2 * n_pose_dimension, n_pose_dimension) = _TyMatrix::Identity();
				small_sys = temp * small_sys * temp.transpose();
			}
			// reorder the system so that the second variable is at the end

			double perm_sys_symmetry_err = (small_sys - small_sys.transpose()).norm();

			Eigen::Block<Eigen::Matrix<double, 3 * n_pose_dimension, 3 * n_pose_dimension>, 2 * n_pose_dimension, 2 * n_pose_dimension> A =
				small_sys.template topLeftCorner<2 * n_pose_dimension, 2 * n_pose_dimension>();
			Eigen::Block<Eigen::Matrix<double, 3 * n_pose_dimension, 3 * n_pose_dimension>, 2 * n_pose_dimension, n_pose_dimension> B =
				small_sys.template topRightCorner<2 * n_pose_dimension, n_pose_dimension>();
			Eigen::Block<Eigen::Matrix<double, 3 * n_pose_dimension, 3 * n_pose_dimension>, n_pose_dimension, 2 * n_pose_dimension> C =
				small_sys.template bottomLeftCorner<n_pose_dimension, 2 * n_pose_dimension>();
			Eigen::Block<Eigen::Matrix<double, 3 * n_pose_dimension, 3 * n_pose_dimension>, n_pose_dimension, n_pose_dimension> D =
				small_sys.template bottomRightCorner<n_pose_dimension, n_pose_dimension>();
			Eigen::Matrix<double, 2 * n_pose_dimension, 2 * n_pose_dimension> SC = A - B * D.inverse() * C;
			// calculate the SC

			double sc_symmetry_err = (SC - SC.transpose()).norm();

			//printf("%g, %g, %g\n", small_sys.inverse().norm(), SC.inverse().norm(), small_sys.inverse().norm() - SC.inverse().norm());
			/*double f_margs_sys = sqrt(small_sys.inverse().eval().template topLeftCorner<n_pose_dimension, n_pose_dimension>().eval().squaredNorm() + // var 0
				small_sys.inverse().eval().template block<n_pose_dimension, n_pose_dimension>(n_pose_dimension, n_pose_dimension).eval().squaredNorm()); // var 1
			double f_margs_sc = sqrt(SC.inverse().eval().template topLeftCorner<n_pose_dimension, n_pose_dimension>().eval().squaredNorm() + // var 0
				SC.inverse().eval().template bottomRightCorner<n_pose_dimension, n_pose_dimension>().eval().squaredNorm()); // var 1 // todo - file eigen bug when compiling without eval
			printf("margs sys %-20g, margs sc %-20g, diff: %-20g\n", f_margs_sys, f_margs_sc, f_margs_sc - f_margs_sys);*/
			// debug

			Eigen::Matrix<double, n_pose_dimension, 2 * n_pose_dimension> J;
			J.template leftCols<n_pose_dimension>() = L0;
			J.template rightCols<n_pose_dimension>() = L1;
			_TyMatrix t_recovered_sigma = (J * SC.inverse() * J.transpose()).inverse();

			double recovered_sigma_symmetry_err = (t_recovered_sigma - t_recovered_sigma.transpose()).norm();

			/*Eigen::Matrix<double, 3 * n_pose_dimension, 3 * n_pose_dimension> small_cov = small_sys.inverse();
			Eigen::Matrix<double, 2 * n_pose_dimension, 2 * n_pose_dimension> sigma_dense;
			{
				Eigen::Matrix<double, 2 * n_pose_dimension, 2 * n_pose_dimension> smaller_sys;
				smaller_sys.setZero();

				_TyMatrix _K0, _K1;
				_TyVector v_nothing, v_origin = system.r_Vertex_Pool()[system.r_Vertex_Pool().n_Size() - 1].v_State();
				_TyMatrix t_sigma_0 = r_marginals.t_GetBlock_Log(r_marginals.n_BlockColumn_Num() - 1, r_marginals.n_BlockColumn_Num() - 1);
				Relative_to_Absolute(v_origin, r_edge.v_measurement, v_nothing, _K0, _K1); // the first and the third argument cannot be the same
				smaller_sys.template topLeftCorner<n_pose_dimension, n_pose_dimension>() = _K0.transpose() * t_sigma_inv1 * _K0 + t_sigma_0;
				smaller_sys.template topRightCorner<n_pose_dimension, n_pose_dimension>() = _K0.transpose() * t_sigma_inv1 * _K1;
				smaller_sys.template bottomLeftCorner<n_pose_dimension, n_pose_dimension>() =
					smaller_sys.template topRightCorner<n_pose_dimension, n_pose_dimension>().transpose();
				smaller_sys.template bottomRightCorner<n_pose_dimension, n_pose_dimension>() = _K1.transpose() * t_sigma_inv1 * _K1;

				sigma_dense = smaller_sys.inverse();
			}
			t_recovered_sigma = (J * sigma_dense * J.transpose()).inverse();*/
			// the approach with Sigma_d - the same result as SC but a slightly larger matrix to invert

			v_cum_measurement = v_composite_measurement;
			t_cum_information = t_recovered_sigma;

			printf("debug: norm of recovered sigma inv: %g, norm of edge sigma inv: %g\n",
				t_cum_information.norm(), r_edge.t_information.norm());
		}
		// see if this works

		/*_TyMatrix J0, J1;
		_TyVector temp;
		Relative_to_Absolute(v_cum_measurement, r_edge.v_measurement, temp, J0, J1); // the first and the third argument cannot be the same
		v_cum_measurement = temp;*/
		// accumulate measurements and get jacobians

		/*if(var_sys.rows() == n_pose_dimension)
			var_sys.template leftCols<n_pose_dimension>() = t_cum_information.llt().matrixU(); // put UF the first time around
		var_sys.conservativeResize(var_sys.rows() + n_pose_dimension, var_sys.cols() + n_pose_dimension);
		var_sys.template bottomRows<n_pose_dimension>().setZero(); // zeroes slightly more than required
		var_sys.template rightCols<n_pose_dimension>().setZero(); // zeroes slightly more than required
		_TyMatrix t_sqrt_sigma_inv = r_edge.t_information.llt().matrixU();
		var_sys.template bottomRightCorner<n_pose_dimension, 2 * n_pose_dimension>().template leftCols<n_pose_dimension>() = t_sqrt_sigma_inv * J0;
		var_sys.template bottomRightCorner<n_pose_dimension, 2 * n_pose_dimension>().template rightCols<n_pose_dimension>() = t_sqrt_sigma_inv * J1;
		t_cum_information = (var_sys.transpose() * var_sys).inverse().template bottomRightCorner<n_pose_dimension, n_pose_dimension>().inverse();
		// accumulate information by running a variable size filter*/

		/*enum {
			n_double_pose_dimension = 2 * n_pose_dimension
		};
		_TyMatrix t_sigma_0 = t_cum_information;
		_TyMatrix t_sigma_inv = r_edge.t_information;
		Eigen::Matrix<double, n_double_pose_dimension, n_double_pose_dimension> small_sys;
		small_sys.template topLeftCorner<n_pose_dimension, n_pose_dimension>() = J0.transpose() * t_sigma_inv * J0 + t_sigma_0;
		small_sys.template topRightCorner<n_pose_dimension, n_pose_dimension>() = J0.transpose() * t_sigma_inv * J1;
		small_sys.template bottomLeftCorner<n_pose_dimension, n_pose_dimension>() =
			small_sys.template topRightCorner<n_pose_dimension, n_pose_dimension>().transpose();
		small_sys.template bottomRightCorner<n_pose_dimension, n_pose_dimension>() = J1.transpose() * t_sigma_inv * J1;
		t_cum_information = small_sys.inverse().template bottomRightCorner<n_pose_dimension, n_pose_dimension>().inverse();*/
		// accumulate information by running a small filter (the same results as the variable size filter)

		/*solver.Optimize(0); // make sure that lambda is up to date // no need to time this, all this is solver time
		const CUberBlockMatrix &r_lambda = solver.r_Lambda();
		CUberBlockMatrix omega;
		{
			_TyMatrix t_sigma_0 = t_cum_information;
			_TyMatrix t_sigma_inv = r_edge.t_information;
			omega.t_Append_Block_Log(J0.transpose() * t_sigma_inv * J0 + t_sigma_0, 0, 0);
			omega.t_Append_Block_Log((J0.transpose() * t_sigma_inv * J1).transpose(), 1, 0);
			omega.t_Append_Block_Log(J0.transpose() * t_sigma_inv * J1, 0, 1);
			omega.t_Append_Block_Log(J1.transpose() * t_sigma_inv * J1, 1, 1);
			_ASSERTE(omega.n_Row_Num() == omega.n_Column_Num() && omega.n_Column_Num() == 2 * n_pose_dimension);
		}
		// get lambda and omega

		CUberBlockMatrix b;
		r_lambda.SliceTo(b, 0, r_lambda.n_BlockRow_Num() - 1,
			r_lambda.n_BlockColumn_Num() - 1, r_lambda.n_BlockColumn_Num(), true);
		_ASSERTE(b.n_BlockRow_Num() == r_lambda.n_BlockRow_Num() - 1 && b.n_BlockColumn_Num() == 1);
		// get the last column of lambda, minus the last block

		CUberBlockMatrix b_omega12T = b;
		b_omega12T.Append_Block_Log(omega.t_GetBlock_Log(0, 1).transpose(), b.n_BlockRow_Num(), 1);
		CUberBlockMatrix b_omega12TT;
		b_omega12TT.TransposeOf(b_omega12T);
		// append omega12T, get transpose

		CUberBlockMatrix c_omega11;
		c_omega11.Append_Block_Log(r_lambda.t_GetBlock_Log(r_lambda.n_BlockColumn_Num() - 1, r_lambda.n_BlockColumn_Num() - 1) + omega.t_GetBlock_Log(0, 0), 0, 0);
		_ASSERTE(c_omega11.n_Row_Num() == c_omega11.n_Column_Num() && c_omega11.n_Column_Num() == n_pose_dimension);
		// get the last block of lambda

		CUberBlockMatrix b_omega12T_c_omega11;
		b_omega12T_c_omega11.ProductOf(b_omega12T, c_omega11);
		CUberBlockMatrix SC;
		SC.ProductOf(b_omega12T_c_omega11, b_omega12TT);
		// calculate schur complement

		// todo - simulate adding and marginalizing a pose out*/
#else // 0
		_TyMatrix J0, J1;
		_TyVector temp;
		Relative_to_Absolute(v_cum_measurement, r_edge.v_measurement, temp, J0, J1); // the first and the third argument cannot be the same
		v_cum_measurement = temp;
		// accumulate measurements and get jacobians

		_TyMatrix t_cov = J0 * t_cum_information.inverse() * J0.transpose() +
			J1 * r_edge.t_information.inverse() * J1.transpose();
		t_cum_information = t_cov.inverse();
		// over-confident (Randall Smith, Matthew Self, Peter Cheeseman)

		//printf("debug: norm of cheeseman sigma inv: %g, norm of edge sigma inv: %g\n",
		//	t_cum_information.norm(), r_edge.t_information.norm());

		//_TyMatrix t_cov = J0 * (t_cum_information * .5).inverse() * J0.transpose() +
		//	J1 * (r_edge.t_information * .5).inverse() * J1.transpose();
		//t_cum_information = (/*J1.inverse() **/ t_cov /** J1.inverse().transpose()*/).inverse();
		// this would work but is rubbish

		/*_TyMatrix t_cov = J0 * t_cum_information.inverse().llt().matrixU() * J0.transpose() +
			J1 * r_edge.t_information.inverse().llt().matrixU() * J1.transpose();
		t_cum_information = (t_cov.transpose() * t_cov).inverse();*/
		// this would also work and appears to be conservative
#endif // 0
	}
};

void PrintHelp()
{
	printf("use: slam_compact_pose_ijrr <input-file> [options]\n\n"
		"where:\n"
		"--help | -h shows this help screen\n"
		"--config | -c <config file> specifies configuration file with the thresholds\n"
		"--write-bitmaps | -wb writes images of the solution at each step\n"
		"--no-incremental-bitmaps | -nib writes only the images at the end (-wb must be specified)\n"
		"--bitmaps-xz-plane | -xz changes the images to be on x-z plane (default x-y)\n"
		"--highlight-loops | -hl plots loop closures in bright green (default off)\n"
		"--record-cumtime | -rc records cummulative time at each step\n"
		"--record-mem-usage | -rmu records memory usage and nnz of the factor\n"
		"--bitmap-resolution | -br <resolution> sets bitmap resolution, in pixels (default 720)\n"
		"--bitmap-bbox | -bb \"[<min-x> <min-y>] [<max-x> <max-y>]\" specifies the bounding box for\n"
		"                                                          the bitmaps, in x-y pose space\n"
		"--write-systems | -ws writes snapshots of the state at each step\n"
		"--write-animdata | -wa writes binary data for the ICRA animation rendering\n"
		"--write-covmats | -wc writes images of covariance matrix sparsity patterns at each step\n"
		"--write-norms | -wn writes norms of the full covariance matrix at each step\n"
		"--write-compact-edge-lengths | -wcl writes a vector of compact edge lengths\n"
		"--threshold-analysis | -ta accepts all poses and loops and performs threshold analysis\n"
		"--random-prune | -rp <X>p:<Y>l prunes the dataset down to X poses and Y loops\n"
		"--random-seed | -rs <seed> sets random seed for the pruning (default 12345)\n");
}

void TParams::Defaults(bool b_2D_SLAM)
{
	p_s_input_file = 0;
	p_s_config_file = 0; // configuration file - if null then use defaults
	b_write_bitmaps = false;
	b_disable_incremental_bitmaps = false;
	b_bitmaps_xz_plane = false;
	b_bitmaps_highlight_loops = false;
	b_write_systems = false;
	b_write_animdata = false;
	b_write_covmats = false;
	b_write_cov_norms = false;
	b_write_cumtime = false;
	b_write_mem_usage = false;
	b_write_compact_edge_lengths = false;
	b_do_thresh_analysis = false;
	b_do_thresh_sensitivity_analysis = false;
	// set defaults

	//v_min = Eigen::Vector2d(-12, -9), v_max = Eigen::Vector2d(8.5, 9); // accomodates the first 100 poses of molson
	//v_min = Eigen::Vector2d(-30, -20), v_max = Eigen::Vector2d(8.5, 9); // accomodates slightly more of molson
	v_min = Eigen::Vector2d(-12, -24), v_max = Eigen::Vector2d(18, 6); // accomodates intel
	n_resolution = 720;
	// image parameters

	v_sensor_range.resize((b_2D_SLAM)? 3 : 4);
	if(b_2D_SLAM) {
		//v_sensor_range << 3, 3, 2 * M_PI; // manhattan (has some long loop closures)
		v_sensor_range << 3, 2, 2 * M_PI; // intel
	} else
		v_sensor_range << 2, 2, 2, 2 * M_PI; // 3D (guess)
	// sensor range (a tolerance, affects which poses will be matched)

	f_loop_probability_dist_threshold = 0.15;//0.1578;//0.1; // as in TRO
	// poses with higher loop probability will be candidates for sensor matching

	f_keep_pose_gain_threshold = 0.1;//0.05;//0.25 // experimentally set
	f_loop_closure_gain_threshold = f_keep_pose_gain_threshold; // initially the same but can be set differently if needed
	// minimal information gain to not extend an edge

	if(b_2D_SLAM)
		t_Sigma_bar = Eigen::MatrixXd::Identity(3, 3);
	else
		t_Sigma_bar = Eigen::MatrixXd::Identity(6, 6);
	// a default value for sigma bar

	n_max_pose_num = size_t(-1);
	n_max_loop_num = size_t(-1);
	n_random_seed = 12345;
	// dataset trim

	p_s_gt_file = 0;
	p_s_ref_file = 0;
	// name of file containing ground truth / reference vertices

	p_s_compact_edge_lengths_file = 0;
	// compact lengths file (for input only - exclusive with b_write_compact_edge_lengths)

	n_omp_threads = 0;
	b_omp_dynamic = false;
	// OpenMP overrides
}

int TParams::n_ParseCommandline(int n_arg_num, const char **p_arg_list)
{
	_ASSERTE(n_arg_num > 1); // checked before
	p_s_input_file = p_arg_list[1];

	for(int i = 2; i < n_arg_num; ++ i) { // start at 2, 1 is the input file
		if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
			PrintHelp();
			return 0;
		} else if(!strcmp(p_arg_list[i], "--write-bitmaps") || !strcmp(p_arg_list[i], "-wb"))
			b_write_bitmaps = true;
		else if(!strcmp(p_arg_list[i], "--no-incremental-bitmaps") || !strcmp(p_arg_list[i], "-nib"))
			b_disable_incremental_bitmaps = true;
		else if(!strcmp(p_arg_list[i], "--bitmaps-xz-plane") || !strcmp(p_arg_list[i], "-xz"))
			b_bitmaps_xz_plane = true;
		else if(!strcmp(p_arg_list[i], "--highlight-loops") || !strcmp(p_arg_list[i], "-hl"))
			b_bitmaps_highlight_loops = true;
		else if(!strcmp(p_arg_list[i], "--write-systems") || !strcmp(p_arg_list[i], "-ws"))
			b_write_systems = true;
		else if(!strcmp(p_arg_list[i], "--write-animdata") || !strcmp(p_arg_list[i], "-wa"))
			b_write_animdata = true;
		else if(!strcmp(p_arg_list[i], "--write-covmats") || !strcmp(p_arg_list[i], "-wc"))
			b_write_covmats = true;
		else if(!strcmp(p_arg_list[i], "--write-norms") || !strcmp(p_arg_list[i], "-wn"))
			b_write_cov_norms = true;
		else if(!strcmp(p_arg_list[i], "--record-cumtime") || !strcmp(p_arg_list[i], "-rc"))
			b_write_cumtime = true;
		else if(!strcmp(p_arg_list[i], "--record-mem-usage") || !strcmp(p_arg_list[i], "-rmu"))
			b_write_mem_usage = true;
		else if(!strcmp(p_arg_list[i], "--write-compact-edge-lengths") || !strcmp(p_arg_list[i], "-wcl"))
			b_write_compact_edge_lengths = true;
		else if(!strcmp(p_arg_list[i], "--threshold-analysis") || !strcmp(p_arg_list[i], "-ta"))
			b_do_thresh_analysis = true;
		else if(!strcmp(p_arg_list[i], "--threshold-sensitivity-analysis") || !strcmp(p_arg_list[i], "-tsa"))
			b_do_thresh_sensitivity_analysis = true;
		else if(i + 1 == n_arg_num) {
			fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
			return -1;
		} else if(!strcmp(p_arg_list[i], "--config") || !strcmp(p_arg_list[i], "-c"))
			p_s_config_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--random-prune") || !strcmp(p_arg_list[i], "-rp")) {
			size_t n_max_poses, n_max_edges;
			if(sscanf(p_arg_list[++ i], PRIsize "p:" PRIsize "l", &n_max_poses, &n_max_edges) != 2) {
				fprintf(stderr, "error: invalid pruning specifier (\'%s\'): "
					"use e.g. 123p:456l to prune down to 123 poses and 456 loops\n", p_arg_list[i]);
				return -1;
			} else {
				n_max_pose_num = n_max_poses;
				n_max_loop_num = n_max_edges;
			}
		} else if(!strcmp(p_arg_list[i], "--random-seed") || !strcmp(p_arg_list[i], "-rs"))
			n_random_seed = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--ground-truth") || !strcmp(p_arg_list[i], "-gt"))
			p_s_gt_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--reference-trajectory") || !strcmp(p_arg_list[i], "-rt"))
			p_s_ref_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--marginalize-norms") || !strcmp(p_arg_list[i], "-mn"))
			p_s_compact_edge_lengths_file = p_arg_list[++ i];
		else if(!strcmp(p_arg_list[i], "--bitmap-resolution") || !strcmp(p_arg_list[i], "-br"))
			n_resolution = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--bitmap-bbox") || !strcmp(p_arg_list[i], "-bb")) {
			double f_min_x, f_min_y, f_max_x, f_max_y;
			if(sscanf(p_arg_list[++ i], "[%lf %lf] [%lf %lf]",
			   &f_min_x, &f_min_y, &f_max_x, &f_max_y) != 4 ||
			   f_min_x > f_max_x || f_min_y > f_max_y) {
				fprintf(stderr, "warning: invalid bounding box specifier (\'%s\'): "
					"use e.g. \"[-12 -24] [18 6]\" for intel.txt\n", p_arg_list[i]);
			} else {
				v_min << f_min_x, f_min_y;
				v_max << f_max_x, f_max_y;
			}
		} else if(!strcmp(p_arg_list[i], "--omp-set-num-threads"))
			n_omp_threads = atol(p_arg_list[++ i]);
		else if(!strcmp(p_arg_list[i], "--omp-set-dynamic"))
			b_omp_dynamic = (atol(p_arg_list[++ i]) != 0);
		else {
			fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
			return -1;
		}
	}
	// "parse" commandline

	if(p_s_compact_edge_lengths_file && b_write_compact_edge_lengths) {
		fprintf(stderr, "error: can't simultaneously write and read compact lengths\n");
		// mostly a precaution to sourcing the file from the same folder where we are trying to write

		return -1;
	}
	if(p_s_compact_edge_lengths_file && !b_write_cov_norms) {
		fprintf(stderr, "warning: compact lengths file specified but marginal norms not calculated\n");
		//b_write_cov_norms = true; // do not auto-enable to avoid overwriting some data
	}
	// complications ...

	return 1;
}

bool TParams::Load_Config(const char *p_s_config_file, bool b_2D_SLAM)
{
	FILE *conf;
	if(!(conf = fopen(p_s_config_file, "r")))
		return false;

	std::string tag;
	while(!feof(conf)) {
		if(!CParserBase::ReadField(tag, conf))
			break; // i/o error
		if(tag.empty())
			continue;
		if(tag == "t_sensor_range") { // sensor range
			// load 3 values for 2d, 4 values for 3d
			double vals[4];
			if(b_2D_SLAM) {
				if(fscanf(conf, "%lf %lf %lf", vals, vals + 1, vals + 2) != 3)
					break;
				v_sensor_range << vals[0], vals[1], vals[2];
			} else {
				if(fscanf(conf, "%lf %lf %lf %lf", vals, vals + 1, vals + 2, vals + 3) != 4)
					break;
				v_sensor_range << vals[0], vals[1], vals[2], vals[3];
			}
		} else if(tag == "t_distance_probability") { // probabilities
			// load 3 values for 2d, 4 values for 3d
			double vals[4];
			if(b_2D_SLAM) {
				if(fscanf(conf, "%lf"/*" %lf %lf"*/, vals/*, vals + 1, vals + 2*/) != 1/*3*/)
					break;
				//v_sensor_range << vals[0], vals[1], vals[2];
			} else {
				if(fscanf(conf, "%lf"/*" %lf %lf %lf"*/, vals/*, vals + 1, vals + 2, vals + 3*/) != 1/*4*/)
					break;
				//v_sensor_range << vals[0], vals[1], vals[2], vals[3];
			}
			f_loop_probability_dist_threshold = vals[0];
		} else if(tag == "t_min_info_add_pose") { // min gain to add edge / keep edge threshold
			double vals;
			if(!fscanf(conf, "%lf", &vals))
				break;
			f_keep_pose_gain_threshold = vals;
		} else if(tag == "t_min_info_close_loop") { // min gain to close a loop
			double vals;
			if(!fscanf(conf, "%lf", &vals))
				break;
			f_loop_closure_gain_threshold = vals;
		} else if(tag == "sigma_bar") { // \f$\bar\Sigma^^{-1}\f$
			double vals[(6 * 5) / 2 + 6]; // upper triangular of an information matrix
			if(b_2D_SLAM) {
				if(fscanf(conf, "%lf %lf %lf"
									" %lf %lf"
										" %lf",
				   &vals[0], &vals[1], &vals[2],
							 &vals[3], &vals[4],
									   &vals[5]) != ((3 * 2) / 2 + 3)) {
					fprintf(stderr, "error: failed to parse sigma_bar (2D)\n");
					break;
				}
				t_Sigma_bar << vals[0], vals[1], vals[2],
							   vals[1], vals[3], vals[4],
							   vals[2], vals[4], vals[5];
			} else {
				if(fscanf(conf, "%lf %lf %lf %lf %lf %lf"
									" %lf %lf %lf %lf %lf"
										" %lf %lf %lf %lf"
											" %lf %lf %lf"
												" %lf %lf"
													" %lf",
				   &vals[0], &vals[1], &vals[ 2], &vals[ 3], &vals[ 4], &vals[ 5],
							 &vals[6], &vals[ 7], &vals[ 8], &vals[ 9], &vals[10],
									   &vals[11], &vals[12], &vals[13], &vals[14],
												  &vals[15], &vals[16], &vals[17],
															 &vals[18], &vals[19],
																		&vals[20]) != ((6 * 5) / 2 + 6)) {
					fprintf(stderr, "error: failed to parse sigma_bar (3D)\n");
					break;
				}
				t_Sigma_bar << vals[ 0], vals[ 1], vals[ 2], vals[ 3], vals[ 4], vals[ 5],
							   vals[ 1], vals[ 6], vals[ 7], vals[ 8], vals[ 9], vals[10],
							   vals[ 2], vals[ 7], vals[11], vals[12], vals[13], vals[14],
							   vals[ 3], vals[ 8], vals[12], vals[15], vals[16], vals[17],
							   vals[ 4], vals[ 9], vals[13], vals[16], vals[18], vals[19],
							   vals[ 5], vals[10], vals[14], vals[14], vals[19], vals[20];
			}
			t_Sigma_bar = t_Sigma_bar.inverse(); // !!
		} else
			fprintf(stderr, "warning: unknown configuration tag \'%s\'\n", tag.c_str());
	}
	fclose(conf);

	return true;
}

int MakeDir(const char *p_s_path)
{
#if defined(_WIN32) || defined(_WIN64)
	return system(((std::string("mkdir ") + p_s_path) + " > nul 2>&1").c_str());
#else // _WIN32 || _WIN64
	return system(((std::string("mkdir ") + p_s_path) + " > /dev/null 2>&1").c_str());
#endif // _WIN32 || _WIN64
}

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
	if(n_arg_num < 2) {
		fprintf(stderr, "error: the first argument must be the path to the input dataset\n");
		PrintHelp();
		return -1;
	}
	// make sure there are arguments (don't parse just yet, can leave that
	// to CCompactPoseSLAM::main(), here we only need the input file)

	uint64_t n_memory_use_after_init = CProcessMemInfo::n_MemoryUsage();
	// get memory usage of the process just after startup

	try {
		bool b_2D_data;
		if(!Detect_DatasetType(p_arg_list[1], b_2D_data)) {
			fprintf(stderr, "error: failed to load \'%s\'\n", p_arg_list[1]);
			return -1;
		}
		// determine the type of the dataset

		if(b_2D_data)
			return CCompactPoseSLAM<true>::main(n_arg_num, p_arg_list, n_memory_use_after_init); // handle 2D
		else
			return CCompactPoseSLAM<false>::main(n_arg_num, p_arg_list, n_memory_use_after_init); // handle 3D
		// run a specialization of compact pose SLAM
	} catch(std::exception &r_exc) {
		fprintf(stderr, "error: uncaught exception: \'%s\'\n", r_exc.what());
		return -1;
	}
}

/**
 *	@brief a simple parse loop
 *
 *	@tparam CEdgeData is edge data type to store
 *	@tparam CEdgeParseType is parsed edge type
 */
template <class CEdgeData = TEdgeData2D, class CEdgeParseType = CParserBase::TEdge2D>
class CMyParseLoop {
protected:
	std::vector<CEdgeData> &m_r_edges; /**< @brief reference to the output vector of edges */

public:
	/**
	 *	@brief default constructor
	 *	@param[in] r_edges is reference to the output vector of edges
	 */
	CMyParseLoop(std::vector<CEdgeData> &r_edges)
		:m_r_edges(r_edges)
	{}

	/**
	 *	@brief appends the system with a measurement
	 *	@param[in] r_t_edge is the measurement to be appended
	 */
	void AppendSystem(const CEdgeParseType &r_t_edge) // throw(std::bad_alloc)
	{
		m_r_edges.push_back(CEdgeData(r_t_edge.m_n_node_0, r_t_edge.m_n_node_1,
			r_t_edge.m_v_delta, r_t_edge.m_t_inv_sigma));
	}
};

/**
 *	@brief a simple parse loop
 */
class CMyDatasetTypeDetector {
protected:
	bool m_b_has_2D; /**< @brief 2D edges presence flag */
	bool m_b_has_3D; /**< @brief 3D edges presence flag */
	bool m_b_has_other; /**< @brief other edges / vertices presence flag */

public:
	/**
	 *	@brief default constructor
	 */
	CMyDatasetTypeDetector()
		:m_b_has_2D(false), m_b_has_3D(false), m_b_has_other(false)
	{}

	/**
	 *	@brief determines whether the type of the dataset is clean cut
	 *	@return Returns true if the dataset contains either purely
	 *		2D or purely 3D edges, otherwise returns false.
	 */
	bool b_DeterminateType() const
	{
		return !(m_b_has_other || (m_b_has_2D && m_b_has_3D) || (!m_b_has_2D && !m_b_has_3D));
		// if there are other types than 2D or if it mixes 2D and 3D or if it does not
		// have either 2D or 3D, then the type cannot be classified as purely 2D or 3D
	}

	/**
	 *	@brief determines whether this dataset is a 2D dataset
	 *	@return Returns true if the dataset is 2D, otherwise returns false.
	 *	@note In case b_DeterminateType() returns true, then the dataset
	 *		is 2D if this returns true or 3D if this returns false. Otherwise
	 *		the type is unclear / other.
	 */
	bool b_Is_2D() const
	{
		return m_b_has_2D;
	}

	/**
	 *	@brief appends the system with a 2D measurement
	 *	@param[in] r_t_edge is the measurement to be appended (value unused)
	 */
	void AppendSystem(const CParserBase::TEdge2D &UNUSED(r_t_edge))
	{
		m_b_has_2D = true;
	}

	/**
	 *	@brief appends the system with a 3D measurement
	 *	@param[in] r_t_edge is the measurement to be appended (value unused)
	 */
	void AppendSystem(const CParserBase::TEdge3D &UNUSED(r_t_edge))
	{
		m_b_has_3D = true;
	}

	/**
	 *	@brief appends the system with a general edge of unknown type
	 *	@tparam COtherEdgeType is edge type, other than CParserBase::TEdge2D or CParserBase::TEdge3D
	 *	@param[in] r_t_edge is the measurement to be appended (value unused)
	 */
	template <class COtherEdgeType>
	void AppendSystem(const COtherEdgeType &UNUSED(r_t_edge))
	{
		m_b_has_other = true;
	}

	/**
	 *	@brief initializes a vertex
	 *	@tparam CVertexType is vertex type (any)
	 *	@param[in] r_t_vertex is the vertex to be initialized (value unused)
	 */
	template <class CVertexType>
	void InitializeVertex(const CVertexType &UNUSED(r_t_vertex))
	{
		m_b_has_other = true;
	}
};

bool Detect_DatasetType(const char *p_s_filename, bool &r_b_is_2D)
{
	typedef CParserTemplate<CMyDatasetTypeDetector, CStandardParsedPrimitives> CParser;

	CMyDatasetTypeDetector detect_type;
	if(!CParser().Parse(p_s_filename, detect_type, 100))
		return false;
	// use the first 100 lines to detect the type

	if(!detect_type.b_DeterminateType()) {
		fprintf(stderr, "error: file \'%s\': unsupported data type\n", p_s_filename);
		return false;
	}
	// can we clearly determine whether it is 2D or 3D?

	r_b_is_2D = detect_type.b_Is_2D();
	// read the type

	return true;
}

bool Load_Dataset(const char *p_s_filename, std::vector<TEdgeData2D> &r_edges)
{
	r_edges.clear(); // !!

	typedef MakeTypelist(CEdge2DParsePrimitive) AllowedEdges;
	typedef CParserTemplate<CMyParseLoop<>, AllowedEdges> CParser;

	CMyParseLoop<> ploop(r_edges);
	if(!CParser().Parse(p_s_filename, ploop))
		return false;

	printf("loaded \'%s\' : " PRIsize " edges\n", p_s_filename, r_edges.size());

	return true;
}

bool Load_Dataset(const char *p_s_filename, std::vector<TEdgeData3D> &r_edges)
{
	r_edges.clear(); // !!

	typedef MakeTypelist(CEdge3DParsePrimitive, CEdge3DParsePrimitiveAxisAngle) AllowedEdges;
	typedef CMyParseLoop<TEdgeData3D, CParserBase::TEdge3D> CMyParseLoop3D;
	typedef CParserTemplate<CMyParseLoop3D, AllowedEdges> CParser;

	CMyParseLoop3D ploop(r_edges);
	if(!CParser().Parse(p_s_filename, ploop))
		return false;

	printf("loaded \'%s\' : " PRIsize " edges\n", p_s_filename, r_edges.size());

	return true;
}

/*
 *	end-of-file
 */
