/*
								+---------------------------------+
								|                                 |
								| ***  Distances calculation  *** |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2013 |
								|                                 |
								|           Distances.h           |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __DISTANCES_INCLUDED
#define __DISTANCES_INCLUDED

/**
 *	@file include/slam/Distances.h
 *	@date 2013
 *	@author V.Ila and -tHE SWINe-
 *	@brief calculation of distances between variables
 */

#include "slam/BlockMatrix.h"
//#include "slam/Timer.h" // included from slam/BlockMatrix.h
//#include "slam/Integer.h" // included from slam/BlockMatrix.h
//#include "eigen/Eigen/Core" // included from slam/BlockMatrix.h

/** \addtogroup covs
 *	@{
 */

/**
 *	@brief identity distance transform; does not change the distance in any way
 */
class CNullDistanceTransform {
public:
	/**
	 *	@brief output dimension traits
	 *	@tparam CEdgeType is edge type for which the dimension is queried
	 */
	template <class CEdgeType>
	class CTransformedDimension {
	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			n_result = CEdgeType::n_residual_dimension /**< @brief transformed distance dimension for CEdgeType */
		};
	};

public:
	/**
	 *	@brief function operator; performs the distance transformation
	 *
	 *	@tparam CVertex0 is type of the first vertex
	 *	@tparam CVertex1 is type of the second vertex
	 *	@tparam CMeanMeasurement is type of mean vector
	 *	@tparam CMeanDistribution is type of Sigma matrix
	 *	@tparam CEdgeTypeList is list of edge types suitable for distance calculation between CVertex0 and CVertex1
	 *
	 *	@param[in] r_vertex_0 is state of the first vertex (unused)
	 *	@param[in] r_vertex_1 is state of the second vertex (unused)
	 *	@param[in,out] r_mean_d is distance between the vertices (unused - left as is)
	 *	@param[in,out] r_t_Sigma_d is distribution of the distance (unused - left as is)
	 *	@param[in] edge_list_tag is tag to carry CEdgeTypeList (value unused)
	 */
	template <class CVertex0, class CVertex1, class CMeanMeasurement, class CMeanDistribution, class CEdgeTypeList>
	void operator ()(const CVertex0 &UNUSED(r_vertex_0), const CVertex1 &UNUSED(r_vertex_1),
		CMeanMeasurement &UNUSED(r_mean_d), CMeanDistribution &UNUSED(r_t_Sigma_d),
		CEdgeTypeList UNUSED(edge_list_tag)) const
	{}
};

/**
 *	@brief transforms SE(3) distance of [x, y, z, a, b, c] to [x, y, z, norm([a b c])]
 */
class CSE3_XYZ_RotationMagnitude_DistanceTransform {
public:
	/**
	 *	@brief output dimension traits
	 *	@tparam CEdgeType is edge type for which the dimension is queried
	 */
	template <class CEdgeType>
	class CTransformedDimension {
	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			n_result = 4 /**< @brief transformed distance dimension for CEdgeType */
		};
	};

public:
	/**
	 *	@brief function operator; performs the distance transformation
	 *
	 *	@tparam CVertex0 is type of the first vertex
	 *	@tparam CVertex1 is type of the second vertex
	 *	@tparam CMeanMeasurement is type of mean vector
	 *	@tparam CMeanDistribution is type of Sigma matrix
	 *	@tparam CEdgeTypeList is list of edge types suitable for distance calculation between CVertex0 and CVertex1
	 *
	 *	@param[in] r_vertex_0 is state of the first vertex
	 *	@param[in] r_vertex_1 is state of the second vertex
	 *	@param[in,out] r_mean_d is distance between the vertices
	 *	@param[in,out] r_t_Sigma_d is distribution of the distance
	 *	@param[in] edge_list_tag is tag to carry CEdgeTypeList (value unused)
	 */
	template <class CVertex0, class CVertex1, class CMeanMeasurement, class CMeanDistribution, class CEdgeTypeList>
	void operator ()(const CVertex0 &r_vertex_0, const CVertex1 &r_vertex_1,
		CMeanMeasurement &r_mean_d, CMeanDistribution &r_t_Sigma_d, CEdgeTypeList UNUSED(edge_list_tag)) const
	{
		Eigen::Matrix<double, 4, 6> H_transform = Eigen::Matrix<double, 4, 6>::Identity();
		const double Aa = r_mean_d(3), Ab = r_mean_d(4), Ac = r_mean_d(5);
		const double D = r_mean_d.template tail<3>().norm();
		if(D) { // otherwise leave as identity
			const double invD = 1 / D;
			H_transform(3, 3) = invD * Aa;
			H_transform(3, 4) = invD * Ab;
			H_transform(3, 5) = invD * Ac;
			// transformation
		} else {
			H_transform(3, 3) = 1 / sqrt(3.0);
			H_transform(3, 4) = 1 / sqrt(3.0);
			H_transform(3, 5) = 1 / sqrt(3.0);
			// limit in zero
		}

		r_mean_d(3) = D;
		r_mean_d.conservativeResize(4);
		// transform mean

		r_t_Sigma_d = H_transform * r_t_Sigma_d * H_transform.transpose();
		// transform the sigma
	}
};

/**
 *	@brief transforms SE(3) distance of [x, y, z, a, b, c] to [x, y, z, angle] where
 *		the angle is angular difference between z+ axes of the identity and the provided pose
 */
class CSE3_XYZ_ViewDirection_DistanceTransform {
public:
	/**
	 *	@brief output dimension traits
	 *	@tparam CEdgeType is edge type for which the dimension is queried
	 */
	template <class CEdgeType>
	class CTransformedDimension {
	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			n_result = 4 /**< @brief transformed distance dimension for CEdgeType */
		};
	};

public:
	/**
	 *	@brief function operator; performs the distance transformation
	 *
	 *	@tparam CVertex0 is type of the first vertex
	 *	@tparam CVertex1 is type of the second vertex
	 *	@tparam CMeanMeasurement is type of mean vector
	 *	@tparam CMeanDistribution is type of Sigma matrix
	 *	@tparam CEdgeTypeList is list of edge types suitable for distance calculation between CVertex0 and CVertex1
	 *
	 *	@param[in] r_vertex_0 is state of the first vertex
	 *	@param[in] r_vertex_1 is state of the second vertex
	 *	@param[in,out] r_mean_d is distance between the vertices
	 *	@param[in,out] r_t_Sigma_d is distribution of the distance
	 *	@param[in] edge_list_tag is tag to carry CEdgeTypeList (value unused)
	 */
	template <class CVertex0, class CVertex1, class CMeanMeasurement, class CMeanDistribution, class CEdgeTypeList>
	void operator ()(const CVertex0 &r_vertex_0, const CVertex1 &r_vertex_1,
		CMeanMeasurement &r_mean_d, CMeanDistribution &r_t_Sigma_d, CEdgeTypeList UNUSED(edge_list_tag)) const
	{
		Eigen::Matrix<double, 4, 6> H_transform = Eigen::Matrix<double, 4, 6>::Identity();
		const double Aa = r_mean_d(3), Ab = r_mean_d(4), Ac = r_mean_d(5);
		const double D = r_mean_d.template tail<3>().squaredNorm(); // !!

		if(!D) {
			double f_cos_angle = 1; // dot product
			double f_angular_distance = 0; // acos(f_cos_angle);
			// calculate the angle

			H_transform(3, 3) = -1 / sqrt(2.0);
			H_transform(3, 4) = -1 / sqrt(2.0); // or negative, when approaching from the right
			//H_transform(3, 3) = -1;
			//H_transform(3, 4) = 0; // (limit from the left) or -1 when approaching from the right, there seems to be a singularity there
			H_transform(3, 5) = 0;
			// unity jacobian? // note that the axis depends on the order of limits (either x or y). sadly, matlab can't calculate limit of several variables at the same time
			// also getting nan nan nan, -inf -inf -inf, nan nan 0, +-1/sqrt(2) +-1/sqrt(2) 0, ... take your pick

			r_mean_d(3) = f_angular_distance;
			r_mean_d.conservativeResize(4);
			// transform mean

			r_t_Sigma_d = H_transform * r_t_Sigma_d * H_transform.transpose();
			// transform the sigma

			return;
		}
		// zero rotation needs to be treated differently as it would cause plentiful division by zero

		const double invD = 1 / D, invD2 = invD * invD, sqrtD = sqrt(D), rsqrtD = 1 / sqrtD;
		double f_cos_angle = 1 + (1 - cos(sqrtD)) * (-Ab * Ab * invD - Aa * Aa * invD);
		const double f_denom = sqrt(1 - f_cos_angle * f_cos_angle/*pow(1 + (1 - cos(sqrtD)) * (-Ab * Ab * invD - Aa * Aa * invD), 2.0)*/);
		H_transform(3, 3) = -(sin(sqrtD) * Aa * rsqrtD * (-Ab * Ab * invD - Aa * Aa * invD) + (1 - cos(sqrtD)) *
			(2 * Ab * Ab * invD2 * Aa - 2 * Aa * invD + 2 * Aa * Aa * Aa * invD2)) / f_denom;
		H_transform(3, 4) = -(sin(sqrtD) * rsqrtD * Ab * (-Ab * Ab * invD - Aa * Aa * invD) + (1 - cos(sqrtD)) *
			(-2 * Ab * invD + 2 * Ab * Ab * Ab * invD2 + 2 * Aa * Aa * invD2 * Ab)) / f_denom;
		H_transform(3, 5) = -(sin(sqrtD) * rsqrtD * Ac * (-Ab * Ab * invD - Aa * Aa * invD) + (1 - cos(sqrtD)) *
			(2 * Ab * Ab * invD2 * Ac + 2 * Aa * Aa * invD2 * Ac)) / f_denom;
		// transformation

		double f_angular_distance = acos(f_cos_angle);
		// calculate the angle

		/*double t1, t2, t3, t4, t6, t7, t8, J[3];
		t1 = Aa * Aa + Ab * Ab + Ac * Ac; // D
		t2 = sin(sqrt(t1) / 2); // real quaternion scaler
		t3 = cos(sqrt(t1) / 2); // imaginary quaternion scaler
		t4 = Aa * Aa + Ab * Ab - Ac * Ac;
		t6 = t3 * t1 * (t1 + t4) - 2 * t2 * sqrt(t1) * t4;
		t7 = t2 / (pow(t1, 2.5) * sqrt(1 - (t2 * t2 / t1 * t4 - t3 * t3) * (t2 * t2 / t1 * t4 - t3 * t3)));
		t8 = 2 * t2 * sqrt(t1 * t1 * t1);
		J[0] = Aa * t7 * (t6 + t8);
		J[1] = Ab * t7 * (t6 + t8);
		J[2] = Ac * t7 * (t6 - t8);
		// quaternion version of the same jacobian (all hail subexpr(); identical results to the top formula)*/ // todo - use this instead, there's nothing wrong with it

		r_mean_d(3) = f_angular_distance;
		r_mean_d.conservativeResize(4);
		// transform mean

		r_t_Sigma_d = H_transform * r_t_Sigma_d * H_transform.transpose();
		// transform the sigma
	}
};

/**
 *	@brief utility code for distances calculation
 */
class CDistancesUtils {
public:
	/**
	 *	@brief error function
	 *	@param[in] x is value of the argument
	 *	@return Returns approximate <tt>erf(x)</tt>.
	 */
	static double f_Erf(double x)
	{
		const double p = 0.3275911, a1 = 0.254829592, a2 = -0.284496736,
			a3 = 1.421413741, a4 = -1.453152027, a5 = 1.061405429;
		double t = 1 / (1 + p * fabs(x));
		double t2 = t * t;
		double t3 = t2 * t;
		double t4 = t2 * t2;
		double t5 = t4 * t;
		double f_sign = ((x < 0)? -1 : 1);
		double f_exp = exp(-x * x);
		return (1 - (a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5) * f_exp) * f_sign;
	}

	/**
	 *	@brief Gaussian CPD function
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *	@tparam Derived2 is Eigen derived matrix type for the third matrix argument
	 *
	 *	@param[in] r_v_mean_d is mean distance value
	 *	@param[in] r_t_Sigma_d is distance distribution
	 *	@param[in] v_threshold is distance threshold
	 *
	 *	@return Returns the vector of probabilities that the distances are below the threshold.
	 */
	template <class Derived0, class Derived1, class Derived2>
	static typename CDeriveMatrixType<Derived0, true>::_TyResult v_GaussianCPD(const Eigen::MatrixBase<Derived0> &r_v_mean_d,
		const Eigen::MatrixBase<Derived1> &r_t_Sigma_d, const Eigen::MatrixBase<Derived2> &v_threshold)
	{
		typename CDeriveMatrixType<Derived0, true>::_TyResult p = ((v_threshold - r_v_mean_d).array() /
			(r_t_Sigma_d.diagonal() * 2.0).array().sqrt()).matrix();
		for(size_t i = 0, n = p.rows(); i < n; ++ i)
			p(i) = (1 + f_Erf(p(i))) / 2;
		return p;
	}

	/**
	 *	@brief calculates the information gain
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[in] r_t_Sigma_e is the covariance of the measurement
	 *	@param[in] r_t_Sigma_d is the covariance of the update
	 *
	 *	@return Returns the information gain \f$0.5 log(|\Sigma_e^{-1}| * |\Sigma_e + \Sigma_d|)\f$.
	 */
	template <class Derived0, class Derived1>
	static inline double f_Information_Gain(const Eigen::MatrixBase<Derived0> &r_t_Sigma_e, const Eigen::MatrixBase<Derived1> &r_t_Sigma_d)
	{
		double f_det0 = r_t_Sigma_e.determinant(), f_det1 = (r_t_Sigma_e + r_t_Sigma_d).determinant();
		double f_det_prod = f_det1 / f_det0;
		return 0.5 * log(f_det_prod);
	}
};

/**
 *	@brief probability distance calculation
 *
 *	@tparam CSystemType is system type
 *	@tparam CEdgeTypeList is a list of edges to be used for distance calculation (by default all the edges in the list)
 *	@tparam CDistanceTransform is distance transform function (by default CNullDistanceTransform)
 *
 *	@note See compact SLAM example for an example of use.
 */
template <class CSystemType, class CEdgeTypeList = typename CSystemType::_TyEdgeTypelist,
	class CDistanceTransform = CNullDistanceTransform>
class CDistances {
protected:
	/**
	 *	@brief utility class, wraps a pair of vertex types into a single type
	 *
	 *	@tparam CVertex0 is the first vertex type
	 *	@tparam CVertex1 is the second vertex type
	 */
	template <class CVertex0, class CVertex1>
	class CFilterReference {
	public:
		typedef CVertex0 _TyVertex0; /**< @brief the first vertex type */
		typedef CVertex1 _TyVertex1; /**< @brief the second vertex type */
	};

	/**
	 *	@brief edge search predicate; finds a binary edge between the two given vertex types
	 *
	 *	@tparam CEdgeType is edge type
	 *	@tparam CRefType is specialization of CFilterReference (contains the vertex types)
	 */
	template <class CEdgeType, class CRefType>
	class CEdgePredicate {
	protected:
		/**
		 *	@brief intermediates, stored as enum
		 */
		enum {
			n_edge_rank = CEdgeType::n_vertex_num, /**< @brief number of vertices in the edge */
			n_vertex1_index = (n_edge_rank > 1)? 1 : 0 /**< @brief index of the second vertex (falls back to the first vertex to avoid compile problems with unary edges) */
		};

		typedef typename CEdgeType::_TyVertices _TyEdgeVertices; /**< @brief list of vertices in the edge */

		typedef typename CTypelistItemAt<_TyEdgeVertices, 0>::_TyResult _TyEdgeVertex0; /**< @brief type of the first vertex */ // this is always there, we have no rank-0 edges
		typedef typename CTypelistItemAt<_TyEdgeVertices, n_vertex1_index>::_TyResult _TyEdgeVertex1; /**< @brief type of the second vertex */ // for unary edges, this is the same type as _TyEdgeVertex0

		/**
		 *	@brief intermediates, stored as enum
		 */
		enum {
			b_vertices_swapped_int = CEqualType<typename CRefType::_TyVertex1, _TyEdgeVertex0>::b_result &&
				CEqualType<typename CRefType::_TyVertex0, _TyEdgeVertex1>::b_result /**< @brief internal vertex swap flag */
		};

	public:
		/**
		 *	@brief result, stored as enum
		 */
		enum {
			b_result = (n_edge_rank == 2) && (b_vertices_swapped_int || // only binary edges permitted at this point
				(CEqualType<typename CRefType::_TyVertex0, _TyEdgeVertex0>::b_result &&
				CEqualType<typename CRefType::_TyVertex1, _TyEdgeVertex1>::b_result)), /**< @brief set if the edge can be used to calculate the distances */
			b_vertices_swapped = b_result && b_vertices_swapped_int /**< @brief vertex swap flag */
		};
	};

	/**
	 *	@brief distance calculation implementation
	 *
	 *	@tparam CVertex0 is the first vertex type
	 *	@tparam CVertex1 is the second vertex type
	 *	@tparam CValidEdges is typelist of edge types that can be used for distance calculation
	 */
	template <class CVertex0, class CVertex1, class CValidEdges>
	class CCalculateDistance {
	public:
		typedef CVertex0 _TyVertex0; /**< @brief the first vertex type */
		typedef CVertex1 _TyVertex1; /**< @brief the second vertex type */
		typedef CValidEdges _TyValidEdges; /**< @brief list of edge types that can be used for distance calculation */
		typedef typename _TyValidEdges::_TyHead _TyEdge; /**< @brief the edge type to be used for distance calculation */ // we will use this edge type to calculate the jacobians
		// choose an edge to use

		typedef typename _TyEdge::template CVertexTraits<0>::_TyVector _TyState0; /**< @brief state vector of the first vertex */
		typedef typename _TyEdge::template CVertexTraits<1>::_TyVector _TyState1; /**< @brief state vector of the second vertex */
		typedef typename _TyEdge::template CVertexTraits<0>::_TyJacobianMatrix _TyJacobian0; /**< @brief jacobian of the first vertex */
		typedef typename _TyEdge::template CVertexTraits<1>::_TyJacobianMatrix _TyJacobian1; /**< @brief jacobian of the second vertex */
		typedef typename _TyEdge::_TyVector _TyResidualVector; /**< @brief edge residual vector */
		typedef typename _TyEdge::_TyStorageVector _TyMeasurementVector; /**< @brief edge measurement vector */
		// edge matrix and vector types

		/**
		 *	@brief intermediates, stored as enum
		 */
		enum {
			b_edge_type_found = CFindTypelistItem<CEdgeTypeList, _TyEdge>::b_result, /**< @brief should be always true */
			n_edge_type_index = CFindTypelistItem<CEdgeTypeList, _TyEdge>::n_index, /**< @brief index of the edge in the edge type list */

			n_residual_dim = _TyEdge::n_residual_dimension, /**< @brief dimension of the edge residual */
			n_vertex0_dim = _TyEdge::template CVertexTraits<0>::n_dimension, /**< @brief dimension of the first vertex */
			n_vertex1_dim = _TyEdge::template CVertexTraits<1>::n_dimension /**< @brief dimension of the second vertex */
		};

		typedef typename CDistanceTransform::template CTransformedDimension<_TyEdge> CTrDim; // transformed dimension type

		enum {
			n_transformed_dimension = CTrDim::n_result, // todo - make sure this is not Eigen::Dynamic

			n_max_result_dimension = (int(n_residual_dim) > int(n_transformed_dimension))? int(n_residual_dim) : int(n_transformed_dimension),
			n_result_dimension = (int(n_residual_dim) == int(n_transformed_dimension))? int(n_residual_dim) : int(Eigen::Dynamic)
		};

		typedef Eigen::Matrix<double, n_result_dimension, 1, Eigen::ColMajor | Eigen::AutoAlign, n_max_result_dimension, 1> _TyResultVec; /**< @brief result vector type */
		typedef Eigen::Matrix<double, n_result_dimension, n_result_dimension, Eigen::ColMajor |
			Eigen::AutoAlign, n_max_result_dimension, n_max_result_dimension> _TyResultMat; /**< @brief result matrix type */

	protected:
		const CSystemType &r_system; /**< @brief const reference to the optimized system (need to access edge and vertex pools) */
		const CUberBlockMatrix &r_marginals; /**< @brief const reference to the marginals matrix */
		const Eigen::VectorXd **p_distance_threshold_list; /**< @brief list of distance thresholds */
		const _TyVertex0 &r_v0; /**< @brief reference to the first vertex */
		const _TyVertex1 &r_v1; /**< @brief reference to the second vertex */

		CDistanceTransform m_transform; /**< @brief distance transformation object */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] _r_system is const reference to the optimized system
		 *	@param[in] _r_marginals is const reference to the marginals matrix
		 *	@param[in] _p_distance_threshold_list is list of distance thresholds
		 *	@param[in] _r_v0 is reference to the first vertex
		 *	@param[in] _r_v1 is reference to the second vertex
		 *	@param[in] transform is distance transformation object
		 */
		CCalculateDistance(const CSystemType &_r_system, const CUberBlockMatrix &_r_marginals,
			const Eigen::VectorXd **_p_distance_threshold_list,
			const _TyVertex0 &_r_v0, const _TyVertex1 &_r_v1, CDistanceTransform transform)
			:r_system(_r_system), r_marginals(_r_marginals),
			p_distance_threshold_list(_p_distance_threshold_list),
			r_v0(_r_v0), r_v1(_r_v1), m_transform(transform)
		{
			_ASSERTE(b_edge_type_found);
			// should be compile-time, but it should always pass (if it does not, it is an internal error)
		}

		/**
		 *	@brief Gaussian CPD function
		 *
		 *	@param[in] r_v_mean_d is mean distance value
		 *	@param[in] r_t_Sigma_d is distance distribution
		 *	@param[in] v_threshold is distance threshold
		 *
		 *	@return Returns the vector of probabilities that the distances are below the threshold.
		 *
		 *	@note This is not a generic function. It is specialized for the dimensions of this distance.
		 */
		static _TyResultVec v_GaussianCPD(const _TyResultVec &r_v_mean_d,
			const _TyResultMat &r_t_Sigma_d, const Eigen::VectorXd &v_threshold)
		{
			_TyResultVec p = ((v_threshold - r_v_mean_d).array() /
				(r_t_Sigma_d.diagonal() * 2.0).array().sqrt()).matrix();
			_ASSERTE(p.rows() == n_transformed_dimension);
			for(int i = 0; i < n_transformed_dimension; ++ i)
				p(i) = (1 + CDistancesUtils::f_Erf(p(i))) / 2;
			return p;
		}

		/**
		 *	@brief calculates the \f$\Sigma_d\f$ matrix of the vertices specified in the constructor
		 *	@param[out] r_t_Sigma_d is filled with the \f$\Sigma_d\f$ matrix
		 */
		_TyResultVec v_Mean_d_Sigma_d(_TyResultMat &r_t_Sigma_d) const
		{
			_TyJacobian0 J0;
			_TyJacobian1 J1;
			_TyMeasurementVector v_measurement;
			v_measurement.setZero();
			_TyResidualVector v_expectation, v_error;
			_TyEdge::Calculate_Jacobians(J0, J1, v_expectation, v_error, r_v0, r_v1, v_measurement);
			// calculate the jacobians

			// this would have to be called differently for hyperedge - would have to pack vertex refs
			// to a tuple and would have to search for which vertices are the two we are interested in.
			// on top of that, we would have to get the remaining vertices pointers (supposedly not
			// difficult as we know all the types from the edge, we would just need to change the
			// interface a bit to allow for specification of the remaining vertex indices)

			typename CUberBlockMatrix::template CMakeMatrixRef<n_vertex0_dim, n_vertex0_dim>::_TyConst
				Sigma_00(r_marginals.p_FindBlock(r_v0.n_Order(), r_v0.n_Order(), n_vertex0_dim, n_vertex0_dim));
			typename CUberBlockMatrix::template CMakeMatrixRef<n_vertex0_dim, n_vertex1_dim>::_TyConst
				Sigma_01(r_marginals.p_FindBlock(r_v0.n_Order(), r_v1.n_Order(), n_vertex0_dim, n_vertex1_dim));
			typename CUberBlockMatrix::template CMakeMatrixRef<n_vertex1_dim, n_vertex1_dim>::_TyConst
				Sigma_11(r_marginals.p_FindBlock(r_v1.n_Order(), r_v1.n_Order(), n_vertex1_dim, n_vertex1_dim));
			//CUberBlockMatrix::_TyConstMatrixXdRef Sigma_10 = Sigma_01.transpose();
			// get all the blocks
			// could do this with fixed size matrices

			Eigen::Matrix<double, n_residual_dim, n_vertex0_dim + n_vertex1_dim> J_stacked;
			J_stacked.template leftCols<n_vertex0_dim>() = J0;
			J_stacked.template rightCols<n_vertex1_dim>() = J1;
			// stack the jacobians
			// could do this with fixed size matrices

			Eigen::Matrix<double, n_vertex0_dim + n_vertex1_dim, n_vertex0_dim + n_vertex1_dim> sigma_dense;
			sigma_dense.template topLeftCorner<n_vertex0_dim, n_vertex0_dim>() = Sigma_00;
			sigma_dense.template topRightCorner<n_vertex0_dim, n_vertex1_dim>() = Sigma_01;
			sigma_dense.template bottomLeftCorner<n_vertex1_dim, n_vertex0_dim>() = Sigma_01.transpose();
			sigma_dense.template bottomRightCorner<n_vertex1_dim, n_vertex1_dim>() = Sigma_11;
			// stack the sigmas
			// could do this with fixed size matrices

			r_t_Sigma_d = J_stacked * sigma_dense * J_stacked.transpose();
			_ASSERTE(r_t_Sigma_d.rows() == r_t_Sigma_d.cols() && r_t_Sigma_d.cols() == n_residual_dim);
			// perform the actual calculation

			_TyResultVec v_mean_d = v_expectation; // absolute to relative inside Calculate_Jacobians()
			// need as _TyResultVec because of the transform potentially changing the dimension
			// t_odo - use a (statically allocated) dynamic vector with maximum dimensions, we know what will both dimensions be

			const _TyState0 &r_v_mean_0 = r_v0.r_v_State();
			const _TyState1 &r_v_mean_1 = r_v1.r_v_State();

			typedef typename MakeTypelist(_TyEdge) TEdgeTypeTag;
			m_transform(r_v_mean_0, r_v_mean_1, v_mean_d, r_t_Sigma_d, TEdgeTypeTag());
			_ASSERTE(v_mean_d.rows() == n_transformed_dimension);
			_ASSERTE(r_t_Sigma_d.rows() == r_t_Sigma_d.cols() && r_t_Sigma_d.rows() == n_transformed_dimension);
			// transform the distance

			return v_mean_d;
		}

		/**
		 *	@brief calculates the \f$\Sigma_d\f$ matrix of the vertices specified in the constructor
		 *	@return Returns the covariance matrix.
		 */
		_TyResultMat t_Sigma_d() const
		{
			_TyResultMat Sigma_d;
			v_Mean_d_Sigma_d(Sigma_d); // note that there is no overhead here, as the distance transform needs to be called with v_mean_d anyway
			// calculates sigma_d
			// need as Eigen::MatrixXd because of the transform potentially changing the dimension

			return Sigma_d;
		}

		/**
		 *	@brief calculates the distance of the vertices specified in the constructor
		 *	@param[out] r_t_Sigma_d is reference to the \f$\Sigma_d\f$ matrix
		 *	@return Returns the distance probability vector.
		 */
		_TyResultVec v_Distance(_TyResultMat &r_t_Sigma_d) const
		{
			_TyResultVec v_mean_d = v_Mean_d_Sigma_d(r_t_Sigma_d);
			// calculates sigma_d
			// need as _TyResultVec because of the transform potentially changing the dimension

			_ASSERTE(p_distance_threshold_list[n_edge_type_index]->rows() == n_transformed_dimension);
			const Eigen::VectorXd &r_v_thresh = *p_distance_threshold_list[n_edge_type_index];
			// specify the threshold, the dimension matches *after* the transform

#if 0
			Eigen::VectorXd p = ((r_v_thresh - v_mean_d).array() /
				(r_t_Sigma_d.diagonal() * 2.0).array().sqrt()).matrix();
			//_TyResidualVector pr = ((-r_v_thresh - v_mean_d).array() /
			//	(r_t_Sigma_d.diagonal() * 2.0).array().sqrt()).matrix();
			// element-wise quotients, left and right side of the algebraic integral

			for(int i = 0; i < p.rows(); ++ i)
				p(i) = (1 + CDistancesUtils::f_Erf(p(i))) / 2;
				//p(i) = (CDistancesUtils::f_Erf(p(i)) - CDistancesUtils::f_Erf(pr(i))) / 2;
			// calculare element-wise error function

			return p;
#else
			return v_GaussianCPD(v_mean_d, r_t_Sigma_d, r_v_thresh) - v_GaussianCPD(v_mean_d, r_t_Sigma_d, -r_v_thresh);
#endif
		}
	};

	/**
	 *	@brief distance calculation implementation (specialization for a pair of vertices
	 *		for which there is no edge that could be used to calculate the distance)
	 *
	 *	@tparam CVertex0 is the first vertex type
	 *	@tparam CVertex1 is the second vertex type
	 */
	template <class CVertex0, class CVertex1>
	class CCalculateDistance<CVertex0, CVertex1, CTypelistEnd> {
	public:
		enum {
			b_edge_type_found = false
		};

		typedef Eigen::VectorXd _TyResultVec; /**< @brief result vector type */
		typedef Eigen::MatrixXd _TyResultMat; /**< @brief result matrix type */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] _r_system is const reference to the optimized system (unused)
		 *	@param[in] _r_marginals is const reference to the marginals matrix (unused)
		 *	@param[in] _p_distance_threshold_list is list of distance thresholds (unused)
		 *	@param[in] _r_v0 is reference to the first vertex (unused)
		 *	@param[in] _r_v1 is reference to the second vertex (unused)
		 *	@param[in] transform is distance transformation object (unused)
		 */
		inline CCalculateDistance(const CSystemType &UNUSED(_r_system),
			const CUberBlockMatrix &UNUSED(_r_marginals),
			const Eigen::VectorXd **UNUSED(_p_distance_threshold_list),
			const CVertex0 &UNUSED(_r_v0), const CVertex1 &UNUSED(_r_v1),
			CDistanceTransform UNUSED(transform))
		{}

		/**
		 *	@brief calculates the \f$\Sigma_d\f$ matrix of the vertices specified in the constructor
		 *	@return Returns the covariance matrix (returns an empty matrix).
		 */
		_TyResultMat t_Sigma_d() const
		{
			return Eigen::MatrixXd(static_cast<Eigen::MatrixXd::Index>(0),
				static_cast<Eigen::MatrixXd::Index>(0));
		}

		/**
		 *	@brief calculates the distance of the vertices specified in the constructor
		 *	@param[out] r_t_Sigma_d is reference to the \f$\Sigma_d\f$ matrix (unused)
		 *	@return Returns the distance probability vector (returns an empty vector).
		 */
		_TyResultVec v_Distance(_TyResultMat &UNUSED(r_t_Sigma_d)) const
		{
			return Eigen::VectorXd(static_cast<Eigen::VectorXd::Index>(0));
		}
	};

	template <class CVertexType, class CMatrixType>
	struct TResult {
		CVertexType *p_distance; // might be null
		CMatrixType *p_sigma_d; // might be null
		CVertexType *p_mean_d; // might be null
		bool b_success;

		TResult(CVertexType *_p_distance, CMatrixType *_p_sigma_d, CVertexType *_p_mean_d = 0)
			:p_distance(_p_distance), p_sigma_d(_p_sigma_d), p_mean_d(_p_mean_d), b_success(false)
		{}
	};

	/**
	 *	@brief helper for vertex 1 type inference
	 *	@tparam CVertex0 is type of the first vertex
	 */
	template <class CResultType, class CVertex0>
	class CGetType_V1 {
	protected:
		typedef CVertex0 _TyVertex0; /**< @brief type of the first vertex */

		const CSystemType &r_system; /**< @brief const reference to the optimized system (need to access edge and vertex pools) */
		const CUberBlockMatrix &r_marginals; /**< @brief const reference to the marginals matrix */
		const Eigen::VectorXd **p_distance_threshold_list; /**< @brief list of distance thresholds */
		const _TyVertex0 &r_v0; /**< @brief reference to the first vertex */

		//Eigen::VectorXd &m_r_v_distance; /**< @brief reference to the (output) distance probability vector */
		//Eigen::MatrixXd &m_r_Sigma_d; /**< @brief reference to the (output) \f$\Sigma_d\f$ matrix */
		CResultType &m_r_t_result; /**< @brief reference to the result container */

		CDistanceTransform m_transform; /**< @brief distance transformation object */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[out] r_t_result is reference to the results container, filled once the function operator is called
		 *	@param[in] _r_system is const reference to the optimized system
		 *	@param[in] _r_marginals is const reference to the marginals matrix
		 *	@param[in] _p_distance_threshold_list is list of distance thresholds
		 *	@param[in] _r_v0 is reference to the first vertex
		 *	@param[in] transform is distance transformation object
		 */
		// *	@param[out] r_v_distance is reference to the distance probability vector (output, written once the function operator is called)
		// *	@param[out] r_t_Sigma_d is reference to the \f$\Sigma_d\f$ matrix (output, written once the function operator is called)
		CGetType_V1(/*Eigen::VectorXd &r_v_distance, Eigen::MatrixXd &r_t_Sigma_d,*/CResultType &r_t_result,
			const CSystemType &_r_system, const CUberBlockMatrix &_r_marginals,
			const Eigen::VectorXd **_p_distance_threshold_list, const _TyVertex0 &_r_v0, CDistanceTransform transform)
			:r_system(_r_system), r_marginals(_r_marginals),
			p_distance_threshold_list(_p_distance_threshold_list), r_v0(_r_v0),
			/*m_r_v_distance(r_v_distance), m_r_Sigma_d(r_t_Sigma_d),*/
			m_r_t_result(r_t_result), m_transform(transform)
		{}

		/**
		 *	@brief function operator; used to infer the type of vertex 1
		 *	@tparam _TyVertex1 is type of vertex 1
		 *	@param[in] r_v1 is state of vertex 1
		 */
		template <class _TyVertex1>
		void operator ()(const _TyVertex1 &r_v1) // throw(std::bad_alloc)
		{
			// now we have both types _TyVertex0, _TyVertex1 and values r_v0, r_v1

			typedef typename CFilterTypelist<CEdgeTypeList, CFilterReference<_TyVertex0,
				_TyVertex1>, CEdgePredicate>::_TyResult _TyValidEdges;
			// get list of edges that can be used to calculate jacobian of v0, v1
			// note that the list may be empty and we need to solve this gracefully,
			// requiring one more template layer

			typedef CCalculateDistance<_TyVertex0, _TyVertex1, _TyValidEdges> _TyCD;

			m_r_t_result.b_success = _TyCD::b_edge_type_found;
			// see if we can calculate the distance

			_TyCD cd(r_system, r_marginals, p_distance_threshold_list, r_v0, r_v1, m_transform);
			if(m_r_t_result.p_distance) { // want distance
				typename _TyCD::_TyResultMat sigma_d;
				*m_r_t_result.p_distance = cd.v_Distance(sigma_d);
				if(m_r_t_result.p_sigma_d)
					*m_r_t_result.p_sigma_d = sigma_d;
			} else { // want only sigma d
				_ASSERTE(m_r_t_result.p_sigma_d);
				if(m_r_t_result.p_mean_d) {
					typename _TyCD::_TyResultMat sigma_d;
					*m_r_t_result.p_mean_d = cd.v_Mean_d_Sigma_d(sigma_d);
					*m_r_t_result.p_sigma_d = sigma_d;
				} else
					*m_r_t_result.p_sigma_d = cd.t_Sigma_d();
			}
			//m_r_v_distance = cd.v_Distance(m_r_Sigma_d);
			// calculate the distance (or not, if there are no edges)
		}
	};

	/**
	 *	@brief helper for vertex 0 type inference
	 */
	template <class CResultType>
	struct CGetType_V0 {
	protected:
		const CSystemType &r_system; /**< @brief const reference to the optimized system (need to access edge and vertex pools) */
		const CUberBlockMatrix &r_marginals; /**< @brief const reference to the marginals matrix */
		const Eigen::VectorXd **p_distance_threshold_list; /**< @brief list of distance thresholds */
		size_t n_vertex1; /**< @brief id of the second vertex */

		//Eigen::VectorXd &m_r_v_distance; /**< @brief reference to the (output) distance probability vector */
		//Eigen::MatrixXd &m_r_Sigma_d; /**< @brief reference to the (output) \f$\Sigma_d\f$ matrix */
		CResultType &m_r_t_result; /**< @brief reference to the result container */

		CDistanceTransform m_transform; /**< @brief distance transformation object */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[out] r_t_result is reference to the results container, filled once the function operator is called
		 *	@param[in] _r_system is const reference to the optimized system
		 *	@param[in] _r_marginals is const reference to the marginals matrix
		 *	@param[in] _p_distance_threshold_list is list of distance thresholds
		 *	@param[in] _n_vertex1 is id of the second vertex
		 *	@param[in] transform is distance transformation object
		 */
		// *	@param[out] r_v_distance is reference to the distance probability vector (output, written once the function operator is called)
		// *	@param[out] r_t_Sigma_d is reference to the \f$\Sigma_d\f$ matrix (output, written once the function operator is called)
		CGetType_V0(CResultType &r_t_result,
			/*Eigen::VectorXd &r_v_distance, Eigen::MatrixXd &r_t_Sigma_d,*/
			const CSystemType &_r_system, const CUberBlockMatrix &_r_marginals,
			const Eigen::VectorXd **_p_distance_threshold_list, size_t _n_vertex1, CDistanceTransform transform)
			:r_system(_r_system), r_marginals(_r_marginals),
			p_distance_threshold_list(_p_distance_threshold_list),
			n_vertex1(_n_vertex1), /*m_r_v_distance(r_v_distance),
			m_r_Sigma_d(r_t_Sigma_d),*/m_r_t_result(r_t_result), m_transform(transform)
		{}

		/**
		 *	@brief function operator; used to infer the type of vertex 0
		 *	@tparam _TyVertex0 is type of vertex 0
		 *	@param[in] r_v0 is state of vertex 0
		 */
		template <class _TyVertex0>
		void operator ()(const _TyVertex0 &r_v0) // throw(std::bad_alloc)
		{
			r_system.r_Vertex_Pool().For_Each(n_vertex1, n_vertex1 + 1,
				CGetType_V1<CResultType, _TyVertex0>(m_r_t_result/*m_r_v_distance, m_r_Sigma_d*/, r_system, r_marginals,
				p_distance_threshold_list, r_v0, m_transform));
		}
	};

	/**
	 *	@brief debug check of thresholds dimensions
	 */
	class CDimensionCheck {
	protected:
		const Eigen::VectorXd **m_p_distance_threshold_list; /**< @brief array of distance threshold pointers */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] p_distance_threshold_list is array of distance threshold pointers
		 */
		CDimensionCheck(const Eigen::VectorXd **p_distance_threshold_list)
			:m_p_distance_threshold_list(p_distance_threshold_list)
		{}

		/**
		 *	@brief function operator; performs dimension check for a single edge type
		 *
		 *	This checks whether the dimension of the vector in the list matches the dimension
		 *	of the corresponding edge in CEdgeTypeList. These checks are debug-only.
		 *
		 *	@tparam CEdgeType is edge type (this needs to be called with edge types in the order of CEdgeTypeList)
		 */
		template <class CEdgeType>
		inline void operator ()()
		{
			_ASSERTE(*m_p_distance_threshold_list); // make sure it is not null

			_ASSERTE((*m_p_distance_threshold_list)->rows() ==
				CDistanceTransform::template CTransformedDimension<CEdgeType>::n_result);
			// make sure that the residual and the thresghold have the same dimension
			// if this triggers, then most likely the list of edges does not match the list of thresholds

			++ m_p_distance_threshold_list;
		}
	};

protected:
	const CSystemType &m_r_system; /**< @brief const reference to the optimized system (need to access edge and vertex pools) */
	const CUberBlockMatrix &m_r_marginals; /**< @brief const reference to the marginals matrix */
	const Eigen::VectorXd **m_p_distance_threshold_list; /**< @brief list of distance thresholds */ // size not remembered

	std::vector<Eigen::VectorXd> m_thresh_list; /**< @brief stores threshold values @note only used if b_copy_thresholds was set in the constructor */
	std::vector<const Eigen::VectorXd*> m_thresh_ptr_list; /**< @brief stores threshold pointers @note only used if b_copy_thresholds was set in the constructor */
	// only used if b_copy_thresholds was set in the constructor

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] r_system is reference to the optimized system
	 *	@param[in] r_marginals is reference to the marginals matrix
	 *	@param[in] p_distance_threshold_list is a list of pointers to distance
	 *		thresholds (the "sensor precision" / "field of view")
	 *	@param[in] n_distance_threshold_num is number of distance thresholds
	 *		(must equal the number of edge types in CEdgeTypeList)
	 *	@param[in] b_copy_thresholds is local threshold table copy flag
	 *		(if set, the distance threshold is copied to local storage of this
	 *		object; default not set)
	 *
	 *	@note None of the arguments are copied, the referenced objects must
	 *		exist for the (useful) lifetime of this object.
	 *	@note See \ref slam_dataassoc_example/Main.cpp or \ref dataassocexample for example use.
	 *	@note If b_copy_thresholds is set, this function throws std::bad_alloc.
	 */
	CDistances(const CSystemType &r_system, const CUberBlockMatrix &r_marginals,
		const Eigen::VectorXd **p_distance_threshold_list,
		size_t n_distance_threshold_num, bool b_copy_thresholds = false) // throw(std::bad_alloc)
		:m_r_system(r_system), m_r_marginals(r_marginals),
		m_p_distance_threshold_list(p_distance_threshold_list)
	{
		_ASSERTE(m_r_marginals.b_SymmetricLayout()); // this code can only be used on marginals with symmetric structure (addressable by vertex ids); if this triggers the matrix is likely not a marginals matrix
		_ASSERTE(m_r_marginals.n_BlockColumn_Num() == r_system.r_Vertex_Pool().n_Size()); // if this triggers, the marginals are not up to date
		// make sure the marginals matrix looks good

		_ASSERTE(n_distance_threshold_num == CTypelistLength<CEdgeTypeList>::n_result);
		// make sure that there are as many thresholds as there are edges

		CTypelistForEach<CEdgeTypeList, CDimensionCheck>::Run(
			CDimensionCheck(p_distance_threshold_list));
		// make sure that the dimensions of the thresholds match

		if(b_copy_thresholds) {
			m_thresh_list.resize(n_distance_threshold_num);
			m_thresh_ptr_list.resize(n_distance_threshold_num);
			for(size_t i = 0; i < n_distance_threshold_num; ++ i) {
				m_thresh_list[i] = *p_distance_threshold_list[i]; // copy values of the vectors
				m_thresh_ptr_list[i] = &m_thresh_list[i];
			}
			m_p_distance_threshold_list = (m_thresh_ptr_list.empty())? 0 : &m_thresh_ptr_list[0];
		}
		// copy the thresholds table to the local storage, so that it does not need to
		// be kept alongside this object
	}

	/**
	 *	@brief calculates probability that the vertices are below distance threshold
	 *
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *
	 *	@param[out] r_v_distance is filled with distance probability between the specified vertices, after the specified transformation
	 *	@param[in] n_vertex0 is id of the first vertex (must be in the system)
	 *	@param[in] n_vertex1 is id of the second vertex (must be in the system)
	 *	@param[in] transform is distance transform instance
	 *
	 *	@return Returns true on success, or false if there are no edge types that can be used to calculate the distance.
	 */
	template <class Derived>
	bool Calculate_Distance(Eigen::MatrixBase<Derived> &r_v_distance, size_t n_vertex0, size_t n_vertex1, CDistanceTransform transform = CDistanceTransform()) const // throw(std::bad_alloc)
	{
		_ASSERTE(m_r_marginals.b_SymmetricLayout()); // this code can only be used on marginals with symmetric structure (addressable by vertex ids); if this triggers the matrix is likely not a marginals matrix
		_ASSERTE(m_r_marginals.n_BlockColumn_Num() == m_r_system.r_Vertex_Pool().n_Size()); // if this triggers, the marginals are not up to date
		// make sure the marginals matrix looks good (can change over time)

		CTypelistForEach<CEdgeTypeList, CDimensionCheck>::Run(
			CDimensionCheck(m_p_distance_threshold_list)); // if this triggers, the dimensions of thresholds changed; if it crashes, the array with dimensions (or the vectors pointed to) went out of scope and was deleted from memory
		// make sure that the dimensions of the thresholds match (can change over time)

		typedef Eigen::MatrixBase<Derived> _TyDistance;
		typedef TResult<_TyDistance, Eigen::MatrixXd> _TyResult;
		_TyResult result(&r_v_distance, 0);

		m_r_system.r_Vertex_Pool().For_Each(n_vertex0, n_vertex0 + 1,
			CGetType_V0<_TyResult>(result/*r_v_distance, Sigma_d*/, m_r_system, m_r_marginals,
			m_p_distance_threshold_list, n_vertex1, transform));

		return result.b_success;
	}

	/**
	 *	@brief calculates probability that the vertices are below distance threshold and returns both the distance and the covariance
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[out] r_v_distance is filled with distance probability between the specified vertices, after the specified transformation
	 *	@param[out] r_t_Sigma_d is filled with the probability distribution \f$\Sigma_d\f$, after the specified transformation
	 *	@param[in] n_vertex0 is id of the first vertex (must be in the system)
	 *	@param[in] n_vertex1 is id of the second vertex (must be in the system)
	 *	@param[in] transform is distance transform instance
	 *
	 *	@return Returns true on success, or false if there are no edge types that can be used to calculate the distance.
	 */
	template <class Derived0, class Derived1>
	bool Calculate_Distance_Sigma_d(Eigen::MatrixBase<Derived0> &r_v_distance, Eigen::MatrixBase<Derived1> &r_t_Sigma_d,
		size_t n_vertex0, size_t n_vertex1, CDistanceTransform transform = CDistanceTransform()) const
	{
		_ASSERTE(m_r_marginals.b_SymmetricLayout()); // this code can only be used on marginals with symmetric structure (addressable by vertex ids); if this triggers the matrix is likely not a marginals matrix
		_ASSERTE(m_r_marginals.n_BlockColumn_Num() == m_r_system.r_Vertex_Pool().n_Size()); // if this triggers, the marginals are not up to date
		// make sure the marginals matrix looks good (can change over time)

		CTypelistForEach<CEdgeTypeList, CDimensionCheck>::Run(
			CDimensionCheck(m_p_distance_threshold_list)); // if this triggers, the dimensions of thresholds changed; if it crashes, the array with dimensions (or the vectors pointed to) went out of scope and was deleted from memory
		// make sure that the dimensions of the thresholds match (can change over time)

		typedef Eigen::MatrixBase<Derived0> _TyDistance;
		typedef Eigen::MatrixBase<Derived1> _TySigma;
		typedef TResult<_TyDistance, _TySigma> _TyResult;
		_TyResult result(&r_v_distance, &r_t_Sigma_d);

		m_r_system.r_Vertex_Pool().For_Each(n_vertex0, n_vertex0 + 1,
			CGetType_V0<_TyResult>(result/*v_distance, Sigma_d*/, m_r_system, m_r_marginals,
			m_p_distance_threshold_list, n_vertex1, transform));

		return result.b_success;
	}

	/**
	 *	@brief calculates probabilty distribution between two vertices
	 *
	 *	@tparam Derived is Eigen derived matrix type for the first matrix argument
	 *
	 *	@param[out] r_t_Sigma_d is filled with the probability distribution \f$\Sigma_d\f$, after the specified transformation
	 *	@param[in] n_vertex0 is id of the first vertex (must be in the system)
	 *	@param[in] n_vertex1 is id of the second vertex (must be in the system)
	 *	@param[in] transform is distance transform instance
	 *
	 *	@return Returns true on success, or false if there are no edge types that can be used to calculate the distance.
	 *
	 *	@note This is still kind of imperfect, as this does not require the distance list at all, yet one
	 *		needs to specify in the constructor. Could write a static version which would get a reference
	 *		to the system and the marginals only.
	 */
	template <class Derived>
	bool Calculate_Sigma_d(Eigen::MatrixBase<Derived> &r_t_Sigma_d,
		size_t n_vertex0, size_t n_vertex1, CDistanceTransform transform = CDistanceTransform()) const
	{
		_ASSERTE(m_r_marginals.b_SymmetricLayout()); // this code can only be used on marginals with symmetric structure (addressable by vertex ids); if this triggers the matrix is likely not a marginals matrix
		_ASSERTE(m_r_marginals.n_BlockColumn_Num() == m_r_system.r_Vertex_Pool().n_Size()); // if this triggers, the marginals are not up to date
		// make sure the marginals matrix looks good (can change over time)

		CTypelistForEach<CEdgeTypeList, CDimensionCheck>::Run(
			CDimensionCheck(m_p_distance_threshold_list)); // if this triggers, the dimensions of thresholds changed; if it crashes, the array with dimensions (or the vectors pointed to) went out of scope and was deleted from memory
		// make sure that the dimensions of the thresholds match (can change over time)

		typedef Eigen::MatrixBase<Derived> _TySigma;
		typedef TResult<Eigen::VectorXd, _TySigma> _TyResult;
		_TyResult result(0, &r_t_Sigma_d);

		m_r_system.r_Vertex_Pool().For_Each(n_vertex0, n_vertex0 + 1,
			CGetType_V0<_TyResult>(result/*v_distance, Sigma_d*/, m_r_system, m_r_marginals,
			m_p_distance_threshold_list, n_vertex1, transform));

		return result.b_success;
	}

	/**
	 *	@brief calculates mean spatial distance and the associated probabilty distribution between two vertices
	 *
	 *	@tparam Derived0 is Eigen derived matrix type for the first matrix argument
	 *	@tparam Derived1 is Eigen derived matrix type for the second matrix argument
	 *
	 *	@param[out] r_v_mean_d is filled with spatial distance between the specified vertices, after the specified transformation
	 *	@param[out] r_t_Sigma_d is filled with the probability distribution \f$\Sigma_d\f$, after the specified transformation
	 *	@param[in] n_vertex0 is id of the first vertex (must be in the system)
	 *	@param[in] n_vertex1 is id of the second vertex (must be in the system)
	 *	@param[in] transform is distance transform instance
	 *
	 *	@return Returns true on success, or false if there are no edge types that can be used to calculate the distance.
	 *
	 *	@note This is still kind of imperfect, as this does not require the distance list at all, yet one
	 *		needs to specify in the constructor. Could write a static version which would get a reference
	 *		to the system and the marginals only.
	 */
	template <class Derived0, class Derived1>
	bool Calculate_Mean_d_Sigma_d(Eigen::MatrixBase<Derived0> &r_v_mean_d, Eigen::MatrixBase<Derived1> &r_t_Sigma_d,
		size_t n_vertex0, size_t n_vertex1, CDistanceTransform transform = CDistanceTransform()) const
	{
		_ASSERTE(m_r_marginals.b_SymmetricLayout()); // this code can only be used on marginals with symmetric structure (addressable by vertex ids); if this triggers the matrix is likely not a marginals matrix
		_ASSERTE(m_r_marginals.n_BlockColumn_Num() == m_r_system.r_Vertex_Pool().n_Size()); // if this triggers, the marginals are not up to date
		// make sure the marginals matrix looks good (can change over time)

		CTypelistForEach<CEdgeTypeList, CDimensionCheck>::Run(
			CDimensionCheck(m_p_distance_threshold_list)); // if this triggers, the dimensions of thresholds changed; if it crashes, the array with dimensions (or the vectors pointed to) went out of scope and was deleted from memory
		// make sure that the dimensions of the thresholds match (can change over time)

		typedef Eigen::MatrixBase<Derived0> _TyMeanD;
		typedef Eigen::MatrixBase<Derived1> _TySigma;
		typedef TResult<_TyMeanD, _TySigma> _TyResult;
		_TyResult result(0, &r_t_Sigma_d, &r_v_mean_d);

		m_r_system.r_Vertex_Pool().For_Each(n_vertex0, n_vertex0 + 1,
			CGetType_V0<_TyResult>(result/*v_distance, Sigma_d*/, m_r_system, m_r_marginals,
			m_p_distance_threshold_list, n_vertex1, transform));

		return result.b_success;
	}

	/**
	 *	@copydoc CDistancesUtils::f_Information_Gain
	 *	@note This is only a convenience function, it actually does not depend on any of the
	 *		template arguments. It is possible to use CDistancesUtils::f_Information_Gain().
	 */
	template <class Derived0, class Derived1>
	static inline double f_Information_Gain(const Eigen::MatrixBase<Derived0> &r_t_Sigma_e,
		const Eigen::MatrixBase<Derived1> &r_t_Sigma_d)
	{
		return CDistancesUtils::f_Information_Gain(r_t_Sigma_e, r_t_Sigma_d);
	}
};

/** @} */ // end of group

#endif // !__DISTANCES_INCLUDED
