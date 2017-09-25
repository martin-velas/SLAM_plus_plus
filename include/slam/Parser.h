/*
								+-----------------------------------+
								|                                   |
								|      ***  .graph Parser  ***      |
								|                                   |
								|  Copyright  (c) -tHE SWINe- 2012  |
								|                                   |
								|             Parser.h              |
								|                                   |
								+-----------------------------------+
*/

#pragma once
#ifndef __GRAPH_PARSER_INCLUDED
#define __GRAPH_PARSER_INCLUDED

/**
 *	@file include/slam/Parser.h
 *	@brief a simple .graph file parser
 *	@author -tHE SWINe-
 *	@date 2012-03-30
 *
 *	@date 2012-04-02
 *
 *	Added the CParseEntity_Vertex2D and CParseEntity_Edge2D classes.
 *	These contain edge and vertex data for 2D case. Note that these should
 *	not be used directly, use one of the derived classes instead.
 *
 *	Added the explicit "2D" to some type names to be clear that they
 *	are to be used for the 2D SLAM case.
 *
 *	Changed the wording from "pose" to "node", as suggested by Ela.
 *
 *	The matrices are stored as "row-major upper triangular and diagonal".
 *	That is, the sequece of elements (0, 1, 2, 3, 4, 5) make up the following matrix:
 *	@code
 *	|0 1 2|
 *	|1 3 4|
 *	|2 4 5|
 *	@endcode
 *	Which corresponds with the matrix ordering, used in the "french code":
 *	@code
 *	|0    |
 *	|1 3  |
 *	|2 4 5|
 *	@endcode
 *	That is hereby deemed correct (and the small slam example is incorrect).
 *
 *	@date 2012-04-20
 *
 *	Clarified format in which the data come from the parser:
 *
 *	Edges contain the vertices in origin - endpoint order. Sigma matrices actually
 *	contain component-wise square roots of inverse covariance.
 *
 *	And so it shall be (watch out).
 */

#include <string>
#include <vector>
#include <stdio.h>
#include <ctype.h>
#include <map>
#include <set>
#include "slam/TypeList.h"
//#include "slam/Integer.h" // included from slam/TypeList.h
#include "eigen/Eigen/Core"

/** \addtogroup parser
 *	@{
 */

/**
 *	@brief a simple .graph file parser
 */
class CParserBase {
public:
	/**
	 *	@brief base parsed entity class
	 */
	class CParseEntity {};

	/**
	 *	@brief range-bearing measurement base class
	 */
	struct TLandmark2D_RB : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the first node (origin) */
		size_t m_n_node_1; /**< @brief (zero-based) index of the second node (endpoint) */
		Eigen::Vector2d m_v_delta; /**< @brief dealte measurement (also called "z") */
		Eigen::Matrix2d m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the first (origin) node
		 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node
		 *	@param[in] f_range is delta position (the range)
		 *	@param[in] f_bearing is bearing angle
		 *	@param[in] p_upper_matrix_2x2 is row-major upper triangular and diagonal 2x2 matrix
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left-to-right column order. Example:
		 *
		 *	@code
		 *	|0 1|
		 *	|  2|
		 *	@endcode
		 */
		inline TLandmark2D_RB(size_t n_node_0, size_t n_node_1,
			double f_range, double f_bearing, const double *p_upper_matrix_2x2)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1), m_v_delta(f_range, f_bearing)
		{
			m_t_inv_sigma <<
				p_upper_matrix_2x2[0], p_upper_matrix_2x2[1],
				p_upper_matrix_2x2[1], p_upper_matrix_2x2[2];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief delta XY landmark measurement base class
	 */
	struct TLandmark2D_XY : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the first node (origin) */
		size_t m_n_node_1; /**< @brief (zero-based) index of the second node (endpoint) */
		Eigen::Vector2d m_v_delta; /**< @brief dealte measurement (also called "z") */
		Eigen::Matrix2d m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the first (origin) node
		 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node
		 *	@param[in] f_delta_x is delta x position
		 *	@param[in] f_delta_y is delta y position
		 *	@param[in] p_upper_matrix_2x2 is row-major upper triangular and diagonal 2x2 matrix
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left-to-right column order. Example:
		 *
		 *	@code
		 *	|0 1|
		 *	|  2|
		 *	@endcode
		 */
		inline TLandmark2D_XY(size_t n_node_0, size_t n_node_1,
			double f_delta_x, double f_delta_y, const double *p_upper_matrix_2x2)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1), m_v_delta(f_delta_x, f_delta_y)
		{
			m_t_inv_sigma <<
				p_upper_matrix_2x2[0], p_upper_matrix_2x2[1],
				p_upper_matrix_2x2[1], p_upper_matrix_2x2[2];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief XYT measurement base class
	 */
	struct TEdge2D : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the first (origin) node */
		size_t m_n_node_1; /**< @brief (zero-based) index of the second (endpoint) node */
		Eigen::Vector3d m_v_delta; /**< @brief dealte measurement (also called "z") */
		Eigen::Matrix3d m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the first (origin) node
		 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node
		 *	@param[in] f_delta_x is delta x position
		 *	@param[in] f_delta_y is delta y position
		 *	@param[in] f_delta_theta is delta theta
		 *	@param[in] p_upper_matrix_3x3 is row-major upper triangular and diagonal 3x3 matrix
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left to right column order. Example:
		 *	@code
		 *	|0 1 2|
		 *	|  3 4|
		 *	|    5|
		 *	@endcode
		 */
		inline TEdge2D(size_t n_node_0, size_t n_node_1,
			double f_delta_x, double f_delta_y, double f_delta_theta,
			const double *p_upper_matrix_3x3)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1),
			m_v_delta(f_delta_x, f_delta_y, f_delta_theta)
		{
			m_t_inv_sigma <<
				p_upper_matrix_3x3[0], p_upper_matrix_3x3[1], p_upper_matrix_3x3[2],
				p_upper_matrix_3x3[1], p_upper_matrix_3x3[3], p_upper_matrix_3x3[4],
				p_upper_matrix_3x3[2], p_upper_matrix_3x3[4], p_upper_matrix_3x3[5];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief node measurement class ("VERTEX" or "VERTEX2" in the datafile)
	 *	@note Vertices are not processed by the french code.
	 */
	struct TVertex2D : public CParseEntity {
		int m_n_id; /**< @brief vertex id */
		Eigen::Vector3d m_v_position; /**< @brief vertex position (the state vector) */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_id is (zero-based) index of the node
		 *	@param[in] f_x is x position
		 *	@param[in] f_y is y position
		 *	@param[in] f_theta is theta
		 *
		 *	@note The vertices are only used as ground truth. Those are mostly not processed.
		 */
		inline TVertex2D(int n_node_id, double f_x, double f_y, double f_theta)
			:m_n_id(n_node_id), m_v_position(f_x, f_y, f_theta)
		{}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief delta XYZ landmark measurement base class
	 */
	struct TLandmark3D_XYZ : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the first node (origin) */
		size_t m_n_node_1; /**< @brief (zero-based) index of the second node (endpoint) */
		Eigen::Vector3d m_v_delta; /**< @brief delta measurement (also called "z") */
		Eigen::Matrix3d m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the first (origin) node
		 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node
		 *	@param[in] f_delta_x is delta x position
		 *	@param[in] f_delta_y is delta y position
		 *	@param[in] f_delta_z is delta z position
		 *	@param[in] p_upper_matrix_3x3 is row-major upper triangular and diagonal 3x3 matrix
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left-to-right column order. Example:
		 *
		 *	@code
		 *	|0 1 2|
		 *	|  2 4|
		 *	|    5|
		 *	@endcode
		 */
		inline TLandmark3D_XYZ(size_t n_node_0, size_t n_node_1,
			double f_delta_x, double f_delta_y, double f_delta_z, const double *p_upper_matrix_3x3)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1), m_v_delta(f_delta_x, f_delta_y, f_delta_z)
		{
			m_t_inv_sigma <<
						p_upper_matrix_3x3[0], p_upper_matrix_3x3[1], p_upper_matrix_3x3[2],
						p_upper_matrix_3x3[1], p_upper_matrix_3x3[3], p_upper_matrix_3x3[4],
						p_upper_matrix_3x3[2], p_upper_matrix_3x3[4], p_upper_matrix_3x3[5];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief spheron to XYZ landmark measurement
	 */
	struct TEdgeSpheronXYZ : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the first node (origin) */
		size_t m_n_node_1; /**< @brief (zero-based) index of the second node (endpoint) */
		Eigen::Vector3d m_v_delta; /**< @brief dealte measurement (also called "z") */
		Eigen::Matrix3d m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the first (origin) node
		 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node
		 *	@param[in] f_delta_x is delta x position
		 *	@param[in] f_delta_y is delta y position
		 *	@param[in] f_delta_z is delta z position
		 *	@param[in] p_upper_matrix_3x3 is row-major upper triangular and diagonal 3x3 matrix
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left-to-right column order. Example:
		 *
		 *	@code
		 *	|0 1 2|
		 *	|  2 4|
		 *	|    5|
		 *	@endcode
		 */
		inline TEdgeSpheronXYZ(size_t n_node_0, size_t n_node_1,
			double f_delta_x, double f_delta_y, double f_delta_z, const double *p_upper_matrix_3x3)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1), m_v_delta(f_delta_x, f_delta_y, f_delta_z)
		{
			m_t_inv_sigma <<
						p_upper_matrix_3x3[0], p_upper_matrix_3x3[1], p_upper_matrix_3x3[2],
						p_upper_matrix_3x3[1], p_upper_matrix_3x3[3], p_upper_matrix_3x3[4],
						p_upper_matrix_3x3[2], p_upper_matrix_3x3[4], p_upper_matrix_3x3[5];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief XYZRPY measurement base class
	 */
	struct TEdge3D : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the first (origin) node */
		size_t m_n_node_1; /**< @brief (zero-based) index of the second (endpoint) node */
		Eigen::Matrix<double, 6, 1> m_v_delta; /**< @brief dealte measurement (also called "z") */
		Eigen::Matrix<double, 6, 6> m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the first (origin) node
		 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node
		 *	@param[in] f_delta_x is delta x position
		 *	@param[in] f_delta_y is delta y position
		 *	@param[in] f_delta_z is delta z position
		 *	@param[in] f_delta_roll is rotation around X axis
		 *	@param[in] f_delta_pitch is rotation around Y axis
		 *	@param[in] f_delta_yaw is is rotation around Z axis
		 *	@param[in] p_upper_matrix_6x6 is row-major upper triangular and diagonal 6x6 matrix
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left to right column order. Example:
		 *	@code
		 *	|0 1  2  3  4  5|
		 *	|  6  7  8  9 10|
		 *	|    11 12 13 14|
		 *	|       15 16 17|
		 *	|          18 19|
		 *	|             20|
		 *	@endcode
		 */
		inline TEdge3D(size_t n_node_0, size_t n_node_1,
			double f_delta_x, double f_delta_y, double f_delta_z,
			double f_delta_roll, double f_delta_pitch, double f_delta_yaw,
			const double *p_upper_matrix_6x6)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1)
		{
			m_v_delta << f_delta_x, f_delta_y, f_delta_z,
				f_delta_roll, f_delta_pitch, f_delta_yaw;
			// no constructor for 6-valued vector

			const double *p_u = p_upper_matrix_6x6;
			m_t_inv_sigma <<
				p_u[0],  p_u[1],  p_u[2],  p_u[3],  p_u[4],  p_u[5],
				p_u[1],  p_u[6],  p_u[7],  p_u[8],  p_u[9], p_u[10],
				p_u[2],  p_u[7], p_u[11], p_u[12], p_u[13], p_u[14],
				p_u[3],  p_u[8], p_u[12], p_u[15], p_u[16], p_u[17],
				p_u[4],  p_u[9], p_u[13], p_u[16], p_u[18], p_u[19],
				p_u[5], p_u[10], p_u[14], p_u[17], p_u[19], p_u[20];
			// fill the matrix
		}

		/**
		 *	@brief override constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the first (origin) node
		 *	@param[in] n_node_1 is (zero-based) index of the second (endpoint) node
		 *	@param[in] edge is vector containing position and AXIS-ANGLE representation of rotation
		 *	@param[in] info is 6x6 information matrix
		 *		containing the inverse sigma matrix, elements are not square roots
		 */
		inline TEdge3D(size_t n_node_0, size_t n_node_1, const Eigen::Matrix<double, 6, 1> &edge,
				const Eigen::Matrix<double, 6, 6> &info)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1), m_v_delta(edge), m_t_inv_sigma(info)
		{}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief unary factor class
	 */
	struct TUnaryFactor3D : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the node */
		Eigen::Matrix3d m_t_factor; /**< @brief the factor matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the 3D point
		 *	@param[in] p_upper_matrix_3x3 is row-major upper triangular and diagonal 3x3 matrix,
		 *		elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left to right column order. Example:
		 *	@code
		 *	|0 1 2|
		 *	|  3 4|
		 *	|    5|
		 *	@endcode
		 */
		inline TUnaryFactor3D(int n_node_0, const double *p_upper_matrix_3x3)
			:m_n_node_0(n_node_0)
		{
			m_t_factor <<
							p_upper_matrix_3x3[0], p_upper_matrix_3x3[1], p_upper_matrix_3x3[2],
							p_upper_matrix_3x3[1], p_upper_matrix_3x3[3], p_upper_matrix_3x3[4],
							p_upper_matrix_3x3[2], p_upper_matrix_3x3[4], p_upper_matrix_3x3[5];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief node measurement class ("VERTEX3" in the datafile)
	 */
	struct TVertex3D : public CParseEntity {
		int m_n_id; /**< @brief vertex id */
		Eigen::Matrix<double, 6, 1> m_v_position; /**< @brief vertex position (the state vector) */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_id is (zero-based) index of the node
		 *	@param[in] f_x is x position
		 *	@param[in] f_y is y position
		 *	@param[in] f_z is z position
		 *	@param[in] f_ax is x part of axis vector
		 *	@param[in] f_ay is y part of axis vector
		 *	@param[in] f_az is z part of axis vector
		 *
		 *	@note The vertices are only used as ground truth. Those are mostly not processed.
		 */
		inline TVertex3D(int n_node_id, double f_x, double f_y, double f_z, double f_ax, double f_ay, double f_az)
			:m_n_id(n_node_id)
		{
			m_v_position << f_x, f_y, f_z, f_ax, f_ay, f_az;
			// no constructor for 6-valued vector
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief node measurement class ("VERTEX_XYZ" in the datafile)
	 */
	struct TVertexXYZ : public CParseEntity {
		int m_n_id; /**< @brief vertex id */
		Eigen::Matrix<double, 3, 1> m_v_position; /**< @brief vertex position (the state vector) */

		/**
		 *	@brief default constructor
		 */
		inline TVertexXYZ()
		{}

		/**
		 *	@brief constructor
		 *
		 *	@param[in] n_node_id is (zero-based) index of the node
		 *	@param[in] f_x is x position
		 *	@param[in] f_y is y position
		 *	@param[in] f_z is z position
		 *
		 *	@note The vertices are only used as ground truth. Those are mostly not processed.
		 */
		inline TVertexXYZ(int n_node_id, double f_x, double f_y, double f_z)
			:m_n_id(n_node_id)
		{
			m_v_position << f_x, f_y, f_z; // no constructor for 3-valued vector
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief node measurement class ("VERTEX_CAM" in the datafile)
	 */
	struct TVertexCam3D : public CParseEntity {
		int m_n_id; /**< @brief vertex id */
		Eigen::Matrix<double, 11, 1> m_v_position; /**< @brief vertex position (the state vector) */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_id is (zero-based) index of the node
		 *	@param[in] p_x is x position
		 *	@param[in] p_y is y position
		 *	@param[in] p_z is z position
		 *	@param[in] ax is x axis angle
		 *	@param[in] ay is y axis angle
		 *	@param[in] az is z axis angle
		 *	@param[in] fx is focal length
		 *	@param[in] fy is focal length
		 *	@param[in] cx is principal point in x axis
		 *	@param[in] cy is principal point in y axis
		 *	@param[in] d is first distortion coefficient of the radial distortion
		 *
		 *	@note The vertices are only used as ground truth. Those are mostly not processed.
		 */
		inline TVertexCam3D(int n_node_id, double p_x, double p_y, double p_z,
			double ax, double ay, double az, double fx, double fy,
			double cx, double cy, double d)
			:m_n_id(n_node_id)
		{
			d *= .5 * (fx + fy)/*sqrt(fx * fy)*/; // since post-ICRA2015 release the distortion is scaled by focal length in the internal representation
			m_v_position << p_x, p_y, p_z, ax, ay, az, fx, fy, cx, cy, d;
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief node measurement class ("VERTEX_INTRINSICS" in the datafile)
	 */
	struct TVertexIntrinsics : public CParseEntity {
		int m_n_id; /**< @brief vertex id */
		Eigen::Matrix<double, 5, 1> m_v_position; /**< @brief vertex position (the state vector) */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_id is (zero-based) index of the node
		 *	@param[in] fx is focal length
		 *	@param[in] fy is focal length
		 *	@param[in] cx is principal point in x axis
		 *	@param[in] cy is principal point in y axis
		 *	@param[in] d is first distortion coefficient of the radial distortion
		 *
		 *	@note The vertices are only used as ground truth. Those are mostly not processed.
		 */
		inline TVertexIntrinsics(int n_node_id, double fx, double fy,
			double cx, double cy, double d)
			:m_n_id(n_node_id)
		{
			d *= .5 * (fx + fy)/*sqrt(fx * fy)*/; // since post-ICRA2015 release the distortion is scaled by focal length in the internal representation
			m_v_position << fx, fy, cx, cy, d;
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief node measurement class ("VERTEX_CAM" in the datafile)
	 */
	struct TVertexSCam3D : public CParseEntity {
		int m_n_id; /**< @brief vertex id */
		Eigen::Matrix<double, 12, 1> m_v_position; /**< @brief vertex position (the state vector) */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_id is (zero-based) index of the node
		 *	@param[in] p_x is x position
		 *	@param[in] p_y is y position
		 *	@param[in] p_z is z position
		 *	@param[in] ax is x axis angle
		 *	@param[in] ay is y axis angle
		 *	@param[in] az is z axis angle
		 *	@param[in] fx is focal length
		 *	@param[in] fy is fx * (aspect ratio of pixel)
		 *	@param[in] cx is principal point in x axis
		 *	@param[in] cy is principal point in y axis
		 *	@param[in] d is first distortion coefficient of the radial distortion
		 *	@param[in] b is base line
		 *
		 *	@note The vertices are only used as ground truth. Those are mostly not processed.
		 */
		inline TVertexSCam3D(int n_node_id, double p_x, double p_y, double p_z,
			double ax, double ay, double az, double fx, double fy,
			double cx, double cy, double d, double b)
			:m_n_id(n_node_id)
		{
			d *= .5 * (fx + fy)/*sqrt(fx * fy)*/; // since post-ICRA2015 release the distortion is scaled by focal length in the internal representation
			m_v_position << p_x, p_y, p_z, ax, ay, az, fx, fy, cx, cy, d, b;
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief node measurement class ("VERTEX_SPHERON" in the datafile)
	 */
	struct TVertexSpheron : public CParseEntity {
		int m_n_id; /**< @brief vertex id */
		Eigen::Matrix<double, 6, 1> m_v_position; /**< @brief vertex position (the state vector) */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_id is (zero-based) index of the node
		 *	@param[in] p_x is x position
		 *	@param[in] p_y is y position
		 *	@param[in] p_z is z position
		 *	@param[in] ax is x axis angle
		 *	@param[in] ay is y axis angle
		 *	@param[in] az is z axis angle
		 *
		 *	@note The vertices are only used as ground truth. Those are mostly not processed.
		 */
		inline TVertexSpheron(int n_node_id, double p_x, double p_y, double p_z,
			double ax, double ay, double az)
			:m_n_id(n_node_id)
		{
			m_v_position << p_x, p_y, p_z, ax, ay, az;
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief P2C measurement base class
	 */
	struct TEdgeP2C3D : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the 3D point */
		size_t m_n_node_1; /**< @brief (zero-based) index of the camera vertex */
		Eigen::Matrix<double, 2, 1> m_v_delta; /**< @brief dealte measurement (also called "z") */
		Eigen::Matrix<double, 2, 2> m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 */
		inline TEdgeP2C3D()
		{}

		/**
		 *	@brief constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the 3D point
		 *	@param[in] n_node_1 is (zero-based) index of the camera vertex
		 *	@param[in] f_delta_x is delta x position
		 *	@param[in] f_delta_y is delta y position
		 *	@param[in] p_upper_matrix_2x2 is row-major upper triangular and diagonal 2x2 matrix,
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left to right column order. Example:
		 *	@code
		 *	|0 1|
		 *	|  2|
		 *	@endcode
		 */
		inline TEdgeP2C3D(size_t n_node_0, size_t n_node_1,
			double f_delta_x, double f_delta_y, const double *p_upper_matrix_2x2)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1)
		{
			m_v_delta << f_delta_x, f_delta_y;
			// no constructor for 2-valued vector

			m_t_inv_sigma << p_upper_matrix_2x2[0], p_upper_matrix_2x2[1],
							 p_upper_matrix_2x2[1], p_upper_matrix_2x2[2];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief P2C measurement base class
	 */
	struct TEdgeP2CI3D : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the 3D point */
		size_t m_n_node_1; /**< @brief (zero-based) index of the camera vertex */
		size_t m_n_node_2; /**< @brief (zero-based) index of the intrinsics vertex */
		Eigen::Matrix<double, 2, 1> m_v_delta; /**< @brief dealte measurement (also called "z") */
		Eigen::Matrix<double, 2, 2> m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the 3D point
		 *	@param[in] n_node_1 is (zero-based) index of the camera vertex
		 *	@param[in] n_node_2 is (zero-based) index of the intrinsics vertex
		 *	@param[in] f_delta_x is delta x position
		 *	@param[in] f_delta_y is delta y position
		 *	@param[in] p_upper_matrix_2x2 is row-major upper triangular and diagonal 2x2 matrix,
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left to right column order. Example:
		 *	@code
		 *	|0 1|
		 *	|  2|
		 *	@endcode
		 */
		inline TEdgeP2CI3D(size_t n_node_0, size_t n_node_1, size_t n_node_2,
			double f_delta_x, double f_delta_y, const double *p_upper_matrix_2x2)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1), m_n_node_2(n_node_2)
		{
			m_v_delta << f_delta_x, f_delta_y;
			// no constructor for 2-valued vector

			m_t_inv_sigma <<
							p_upper_matrix_2x2[0], p_upper_matrix_2x2[1],
							p_upper_matrix_2x2[1], p_upper_matrix_2x2[2];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief P2C measurement base class
	 */
	struct TEdgeP2SC3D : public CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the 3D point */
		size_t m_n_node_1; /**< @brief (zero-based) index of the camera vertex */
		Eigen::Matrix<double, 3, 1> m_v_delta; /**< @brief dealte measurement (also called "z") */
		Eigen::Matrix<double, 3, 3> m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node_0 is (zero-based) index of the 3D point
		 *	@param[in] n_node_1 is (zero-based) index of the camera vertex
		 *	@param[in] f_delta_x1 is delta x position - left cam
		 *	@param[in] f_delta_y is delta y position
		 *	@param[in] f_delta_x2 is delta x position - right cam
		 *	@param[in] p_upper_matrix_3x3 is row-major upper triangular and diagonal 3x3 matrix,
		 *		elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left to right column order. Example:
		 *	@code
		 *	|0 1 2|
		 *	|  3 4|
		 *	|    5|
		 *	@endcode
		 */
		inline TEdgeP2SC3D(size_t n_node_0, size_t n_node_1,
			double f_delta_x1, double f_delta_y, double f_delta_x2, const double *p_upper_matrix_3x3)
			:m_n_node_0(n_node_0), m_n_node_1(n_node_1)
		{
			m_v_delta << f_delta_x1, f_delta_y, f_delta_x2;
			// no constructor for 2-valued vector

			m_t_inv_sigma <<
							p_upper_matrix_3x3[0], p_upper_matrix_3x3[1], p_upper_matrix_3x3[2],
							p_upper_matrix_3x3[1], p_upper_matrix_3x3[3], p_upper_matrix_3x3[4],
							p_upper_matrix_3x3[2], p_upper_matrix_3x3[4], p_upper_matrix_3x3[5];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

//	/**
//	 *	@brief Camera-to-Camera Epipolar constraint edge (C2CE)
//	 *
//	 *	This edge represents epipolar constraint
//	 *	between two 2D images \f$p0,p1\f$ of a 3D point:
//	 *	\f{equation}{ p1^T * F * p0 = 0 \f}
//	 *	where F is a fundamental matrix: \f$F = K2 * E * K1 \f$, \f$E = [t]_x * R \f$,
//	 *	\f$K\f$ is as calibration matrix, and \f$[R|t]\f$
//	 *	is a relative translation between cameras.
//	 */
//	struct TEdgeC2CE : public CParseEntity {
//		size_t m_n_node_0; /**< @brief (zero-based) index of the camera 0 */
//		size_t m_n_node_1; /**< @brief (zero-based) index of the camera 1 */
//		Eigen::Matrix<double, 4, 1> m_v_delta; /**< @brief delta measurement (also called "z") - 2d corresponding points */
//		Eigen::Matrix<double, 4, 4> m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */
//
//		/**
//		 *	@brief default constructor
//		 *
//		 *	@param[in] n_node_0 is (zero-based) index of the camera 0
//		 *	@param[in] n_node_1 is (zero-based) index of the camera 1
//		 *	@param[in] f_delta_x0 is delta x position of a corresponding point in camera 0
//		 *	@param[in] f_delta_y0 is delta y position of a corresponding point in camera 0
//		 *	@param[in] f_delta_x1 is delta x position of a corresponding point in camera 1
//		 *	@param[in] f_delta_y1 is delta y position of a corresponding point in camera 1
//		 *	@param[in] p_upper_matrix_4x4 is row-major upper triangular and diagonal 4x4 matrix,
//		 *		elements are not square roots
//		 *
//		 *	The matrix is stored row by row from top to bottom,
//		 *	with left to right column order. Example:
//		 *	@code
//		 *	|0 1 2 3|
//		 *	|  4 5 6|
//		 *	|    7 8|
//		 *	|      9|
//		 *	@endcode
//		 */
//		inline TEdgeC2CE(size_t n_node_0, size_t n_node_1,
//			double f_delta_x0, double f_delta_y0, double f_delta_x1, double f_delta_y1, const double *p_upper_matrix_4x4)
//			:m_n_node_0(n_node_0), m_n_node_1(n_node_1)
//		{
//			m_v_delta << f_delta_x0, f_delta_y0, f_delta_x1, f_delta_y1;
//			// no constructor for 2-valued vector
//
//			m_t_inv_sigma <<
//					p_upper_matrix_4x4[0], p_upper_matrix_4x4[1], p_upper_matrix_4x4[2], p_upper_matrix_4x4[3],
//					p_upper_matrix_4x4[1], p_upper_matrix_4x4[4], p_upper_matrix_4x4[5], p_upper_matrix_4x4[6],
//					p_upper_matrix_4x4[2], p_upper_matrix_4x4[3], p_upper_matrix_4x4[7], p_upper_matrix_4x4[8],
//					p_upper_matrix_4x4[3], p_upper_matrix_4x4[6], p_upper_matrix_4x4[8], p_upper_matrix_4x4[9];
//			// fill the matrix
//		}
//
//		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//	};

	/**
	 *	@brief ground truth vertex measurement class ("ROCV:RECEIVER_GT" in the datafile)
	 */
	struct TVertex3D_Reference : public CParserBase::TVertex3D {
		/**
		 *	@copydoc CParserBase::TVertex3D::TVertex3D()
		 */
		TVertex3D_Reference(int n_node_id, double f_x, double f_y, double f_z, double f_ax, double f_ay, double f_az)
			:CParserBase::TVertex3D(n_node_id, f_x, f_y, f_z, f_ax, f_ay, f_az)
		{}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief range-only constant velocity parsed range edge
	 */
	struct TROCV_RangeEdge : public CParserBase::CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the 3D point */
		size_t m_n_node_1; /**< @brief (zero-based) index of the landmark */
		Eigen::Matrix<double, 1, 1> m_v_delta; /**< @brief measurement */
		Eigen::Matrix<double, 1, 1> m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node0 is (zero-based) index of the 3D point
		 *	@param[in] n_node1 is (zero-based) index of the landmark
		 *	@param[in] f_range is range measurement
		 *	@param[in] f_inv_sigma is 1x1 matrix, containing the inverse sigma matrix, elements are not square roots
		 */
		TROCV_RangeEdge(size_t n_node0, size_t n_node1, double f_range, double f_inv_sigma)
			:m_n_node_0(n_node0), m_n_node_1(n_node1)
		{
			m_v_delta << f_range;
			m_t_inv_sigma << f_inv_sigma;
			// initialize the matrices
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief range-only constant velocity parsed delta-time edge
	 */
	struct TROCV_DeltaTimeEdge : public CParserBase::CParseEntity {
		size_t m_n_node_0; /**< @brief (zero-based) index of the first 3D point */
		size_t m_n_node_1; /**< @brief (zero-based) index of the second 3D point */
		Eigen::Matrix<double, 1, 1> m_v_delta; /**< @brief measurement */
		Eigen::Matrix<double, 6, 6> m_t_inv_sigma; /**< @brief inverse sigma matrix, elements are not square roots */

		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_node0 is (zero-based) index of the first 3D point
		 *	@param[in] n_node1 is (zero-based) index of the second 3D point
		 *	@param[in] f_delta_time is delta-time measurement
		 *	@param[in] p_upper_matrix_6x6 is row-major upper triangular and diagonal 6x6 matrix
		 *		containing the inverse sigma matrix, elements are not square roots
		 *
		 *	The matrix is stored row by row from top to bottom,
		 *	with left to right column order. Example:
		 *	@code
		 *	|0 1  2  3  4  5|
		 *	|  6  7  8  9 10|
		 *	|    11 12 13 14|
		 *	|       15 16 17|
		 *	|          18 19|
		 *	|             20|
		 *	@endcode
		 */
		TROCV_DeltaTimeEdge(size_t n_node0, size_t n_node1, double f_delta_time, const double *p_upper_matrix_6x6)
			:m_n_node_0(n_node0), m_n_node_1(n_node1)
		{
			m_v_delta(0) = f_delta_time;

			const double *p_u = p_upper_matrix_6x6;
			m_t_inv_sigma <<
				p_u[0],  p_u[1],  p_u[2],  p_u[3],  p_u[4],  p_u[5],
				p_u[1],  p_u[6],  p_u[7],  p_u[8],  p_u[9], p_u[10],
				p_u[2],  p_u[7], p_u[11], p_u[12], p_u[13], p_u[14],
				p_u[3],  p_u[8], p_u[12], p_u[15], p_u[16], p_u[17],
				p_u[4],  p_u[9], p_u[13], p_u[16], p_u[18], p_u[19],
				p_u[5], p_u[10], p_u[14], p_u[17], p_u[19], p_u[20];
			// fill the matrix
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	/**
	 *	@brief a simple callback class, to be used by the parser
	 *	@note This has the disadvantage of having to be modified after adding a new
	 *		parsed types, and having to rewrite all implementations of this. This is
	 *		now only used by CDatasetPeeker.
	 */
	class CParserAdaptor {
	public:
		/**
		 *	@brief appends the system with an odometry measurement
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TEdge2D &r_t_edge) = 0;

		/**
		 *	@brief appends the system with a landmark measurement
		 *	@param[in] r_t_landmark is the measurement to be appended
		 */
		virtual void AppendSystem(const TLandmark2D_XY &r_t_landmark) = 0;

		/**
		 *	@brief appends the system with a landmark measurement
		 *	@param[in] r_t_landmark is the measurement to be appended
		 */
		virtual void AppendSystem(const TLandmark2D_RB &r_t_landmark) = 0;

		/**
		 *	@brief appends the system with vertex position
		 *	@param[in] r_t_vertex is the vertex to be appended
		 *	@note The vertices can be ignored in most of the solvers.
		 */
		virtual void InitializeVertex(const TVertex2D &r_t_vertex) = 0;

		/**
		 *	@brief appends the system with an odometry measurement
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TEdge3D &r_t_edge) = 0;

		/**
		 *	@brief appends the system with an 3d landmark measurement
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TLandmark3D_XYZ &r_t_edge) = 0;

		/**
		 *	@brief appends the system with vertex position
		 *	@param[in] r_t_vertex is the vertex to be appended
		 *	@note The vertices can be ignored in most of the solvers.
		 */
		virtual void InitializeVertex(const TVertex3D &r_t_vertex) = 0;

		/**
		 *	@brief appends the system with vertex position
		 *	@param[in] r_t_vertex is the vertex to be appended
		 *	@note The vertices can be ignored in most of the solvers.
		 */
		virtual void InitializeVertex(const TVertexXYZ &r_t_vertex) = 0;

		/**
		 *	@brief appends the system with camera vertex position and parameters
		 *	@param[in] r_t_vertex is the vertex to be appended
		 */
		virtual void InitializeVertex(const TVertexCam3D &r_t_vertex) = 0;

		/**
		 *	@brief appends the system with intrinsics vertex
		 *	@param[in] r_t_vertex is the vertex to be appended
		 */
		virtual void InitializeVertex(const TVertexIntrinsics &r_t_vertex) = 0;

		/**
		 *	@brief appends the system with camera vertex position and parameters
		 *	@param[in] r_t_vertex is the vertex to be appended
		 */
		virtual void InitializeVertex(const TVertexSpheron &r_t_vertex) = 0;

		/**
		 *	@brief appends the system with camera vertex position and parameters
		 *	@param[in] r_t_vertex is the vertex to be appended
		 */
		virtual void InitializeVertex(const TVertexSCam3D &r_t_vertex) = 0;

		/**
		 *	@brief appends the system with an camera measurement
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TEdgeP2C3D &r_t_edge) = 0;

		/**
		 *	@brief appends the system with an camera+intrinsics measurement
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TEdgeP2CI3D &r_t_edge) = 0;

//		/**
//		 *	@brief appends the system with an camera+intrinsics measurement
//		 *	@param[in] r_t_edge is the measurement to be appended
//		 */
//		virtual void AppendSystem(const TEdgeC2CE &r_t_edge) = 0;

		/**
		 *	@brief appends the system with an camera measurement
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TEdgeSpheronXYZ &r_t_edge) = 0;

		/**
		 *	@brief appends the system with an camera measurement
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TEdgeP2SC3D &r_t_edge) = 0;

		/**
		 *	@brief appends the system with a reference 3D pose
		 *	@param[in] r_t_vertex is the vertex to be appended
		 *	@note The vertices can be ignored in most of the solvers.
		 */
		virtual void InitializeVertex(const TVertex3D_Reference &r_t_vertex) = 0;

		/**
		 *	@brief appends the system with a delta-time edge
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TROCV_DeltaTimeEdge &r_t_edge) = 0;

		/**
		 *	@brief appends the system with a range measurement edge
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TROCV_RangeEdge &r_t_edge) = 0;

		/**
		 *	@brief appends the system with a range measurement edge
		 *	@param[in] r_t_edge is the measurement to be appended
		 */
		virtual void AppendSystem(const TUnaryFactor3D &r_t_edge) = 0;
	};

protected:
	/**
	 *	@brief a simple function object which adds token names to a lookup map
	 *	@note This is actually used by CParserTemplate, but it is not dependent on its parameters.
	 */
	class CAddToMap {
	protected:
		int m_n_index; /**< @brief index of the current parse primitive */
		std::map<std::string, int> &m_r_token_map; /**< @brief reference to the token map */

	public:
		/**
		 *	@brief default constructor
		 *	@param[in] r_token_map is a reference to the token map (output)
		 */
		inline CAddToMap(std::map<std::string, int> &r_token_map)
			:m_n_index(0), m_r_token_map(r_token_map)
		{}

		/**
		 *	@brief function operator
		 *	@tparam CParsePrimitive is a parse primitive type to be added to the map
		 *	@note This function throws std::bad_alloc.
		 */
		template <class CParsePrimitive>
		inline void operator ()() // throw(std::bad_alloc)
		{
			CParsePrimitive::EnumerateTokens(m_r_token_map, m_n_index);
			++ m_n_index; // the original version had this one at compile time; a bit more elegant
			// add all tokens for this primitive type
		}
	};

	static std::map<std::string, std::pair<std::set<std::string>,
		std::pair<bool, bool> > > m_per_file_warned_token_set; /**< @brief contains persistent warned tokens per file, in order to not warn twice (once per dataset peeker, once per main loop) */

public:
	/**
	 *	@brief removes whitespace from the beginning and from the end of the string
	 *	@param[in,out] r_s_string is the string to remove whitespace from
	 */
	static void TrimSpace(std::string &r_s_string);

	/**
	 *	@brief reads line form a file
	 *
	 *	@param[out] r_s_line is output string, containing one line read from a file
	 *	@param[in] p_fr is pointer to a file
	 *
	 *	@return Returns true on success, false on failure (not enough memory / input error).
	 *
	 *	@note In case file is at it's end, output lines are empty, but the function still succeeds.
	 *	@note Output lines may contain carriage-return character(s), for example if the file
	 *		is opened for binary reading. Line-feed character marks end of line and is never
	 *		included.
	 */
	static bool ReadLine(std::string &r_s_line, FILE *p_fr);

	/**
	 *	@brief reads line form a file
	 *
	 *	@param[out] r_s_line is output string, containing one line read from a file
	 *	@param[in] p_fr is pointer to a file
	 *
	 *	@return Returns true on success, false on failure (not enough memory / input error).
	 *
	 *	@note In case file is at it's end, output lines are empty, but the function still succeeds.
	 *	@note Output lines may contain carriage-return character(s), for example if the file
	 *		is opened for binary reading. Line-feed character marks end of line and is never
	 *		included.
	 */
	static bool ReadField(std::string &r_s_line, FILE *p_fr);

	/**
	 *	@brief splits a string by a separator
	 *
	 *	@param[out] r_s_dest is destination vector for substrings
	 *	@param[in] r_s_string is the input string
	 *	@param[in] p_s_separator is the separator
	 *	@param[in] n_thresh is minimal output string threshold
	 *		(only strings longer than threshold are stored in r_s_dest)
	 *
	 *	@return Returns true on success, false on failure (not enough memory).
	 */
	static bool Split(std::vector<std::string> &r_s_dest, const std::string &r_s_string,
		const char *p_s_separator, int n_thresh);
};

/**
 *	@brief template of a parser implementation with a list of parsed primitives
 *
 *	@tparam CParseLoopType is a type of parse loop (the sink); it can be CParserBase::CParserAdaptor,
 *		a specialization of CParseLoop or something completely different
 *	@tparam CParsePrimitiveTypelist is a list of parse primitive handlers
 */
template <class CParseLoopType, class CParsePrimitiveTypelist>
class CParserTemplate : public CParserBase {
protected:
	typedef CParseLoopType _TyParseLoop; /**< @brief parse loop type */
	typedef typename CUniqueTypelist<CParsePrimitiveTypelist>::_TyResult
		_TyParsePrimitiveTypelist; /**< @brief a list of parse primitive handlers */

	/**
	 *	@brief a simple function object that passes a line of text to
	 *		a parse primitive and dispatches the parsed item to parse loop
	 */
	class CPrimitiveFire {
	protected:
		size_t m_n_line_no; /**< @brief current line number (zero-based, for error reporting) */
		const std::string &m_r_s_line; /**< @brief the current line without the token */
		const std::string &m_r_s_token; /**< @brief the name of the token */
		_TyParseLoop &m_r_parse_loop; /**< @brief parse loop to pass the parsed item to */
		bool m_b_result; /**< @brief the result of the operation */

	public:
		/**
		 *	@brief default constructor; just copies the arguments
		 *
		 *	@param[in] n_line_no is current line number (zero-based, for error reporting)
		 *	@param[in] r_s_line is the current line without the token
		 *	@param[in] r_s_token is the name of the token
		 *	@param[in,out] r_parse_loop is the parse loop to pass the parsed item to
		 */
		inline CPrimitiveFire(size_t n_line_no, const std::string &r_s_line,
			const std::string &r_s_token, _TyParseLoop &r_parse_loop)
			:m_n_line_no(n_line_no), m_r_s_line(r_s_line),
			m_r_s_token(r_s_token), m_r_parse_loop(r_parse_loop), m_b_result(false)
		{}

		/**
		 *	@brief performs the parsing using the selected parse primitive
		 *	@tparam CParsePrimitive is the selected parse primitive for the current token
		 */
		template <class CParsePrimitive>
		inline void operator ()()
		{
			m_b_result = CParsePrimitive::Parse_and_Dispatch(m_n_line_no,
				m_r_s_line, m_r_s_token, m_r_parse_loop);
			// parse this using the primitive
		}

		/**
		 *	@brief gets the result of the parsing
		 *	@return Returns true on success, false on failure.
		 */
		operator bool() const
		{
			return m_b_result;
		}
	};

public:
	/**
	 *	@brief parses a .graph file, emitting parsed entities in the process
	 *
	 *	@param[in] p_s_filename is a null-terminated string containing input file name
	 *	@param[in] r_callback is a reference to the callback object that will receive parsed entities
	 *	@param[in] n_line_limit is the limit of the number of processed lines (use e.g. 1500 for debugging)
	 *	@param[in] b_ignore_case is case sensitivity flag; if set, token case is ignored
	 *	@param[in] b_comments is comments handling flag; if set, comments are started using '#' or '%'
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Parse(const char *p_s_filename, CParseLoopType &r_callback,
		size_t n_line_limit = 0, bool b_ignore_case = false, bool b_comments = true)
	{
		FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fr, p_s_filename, "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fr = fopen(p_s_filename, "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return false;
		// open the input file

		try {
			std::map<std::string, int> token_name_map;
			CTypelistForEach<_TyParsePrimitiveTypelist, CAddToMap>::Run(CAddToMap(token_name_map));
			// fill the token map, based on the parsed primitives

			bool b_new_file = m_per_file_warned_token_set.find(std::string(p_s_filename)) ==
				m_per_file_warned_token_set.end();
			bool &r_b_warned_about_duplicates =
				m_per_file_warned_token_set[std::string(p_s_filename)].second.first;
			bool &r_b_warned_about_reverse_order =
				m_per_file_warned_token_set[std::string(p_s_filename)].second.second;
			if(b_new_file) {
				r_b_warned_about_duplicates = false;
				r_b_warned_about_reverse_order = false;
			}
			std::set<std::string> &r_bad_token_name_set =
				m_per_file_warned_token_set[std::string(p_s_filename)].first;
			// set of unknown token names (remember them for application runtime)

			std::string s_line, s_token;
			for(size_t n_line_no = 0; !feof(p_fr) && (!n_line_limit || n_line_no < n_line_limit); ++ n_line_no) {
				if(!ReadLine(s_line, p_fr)) {
					fclose(p_fr);
					return false;
				}
				if(b_comments) {
					size_t n_comment_pos;
					if((n_comment_pos = s_line.find_first_of("%#")) != std::string::npos)
						s_line.erase(n_comment_pos);
					// handle line comments
				}
				TrimSpace(s_line);
				if(s_line.empty())
					continue;
				// read the file line by line, skip the empty lines

				s_token.erase();
				size_t n_pos;
				if((n_pos = s_line.find_first_of(" \t")) != std::string::npos) {
					s_token.insert(s_token.begin(), s_line.begin(), s_line.begin() + n_pos);
					s_line.erase(0, n_pos + 1);
					TrimSpace(s_line); // ...
				} else {
					s_token.insert(s_token.begin(), s_line.begin(), s_line.end());
					s_line.erase();
				}
				// read the token (separated by space; otherwise the entire line is a token)
				// and remove the token from the line

				if(b_ignore_case) {
					for(std::string::iterator b = s_token.begin(), e = s_token.end(); b != e; ++ b)
						*b = toupper(*b);
					//std::transform(s_token.begin(), s_token.end(), s_token.begin(), toupper); // g++4.8 contains overloads of toupper and is not able to specialize this due to ambiguity
				}
				// map is case sensitive, if there is dataset with lowercase tokens, it needs to be fixed

				std::map<std::string, int>::const_iterator p_tok_it;
				if((p_tok_it = token_name_map.find(s_token)) == token_name_map.end()) {
					if(r_bad_token_name_set.find(s_token) == r_bad_token_name_set.end()) {
						fprintf(stderr, "warning: unknown token: \'%s\' (ignored)\n", s_token.c_str());
						r_bad_token_name_set.insert(s_token);
					}
					// t_odo - add set of unknown token names so each warning is only printed once

					continue;
				}
				// identify the token, skip unknown tokens

				size_t n_token_type = (*p_tok_it).second;
				CPrimitiveFire dispatcher(n_line_no, s_line, s_token, r_callback);
				if(!CTypelistItemSelect<_TyParsePrimitiveTypelist,
				   CPrimitiveFire>::Select(n_token_type, dispatcher)) {
					fprintf(stderr, "warning: parser failed in parse primitive "
						PRIsize " on line " PRIsize "\n", n_token_type, n_line_no + 1); // line numbers are zero-based
					fclose(p_fr);
					return false;
				}
				// pass the token to the appropriate implementation, parse it and dispatch it to the callback
			}
		} catch(std::bad_alloc&) {
			fclose(p_fr);
			fprintf(stderr, "warning: caught std::bad_alloc in the parse loop\n");
			return false;
		}

		return true;
	}
};

/** @} */ // end of group

#endif // !__GRAPH_PARSER_INCLUDED
