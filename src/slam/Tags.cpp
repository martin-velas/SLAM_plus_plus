/*
								+----------------------------------+
								|                                  |
								|   ***  Constants and tags  ***   |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2015  |
								|                                  |
								|             Tags.cpp             |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file src/slam/Tags.cpp
 *	@brief static constants and tags
 *	@author -tHE SWINe-
 *	@date 2015-08-27
 */

#include "slam/IncrementalPolicy.h"

namespace solve {

const linear_tag linear = {};
const nonlinear_tag nonlinear = {};

} // ~solve
// tags for incremental solving policies

#include "slam/Sim3SolverBase.h"
#include "slam/3DSolverBase.h"
#include "slam/ConfigSolvers.h" // not sure if this will lead to problems later if someone wants to disable support of some type of solver
#include "slam/BaseTypes.h"

const CBaseEdge::null_initialize_vertices_tag CBaseEdge::null_initialize_vertices = {};
const CBaseEdge::explicitly_initialized_vertices_tag CBaseEdge::explicitly_initialized_vertices = {};
// tags for vertex initialization in the edges

const C3DJacobians::TSE3::from_se3_vector_tag C3DJacobians::TSE3::from_se3_vector = {};
const C3DJacobians::TSE3::from_tR_vector_tag C3DJacobians::TSE3::from_tR_vector = {};
// tags for SE(3) pose initialization disambiguation

const CSim3Jacobians::TSim3::from_sim3_vector_tag CSim3Jacobians::TSim3::from_sim3_vector = {};
const CSim3Jacobians::TSim3::from_tRs_vector_tag CSim3Jacobians::TSim3::from_tRs_vector = {};
// tags for Sim(3) pose initialization disambiguation

#include "slam/BlockMatrixVBR.h"

const TBLAS_SparseBlockMatrix::TFromTransposeMatrix_Token TBLAS_SparseBlockMatrix::from_transpose_matrix = {};
// tags for TBLAS_SparseBlockMatrix constructor with CUberBlockMatrix
