
# Set required variables.
SET(SLAM_EXAMPLE_BINARY_NAME "ba_parameter_acra_gxyz")

FILE(GLOB_RECURSE SLAM_EXAMPLE_HEADERS ${SLAM_P_P_SOURCE_DIR}/include/ba_parameter_acra/*.h ${SLAM_P_P_SOURCE_DIR}/include/slam/*.h)

IF(SLAM_P_P_SHORT_CXSPARSE)
	ADD_DEFINITIONS(-D__CXSPARSE_SHORT)
ENDIF(SLAM_P_P_SHORT_CXSPARSE)

IF(SLAM_P_P_SHORT_CHOLMOD)
	ADD_DEFINITIONS(-D__CHOLMOD_SHORT)
ENDIF(SLAM_P_P_SHORT_CHOLMOD)

# support for GPU BLAS
IF(SLAM_P_P_GPU_BLAS)
	ADD_DEFINITIONS(
		-DGPU_BLAS
		-D__CHANNEL_DESCRIPTOR_H__
		-D__CUDA_RUNTIME_H__)
ENDIF(SLAM_P_P_GPU_BLAS)

ADD_DEFINITIONS(-DCERES_NO_PROTOCOL_BUFFERS)

SET(SLAM_EXAMPLE_SOURCES
  BAOptimizer.cpp)

#SET(FRONT_END_SOURCES
#  ARDriver.cpp
#  ARGui.cpp
#  Bundle.cpp
#  BundleAPI.cpp
#  FivePoint.cpp
#  GLWindow2.cpp
#  Graph_api.cpp
#  GraphViewer.cpp
#  ImageGrabber.cpp
#  ImageGraph.cpp
#  kmeans.cpp
#  Matcher.cpp
#  PatchFinder.cpp
#  slam.cpp
#  SyntDataParser.cpp
#  Tracker.cpp
#  TrackerData.cpp
#  Triangulation.cpp
#  Vertex.cpp
#  VocabTree.cpp
#  VocabTreeBuild.cpp
#  VocabTreeIO.cpp
#  VocabTreeUtil.cpp
#  vPoint.cpp
#  BAOptimizer.cpp
#)

#SET(FINAL_VIEWER_SOURCES GLWindow2.cpp)
  
#SET(FINAL_VIEWER_LIB cvd GVars3 GL GLU)
  
#SET(MY_LIB cvd GVars3 toontag lapack opencv_core opencv_features2d opencv_flann GL GLU gomp)

SET(FRONT_END_LIB
  #${MY_LIB}
  ${SLAM_EXAMPLE_LIBRARIES})

IF(SLAM_P_P_LINEAR_SOLVER_TYPE)
	ADD_DEFINITIONS("-D__LINEAR_SOLVER_OVERRIDE=${SLAM_P_P_LINEAR_SOLVER_TYPE}")
ENDIF(SLAM_P_P_LINEAR_SOLVER_TYPE)

ADD_DEFINITIONS("-DRUN_STANDALONE")

# Build and install SLAM_plus_plus executable.
ADD_EXECUTABLE(${SLAM_EXAMPLE_BINARY_NAME}
	${SLAM_EXAMPLE_SOURCES} MainL.cpp
	${SLAM_EXAMPLE_HEADERS})
set_target_properties(${SLAM_EXAMPLE_BINARY_NAME} PROPERTIES COMPILE_DEFINITIONS "LANDMARK_TYPE_XYZ;LANDMARKS_GLOBAL")
TARGET_LINK_LIBRARIES(${SLAM_EXAMPLE_BINARY_NAME}
	${SLAM_EXAMPLE_LIBRARIES})

INSTALL(TARGETS ${SLAM_EXAMPLE_BINARY_NAME}
	RUNTIME DESTINATION bin)

# build and install front_end
#ADD_EXECUTABLE(front_end Main.cpp ${FRONT_END_SOURCES})
#TARGET_LINK_LIBRARIES(front_end ${FRONT_END_LIB})
#INSTALL(TARGETS front_end RUNTIME DESTINATION bin)

#build and install fibal_viewer
#ADD_EXECUTABLE(final_viewer FinalViewer.cpp ${FINAL_VIEWER_SOURCES})
#TARGET_LINK_LIBRARIES(final_viewer ${FINAL_VIEWER_LIB})
#INSTALL(TARGETS final_viewer RUNTIME DESTINATION bin)

# note that these still rebuild MainL.cpp and Eigenvalues.cpp for every config, even though that is not required. could make a library out of it and link to that. lazy.

ADD_EXECUTABLE(       ba_parameter_acra_linvdepth ${SLAM_EXAMPLE_SOURCES} MainL.cpp)
set_target_properties(ba_parameter_acra_linvdepth PROPERTIES COMPILE_DEFINITIONS "LANDMARK_TYPE_InvDepth;LANDMARKS_LOCAL")
TARGET_LINK_LIBRARIES(ba_parameter_acra_linvdepth ${SLAM_EXAMPLE_LIBRARIES})
INSTALL(TARGETS       ba_parameter_acra_linvdepth RUNTIME DESTINATION bin)

ADD_EXECUTABLE(       ba_parameter_acra_linvdist ${SLAM_EXAMPLE_SOURCES} MainL.cpp)
set_target_properties(ba_parameter_acra_linvdist PROPERTIES COMPILE_DEFINITIONS "LANDMARK_TYPE_InvDist;LANDMARKS_LOCAL")
TARGET_LINK_LIBRARIES(ba_parameter_acra_linvdist ${SLAM_EXAMPLE_LIBRARIES})
INSTALL(TARGETS       ba_parameter_acra_linvdist RUNTIME DESTINATION bin)

ADD_EXECUTABLE(       ba_parameter_acra_lxyz ${SLAM_EXAMPLE_SOURCES} MainL.cpp)
set_target_properties(ba_parameter_acra_lxyz PROPERTIES COMPILE_DEFINITIONS "LANDMARK_TYPE_XYZ;LANDMARKS_LOCAL")
TARGET_LINK_LIBRARIES(ba_parameter_acra_lxyz ${SLAM_EXAMPLE_LIBRARIES})
INSTALL(TARGETS       ba_parameter_acra_lxyz RUNTIME DESTINATION bin)

ADD_EXECUTABLE(       ba_parameter_acra_ginvdepth ${SLAM_EXAMPLE_SOURCES} MainL.cpp)
set_target_properties(ba_parameter_acra_ginvdepth PROPERTIES COMPILE_DEFINITIONS "LANDMARK_TYPE_InvDepth;LANDMARKS_GLOBAL")
TARGET_LINK_LIBRARIES(ba_parameter_acra_ginvdepth ${SLAM_EXAMPLE_LIBRARIES})
INSTALL(TARGETS       ba_parameter_acra_ginvdepth RUNTIME DESTINATION bin)

ADD_EXECUTABLE(       ba_parameter_acra_ginvdist ${SLAM_EXAMPLE_SOURCES} MainL.cpp)
set_target_properties(ba_parameter_acra_ginvdist PROPERTIES COMPILE_DEFINITIONS "LANDMARK_TYPE_InvDist;LANDMARKS_GLOBAL")
TARGET_LINK_LIBRARIES(ba_parameter_acra_ginvdist ${SLAM_EXAMPLE_LIBRARIES})
INSTALL(TARGETS       ba_parameter_acra_ginvdist RUNTIME DESTINATION bin)

#ADD_EXECUTABLE(       ba_parameter_acra_gxyz ${SLAM_EXAMPLE_SOURCES} MainL.cpp)
#set_target_properties(ba_parameter_acra_gxyz PROPERTIES COMPILE_DEFINITIONS "LANDMARK_TYPE_XYZ;LANDMARKS_GLOBAL")
#TARGET_LINK_LIBRARIES(ba_parameter_acra_gxyz ${SLAM_EXAMPLE_LIBRARIES})
#INSTALL(TARGETS       ba_parameter_acra_gxyz RUNTIME DESTINATION bin)
