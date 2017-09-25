This file briefly describes format and sources of 2D dataset files for SLAM++.

There are several conventions for the 2D graph SLAM datasets, each with its own
shortcomings, some of them requiring additional processing for incremental
scenarios. In order to run our code with several already existing datasets we
use a unified graph SLAM dataset format.

---------------------------------------------------
- SLAM++ 2D format specifications
---------------------------------------------------

vertices:
    VERTEX_SE2/VERTEX2 <node_id> <X> <Y> <Theta>

edges:
    EDGE_SE2/EDGE2/ODOMETRY <node_id_from> <node_id_to> <X> <Y> <Theta> <XX> <XY> <XT> <YY> <YT> <TT>
    where node_id_from < node_id_to

Variable initialization entries are supported and marked by tag VERTEX_SE2 or VERTEX2.
Edges are marked by tags EDGE_SE2, EDGE2 or ODOMETRY.
Angles are in radians.
The information matrix associated to each edge is NOT in squared form. The
information is in upper-triangular form (xx xy xy yy yt yy). The matrix is
stored row by row from top to bottom, with left to right column order. Example:

|0 1 2|
|  3 4|
|    5|

Note that the 3D format datasets are described in the documentation.

---------------------------------------------------
- SLAM++ Incremental BA format specifications
---------------------------------------------------

The Bundle Adjustment datasets are usually unordered, e.g. g2o's Venice871 dataset
is just 871 camera vertices followed by a bunch of landmark vertices followed by
a bunch of observations. It is not easy for the incremental solver to process such
a file without reading all of it first and performing some graph analysis.

We have implemented a bash script which parses a BA dataset, constructs a spanning
tree and then orders the vertices as if captured by a SfM system, ready for incremental
processing. To make the life of the optimizer easier, it also inserts markers where
the system is in consistent state and can be optimized (there are no missing / orphaned
vertices and all the observations are in place). The said script can be found at:

https://sourceforge.net/p/slam-plus-plus/code/HEAD/tree/trunk/scripts/incremental_BA/

The format is derived from the same one as that of g2o, i.e.:

VERTEX_CAM <id> <x> <y> <z> <qx> <qy> <qz> <qw> <fx> <fy> <cx> <cy> <d>
VERTEX_XYZ <id> <x> <y> <z>
EDGE_PROJECT_P2MC <pt-id> <cam-id> <ox> <oy> <XX> <XY> <YY>
CONSISTENCY_MARKER

where <x>, <y> and <z> are Euclidean coordinates of the point and camera
in a global coordinate frame, <qx>, <qy>, <qz> ans <qw> is a quaternion
specifying the rotation of the camera, <fx> and <fy> is the focal length,
<cx> and <cy> is the principal point position and <d> is the first radial
distortion parameter.

For the edge, <ox> and <oy> are the observed coordinates of the point and
<XX> <XY> <YY> specify the upper triangle of the covariance matrix.

The consistency marker identifies positions where one can optimize.

There are currently two incremental BA datasets, the Guildford cathedral
created from stills from the IMPART dataset (http://cvssp.org/impart/)
and the Venice dataset originally published in [6].

---------------------------------------------------
- Datasets
---------------------------------------------------

The following text contains information about sources of dataset files
available to download.

Guildford cathedral [7]
---------------------------------------------------
download: http://sourceforge.net/projects/slam-plus-plus/files/data/cathedral.zip/download

Guildford cathedral - incremental version [7]
---------------------------------------------------
download: http://sourceforge.net/projects/slam-plus-plus/files/data/cathedral_inc.zip/download

Venice - incremental version [6]
---------------------------------------------------
download: http://sourceforge.net/projects/slam-plus-plus/files/data/venice_inc.zip/download

intel [4]
---------------------------------------------------
download: http://sourceforge.net/projects/slam-plus-plus/files/data/intel.txt/download

manhattanOlson3500 [2]
---------------------------------------------------
modification: the information values have been squared
download: http://sourceforge.net/projects/slam-plus-plus/files/data/manhattanOlson3500.txt/download

10kHog-man  [1]
---------------------------------------------------
modification: order of information values has been changed
download: http://sourceforge.net/projects/slam-plus-plus/files/data/10kHog-man.txt/download

10k [1]
---------------------------------------------------
modification: order of information values has been changed
download: http://sourceforge.net/projects/slam-plus-plus/files/data/10k.txt/download

100k [1]
---------------------------------------------------
modification: order of information values has been changed
download: http://sourceforge.net/projects/slam-plus-plus/files/data/100k.txt/download

city10k [3]
---------------------------------------------------
modification: the information values have been squared
download: http://sourceforge.net/projects/slam-plus-plus/files/data/city10k.txt/download

cityTrees10k [3]
---------------------------------------------------
modification: the information values have been squared
download: http://sourceforge.net/projects/slam-plus-plus/files/data/cityTrees10k.txt/download

Victoria park [4]
---------------------------------------------------
modification: the information values have been squared
download: http://sourceforge.net/projects/slam-plus-plus/files/data/victoria-park.txt/download

Killian court [5]
---------------------------------------------------
modification: the information values have been squared and the order changed
download: http://sourceforge.net/projects/slam-plus-plus/files/data/killian-court.txt/download

sphere2500.txt [3]
---------------------------------------------------
modification: the information values have been converted from RPY to axis angle
download: http://sourceforge.net/projects/slam-plus-plus/files/data/sphere2500.txt/download

parking-garage.txt [6]
---------------------------------------------------
modification: the information values have been converted from RPY to axis angle
download: http://sourceforge.net/projects/slam-plus-plus/files/data/parking-garage.txt/download

[1] G. Grisetti, C. Stachniss, S. Grzonka, and W. Burgard, "A tree parameterization for efficiently computing maximum likelihood maps using gradient descent," in Robotics: Science and Systems (RSS), June 2007.
[2] E. Olson, "Robust and efficient robot mapping," Ph.D. dissertation, Massachusetts Institute of Technology, 2008.
[3] M. Kaess, A. Ranganathan, and F. Dellaert, "iSAM: Fast incremental smoothing and mapping with efficient data association," in IEEE Intl. Conf. on Robotics and Automation (ICRA), Rome, Italy, April 2007, pp. 1670-1677.
[4] A. Howard and N. Roy, "The robotics data set repository (Radish)," 2003. [Online]. Available: http://radish.sourceforge.net/
[5] M. Bosse, P. Newman, J. Leonard, and S. Teller, "Simultaneous localization and map building in large-scale cyclic environments using the Atlas framework," Intl. J. of Robotics Research, vol. 23, no. 12, pp. 1113-1139, Dec 2004.
[6] R. Kuemmerle, G. Grisetti, H. Strasdat, K. Konolige, and W. Burgard: "g2o: A General Framework for Graph Optimization", IEEE Intl. Conf. on Robotics and Automation (ICRA), 2011
[7] H. Kim and A. Hilton, "Influence of Colour and Feature Geometry on Multi-modal 3D Point Clouds Data Registration," Proc. 3DV, 2014.

---------------------------------------------------
- A note on the sphere2500 dataset. 
---------------------------------------------------

The sphere2500 used in SLAM++ evaluation is the iSAM sphere2500 dataset.
Being a popular shape in robotics, there is another sphere2500 dataset generated by g2o (https://github.com/RainerKuemmerle/g2o/tree/master/g2o/examples/sphere) which is sometimes called sphere2500a. It is more connected than the iSAM sphere and thus more difficult to solve.
There is however another sphere which was created by Giorgio Grisetti while implementing TORO in its 3D variant. It's called sphere_bignoise_vertex3.g2o and can be found inside the repository at http://www.openslam.org/g2o.html.

Special thanks to Rainer Kümmerle for his wisdom on spheres.
