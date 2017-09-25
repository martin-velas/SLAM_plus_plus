=== Building ===

This library was tested on Windows (using MSVC 2008 and above),
Mac, Linux and BSD. There was an optimizer issue in g++ 4.2 or earlier
which causes the code to crash at runtime.

You can download thestable sources in a zip file from SourceForge,
or checkout the newest sources from the svn:

$ mkdir slam
$ cd slam
$ svn checkout svn://svn.code.sf.net/p/slam-plus-plus/code/trunk .

There is a CMakeFile. To be able to change the code and commit back
to the svn, do an out-of-source build, like this:

$ cd build
$ cmake ..

This will configure the project without any configuration. To change
configuration, run cmake -i .. instead of the last line above (or at
a later time, should a change in the configuration be needed). One
interesting option is to specify the default linear solver. Supernodal
CHOLMOD or block Cholesky are the fastest, CSparse is slightly slower
and simplical CHOLMOD is the slowest. Another option is to enable GPU
acceleration support, which currently applies to the Schur complement
solver only (specify -us when running SLAM ++; requires CUDA and CULA
toolkits).

On Mac, you might want to configure your C and C++ compilers to be the
GNU ones (e.g. from the MacPorts project) rather than Clang which does
not support OpenMP nowadays and your code will be somewhat slower. You
can do that by using:

$ cmake -D CMAKE_C_COMPILER=/opt/local/bin/gcc-mp-4.7 \
$	-D CMAKE_CXX_COMPILER=/opt/local/bin/g++-mp-4.7 ..

Where you might want to change the version to the latest one that you
have installed. But it will build and run correctly even without that.

$ make 

Will finish the building process. You should now be able to run by typing:

$ ../bin/slam_plus_plus --help 

You can also use fast parallel build, like this:

$ make -j

And that should take care of the build.

In case you are working on Windows, there is a pre-configured workspace inside
the build15 folder in the source code distribution. In the previous CMake
versions, there were varius issues with Windows. Since CMake 3.x, you can easily
generate Visual Studio workspace using cmake-gui, that will work just as well.
It only does not have project grouping, which is a very minor cosmetic issue.
We tested this with Visual Studio 2008, 2012, 2013, 2015 and 2017.

There is an internal compiler error in Visual Studio 2012 if Eigen complex BLAS
is enabled (added an extra CMake option to enable it, as it is usually not needed).

Some Ubuntu distributions have linking problem which yields errors such as:

$ Timer.cpp:(.text+0x18): undefined reference to `clock_gettime'

This is solved by adding the "-Wl,--no-as-needed" (written together,
exactly like that) in the EXE_LINKER_FLAGS field in CMake (the "-lrt"
option is already there by default and adding it won't change anything).

If you have trouble building or during your development, you can visit
https://sourceforge.net/p/slam-plus-plus/wiki/Bug%20Atlas to see descriptions
of the commonly observed errors and ways to fix them.

=== Data ===

Data can be downloaded from SourceForge, at:
http://sourceforge.net/projects/slam-plus-plus/files/data/

=== References (please cite these if using this software) ===

V Ila, L Polok, M Solony and K Istenic, "Fast Incremental Bundle
Adjustment with Covariance Recovery", proceedings of the International
Conference on 3D Vision (3DV). Qingdao, China, 2017.

L Polok and P Smrž, "Pivoting Strategy for Fast LU decomposition
of Sparse Block Matrices", proceedings of the 25th High Performance
Computing Symposium. Virginia Beach, USA, 2017.

V Ila, L Polok, M Šolony and P Svoboda, "SLAM++. A Highly Efficient
and Temporally Scalable Incremental SLAM Framework", The International
Journal of Robotics Research, Online First, 2017, ISSN 1741-3176,
DOI 10.1177/0278364917691110.

L Polok, V Ila and P Smrž, "3D Reconstruction Quality Analysis and
Its Acceleration on GPU Clusters," in proceedings of European Signal
Processing Conference 2016. Budapest, Hungary, 2016.

L Polok, V Lui, V Ila, T Drummond, R Mahony, "The Effect of Different
Parameterisations in Incremental Structure from Motion," proceedings
of the Australian Conference on Robotics and Automation. Australia, 2015.

L Polok and P Smrz, "Increasing Double Precision Throughput on NVIDIA
Maxwell GPUs," proceedings of the 24th High Performance Computing
Symposium. Pasadena / Los Angeles, USA, 2016.

S Pabst, H Kim, L Polok, V Ila, T Waine, A Hilton, J Clifford
and P Smrz, "Jigsaw - Multi-Modal Big Data Management in Digital Film
Production," SIGGRAPH (poster). Los Angeles, USA, 2015.

L Polok, S Pabst, J Clifford, "A GPU-Accelerated Bundle Adjustment
Solver," GPU Technology Conference 2015. San Diego, USA, 2015.

M Solony, E Imre, V Ila, L Polok, H Kim and P Zemcik, "Fast and
Accurate Refinement Method for 3D Reconstruction from Stereo Spherical
Images," proceedings of the 10th International Conference on Computer
Vision Theory and Applications. Berlin: Institute of Electrical and
Electronics Engineers. Berlin, Germany, 2015.

V Ila, L Polok, M Solony, P Zemcik, P Smrz, "Fast covariance recovery
in incremental nonlinear least square solvers," proceedings of IEEE
International Conference on Robotics and Automation (ICRA). Seatle,
USA, 2015.

L Polok, V Ila, P Smrz, "Fast Sparse Matrix Multiplication on GPU,"
23rd High Performance Computing Symposium. Alexandria, USA, 2015

L Polok, V Ila, P Smrz, "Fast Radix Sort for Sparse Linear Algebra
on GPU," 22nd High Performance Computing Symposium. Tampa, USA, 2014

L Polok, V Ila, M Solony, P Smrz and P Zemcik, "Incremental Block
Cholesky Factorization for Nonlinear Least Squares in Robotics," in
Proceedings of Robotics: Science and Systems 2013. MIT Press, 2013

L Polok, M Solony, V Ila, P Smrz and P Zemcik, "Incremental Cholesky
Factorization for Least Squares Problems in Robotics," in Proceedings
of The 2013 IFAC Intelligent Autonomous Vehicles Symposium (IFAC). 2013.

L Polok, M Solony, V Ila, P Zemcik, and P Smrz, "Efficient
implementation for block matrix operations for nonlinear least squares
problems in robotic applications," in Proceedings of the IEEE
International Conference on Robotics and Automation. IEEE, 2013.

L Polok, V Ila and P Smrz, "Cache Efficient Implementation for Block
Matrix Operations," in Proceedings of the 21st High Performance
Computing Symposium. ACM, 2013.

=== Contact us ===

If a need arises to contact the authors of this implementation, use:

viorela.ila 'at' anu.edu.au or lukas 'at' lukas-polok.cz or
isolony 'at' fit.vutbr.cz
