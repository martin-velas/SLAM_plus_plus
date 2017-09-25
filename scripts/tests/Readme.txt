This is for internal unit testing and possibly also benchmarking.

The make_unittest.sh is a simple script that records a reference
result. This is only supposed to be run once, using a highly stable
version of SLAM ++. It prints parts of the second script, which
verifies these results using the current version of SLAM ++ and
prints any differences.

unit_tests.sh [-v|--verbose|-q|--quiet] [-rt|--robust-timing]
	[-i|--data-path <path-to-SLAM_plus_plus/data>]

runs the unit tests with the specified options. The ground truth
part of the script is not supposed to be modified under normal
conditions.

In case the data path is not specified, the script tries to read
the SLAMPP_DATA environment variable and if that is not set, falls
back to the default "../data" (which works, assuming this is being
run from build).

----

The test_eigen33.sh is another simple benchmark, testing performance
of the BA solver under different build configurations of Eigen 3.3.
It compares the effects of AVX and unaligned object vectorization.
It builds all of SLAM++ several times. It is supposed to be run from
the root (where src and include directories are).

----

build_tests.sh is yet another test. It builds the whole set of SLAM++
examples and apps, using several different CMake configurations. It
also tests that each configuration produces a working binaries. This
takes about five hours on a decent computer. It is also supposed to
be run from the root (where src and include directories are).
