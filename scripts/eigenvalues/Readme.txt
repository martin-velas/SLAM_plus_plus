Here are some windows scripts which were used to unit test the eigenvalues
module. Make a folder data/bench_eigs in the working directory and fill it
with paths to matrices from University of Florida Sparse Matrix Collection
to be used it the test. Then apply the scripts in the following order:

list.bat

Makes a list of .mtx files in the current working directory.

get_eigs_gt.m

Takes matrix.mtx as input and writes eigvecs.mtx and eigvals.mtx, a matrix
of eigenvectors and a vector of eigenvalues, despite dense both are stored
in the sparse matrix market format which SLAM++ can read.

get_eigs_gt.bat

Iterates through all the .mtx files in the current working directory, runs
get_eigs_gt.m for each and puts the outputs in the eigs subdirectory. Once
the outputs are moved from eigs back to the parent folder, the eigensolver
unit tests can be run using

void Eigenvalues_UnitTest();

which is declated in Eigenvalues.cpp.
