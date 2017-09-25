This is a tool which can be used to download the University of Florida
Sparse Matrix Collection (UFLSMC) and to run some analysis on the matrices.

The basic usage is just:
> ./getmymatrices

And that downloads all of the UFLSMC in the compressed form.

To get matrices for a benchmark, one uses e.g.:
> ./getmymatrices -i uflsmc --real --pos-def

And that makes a folder benchmark with a list.txt in it which contains
the list of real and positive definite matrices (use -h or --help to get
all the available options). And of course, there are the matrices,
uncompressed from the downloaded archives. On Windows, 7-zip or some
working implementation of tar and gzip is needed.

SLAM++ however employs sparse *block* matrices. This tool can do block
structure analysis.

To filter out block matrices, one uses:
> ./getmymatrices -i uflsmc --generate-block-layouts -o uflsmc_bla

Most of those layouts still have a lot of 1x1 blocks. It is possible
to fuse them together to form larger blocks (which introduces some
fill-in). It is possible to use e.g.:
> ./getmymatrices -i uflsmc --block-sizes 3,7:20 -blaf uflsmc_bla

And this collects matrices with 3x3, 3x7, 7x3 and 7x7 blocks in them,
without introducing less than 1/20 of fill-in.

Since this is a time-consuming process, there are also scripts for PBS
enabled clusters. Use:
> ./resubmit_bsn.sh

To submit (and re-submit) tasks with checkpointing which process all
of the UFLSMC and yield common block sizes.

Use:
> ./show_gmm_progress.sh

To show the progress of this processing.

Note that you may need to modify paths in both scripts, depending on
your set-up.
