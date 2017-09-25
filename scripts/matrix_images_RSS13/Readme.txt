Here are the scripts that were used in the RSS2013 evaluation
and matrix sparsity pattern video preparation.

transpose_results.sh scans all the text files in the path and
outputs them transposed as comma separated values. If used on
0000*_9_stats.txt files then the columns are filename, number
of nnz blocks in lambda, number of nny blocks in R and matrix
cut (if the incremental reordering was employed in that given
step).

kill_Rs.bat deletes all the 0000*_7_R.tga images (windows).

kill_txts.bat moves all the 0000*_9_stats.txt files to folder
named stats (in case it did not exist before, it is created).

make_video.bat makes a video from the pngs in the path (it is
somewhat non-portable).

*.xbs are the XnView scripts, which were used to make all the
images the same size for the matrix sparsity pattern videos.
