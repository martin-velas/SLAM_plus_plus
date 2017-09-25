#!/bin/sh

# this builds all the files required for the demo from only venice871.g2o and cathedral_2014-10-02_.txt
# this script takes about 9 minutes to execute on a decent machine

gf_num=6
if [ ! -f venice871.g2o ]; then
	>&2 echo "warning: venice871.g2o not found"
	gf_num=$(( $gf_num - 3 ))
fi
if [ ! -f cathedral_2014-10-02_.txt ]; then
	>&2 echo "warning: cathedral_2014-10-02_.txt not found"
	gf_num=$(( $gf_num - 3 ))
fi
if [[ ! -f venice871.g2o && ! -f cathedral_2014-10-02_.txt ]]; then
	>&2 echo "no data found"
	exit
fi
#if [[ -f venice871.g2o && -f cathedral_2014-10-02_.txt ]]; then
#	>&2 echo "good, have both venice and cathedral"
#fi

echo "patience, my young padawan ..."

if [ -f venice871.g2o ]; then
	./graph_get_cam_connectivity.sh venice871.g2o > venice_cconn.txt
fi
if [ -f cathedral_2014-10-02_.txt ]; then
	./graph_get_cam_connectivity.sh cathedral_2014-10-02_.txt > cathedral_cconn.txt
fi

echo "    [camera connectivity extracted]"

if [ -f venice871.g2o ]; then
	./cconn_to_matrixmarket.sh venice_cconn.txt > venice_cconn.mtx
fi
if [ -f cathedral_2014-10-02_.txt ]; then
	./cconn_to_matrixmarket.sh cathedral_cconn.txt > cathedral_cconn.mtx
fi

echo "    [camera connectivity converted to matrix market]"

if [ -f venice871.g2o ]; then
	./cconn_spanning_tree.sh venice_cconn.txt 0 > venice_camorder_DFS.txt
	./cconn_spanning_tree.sh venice_cconn.txt 1 > venice_camorder_BFS.txt
	./cconn_spanning_tree.sh venice_cconn.txt 2 > venice_camorder_BFSpq.txt
fi
if [ -f cathedral_2014-10-02_.txt ]; then
	./cconn_spanning_tree.sh cathedral_cconn.txt 0 > cathedral_camorder_DFS.txt
	./cconn_spanning_tree.sh cathedral_cconn.txt 1 > cathedral_camorder_BFS.txt
	./cconn_spanning_tree.sh cathedral_cconn.txt 2 > cathedral_camorder_BFSpq.txt
fi

echo "    [spanning trees generated]"

if [[ -f venice871.g2o && -f cathedral_2014-10-02_.txt ]]; then
	echo 'function [v_order_DFS, v_order_BFS, v_order_BFSpq, c_order_DFS, c_order_BFS, c_order_BFSpq]=orders()' > orders.m

	cat venice_camorder_DFS.txt | grep "ORDER" | gawk '{ list = list " " $2; } END { print "v_order_DFS = [" substr(list, 2) "] + 1;"; }' >> orders.m
	cat venice_camorder_BFS.txt | grep "ORDER" | gawk '{ list = list " " $2; } END { print "v_order_BFS = [" substr(list, 2) "] + 1;"; }' >> orders.m
	cat venice_camorder_BFSpq.txt | grep "ORDER" | gawk '{ list = list " " $2; } END { print "v_order_BFSpq = [" substr(list, 2) "] + 1;"; }' >> orders.m

	cat cathedral_camorder_DFS.txt | grep "ORDER" | gawk '{ list = list " " $2; } END { print "c_order_DFS = [" substr(list, 2) "] + 1;"; }' >> orders.m
	cat cathedral_camorder_BFS.txt | grep "ORDER" | gawk '{ list = list " " $2; } END { print "c_order_BFS = [" substr(list, 2) "] + 1;"; }' >> orders.m
	cat cathedral_camorder_BFSpq.txt | grep "ORDER" | gawk '{ list = list " " $2; } END { print "c_order_BFSpq = [" substr(list, 2) "] + 1;"; }' >> orders.m
else
	>&2 echo "warning: the matlab script not generated, some data files missing"
fi

echo "    [camera orders generated]"

if [ -f cathedral_2014-10-02_.txt ]; then
	./graph_reorder_cameras.sh cathedral_2014-10-02_.txt cathedral_camorder_DFS.txt > cathedral_2014-10-02_incremental_DFS.txt
	echo "    [1 / $gf_num graph files generated]" | gawk '{ printf("%s\r", $0); }'
	./graph_reorder_cameras.sh cathedral_2014-10-02_.txt cathedral_camorder_BFS.txt > cathedral_2014-10-02_incremental_BFS.txt
	echo "    [2 / $gf_num graph files generated]" | gawk '{ printf("%s\r", $0); }'
	./graph_reorder_cameras.sh cathedral_2014-10-02_.txt cathedral_camorder_BFSpq.txt > cathedral_2014-10-02_incremental_BFSpq.txt
	echo "    [3 / $gf_num graph files generated]" | gawk '{ printf("%s\r", $0); }'
fi
if [ -f venice871.g2o ]; then
	./graph_reorder_cameras.sh venice871.g2o venice_camorder_DFS.txt > venice_incremental_DFS.txt
	echo "    [$(( $gf_num - 2 )) / $gf_num graph files generated]" | gawk '{ printf("%s\r", $0); }'
	./graph_reorder_cameras.sh venice871.g2o venice_camorder_BFS.txt > venice_incremental_BFS.txt
	echo "    [$(( $gf_num - 1 )) / $gf_num graph files generated]" | gawk '{ printf("%s\r", $0); }'
	./graph_reorder_cameras.sh venice871.g2o venice_camorder_BFSpq.txt > venice_incremental_BFSpq.txt
	echo "    [$(( $gf_num - 0 )) / $gf_num graph files generated]" | gawk '{ printf("%s\r", $0); }'
fi
if [[ -f cathedral_2014-10-02_.txt || -f venice871.g2o ]]; then
	echo ""
	# print a line feed
fi

if [[ -f venice871.g2o && -f cathedral_2014-10-02_.txt ]]; then
	#md5_ref="d361c1e7ff00bcd8316629baa1e74bbb"
	md5_ref="43943ad6119ea55cb8e9dd6879db06b2" # after adding sorting at 2015-06-29
	md5=`md5sum *_incremental_*.txt | md5sum | gawk '{ print $1; }'`
	if [[ ! "$md5_ref" == "$md5" ]]; then
		>&2 echo "warning: your results seem to be slightly different, maybe you have different datasets?"
	fi
fi

# post-2015-06-29:
# running `md5sum *_incremental_*.txt` yields:
# 3fcbbdf5cc3420114c60e4cb24c7ddb1  cathedral_2014-10-02_incremental_BFSpq.txt
# 779df3c2c5f442e1e70bb78255ada341  cathedral_2014-10-02_incremental_BFS.txt
# b8f96eaa734be0b1d2b6ecbd39a3ecc6  cathedral_2014-10-02_incremental_DFS.txt
# abaeae80619681339d235b119a477e78  venice_incremental_BFSpq.txt
# 82b51f21cf8c9210900857b842e2d6a8  venice_incremental_BFS.txt
# 4092b3900832004032d5847f0de05d6d  venice_incremental_DFS.txt
#
# to get a single checksum, use `md5sum *_incremental_*.txt | md5sum`
# 43943ad6119ea55cb8e9dd6879db06b2  -

# pre-2015-06-29:
# running `md5sum *_incremental_*.txt` yields:
# 7f0802e1846e53508d1ace1635fe9bd2  cathedral_2014-10-02_incremental_BFSpq.txt
# e56ecc8a12603a64b56ba4a222a29972  cathedral_2014-10-02_incremental_BFS.txt
# 4573dd1e3f8dfc521d1a25ac0e168d36  cathedral_2014-10-02_incremental_DFS.txt
# f00f869760cebcd426b8a28ff5048f21  venice_incremental_BFSpq.txt
# 6e82aecd2c62c3b9e18393cf468cc288  venice_incremental_BFS.txt
# 095631e2d7134d9cac500a292dd02678  venice_incremental_DFS.txt
#
# to get a single checksum, use `md5sum *_incremental_*.txt | md5sum`
# d361c1e7ff00bcd8316629baa1e74bbb  -
