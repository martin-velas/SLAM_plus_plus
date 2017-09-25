#!/bin/sh

mkdir sparse_flops
cd sparse_flops
mkdir submit
cd submit

SPP_BIN=`readlink -f ../../../bin/slam_schur_orderings`
FLAGS="-npt -nbc"

free_submit.sh -nw mkdir ../intel\; cd ../intel\; "$SPP_BIN" ../../../data/SLAM_sys/intel -naf -amics -nex $FLAGS \| tee log.txt
free_submit.sh -nw mkdir ../killian\; cd ../killian\; "$SPP_BIN" ../../../data/SLAM_sys/killian-court -naf -amics -nex $FLAGS \| tee log.txt
free_submit.sh -nw mkdir ../molson\; cd ../molson\; "$SPP_BIN" ../../../data/SLAM_sys/manhattanOlson3500 -naf -amics -nex $FLAGS \| tee log.txt
free_submit.sh -nw mkdir ../garage\; cd ../garage\; "$SPP_BIN" ../../../data/SLAM_sys/parking-garage -naf -amics -nex $FLAGS \| tee log.txt
free_submit.sh -nw mkdir ../sphere\; cd ../sphere\; "$SPP_BIN" ../../../data/SLAM_sys/sphere2500 -naf -amics -nex -ndcc $FLAGS \| tee log.txt
free_submit.sh -nw mkdir ../vp\; cd ../vp\; "$SPP_BIN" ../../../data/SLAM_sys/victoria-park -naf -amics -nex -ndcc $FLAGS \| tee log.txt
free_submit.sh -nw mkdir ../10k\; cd ../10k\; "$SPP_BIN" ../../../data/SLAM_sys/10k -naf -amics -nex -nbc -ndcc $FLAGS \| tee log.txt
#free_submit.sh -nw mkdir ../100k\; cd ../100k\; "$SPP_BIN" ../../../data/SLAM_sys/100k -naf -amics -nex -nbc -ndcc $FLAGS -pc \| tee log.txt
pbs_submit.sh -jt 4 -jq qprod -nw mkdir ../100k\; cd ../100k\; "$SPP_BIN" ../../../data/SLAM_sys/100k -naf -amics -nex -nbc -ndcc $FLAGS -pc \| tee log.txt
free_submit.sh -nw mkdir ../city10k\; cd ../city10k\; "$SPP_BIN" ../../../data/SLAM_sys/city10k -naf -amics -nex -ndcc $FLAGS \| tee log.txt
free_submit.sh -nw mkdir ../trees10k\; cd ../trees10k\; "$SPP_BIN" ../../../data/SLAM_sys/cityTrees10k -naf -amics -nex -ndcc $FLAGS \| tee log.txt

#time (mkdir ../intel; cd ../intel; "$SPP_BIN" ../../../data/SLAM_sys/intel -naf -amics -nex | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../killian; cd ../killian; "$SPP_BIN" ../../../data/SLAM_sys/killian-court -naf -amics -nex | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../molson; cd ../molson; "$SPP_BIN" ../../../data/SLAM_sys/manhattanOlson3500 -naf -amics -nex | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../garage; cd ../garage; "$SPP_BIN" ../../../data/SLAM_sys/parking-garage -naf -amics -nex | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../sphere; cd ../sphere; "$SPP_BIN" ../../../data/SLAM_sys/sphere2500 -naf -amics -nex -ndcc | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../vp; cd ../vp; "$SPP_BIN" ../../../data/SLAM_sys/victoria-park -naf -amics -nex -ndcc | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../10k; cd ../10k; "$SPP_BIN" ../../../data/SLAM_sys/10k -naf -amics -nex -nbc -ndcc | tee log.txt) # brute-force cliques and -emics both take a lot of time
#time (mkdir ../city10k; cd ../city10k; "$SPP_BIN" ../../../data/SLAM_sys/city10k -naf -amics -nex -ndcc | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../trees10k; cd ../trees10k; "$SPP_BIN" ../../../data/SLAM_sys/cityTrees10k -naf -amics -nex -ndcc | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
## brute-force cliques have bad performance on garage (about 5 seconds)
## brute-force cliques have prohibitive performance on 10k (?), but works surprisingly well on city10k or cityTrees10k
# or serial execution

echo waiting for the first round of jobs to finish
while [[ `qstat -u ipolok | grep ipolok | wc -l` -ne 0 ]]; do
	sleep 10
done
echo the first round of jobs finished
# wait for all to finish

free_submit.sh -jn intel -nw cd ../intel/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn 100k -nw cd ../100k/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
# all first-fit

free_submit.sh -jn intel -nw cd ../intel/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn 100k -nw cd ../100k/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
# all quadratically improved first-fit

free_submit.sh -jn intel -nw cd ../intel/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt # emics takes too
free_submit.sh -jn killian -nw cd ../killian/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -amics $FLAGS \| tee log.txt # dense chol or -emics needs too much memory, gets us killed
free_submit.sh -jn molson -nw cd ../molson/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -amics $FLAGS \| tee log.txt # emics takes too long
free_submit.sh -jn garage -nw cd ../garage/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt # emics takes too long
free_submit.sh -jn sphere -nw cd ../sphere/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -amics $FLAGS \| tee log.txt # emics takes too long
free_submit.sh -jn vp -nw cd ../vp/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt # emics takes too long
free_submit.sh -jn 10k -nw cd ../10k/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc $FLAGS \| tee log.txt # emics takes too long
free_submit.sh -jn 100k -nw cd ../100k/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc $FLAGS \| tee log.txt # emics takes too long
free_submit.sh -jn city10k -nw cd ../city10k/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc $FLAGS \| tee log.txt # emics takes too long
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc $FLAGS \| tee log.txt # emics takes too long
# all AMICS

#time (cd ../intel/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../killian/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../molson/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../garage/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../sphere/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../vp/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../10k/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../city10k/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../trees10k/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
## all first-fit
#
#time (cd ../intel/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../killian/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../molson/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../garage/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../sphere/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../vp/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../10k/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../city10k/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../trees10k/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
## all quadratically improved first-fit
#
#time (cd ../intel/schur_AMICS; "$SPP_BIN" . -naf -nex -amics | tee log.txt) # emics takes too long
#time (cd ../killian/schur_AMICS; "$SPP_BIN" . -naf -nex -amics | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (cd ../molson/schur_AMICS; "$SPP_BIN" . -naf -nex -amics | tee log.txt) # emics takes too long
#time (cd ../garage/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics | tee log.txt) # emics takes too long
#time (cd ../sphere/schur_AMICS; "$SPP_BIN" . -naf -nex -amics | tee log.txt) # emics takes too long
#time (cd ../vp/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics | tee log.txt) # emics takes too long
#time (cd ../10k/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc | tee log.txt) # emics takes too long
#time (cd ../city10k/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc | tee log.txt) # emics takes too long
#time (cd ../trees10k/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc | tee log.txt) # emics takes too long
## brute-force cliques have bad performance on intel (38 seconds), killian (1.5 seconds)
## brute-force cliques have prohibitive performance on garage (?), victoria-park (?), 10k (?), city10k (?), cityTrees10k (?)
## all AMICS
# or serial execution

echo waiting for the second round of jobs to finish
while [[ `qstat -u ipolok | grep ipolok | wc -l` -ne 0 ]]; do
	sleep 10
done
echo the second round of jobs finished
# wait for all to finish

free_submit.sh -jn intel -nw cd ../intel/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn 100k -nw cd ../100k/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
# all first-fit

free_submit.sh -jn intel -nw cd ../intel/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics $FLAGS \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn 100k -nw cd ../100k/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_impq/schur_impq\; pwd\; "$SPP_BIN" . -naf -nex -nmics -ndcc $FLAGS \| tee log.txt
# all quadratically improved first-fit

free_submit.sh -jn intel -nw cd ../intel/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn 100k -nw cd ../100k/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
# all AMICS

#time (cd ../intel/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../killian/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../molson/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../garage/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../sphere/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../vp/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../10k/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../city10k/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../trees10k/schur_ff/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
## all first-fit
#
#time (cd ../intel/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../killian/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../molson/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../garage/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../sphere/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../vp/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../10k/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../city10k/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../trees10k/schur_impq/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
# all quadratically improved first-fit
#
#time (cd ../intel/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt) # emics takes too long
#time (cd ../killian/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt) # emics takes too long
#time (cd ../molson/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt) # emics takes too long
#time (cd ../garage/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt) # emics takes too long
#time (cd ../sphere/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt) # emics takes too long
#time (cd ../vp/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt) # emics takes too long
#time (cd ../10k/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt) # emics takes too long
#time (cd ../city10k/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nbc -nex -amics -emics | tee log.txt) # emics takes too long
#time (cd ../trees10k/schur_AMICS/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt) # emics takes too long
## brute-force cliques have prohibitive performance on intel (?), killian (?), molson (?), garage (?), sphere (?), vp (?), 10k (?), city10k (?) trees10k (?)
## all AMICS
# or serial execution

# ---------------------------------------------------------------------------------------------
exit

free_submit.sh -nw mkdir ../intel\; cd ../intel\; "$SPP_BIN" ../../data/SLAM_sys/intel -naf -amics -nex \| tee log.txt
free_submit.sh -nw mkdir ../killian\; cd ../killian\; "$SPP_BIN" ../../data/SLAM_sys/killian-court -naf -amics -nex \| tee log.txt
free_submit.sh -nw mkdir ../molson\; cd ../molson\; "$SPP_BIN" ../../data/SLAM_sys/manhattanOlson3500 -naf -amics -nex \| tee log.txt
free_submit.sh -nw mkdir ../garage\; cd ../garage\; "$SPP_BIN" ../../data/SLAM_sys/parking-garage -naf -amics -nex \| tee log.txt
free_submit.sh -nw mkdir ../sphere\; cd ../sphere\; "$SPP_BIN" ../../data/SLAM_sys/sphere2500 -naf -amics -nex -ndcc \| tee log.txt
free_submit.sh -nw mkdir ../vp\; cd ../vp\; "$SPP_BIN" ../../data/SLAM_sys/victoria-park -naf -amics -nex -ndcc \| tee log.txt
free_submit.sh -nw mkdir ../10k\; cd ../10k\; "$SPP_BIN" ../../data/SLAM_sys/10k -naf -amics -nex -nbc -ndcc \| tee log.txt
free_submit.sh -nw mkdir ../city10k\; cd ../city10k\; "$SPP_BIN" ../../data/SLAM_sys/city10k -naf -amics -nex -ndcc \| tee log.txt
free_submit.sh -nw mkdir ../trees10k\; cd ../trees10k\; "$SPP_BIN" ../../data/SLAM_sys/cityTrees10k -naf -amics -nex -ndcc \| tee log.txt

#time (mkdir ../intel; cd ../intel; "$SPP_BIN" ../../data/SLAM_sys/intel -naf -amics -nex | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../killian; cd ../killian; "$SPP_BIN" ../../data/SLAM_sys/killian-court -naf -amics -nex | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../molson; cd ../molson; "$SPP_BIN" ../../data/SLAM_sys/manhattanOlson3500 -naf -amics -nex | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../garage; cd ../garage; "$SPP_BIN" ../../data/SLAM_sys/parking-garage -naf -amics -nex | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../sphere; cd ../sphere; "$SPP_BIN" ../../data/SLAM_sys/sphere2500 -naf -amics -nex -ndcc | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../vp; cd ../vp; "$SPP_BIN" ../../data/SLAM_sys/victoria-park -naf -amics -nex -ndcc | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../10k; cd ../10k; "$SPP_BIN" ../../data/SLAM_sys/10k -naf -amics -nex -nbc -ndcc | tee log.txt) # brute-force cliques and -emics both take a lot of time
#time (mkdir ../city10k; cd ../city10k; "$SPP_BIN" ../../data/SLAM_sys/city10k -naf -amics -nex -ndcc | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (mkdir ../trees10k; cd ../trees10k; "$SPP_BIN" ../../data/SLAM_sys/cityTrees10k -naf -amics -nex -ndcc | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
## brute-force cliques have bad performance on garage (about 5 seconds)
## brute-force cliques have prohibitive performance on 10k (?), but works surprisingly well on city10k or cityTrees10k
# or serial execution

echo waiting for the first round of jobs to finish
while [[ `qstat -u ipolok | grep ipolok | wc -l` -ne 0 ]]; do
	sleep 10
done
echo the first round of jobs finished
# wait for all to finish

free_submit.sh -jn intel -nw cd ../intel/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_ff\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_ff\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_ff\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
# all first-fit

free_submit.sh -jn intel -nw cd ../intel/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_impq\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_impq\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_impq\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
# all quadratically improved first-fit

free_submit.sh -jn intel -nw cd ../intel/schur_AMICS\; "$SPP_BIN" . -naf -nex -amics \| tee log.txt # emics takes too long
free_submit.sh -jn killian -nw cd ../killian/schur_AMICS\; "$SPP_BIN" . -naf -nex -amics \| tee log.txt # dense chol or -emics needs too much memory, gets us killed
free_submit.sh -jn molson -nw cd ../molson/schur_AMICS\; "$SPP_BIN" . -naf -nex -amics \| tee log.txt # emics takes too long
free_submit.sh -jn garage -nw cd ../garage/schur_AMICS\; "$SPP_BIN" . -naf -nex -nbc -amics \| tee log.txt # emics takes too long
free_submit.sh -jn sphere -nw cd ../sphere/schur_AMICS\; "$SPP_BIN" . -naf -nex -amics \| tee log.txt # emics takes too long
free_submit.sh -jn vp -nw cd ../vp/schur_AMICS\; "$SPP_BIN" . -naf -nex -nbc -amics \| tee log.txt # emics takes too long
free_submit.sh -jn 10k -nw cd ../10k/schur_AMICS\; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc \| tee log.txt # emics takes too long
free_submit.sh -jn city10k -nw cd ../city10k/schur_AMICS\; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc \| tee log.txt # emics takes too long
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_AMICS\; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc \| tee log.txt # emics takes too long
# all AMICS

#time (cd ../intel/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../killian/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../molson/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../garage/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../sphere/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../vp/schur_ff; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../10k/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../city10k/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../trees10k/schur_ff; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
## all first-fit
#
#time (cd ../intel/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../killian/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../molson/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../garage/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../sphere/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../vp/schur_impq; "$SPP_BIN" . -naf -nex -nmics | tee log.txt)
#time (cd ../10k/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../city10k/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
#time (cd ../trees10k/schur_impq; "$SPP_BIN" . -naf -nex -nmics -ndcc | tee log.txt)
## all quadratically improved first-fit
#
#time (cd ../intel/schur_AMICS; "$SPP_BIN" . -naf -nex -amics | tee log.txt) # emics takes too long
#time (cd ../killian/schur_AMICS; "$SPP_BIN" . -naf -nex -amics | tee log.txt) # dense chol or -emics needs too much memory, gets us killed
#time (cd ../molson/schur_AMICS; "$SPP_BIN" . -naf -nex -amics | tee log.txt) # emics takes too long
#time (cd ../garage/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics | tee log.txt) # emics takes too long
#time (cd ../sphere/schur_AMICS; "$SPP_BIN" . -naf -nex -amics | tee log.txt) # emics takes too long
#time (cd ../vp/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics | tee log.txt) # emics takes too long
#time (cd ../10k/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc | tee log.txt) # emics takes too long
#time (cd ../city10k/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc | tee log.txt) # emics takes too long
#time (cd ../trees10k/schur_AMICS; "$SPP_BIN" . -naf -nex -nbc -amics -ndcc | tee log.txt) # emics takes too long
## brute-force cliques have bad performance on intel (38 seconds), killian (1.5 seconds)
## brute-force cliques have prohibitive performance on garage (?), victoria-park (?), 10k (?), city10k (?), cityTrees10k (?)
## all AMICS
# or serial execution

echo waiting for the second round of jobs to finish
while [[ `qstat -u ipolok | grep ipolok | wc -l` -ne 0 ]]; do
	sleep 10
done
echo the second round of jobs finished
# wait for all to finish

free_submit.sh -jn intel -nw cd ../intel/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_ff/schur_ff\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
# all first-fit

free_submit.sh -jn intel -nw cd ../intel/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn killian -nw cd ../killian/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn molson -nw cd ../molson/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn garage -nw cd ../garage/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn sphere -nw cd ../sphere/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn vp -nw cd ../vp/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics \| tee log.txt
free_submit.sh -jn 10k -nw cd ../10k/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
free_submit.sh -jn city10k -nw cd ../city10k/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
free_submit.sh -jn trees10k -nw cd ../trees10k/schur_impq/schur_impq\; "$SPP_BIN" . -naf -nex -nmics -ndcc \| tee log.txt
# all quadratically improved first-fit
