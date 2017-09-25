#!/bin/sh

mkdir sparse_flops
cd sparse_flops
mkdir submit
cd submit

waitJobs() {
	echo waiting for the $1 round of jobs to finish
	while [[ `qstat -u ipolok | grep ipolok | grep -v qlong | wc -l` -ne 0 ]]; do
		sleep 10
	done
	echo the $1 round of jobs finished
	# wait for all to finish
}

waitJobs "any preceding"

SPP_BIN=`readlink -f ../../../bin/slam_schur_orderings`

FLAGS="-npt -nbc"
JTIME="01:00:00"

free_submit.sh -jn kr_s00 -nw mkdir ../kr_seq00\; cd ../kr_seq00\; "$SPP_BIN" ../../../data/BA_sys/karlsruhe_seq00 -naf -nex $FLAGS \| tee log.txt
free_submit.sh -jn kr_loop -nw mkdir ../kr_loop\; cd ../kr_loop\; "$SPP_BIN" ../../../data/BA_sys/karlsruhe_loop -naf -nex $FLAGS \| tee log.txt
free_submit.sh -jn kr_loop41 -nw mkdir ../kr_loop41\; cd ../kr_loop41\; "$SPP_BIN" ../../../data/BA_sys/karlsruhe_loop_41 -naf -nex $FLAGS \| tee log.txt
free_submit.sh -jn kr_loopw20 -nw mkdir ../kr_loopw20\; cd ../kr_loopw20\; "$SPP_BIN" ../../../data/BA_sys/karlsruhe_loop_W20 -naf -nex $FLAGS \| tee log.txt
free_submit.sh -jn kr_loopw6 -nw mkdir ../kr_loopw6\; cd ../kr_loopw6\; "$SPP_BIN" ../../../data/BA_sys/karlsruhe_loop_W6 -naf -nex $FLAGS \| tee log.txt

free_submit.sh -jn ff6 -nw mkdir ../ff6\; cd ../ff6\; "$SPP_BIN" ../../../data/BA_sys/ff6 -naf -nex $FLAGS \| tee log.txt
free_submit.sh -jn cath -nw mkdir ../cath\; cd ../cath\; "$SPP_BIN" ../../../data/BA_sys/cathedral -naf -nex $FLAGS \| tee log.txt
free_submit.sh -jn venice -nw mkdir ../venice\; cd ../venice\; "$SPP_BIN" ../../../data/BA_sys/venice871.g2o -naf -nex $FLAGS \| tee log.txt

#time (mkdir ../ff6; cd ../ff6; "$SPP_BIN" ../../../data/BA_sys/ff6 -naf -nex $FLAGS | tee log.txt)
#time (mkdir ../cath; cd ../cath; "$SPP_BIN" ../../../data/BA_sys/cathedral -naf -nex $FLAGS | tee log.txt)
#time (mkdir ../venice; cd ../venice; "$SPP_BIN" ../../../data/BA_sys/venice871.g2o -naf -nex $FLAGS | tee log.txt)
# or serial

waitJobs "first"

free_submit.sh -jt $JTIME -jn kr_s00 -nw cd ../kr_seq00/schur_gord\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop -nw cd ../kr_loop/schur_gord\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop41 -nw cd ../kr_loop41/schur_gord\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw20 -nw cd ../kr_loopw20/schur_gord\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw6 -nw cd ../kr_loopw6/schur_gord\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt
# no emics here. want all to run

free_submit.sh -jn ff6 -nw cd ../ff6/schur_gord\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS \| tee log.txt
free_submit.sh -jn cath -nw cd ../cath/schur_gord\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
#free_submit.sh -jn venice -nw cd ../venice/schur_gord\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics -dm 10000 $FLAGS \| tee log.txt
pbs_submit.sh -jq qprod -jt 8 -jn venice -nw cd ../venice/schur_gord\; pwd\; "$SPP_BIN" .-naf -nex -amics -ncs 25 -mcs 50 -dm 10000 $FLAGS \| tee log.txt # takes some time
# or qnvidia

#time (cd ../ff6/schur_gord; pwd; "$SPP_BIN" . -naf -nex -nbc -amics $FLAGS | tee log.txt)
#time (cd ../cath/schur_gord; pwd; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS | tee log.txt) # emics finishes in good time, is much better than amics
#time (cd ../venice/schur_gord; pwd; "$SPP_BIN" . -naf -nex -nbc -nmics -dm 10000 $FLAGS | tee log.txt)
# or serial

waitJobs "second"

free_submit.sh -jn ff6 -nw cd ../ff6/schur_gord/schur_AMICS/\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn cath -nw cd ../cath/schur_gord/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics $FLAGS \| tee log.txt
free_submit.sh -jn venice -nw cd ../venice/schur_gord/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -amics -emics -dm 10000 $FLAGS \| tee log.txt
# -emics works and is crucial for performance

free_submit.sh -jn ff6 -nw cd ../ff6/schur_gord/schur_ff/\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jn cath -nw cd ../cath/schur_gord/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jn venice -nw cd ../venice/schur_gord/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics -dm 10000 $FLAGS \| tee log.txt
# also ff only

#time (cd ../ff6/schur_gord/schur_AMICS; pwd; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt)
#time (cd ../cath/schur_gord/schur_AMICS; pwd; "$SPP_BIN" . -naf -nex -nbc -amics -emics | tee log.txt)
#time (cd ../venice/schur_gord/schur_ff; pwd; "$SPP_BIN" . -naf -nex -nbc -amics -emics -dm 10000 | tee log.txt)
# or serial

free_submit.sh -jt $JTIME -jn kr_s00 -nw cd ../kr_seq00/schur_gord/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop -nw cd ../kr_loop/schur_gord/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop41 -nw cd ../kr_loop41/schur_gord/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw20 -nw cd ../kr_loopw20/schur_gord/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw6 -nw cd ../kr_loopw6/schur_gord/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt

free_submit.sh -jt $JTIME -jn kr_s00 -nw cd ../kr_seq00/schur_gord/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop -nw cd ../kr_loop/schur_gord/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop41 -nw cd ../kr_loop41/schur_gord/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw20 -nw cd ../kr_loopw20/schur_gord/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw6 -nw cd ../kr_loopw6/schur_gord/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
# also ff only

waitJobs "third"

free_submit.sh -jt $JTIME -jn kr_s00 -nw cd ../kr_seq00/schur_gord/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop -nw cd ../kr_loop/schur_gord/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop41 -nw cd ../kr_loop41/schur_gord/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw20 -nw cd ../kr_loopw20/schur_gord/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw6 -nw cd ../kr_loopw6/schur_gord/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt

free_submit.sh -jt $JTIME -jn kr_s00 -nw cd ../kr_seq00/schur_gord/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop -nw cd ../kr_loop/schur_gord/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop41 -nw cd ../kr_loop41/schur_gord/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw20 -nw cd ../kr_loopw20/schur_gord/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw6 -nw cd ../kr_loopw6/schur_gord/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
# also ff only

waitJobs "fourth"

free_submit.sh -jt $JTIME -jn kr_s00 -nw cd ../kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop -nw cd ../kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop41 -nw cd ../kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw20 -nw cd ../kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw6 -nw cd ../kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS\; pwd\; "$SPP_BIN" . -naf -nex -nbc -emics $FLAGS \| tee log.txt

free_submit.sh -jt $JTIME -jn kr_s00 -nw cd ../kr_seq00/schur_gord/schur_ff/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop -nw cd ../kr_loop/schur_gord/schur_ff/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loop41 -nw cd ../kr_loop41/schur_gord/schur_ff/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw20 -nw cd ../kr_loopw20/schur_gord/schur_ff/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
free_submit.sh -jt $JTIME -jn kr_loopw6 -nw cd ../kr_loopw6/schur_gord/schur_ff/schur_ff/schur_ff\; pwd\; "$SPP_BIN" . -naf -nex -nbc -nmics $FLAGS \| tee log.txt
# also ff only

#waitJobs "fifth"
