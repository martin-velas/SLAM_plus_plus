#!/bin/sh

mkdir BA_flops_gpuinv2
cd BA_flops_gpuinv2

SPP_BIN=`readlink -f ../../bin/slam_schur_orderings`

free_GPU_submit.sh -jn kr_s00-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/\; done
free_GPU_submit.sh -jn kr_loop-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/\; done
free_GPU_submit.sh -jn kr_loop41-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/\; done
free_GPU_submit.sh -jn kr_loopw20-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/\; done
free_GPU_submit.sh -jn kr_loopw6-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/\; done
# evaluate gord perf (no images written)

free_GPU_submit.sh -jn kr_s00-AM4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop-AM4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop41-AM4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw20-AM4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw6-AM4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done

free_GPU_submit.sh -jn kr_s00-AM3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop-AM3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop41-AM3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw20-AM3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw6-AM3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done

free_GPU_submit.sh -jn kr_s00-AM2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop-AM2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop41-AM2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw20-AM2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw6-AM2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/\; done
# evaluate nested AMICS perf (no images written)

free_GPU_submit.sh -jn kr_s00-ff1g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_ff/\; done
free_GPU_submit.sh -jn kr_loop-ff1g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_ff/\; done
free_GPU_submit.sh -jn kr_loop41-ff1g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_ff/\; done
free_GPU_submit.sh -jn kr_loopw20-ff1g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_ff/\; done
free_GPU_submit.sh -jn kr_loopw6-ff1g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_ff/\; done

free_GPU_submit.sh -jn kr_s00-ff2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loop-ff2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loop41-ff2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loopw20-ff2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loopw6-ff2g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_ff/schur_ff/\; done

free_GPU_submit.sh -jn kr_s00-ff3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_ff/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loop-ff3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_ff/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loop41-ff3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_ff/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loopw20-ff3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_ff/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loopw6-ff3g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_ff/schur_ff/schur_ff/\; done

free_GPU_submit.sh -jn kr_s00-ff4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loop-ff4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loop41-ff4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loopw20-ff4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn kr_loopw6-ff4g -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/\; done
# evaluate nested ff perf (no images written)

free_GPU_submit.sh -jn kr_s00-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_seq00 -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop41-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_41 -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw20-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W20 -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw6-AM1 -nw for i in 1 2\; do $SPP_BIN ../../data/BA_sys/karlsruhe_loop_W6 -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/\; done

free_GPU_submit.sh -jn veni-AM2 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_AMICS/schur_AMICS/ -dm 10000\; done
free_GPU_submit.sh -jn veni-AM1 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_AMICS/ -dm 10000\; done
free_GPU_submit.sh -jn veni-ff2 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_ff/schur_ff/ -dm 10000\; done
free_GPU_submit.sh -jn veni-ff1 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_ff/ -dm 10000\; done
free_GPU_submit.sh -jn veni-gord -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/ -dm 10000\; done

#exit

free_GPU_submit.sh -jn kr_s00-AM2 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_seq00/schur_gord -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop-AM2 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loop/schur_gord -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop41-AM2 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loop41/schur_gord -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw20-AM2 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loopw20/schur_gord -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw6-AM2 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loopw6/schur_gord -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/\; done

free_GPU_submit.sh -jn kr_s00-AM3 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_seq00/schur_gord -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop-AM3 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loop/schur_gord -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop41-AM3 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loop41/schur_gord -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw20-AM3 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loopw20/schur_gord -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw6-AM3 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loopw6/schur_gord -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/\; done

free_GPU_submit.sh -jn kr_s00-AM4 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_seq00/schur_gord -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop-AM4 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loop/schur_gord -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loop41-AM4 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loop41/schur_gord -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw20-AM4 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loopw20/schur_gord -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw6-AM4 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loopw6/schur_gord -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn kr_loopw20-AM4 -nw for i in 1 2\; do $SPP_BIN ../sparse_flops/kr_loopw20/schur_gord -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_MICS/\; done

free_GPU_submit.sh -jn ff6-AM2 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn ff6-MICS -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_MICS/\; done
free_GPU_submit.sh -jn ff6-AM1 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/\; done
free_GPU_submit.sh -jn ff6-ff2 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn ff6-ff1 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_ff/\; done
free_GPU_submit.sh -jn ff6-gord -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/\; done

free_GPU_submit.sh -jn cath-AM2 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_AMICS/\; done
free_GPU_submit.sh -jn cath-MICS -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_MICS/\; done
free_GPU_submit.sh -jn cath-AM1 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/\; done
free_GPU_submit.sh -jn cath-ff2 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_ff/schur_ff/\; done
free_GPU_submit.sh -jn cath-ff1 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_ff/\; done
free_GPU_submit.sh -jn cath-gord -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/\; done

free_GPU_submit.sh -jn veni-AM2 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_AMICS/schur_AMICS/ -dm 10000\; done
free_GPU_submit.sh -jn veni-MICS -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_AMICS/schur_MICS/ -dm 10000\; done
free_GPU_submit.sh -jn veni-AM1 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_AMICS/schur_AMICS/ -dm 10000\; done
free_GPU_submit.sh -jn veni-ff2 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_ff/schur_ff/ -dm 10000\; done
free_GPU_submit.sh -jn veni-ff1 -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/schur_ff/ -dm 10000\; done
free_GPU_submit.sh -jn veni-gord -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/venice/schur_gord/ -dm 10000\; done

echo "waiting for the jobs to finish"
while [[ `qstat -u ipolok | grep ipolok | wc -l` -ne 0 ]]; do
	sleep 60
	echo -n "."
done
echo -e "\nthe jobs have finished"
# wait for all to finish

(../parse_nested_schur_analysis.sh -5l *.OU | head -n 1; ../parse_nested_schur_analysis.sh -5l *.OU | gawk '{ printf("%-20s%-20s|", $2, $1); print $0; }' | tail -n +2 | sort | cut -d "|" -f 2) > results.txt

exit

../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_MICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_MICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_MICS/

../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/

../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/ff6 -nes ../sparse_flops/ff6/schur_gord/

../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_MICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_MICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_MICS/

../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/schur_AMICS/

../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/cathedral -nes ../sparse_flops/cath/schur_gord/

../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/schur_AMICS/

../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/schur_MICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/schur_MICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/schur_MICS/

../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/schur_ff/

../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/BA_sys/venice871.g2o -nes ../sparse_flops/cath/schur_gord/
