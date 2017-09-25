#!/bin/sh

mkdir SLAM_flops
cd SLAM_flops

IS_SALOMON=0
if [[ `qstat | grep ".isrv" | wc -l` -gt 0 ]]; then
	IS_SALOMON=1
fi
# detect whether we are on Salomon

if [[ $IS_SALOMON -eq 0 ]]; then
	WHAT=free_GPU_submit.sh
else
	WHAT=free_submit.sh
fi
# parallel submit with GPUs on Anselm

$WHAT -jn 10k-impq -nw      for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn 100k-impq -nw      for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/100k -nes ../sparse_flops/100k/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn city10k-impq -nw  for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn garage-impq -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn intel-impq -nw    for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn killian-impq -nw  for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn molson-impq -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn sphere-impq -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn trees10k-impq -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_impq/schur_impq/schur_impq/ \; done
$WHAT -jn vp-impq -nw       for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_impq/schur_impq/schur_impq/ \; done

$WHAT -jn 10k-ff -nw      for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn 100k-ff -nw      for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/100k -nes ../sparse_flops/100k/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn city10k-ff -nw  for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn garage-ff -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn intel-ff -nw    for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn killian-ff -nw  for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn molson-ff -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn sphere-ff -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn trees10k-ff -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_ff/schur_ff/schur_ff/ \; done
$WHAT -jn vp-ff -nw       for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_ff/schur_ff/schur_ff/ \; done

$WHAT -jn 10k-AMICS -nw      for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn 100k-AMICS -nw      for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/100k -nes ../sparse_flops/100k/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn city10k-AMICS -nw  for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn garage-AMICS -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn intel-AMICS -nw    for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn killian-AMICS -nw  for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn molson-AMICS -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn sphere-AMICS -nw   for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn trees10k-AMICS -nw for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_AMICS/schur_AMICS/schur_AMICS/ \; done
$WHAT -jn vp-AMICS -nw       for i in 1 2\; do ../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_AMICS/schur_AMICS/schur_AMICS/ \; done

exit

# serial submit anywhere
../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_AMICS/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_AMICS/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_AMICS/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_AMICS/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_AMICS/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_AMICS/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_AMICS/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_AMICS/schur_AMICS/schur_AMICS/

../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_AMICS/schur_AMICS/schur_AMICS/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_AMICS/schur_AMICS/schur_AMICS/

# ----

../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_ff/schur_ff/schur_ff/

../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_ff/schur_ff/schur_ff/

../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_ff/schur_ff/schur_ff/

../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_ff/schur_ff/schur_ff/

../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_ff/schur_ff/schur_ff/

../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_ff/schur_ff/schur_ff/

../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_ff/schur_ff/schur_ff/

../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_ff/schur_ff/schur_ff/

../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_ff/schur_ff/schur_ff/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_ff/schur_ff/schur_ff/

# ----

../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_impq/schur_impq/schur_impq/

../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_impq/schur_impq/schur_impq/

../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_impq/schur_impq/schur_impq/

../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_impq/schur_impq/schur_impq/

../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_impq/schur_impq/schur_impq/

../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_impq/schur_impq/schur_impq/

../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_impq/schur_impq/schur_impq/

../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_impq/schur_impq/schur_impq/

../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_impq/schur_impq/schur_impq/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_impq/schur_impq/schur_impq/

# ----

exit

# have not generated those before, impqw being an outsider ordering before it was improved (then it matches impq except for the landmark datasets)
../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/10k -nes ../sparse_flops/10k/schur_impqw/schur_impqw/schur_impqw/

../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/city10k -nes ../sparse_flops/city10k/schur_impqw/schur_impqw/schur_impqw/

../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/parking-garage -nes ../sparse_flops/garage/schur_impqw/schur_impqw/schur_impqw/

../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/intel -nes ../sparse_flops/intel/schur_impqw/schur_impqw/schur_impqw/

../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/killian-court -nes ../sparse_flops/killian/schur_impqw/schur_impqw/schur_impqw/

../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/manhattanOlson3500 -nes ../sparse_flops/molson/schur_impqw/schur_impqw/schur_impqw/

../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/sphere2500 -nes ../sparse_flops/sphere/schur_impqw/schur_impqw/schur_impqw/

../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/cityTrees10k -nes ../sparse_flops/trees10k/schur_impqw/schur_impqw/schur_impqw/

../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_impqw/schur_impqw/schur_impqw/ 2>&1 > /dev/null
../../bin/slam_schur_orderings ../../data/SLAM_sys/victoria-park -nes ../sparse_flops/vp/schur_impqw/schur_impqw/schur_impqw/
