#!/bin/bash

REPEATS=3
# options

if [[ ! -d src || -d include || -f CMakeLists.txt ]]; then
	>&2 echo "this is supposed to be run from SLAM++ root folder"
	exit -1
fi

if [[ ! -d build_eigperf ]]; then
	mkdir build_eigperf
	if [[ $? -ne 0 ]]; then
		>&2 echo "error: failed to mkdir build_eigperf"
		exit -1
	fi
fi
cd build_eigperf
if [[ $? -ne 0 ]]; then
	>&2 echo "error: failed to cd build_eigperf"
	exit -1
fi
# make very sure that we're in the folder before deleting stuff

if [[ "$SLAMPP_DATA" == "" ]]; then
	SLAMPP_DATA=../data
fi

if [[ ! -f "${SLAMPP_DATA}/venice871.g2o" ]]; then
	if [[ ! -f "../data/venice871.g2o" ]]; then
		>&2 echo "error: can't find ${SLAMPP_DATA}/venice871.g2o"
		exit -1
	else
		SLAMPP_DATA=../data
		# use this one after all
	fi
fi

if [[ `md5sum "${SLAMPP_DATA}/venice871.g2o" | awk '{ print $1; }'` != "54f3f9dd06e781f0c69abd6911967498" ]]; then
	>&2 echo "warning: MD5 sum of ${SLAMPP_DATA}/venice871.g2o does not match; maybe you have a different version of the file?"
else
	echo "debug: will use ${SLAMPP_DATA}/venice871.g2o as the dataset for testing"
fi

if [[ -f Makefile || -f makefile || -f MAKEFILE ]]; then
	make clean
else
	echo [nothing to clean]
fi

MB_PER_BUILD_THREAD=2560
kB_PER_BUILD_THREAD=$(( $MB_PER_BUILD_THREAD * 1024 ))
NUM_CPU_THREADS=`grep "core id" /proc/cpuinfo | wc -l`
MAKEARG=`grep MemTotal /proc/meminfo | awk -v kpt=$kB_PER_BUILD_THREAD -v mtr=$NUM_CPU_THREADS '{ ntr = 4; if($3 == "kB" || $3 == "kiB") { ntr = int($2 / kpt); } if($3 == "MB" || $3 == "MiB") { ntr = int($2 * 1024 / kpt); } if($3 == "B") { ntr = int($2 / 1024 / kpt); } if(ntr <= 1) print ""; else if(ntr > mtr) print "-j"; else print "-j" ntr; }'`
echo "debug: will use make $MAKEARG for building"

COMPUTER=`hostname`
# computer name

TSTAMP=`date +%Y-%m-%dT%H-%M-%S`
# get a timestamp for storing the partial results
# disable this if you need to rerun the script multiple times and compose the results

if [[ ! -f ../venice_${TSTAMP}_eig331_uv1_mnat1.txt ]]; then
	( cmake .. -DSLAM_P_P_EIGEN33=ON -DEIGEN_UNALIGNED_VECTORIZE=ON -DSLAM_P_P_USE_MARCH_NATIVE=ON 2>&1 ) | tee ../eig331_uv1_mnat1_${TSTAMP}_build.log
	( ( time make $MAKEARG ) 2>&1 ) | tee -a ../eig331_uv1_mnat1_${TSTAMP}_build.log ../eig331_uv1_mnat1_${TSTAMP}_build.log
	( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee ../venice_${TSTAMP}_eig331_uv1_mnat1.txt
	for (( i=1; i<$REPEATS; i++ )); do
		( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee -a ../venice_${TSTAMP}_eig331_uv1_mnat1.txt
	done
fi

if [[ ! -f ../venice_${TSTAMP}_eig331_uv0_mnat1.txt ]]; then
	( cmake .. -DSLAM_P_P_EIGEN33=ON -DEIGEN_UNALIGNED_VECTORIZE=OFF -DSLAM_P_P_USE_MARCH_NATIVE=ON 2>&1 ) | tee ../eig331_uv0_mnat1_${TSTAMP}_build.log
	( ( time make $MAKEARG ) 2>&1 ) | tee -a ../eig331_uv0_mnat1_${TSTAMP}_build.log
	( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee ../venice_${TSTAMP}_eig331_uv0_mnat1.txt
	for (( i=1; i<$REPEATS; i++ )); do
		( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee -a ../venice_${TSTAMP}_eig331_uv0_mnat1.txt
	done
fi

if [[ ! -f ../venice_${TSTAMP}_eig331_uv1_mnat0.txt ]]; then
	( cmake .. -DSLAM_P_P_EIGEN33=ON -DEIGEN_UNALIGNED_VECTORIZE=ON -DSLAM_P_P_USE_MARCH_NATIVE=OFF 2>&1 ) | tee ../eig331_uv1_mnat0_${TSTAMP}_build.log
	( ( time make $MAKEARG ) 2>&1 ) | tee -a ../eig331_uv1_mnat0_${TSTAMP}_build.log
	( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee ../venice_${TSTAMP}_eig331_uv1_mnat0.txt
	for (( i=1; i<$REPEATS; i++ )); do
		( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee -a ../venice_${TSTAMP}_eig331_uv1_mnat0.txt
	done
fi

if [[ ! -f ../venice_${TSTAMP}_eig331_uv0_mnat0.txt ]]; then
	( cmake .. -DSLAM_P_P_EIGEN33=ON -DEIGEN_UNALIGNED_VECTORIZE=OFF -DSLAM_P_P_USE_MARCH_NATIVE=OFF 2>&1 ) | tee ../eig331_uv0_mnat0_${TSTAMP}_build.log
	( ( time make $MAKEARG ) 2>&1 ) | tee -a ../eig331_uv0_mnat0_${TSTAMP}_build.log
	( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee ../venice_${TSTAMP}_eig331_uv0_mnat0.txt
	for (( i=1; i<$REPEATS; i++ )); do
		( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee -a ../venice_${TSTAMP}_eig331_uv0_mnat0.txt
	done
fi

if [[ ! -f ../venice_${TSTAMP}_eig330_uv0_mnat0.txt ]]; then
	( cmake .. -DSLAM_P_P_EIGEN33=OFF -DEIGEN_UNALIGNED_VECTORIZE=OFF -DSLAM_P_P_USE_MARCH_NATIVE=OFF 2>&1 ) | tee ../eig330_uv0_mnat0_${TSTAMP}_build.log
	( ( time make $MAKEARG ) 2>&1 ) | tee -a ../eig330_uv0_mnat0_${TSTAMP}_build.log
	( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee ../venice_${TSTAMP}_eig330_uv0_mnat0.txt
	for (( i=1; i<$REPEATS; i++ )); do
		( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/venice871.g2o" -us 2>&1 ) | tee -a ../venice_${TSTAMP}_eig330_uv0_mnat0.txt
	done
fi

echo done

cd ..

CPUN=`grep "model name" /proc/cpuinfo | head -n 1 | cut -d ":" -f 2 | sed s/^[[:space:]]*//`
compiler=`g++ --version | head -n 1`
T_eig331_uv1_mnat1=`grep "done. it took" venice_${TSTAMP}_eig331_uv1_mnat1.txt | sed s/[\(\)]//g | awk 'BEGIN { count = 0; } { if(!count || best > $5) best = $5; ++ count; } END { if(count) print best; }'`
T_eig331_uv0_mnat1=`grep "done. it took" venice_${TSTAMP}_eig331_uv0_mnat1.txt | sed s/[\(\)]//g | awk 'BEGIN { count = 0; } { if(!count || best > $5) best = $5; ++ count; } END { if(count) print best; }'`
T_eig331_uv1_mnat0=`grep "done. it took" venice_${TSTAMP}_eig331_uv1_mnat0.txt | sed s/[\(\)]//g | awk 'BEGIN { count = 0; } { if(!count || best > $5) best = $5; ++ count; } END { if(count) print best; }'`
T_eig331_uv0_mnat0=`grep "done. it took" venice_${TSTAMP}_eig331_uv0_mnat0.txt | sed s/[\(\)]//g | awk 'BEGIN { count = 0; } { if(!count || best > $5) best = $5; ++ count; } END { if(count) print best; }'`
T_eig330_uv0_mnat0=`grep "done. it took" venice_${TSTAMP}_eig330_uv0_mnat0.txt | sed s/[\(\)]//g | awk 'BEGIN { count = 0; } { if(!count || best > $5) best = $5; ++ count; } END { if(count) print best; }'`

BT_eig331_uv1_mnat1=`tail -n 10 eig331_uv1_mnat1_${TSTAMP}_build.log | grep user | awk '{ print $2; }' | tail -n 1`
BT_eig331_uv0_mnat1=`tail -n 10 eig331_uv0_mnat1_${TSTAMP}_build.log | grep user | awk '{ print $2; }' | tail -n 1`
BT_eig331_uv1_mnat0=`tail -n 10 eig331_uv1_mnat0_${TSTAMP}_build.log | grep user | awk '{ print $2; }' | tail -n 1`
BT_eig331_uv0_mnat0=`tail -n 10 eig331_uv0_mnat0_${TSTAMP}_build.log | grep user | awk '{ print $2; }' | tail -n 1`
BT_eig330_uv0_mnat0=`tail -n 10 eig330_uv0_mnat0_${TSTAMP}_build.log | grep user | awk '{ print $2; }' | tail -n 1`

echo "CPU,compiler,eig330_uv0_mnat0,eig331_uv1_mnat1,eig331_uv0_mnat1,eig331_uv1_mnat0,eig331_uv0_mnat0,summary,Benefit of Eigen 3.3,Benefit of AVX,Benefit of unaligned vectorization,Eigen 3.2,Eigen 3.3,Eigen 3.3 + AVX,bt_eig330_uv0_mnat0,bt_eig331_uv1_mnat1,bt_eig331_uv0_mnat1,bt_eig331_uv1_mnat0,bt_eig331_uv0_mnat0" > eigen_bench_${COMPUTER}_${TSTAMP}.csv
echo "${CPUN},${compiler},${T_eig330_uv0_mnat0},${T_eig331_uv1_mnat1},${T_eig331_uv0_mnat1},${T_eig331_uv1_mnat0},${T_eig331_uv0_mnat0},=IF(OR(I2<=0;J2<=0);\":)\";\":'(\"),=MIN(D2:G2)-C2,=MIN(D2:E2)-MIN(F2:G2),=MIN(D2;F2)-MIN(E2;G2),=C2,=MIN(F2:G2),=MIN(D2:E2),${BT_eig330_uv0_mnat0},${BT_eig331_uv1_mnat1},${BT_eig331_uv0_mnat1},${BT_eig331_uv1_mnat0},${BT_eig331_uv0_mnat0}" >> eigen_bench_${COMPUTER}_${TSTAMP}.csv
# this opens fine in openoffice; ms excel may require manual replacement of ";" by "," in the formulas, also depending on the language

echo "the results are in eigen_bench_${COMPUTER}_${TSTAMP}.csv"
