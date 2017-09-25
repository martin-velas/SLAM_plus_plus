#!/bin/bash

USE_SECOND_SCREEN=0
which infocmp 1>/dev/null 2>/dev/null
if [[ $? -eq 0 ]]; then
	if [[ `infocmp -1 | grep -E "smcup|rmcup" | wc -l` == "2" ]]; then
		USE_SECOND_SCREEN=1
		echo "debug: will use second screen capabilities of your terminal"
	fi
fi
# detect second screen caps of the terminal

if [[ ! -d src || ! -d include || ! -f CMakeLists.txt ]]; then
	>&2 echo "this is supposed to be run from SLAM++ root folder"
	exit -1
fi

if [[ ! -d build_buildtests ]]; then
	mkdir build_buildtests
	if [[ $? -ne 0 ]]; then
		>&2 echo "error: failed to mkdir build_buildtests"
		exit -1
	fi
fi
cd build_buildtests
if [[ $? -ne 0 ]]; then
	>&2 echo "error: failed to cd build_buildtests"
	exit -1
fi
# make very sure that we're in the folder before deleting stuff

if [[ "$SLAMPP_DATA" == "" ]]; then
	SLAMPP_DATA=../data
fi

if [[ ! -f "${SLAMPP_DATA}/cathedral_inc_sorted.txt" ]]; then
	if [[ ! -f "../data/cathedral_inc_sorted.txt" ]]; then
		>&2 echo "error: can't find ${SLAMPP_DATA}/cathedral_inc_sorted.txt"
		exit -1
	else
		SLAMPP_DATA=../data
		# use this one after all
	fi
fi

if [[ `md5sum "${SLAMPP_DATA}/cathedral_inc_sorted.txt" | awk '{ print $1; }'` != "d773839367b530d2b0ae269c9fbe764c" ]]; then
	>&2 echo "warning: MD5 sum of ${SLAMPP_DATA}/cathedral_inc_sorted.txt does not match; maybe you have a different version of the file?"
else
	echo "debug: will use ${SLAMPP_DATA}/cathedral_inc_sorted.txt as the dataset for testing"
fi

MB_PER_BUILD_THREAD=2560
kB_PER_BUILD_THREAD=$(( $MB_PER_BUILD_THREAD * 1024 ))
NUM_CPU_THREADS=`grep "core id" /proc/cpuinfo | wc -l`
MAKEARG=`grep MemTotal /proc/meminfo | awk -v kpt=$kB_PER_BUILD_THREAD -v mtr=$NUM_CPU_THREADS '{ ntr = 4; if($3 == "kB" || $3 == "kiB") { ntr = int($2 / kpt); } if($3 == "MB" || $3 == "MiB") { ntr = int($2 * 1024 / kpt); } if($3 == "B") { ntr = int($2 / 1024 / kpt); } if(ntr <= 1) print ""; else if(ntr > mtr) print "-j"; else print "-j" ntr; }'`
echo "debug: will use make $MAKEARG for building"

COMPUTER=`hostname`

ON_ONLY="ON"
OFF_ONLY="OFF"
BOOL_OPT="OFF ON"
DBG_REL="Release Debug"

rm -f CMakeCache.txt

GR='\033[0;32m'
RD='\033[0;31m'
NC='\033[0m'

TSTAMP=`date +%Y-%m-%dT%H-%M-%S`
# get a timestamp for storing the partial results

FLAWLESS_CONFIGS=0
BUGGY_CONFIGS=0
for BTYPE in $DBG_REL; do
	for XTARGETS in $BOOL_OPT; do
		for EIG33 in $BOOL_OPT; do
			if [[ "$EIG33" == "ON" ]]; then
				EIG33_EXTOPTS="$BOOL_OPT"
			else
				EIG33_EXTOPTS="$OFF_ONLY"
			fi

			for UNVEC in $EIG33_EXTOPTS; do
				CONFIG="-DSLAM_P_P_EIGEN33=$EIG33 -DEIGEN_UNALIGNED_VECTORIZE=$UNVEC -DCMAKE_BUILD_TYPE=$BTYPE -DSLAM_P_P_BUILD_TOMS_EXAMPLE=$XTARGETS -DSLAM_P_P_MAIN_APP_PERF_TESTS=$XTARGETS -DSLAM_P_P_MAIN_APP_UNIT_TESTS=$XTARGETS"
				echo -e "configure with '${GR}${CONFIG}${NC}'"

				CONFIG_N=`echo $CONFIG | sed "s/-D//g" | sed "s/SLAM_P_P_EIGEN33/eig33/g" | sed "s/EIGEN_UNALIGNED_VECTORIZE/eig_uv/g" | sed "s/CMAKE_BUILD_TYPE/btype/g" | sed "s/SLAM_P_P_BUILD_TOMS_EXAMPLE/buildTOMS/g" | sed "s/SLAM_P_P_MAIN_APP_PERF_TESTS/withPerf/g" | sed "s/SLAM_P_P_MAIN_APP_UNIT_TESTS/withTests/g" | sed "s/[ ?:\\/]/_/g"`
				CONFIG_N="_run_${TSTAMP}_$CONFIG_N"

				if [[ -f Makefile || -f makefile || -f MAKEFILE ]]; then
					make clean
				else
					echo [nothing to clean]
				fi
				( cmake .. --no-warn-unused-cli $CONFIG 2>&1 ) | tee ${CONFIG_N}.cmake.log
				if [[ $USE_SECOND_SCREEN -eq 1 ]]; then
					tput smcup # switch to alt-screen
				fi
				( make $MAKEARG 2>&1 ) | tee ${CONFIG_N}.make.log
				if [[ $USE_SECOND_SCREEN -eq 1 ]]; then
					tput rmcup # restore screen
					echo "[build log omitted; see ${CONFIG_N}.make.log]"
				fi
				( ../bin/slam_plus_plus -i "${SLAMPP_DATA}/cathedral_inc_sorted.txt" -us 2>&1 ) | tee ${CONFIG_N}.run.log

				grep -E "error|fault|fail" ${CONFIG_N}.cmake.log | grep -v "default" | grep -v "chi2 error" | grep -v "/cholmod_error.o" > ${CONFIG_N}_errors.txt
				grep -E "error|fault|fail" ${CONFIG_N}.make.log | grep -v "default" | grep -v "chi2 error" | grep -v "/cholmod_error.o" >> ${CONFIG_N}_errors.txt
				grep -E "error|fault|fail" ${CONFIG_N}.run.log | grep -v "default" | grep -v "chi2 error" | grep -v "/cholmod_error.o" >> ${CONFIG_N}_errors.txt

				if [[ `grep "done. it took" ${CONFIG_N}.run.log | wc -l` == "0" ]]; then
					echo "error: the run log does not contain 'done'" >> ${CONFIG_N}_errors.txt
				fi
				if [[ `grep "solver took 5 iterations" ${CONFIG_N}.run.log | wc -l` == "0" ]]; then
					echo "error: the run log does not contain 'solver took 5 iterations'" >> ${CONFIG_N}_errors.txt
				fi
				if [[ `grep "denormalized chi2 error:" ${CONFIG_N}.run.log | tail -n 1 | awk '{ err = ($4 - 3372728.30) / 3372728.30 * 100; if(err > 1 || err < -1) print "0"; else print "1"; }'` != "1" ]]; then
					echo "error: the final chi2 is different than expected (3372728.30)" >> ${CONFIG_N}_errors.txt
				fi
				# detect what is expected in a valid result log

				if [[ `cat ${CONFIG_N}_errors.txt | wc -l` -eq 0 ]]; then
					FLAWLESS_CONFIGS=$(( $FLAWLESS_CONFIGS + 1 ))
					echo -e "${GR}PASS (so far ${FLAWLESS_CONFIGS} passed (including this one) and ${BUGGY_CONFIGS} failed)${NC}"
				else
					BUGGY_CONFIGS=$(( $BUGGY_CONFIGS + 1 ))
					echo -e "${RD}FAIL (so far ${FLAWLESS_CONFIGS} passed and ${BUGGY_CONFIGS} failed (including this one))${NC}"
				fi
			done
			# loop unaligned vectorization
		done
		# loop eigen versions
	done
	# loop extra targets (TOMS, unit tests, perf tests)
done
# loop debug / release

echo ""
echo -e "done. there were ${RD}${BUGGY_CONFIGS}${NC} buggy configs and ${GR}${FLAWLESS_CONFIGS}${NC} good configs"
