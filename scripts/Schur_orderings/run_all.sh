#!/bin/sh

# use (./run_all.sh 2>&1) | tee alllog.txt

# 0 -eq 1 && # use this to disable below
if [[ -f "CMakeCache.txt" ]]; then
	if [[ `cat CMakeCache.txt | grep -E GPU_ENABLED:.*= | wc -l` -eq 1 ]]; then
		if [[ `cat CMakeCache.txt | grep -E GPU_ENABLED:.*= | cut -d = -f 2` == "ON" ]]; then
			>&2 echo "warning: GPU_ENABLED is ON: won't be able to run on ordinary nodes"

			cmake .. -DGPU_ENABLED=OFF
		fi
	else
		>&2 echo "warning: 'CMakeCache.txt' in unknown format: unable to make sure that this is not linked with libcuda.so"
	fi
else
	>&2 echo "warning: can't find 'CMakeCache.txt', unable to make sure that this is not linked with libcuda.so"
fi
# make sure it is built without GPU support for the first round of tests

BUILDLOG=`mktemp`
(make5 -j 2>&1) | tee "$BUILDLOG"

RESULT=$?
if [[ $RESULT -ne 0 || `grep -e "error:" "$BUILDLOG" | wc -l` -ne 0 ]]; then
	>&2 echo "warning: build not ok (result $RESULT, the last line says '`tail -n 1 "$BUILDLOG"`'), stopping tests"
	rm -f "$BUILDLOG"
	exit
fi
echo "debug: build ok (the last line says '`tail -n 1 "$BUILDLOG"`')"
rm -f "$BUILDLOG"
# build always, just to make sure that everything is up to date

#exit #debug

IS_SALOMON=0
if [[ `qstat | grep ".isrv" | wc -l` -gt 0 ]]; then
	IS_SALOMON=1
fi
# detect whether we are on Salomon (if we are, we need to alloc more cores per node in accordance with the job submission policy)

./run_SLAMs.sh
./run_BAs.sh
# run the first tests

waitJobs() {
	echo "waiting for the preceding jobs to finish"
	i=0
	while [[ $i -lt 5 ]]; do
		i=$(( $i + 1 ))
		something=0
		while [[ `qstat -u ipolok | grep ipolok | wc -l` -ne 0 ]]; do
			sleep 10
			something=1
			i=0 # jobs running again!
		done
		# wait for all to finish

		if [[ $something -eq 0 ]]; then
			if [[ $i -eq 1 ]]; then
				echo "    nothing"
			else
				echo "    still nothing"
			fi
		else
			echo "new jobs spawned; will wait some more"
		fi

		sleep 10
	done
	echo "the preceding jobs finished"
}

waitJobs
# wait for everything to finish, make sure nothing starts in a while

if [[ $IS_SALOMON -eq 0 ]]; then
	if [[ ! -f "CMakeCache.txt" || `cat CMakeCache.txt | grep -E GPU_ENABLED:.*= | wc -l` -ne 1 || `cat CMakeCache.txt | grep -E GPU_ENABLED:.*= | cut -d = -f 2` == "OFF" ]]; then
		>&2 echo "debug: setting GPU_ENABLED to ON"
		cmake .. -DGPU_ENABLED=ON
		# only enable if needed
	fi
	CWD=`pwd`
	BUILDLOG=`mktemp "--tmpdir=$CWD"` # the remote machine may not have temp in the same path
	free_GPU_submit.sh -jn build -fl module load cuda\; make5 -j 2\>\&1 \| tee "$BUILDLOG"

	RESULT=$?
	if [[ $RESULT -ne 0 || `grep -e "error:" "$BUILDLOG" | wc -l` -ne 0 ]]; then
		>&2 echo "warning: build not ok (result $RESULT, the last line says '`tail -n 1 "$BUILDLOG"`'), stopping tests"
		rm -f "$BUILDLOG"
		exit
	fi
	echo "debug: build ok (the last line says '`tail -n 1 "$BUILDLOG"`')"
	rm -f "$BUILDLOG"
fi
# make sure it is built with GPU support for the final round of tests

#exit #debug

./run_nested_SLAM_schur_analysis.sh
./run_nested_BA_schur_analysis.sh
# run the analyses, this runs on GPU on Anselm so that we have GPU cholesky results as well

waitJobs
# wait for everything to finish, make sure nothing starts in a while

./parse_nested_schur_analysis.sh SLAM_flops/*.OU | head -n 1 > SLAM_flops/results.txt
./parse_nested_schur_analysis.sh SLAM_flops/*.OU | gawk '{ printf("%-20s%-20s|", $2, $1); print $0; }' | tail -n +2 | sort | cut -d "|" -f 2 >> SLAM_flops/results.txt

./parse_nested_schur_analysis.sh -5l BA_flops/*.OU | head -n 1 > BA_flops/results.txt
./parse_nested_schur_analysis.sh -5l BA_flops/*.OU | gawk '{ printf("%-20s%-20s|", $2, $1); print $0; }' | tail -n +2 | sort | cut -d "|" -f 2 >> BA_flops/results.txt

(cd sparse_flops; time tgatopng.sh -j)
# convert images
