#!/bin/sh

ROOTS="../data/LU/LU_bench_SLAM,0,bs_slam,0 ../data/LU/LU_bench_BA_SC,0,bs_ba,1 ../data/LU/LU_bench_lg_bs3,1,bs3,0 ../data/LU/LU_bench_lg_bs4,1,bs4,0 ../data/LU/LU_bench_lg_bs5,1,bs5,0 ../data/LU/LU_bench_lg_bs6,1,bs6,0"
# quadruplets of root, copy aside, benchmark designation, use Schur complement

for DATASET in $ROOTS; do
	ROOT=`echo $DATASET | cut -d , -f 1`
	COPY_ASIDE=`echo $DATASET | cut -d , -f 2`
	BENCH=`echo $DATASET | cut -d , -f 3`
	SCHUR_COMPL=`echo $DATASET | cut -d , -f 4`
	echo "root: $ROOT, copy aside: $COPY_ASIDE, bench designation: $BENCH, Schur: $SCHUR_COMPL"

	FILES=`cat "$ROOT/list.txt" | sed 's/\r//'`

	if [[ "$SCHUR_COMPL" == "1" ]]; then
		OPTION="-tSCLUo"
	else
		OPTION="-tLUo"
	fi
	# option for the benchmark

	for f in $FILES; do
		blaf=`echo $f | sed "s/\\.mtx$/.bla/g"`
		name=`echo $f | sed "s/\\.mtx$//g" | sed "s/^.*\\///"`

		#echo $f $blaf $name # test

		if [[ `qstat -u ipolok | grep ipolok | wc -l` -ge 95 ]]; then
			echo "too many jobs submitted, need to wait a bit"
			while [[ `qstat -u ipolok | grep ipolok | wc -l` -ge 95 ]]; do
				echo -n .
				sleep 10
			done
			echo -e -n "\ndone waiting\n"
		fi
		# wait for some jobs to finish if we submitted too many of them (job limit specific to IT4I)

		if [[ -f ${BENCH}_${name}.log ]]; then
			rm ${BENCH}_${name}.log
		fi
		# make sure the logs are removed and not reused from the last time

		if [[ "$COPY_ASIDE" == "1" ]]; then
			JOBIDFILE=`mktemp`
			pbs_submit.sh -jt 10 -jn "1${name}" -nw \( mkdir -p $name \&\& cp "$ROOT/$f" $name/system.mtx \&\& cp "$ROOT/$blaf" $name/system.bla \&\& echo $name \&\& ../data/BA_sys/blanalyze.sh $name/system.bla \&\& ../bin/ba_margs_example $name $OPTION \) \> ${BENCH}_${name}.log | tee $JOBIDFILE
			LASTJOBID=`tail -n 1 $JOBIDFILE`
			pbs_submit.sh -jt 10 -jn "2${name}" -nw -wf $LASTJOBID \( ../bin/ba_margs_example $name $OPTION \) \>\> ${BENCH}_${name}.log | tee $JOBIDFILE
			LASTJOBID=`tail -n 1 $JOBIDFILE`
			pbs_submit.sh -jt 10 -jn "3${name}" -nw -wf $LASTJOBID \( ../bin/ba_margs_example $name $OPTION \&\& echo ---- \&\& rm $name/system.mtx $name/system.bla \&\& rmdir $name \) \>\> ${BENCH}_${name}.log
			rm $JOBIDFILE
			# submit as three jobs (could repeat the middle segment up to N times), make them depend on one another to enforce ordering in output to the file, the last job deletes the matrices
			# (could run them with output to several files and merge that after-the-fact, this is more of an experiment with job dependencies)

		else
			srcpath="$ROOT/$f"
			JOBIDFILE=`mktemp`
			pbs_submit.sh -jt 10 -jn "1${name}" -nw \( echo $name \&\& ../data/BA_sys/blanalyze.sh $srcpath/system.bla \&\& ../bin/ba_margs_example $srcpath $OPTION \) \> ${BENCH}_${name}.log | tee $JOBIDFILE
			LASTJOBID=`tail -n 1 $JOBIDFILE`
			pbs_submit.sh -jt 10 -jn "2${name}" -nw -wf $LASTJOBID \( ../bin/ba_margs_example $srcpath $OPTION \) \>\> ${BENCH}_${name}.log | tee $JOBIDFILE
			LASTJOBID=`tail -n 1 $JOBIDFILE`
			pbs_submit.sh -jt 10 -jn "3${name}" -nw -wf $LASTJOBID \( ../bin/ba_margs_example $srcpath $OPTION \&\& echo ---- \) \>\> ${BENCH}_${name}.log
			rm $JOBIDFILE
			# submit as three jobs (could repeat the middle segment up to N times), make them depend on one another to enforce ordering in output to the file
			# (could run them with output to several files and merge that after-the-fact, this is more of an experiment with job dependencies)

		fi
		# submit this job, depending on how the files are prepared (already prepared / have to be copied aside and deleted after the benchmark)
		# could use symlinks rather than copy
	done
done
# go through all the roots, all the datasets and submit the jobs

pbs_follow_my_queue.sh
# wait for the jobs to finish

TSTAMP=`date +%Y-%m-%dT%H-%M-%S`
# get a timestamp for storing the partial results

if [[ -f hpc17_results-all.txt ]]; then
	rm hpc17_results-all.txt
fi

echo "type,matrix,rows,cols,nnz,CS time,CS time,CS time,CS nnz,CS error,LUB time,LUB time,LUB time,LUB nnz,LUB error" > hpc17_results-all.csv
# add a header in the global log (dirty, should be done by parselog.sh)

for DATASET in $ROOTS; do
	ROOT=`echo $DATASET | cut -d , -f 1`
	COPY_ASIDE=`echo $DATASET | cut -d , -f 2`
	BENCH=`echo $DATASET | cut -d , -f 3`
	SCHUR_COMPL=`echo $DATASET | cut -d , -f 4`
	#echo "root: $ROOT, copy aside: $COPY_ASIDE, bench designation: $BENCH, Schur: $SCHUR_COMPL"

	FILES=`cat "$ROOT/list.txt" | sed 's/\r//'`

	LOGFILE=${BENCH}_log_all.txt

	if [[ -f $LOGFILE ]]; then
		rm $LOGFILE
	fi
	# remove the logfile

	for f in $FILES; do
		blaf=`echo $f | sed "s/\\.mtx$/.bla/g"`
		name=`echo $f | sed "s/\\.mtx$//g" | sed "s/^.*\\///"`

		if [[ ! -f ${BENCH}_${name}.log ]]; then
			>&2 echo "error: ${BENCH}_${name}.log not found (benchmark job deleted or timed out?)"
			continue
		fi
		# see if the logfile exists at all

		cat ${BENCH}_${name}.log >> $LOGFILE
		if [[ `tail -n 1 ${BENCH}_${name}.log` != "----" ]]; then
			echo ---- >> $LOGFILE
		fi
		#rm ${BENCH}_${name}.log
		mv ${BENCH}_${name}.log ${BENCH}_${TSTAMP}_${name}.log # keep the logs

		if [[ "$COPY_ASIDE" == "1" && -d $name ]]; then # if the folder still exists
			rm $name/system.mtx $name/system.bla 2> /dev/null
			rmdir $name 2> /dev/null
		fi
		# remove in case it did not finish, ignore errors

	done

	./hpc17_parsebench.sh $LOGFILE $BENCH > ${BENCH}_log_all.csv
	cp ${BENCH}_log_all.csv ${BENCH}_log_all_${TSTAMP}.csv

	cat $LOGFILE >> hpc17_results-all.txt
	cat ${BENCH}_log_all.csv >> hpc17_results-all.csv
	#concat all the logfiles
done

#./hpc17_parsebench.sh hpc17_results-all.txt | tee hpc17_results-all.csv # concatenated from the individual logs, while keeping the information about benchmark type
cat hpc17_results-all.csv
cp hpc17_results-all.csv hpc17_results-all_${TSTAMP}.csv
cp hpc17_results-all.txt hpc17_results-all_${TSTAMP}.txt # the source too

LOGFILES=`ls *${TSTAMP}* | grep -v hpc17_results-all`
if [[ `echo $LOGFILES | wc -w` -ne 0 ]]; then
	if [[ ! -d hpc17_archived_results ]]; then
		mkdir hpc17_archived_results
	fi
	mv -t hpc17_archived_results $LOGFILES
fi
# move all the partial logfiles in the archive directory
