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

		if [[ -f FLOPs_${BENCH}_${name}.log ]]; then
			rm FLOPs_${BENCH}_${name}.log
		fi
		# make sure the logs are removed and not reused from the last time

		if [[ "$COPY_ASIDE" == "1" ]]; then
			pbs_submit.sh -jt 10 -jn "F${name}" -nw \( mkdir -p $name \&\& cp "$ROOT/$f" $name/system.mtx \&\& cp "$ROOT/$blaf" $name/system.bla \&\& echo $name \&\& ../data/BA_sys/blanalyze.sh $name/system.bla \&\& ../bin/ba_margs_example_cflops $name $OPTION -cFLOPs \&\& echo ---- \&\& rm $name/system.mtx $name/system.bla \&\& rmdir $name \) \> FLOPs_${BENCH}_${name}.log
		else
			srcpath="$ROOT/$f"
			pbs_submit.sh -jt 10 -jn "F${name}" -nw \( echo $name \&\& ../data/BA_sys/blanalyze.sh $srcpath/system.bla \&\& ../bin/ba_margs_example_cflops $srcpath $OPTION -cFLOPs \&\& echo ---- \) \> FLOPs_${BENCH}_${name}.log
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

if [[ -f hpc17_FLOPs-all.txt ]]; then
	rm hpc17_FLOPs-all.txt
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

	LOGFILE=FLOPs_${BENCH}_log_all.txt

	if [[ -f $LOGFILE ]]; then
		rm $LOGFILE
	fi
	# remove the logfile

	for f in $FILES; do
		blaf=`echo $f | sed "s/\\.mtx$/.bla/g"`
		name=`echo $f | sed "s/\\.mtx$//g" | sed "s/^.*\\///"`

		if [[ ! -f FLOPs_${BENCH}_${name}.log ]]; then
			>&2 echo "error: FLOPs_${BENCH}_${name}.log not found (benchmark job deleted or timed out?)"
			continue
		fi
		# see if the logfile exists at all

		cat FLOPs_${BENCH}_${name}.log >> $LOGFILE
		if [[ `tail -n 1 FLOPs_${BENCH}_${name}.log` != "----" ]]; then
			echo ---- >> $LOGFILE
		fi
		#rm ${BENCH}_${name}.log
		mv FLOPs_${BENCH}_${name}.log FLOPs_${BENCH}_${TSTAMP}_${name}.log # keep the logs

		if [[ "$COPY_ASIDE" == "1" && -d $name ]]; then # if the folder still exists
			rm $name/system.mtx $name/system.bla 2> /dev/null
			rmdir $name 2> /dev/null
		fi
		# remove in case it did not finish, ignore errors

	done

	./hpc17_parseFLOPsbench.sh $LOGFILE $BENCH > FLOPs_${BENCH}_log_all.csv
	cp FLOPs_${BENCH}_log_all.csv FLOPs_${BENCH}_log_all_${TSTAMP}.csv

	cat $LOGFILE >> hpc17_FLOPs-all.txt
	cat FLOPs_${BENCH}_log_all.csv >> hpc17_FLOPs-all.csv
	#concat all the logfiles
done

cat hpc17_FLOPs-all.csv
cp hpc17_FLOPs-all.csv hpc17_FLOPs-all_${TSTAMP}.csv
cp hpc17_FLOPs-all.txt hpc17_FLOPs-all_${TSTAMP}.txt # the source too

LOGFILES=`ls *${TSTAMP}* | grep -v hpc17_FLOPs-all`
if [[ `echo $LOGFILES | wc -w` -ne 0 ]]; then
	if [[ ! -d hpc17_archived_flops ]]; then
		mkdir hpc17_archived_flops
	fi
	mv -t hpc17_archived_flops $LOGFILES
fi
# move all the partial logfiles in the archive directory
