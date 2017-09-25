#!/bin/bash

NUMMATS=$(( `./getmymatrices -i ../uflsmc/ -lpt | grep "filtering finished. selected" | cut -d " " -f 4` - 1 ))

for tol in 20 50; do
	for BS in 2 3 4 5 6 7 8 16 2,3 2,4 3,4 3,6 3,7 5,6 5,7 5,8; do
		BSS=`echo $BS | sed s/,//g`
		#echo $BS $BSS;

		LOGFILE="matrices_bs${BSS}n${tol}.log"
		DESTDIR="../matrices_bs${BSS}n${tol}"

		touch $LOGFILE
		if [[ `grep -E "analysing matrix [0-9]+" $LOGFILE | wc -l` -ne 0 ]]; then
			PROCESSED=$((`grep -E "analysing matrix [0-9]+" $LOGFILE | tail -n 1 | awk '{ for(i = NF - 1; i > 1; -- i) { if(index($(i - 2), "analysing") != 0 && $(i - 1) == "matrix") { print $i; break; } } }' | sed s/://g` - 1))
		else
			PROCESSED=0
		fi

		if [[ $NUMMATS -eq $PROCESSED ]]; then
			echo "job -bs $BS already processed all $PROCESSED matrices; the results are in $DESTDIR, the log is in $LOGFILE"
		else 
			echo "will do -bs $BS starting with matrix $PROCESSED to $DESTDIR with log $LOGFILE"
			free_submit.sh -nw -jn GMM_${BSS}_${tol} -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory $DESTDIR -bs ${BS}:${tol} -sf $PROCESSED \| tee -a $LOGFILE
		fi
	done
done

exit

tol=20; BS=3,4; BSS=`echo $BS | sed s/,//g`; LOGFILE="matrices_bs${BSS}n${tol}.log"; DESTDIR="../matrices_bs${BSS}n${tol}"; if [[ -f $LOGFILE ]]; then PROCESSED=$((`grep -E "analysing matrix [0-9]+" $LOGFILE | tail -n 1 | awk '{ for(i = NF - 1; i > 1; -- i) { if(index($(i - 2), "analysing") != 0 && $(i - 1) == "matrix") { print $i; break; } } }' | sed s/://g` - 1)); else PROCESSED=0; fi; ./getmymatrices -i ../uflsmc/ --real -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory $DESTDIR -bs ${BS}:${tol} -sf $PROCESSED | tee -a $LOGFILE
# one-liner for just one size

for i in {1..25}; do echo "*** this is run $i ***"; ./show_gmm_progress.sh main; pbs_follow_my_queue.sh 60; ./resubmit_bsn.sh; done | tee -a globlog.txt
# driver to repeatedly submit until everything is processed

# below is madness

touch $LOGFILE
if [[ `grep -E "$analysing matrix [0-9]+" $LOGFILE | wc -l` -ne 0 ]]; then
	PROCESSED=$((`grep -E "$analysing matrix [0-9]+" $LOGFILE | tail -n 1 | awk '{ for(i = NF - 1; i > 1; -- i) { if(index($(i - 2), "analysing") != 0 && $(i - 1) == "matrix") { print $i; break; } } }' | sed s/://g` - 1))
else
	PROCESSED=0
fi
echo "will do -bs $BS starting with matrix $PROCESSED to $DESTDIR with log $LOGFILE"
./getmymatrices -i ../uflsmc/ --real -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory $DESTDIR -bs ${BS}:${tol} -sf $PROCESSED | tee -a $LOGFILE

pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs2n50 -bs 2:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs2n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs2n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs3n50 -bs 3:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs3n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs3n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs4n50 -bs 4:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs4n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs4n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs5n50 -bs 5:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs5n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs5n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs6n50 -bs 6:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs6n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs6n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs7n50 -bs 7:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs7n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs7n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs8n50 -bs 8:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs8n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs8n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs16n50 -bs 16:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs16n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \| tee -a matrices_bs16n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs36n50 -bs 3,6:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs36n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \| tee -a matrices_bs36n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs37n50 -bs 3,7:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs37n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \| tee -a matrices_bs37n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs24n50 -bs 2,4:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs24n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \| tee -a matrices_bs24n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs34n50 -bs 3,4:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs34n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \| tee -a matrices_bs34n50.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs56n50 -bs 5,6:50 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs56n50.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \| tee -a matrices_bs56n50.log

pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs2n20 -bs 2:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs2n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs2n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs3n20 -bs 3:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs3n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs3n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs4n20 -bs 4:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs4n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs4n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs5n20 -bs 5:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs5n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs5n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs6n20 -bs 6:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs6n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs6n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs7n20 -bs 7:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs7n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs7n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs8n20 -bs 8:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs8n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  tee -a matrices_bs8n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs16n20 -bs 16:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs16n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \| tee -a matrices_bs16n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs36n20 -bs 3,6:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs36n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  \| tee -a matrices_bs36n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs37n20 -bs 3,7:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs37n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  \| tee -a matrices_bs37n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs24n20 -bs 2,4:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs24n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  \| tee -a matrices_bs24n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs34n20 -bs 3,4:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs34n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  \| tee -a matrices_bs34n20.log
pbs_submit.sh -nw -jn GMM_sbsn -jq qlong -jt 12:00:00 -- ./getmymatrices -i ../uflsmc/ --real --temp /lscratch/\$PBS_JOBID -blaf ../uflsmc_clean_blocks_exact_fixfixed --dest-directory ../matrices_bs56n20 -bs 5,6:20 -sf $((`grep -E "$analysing matrix [0-9]+" matrices_bs56n20.log | tail -n 1 | awk '{ print $4; }' \| sed s/://g` - 1)) \|  \| tee -a matrices_bs56n20.log

