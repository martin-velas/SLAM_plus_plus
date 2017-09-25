#!/bin/sh

touch show_gmm_progress$1.last
mv show_gmm_progress$1.last show_gmm_progress$1.cmp
BLOCKMATLIST=`mktemp`
BLOCKMATS=0
QSTATS=`mktemp`
qstat -u ipolok > $QSTATS
for TOL in 20 50; do
	FILES=*${TOL}.log
	for f in $FILES; do
		LOGFILE_PROGRESS=`grep -E "analysing matrix [0-9]+" $f | tail -n 1 | awk '{ for(i = NF - 1; i > 1; -- i) { if(index($(i - 2), "analysing") != 0 && $(i - 1) == "matrix") { print $i; break; } } }' | sed s/://`
		BSS=`echo $f | sed s/^.*_bs// | sed s/n${TOL}\.log//`
		RUNNING=`grep GMM_${BSS}_${TOL} $QSTATS | awk 'BEGIN { n = 0; } { if(++ n == 1) { if($10 == "R") print $10 " (" $11 " / " $9 ")"; else print $10; } }'`
		LASTPRG=`cat show_gmm_progress$1.cmp | grep $f | cut -d " " -f 2`
		LASTMATS=`cat show_gmm_progress$1.cmp | grep $f | cut -d " " -f 4`
		find ../matrices_bs${BSS}n${TOL}/ -print | grep -E "\\.\\./matrices_bs[0-9]+n[0-9]+/.*\\.bla" | cut -d / -f 3 | sed s/\\.bla// | sort >> $BLOCKMATLIST
		BLOCKMATS1=`cat $BLOCKMATLIST | wc -l`
		BLOCKMATSH=$(( $BLOCKMATS1 - $BLOCKMATS ))
		BLOCKMATS=$BLOCKMATS1
		if [[ "$LASTPRG" != "" ]]; then
			NEWPRG=$(( $LOGFILE_PROGRESS - $LASTPRG ))
			NEWMATS=$(( $BLOCKMATSH - $LASTMATS ))
		else
			NEWPRG=$LOGFILE_PROGRESS
			NEWMATS=$BLOCKMATSH
		fi
		echo -e "$f \t$LOGFILE_PROGRESS \t+$NEWPRG \t$BLOCKMATSH \t+$NEWMATS \t$RUNNING"
	done | tee -a show_gmm_progress$1.last | awk '{ sum += $2; print $0; } END {print sum; }'
	BLOCKMATS=`cat $BLOCKMATLIST | wc -l` # the loop doesn't export its variables ourside, need to update the value in this scope
done
rm -f $QSTATS show_gmm_progress$1.cmp

#SPDLIST=`mktemp`
#./getmymatrices -i ../uflsmc -lmn _ --real --pos-def | tail -n +2 | awk '{for(i = 0; i < 100; ++ i) print}' | sort > $SPDLIST
# there can be up to number of $FILES duplicates (currently that's 32); should use awk to grep the matrices against the pos-def ones though

SPDREGEXP=`./getmymatrices -i ../uflsmc -lmn _ --real --pos-def | tail -n +2 | awk 'BEGIN { n = 0; } { printf((++ n == 1)? "%s" : "|%s", $1); }'`

BLOCKMATLISTS=$BLOCKMATLIST # grep doesnt need it sorted
#BLOCKMATLISTS=`mktemp`
#cat $BLOCKMATLIST | sort > $BLOCKMATLISTS

UNIQUEMATS=`cat $BLOCKMATLISTS | sort | uniq | wc -l`
SPDMATS=`grep -E "$SPDREGEXP" $BLOCKMATLISTS | wc -l`
SPDMATSU=`grep -E "$SPDREGEXP" $BLOCKMATLISTS | sort | uniq | wc -l`

#SPDMATS=`comm -12 $BLOCKMATLISTS $SPDLIST | wc -l`
#SPDMATSU=`comm -12 $BLOCKMATLISTS $SPDLIST | sort | uniq | wc -l`

echo "found $BLOCKMATS block matrices ($SPDMATS pos-def, $UNIQUEMATS unique, $SPDMATSU unique pos-def)"
rm -f $BLOCKMATLIST
#rm -f $BLOCKMATLISTS
rm -f $SPDLIST
