#!/bin/sh

# the first argument is always the input file name
# the following arguments are passed to SLAM ++ as is
# this will then print a shell source code with this
# run as a reference for unit test.

if [[ $# -eq 0 ]]; then
	>&2 echo "ettot: need to supply input file name"
	exit
fi

infile="$1"
# the first option must be the input file name

opts=""
shift
while [[ $# -ge 1 ]]; do
	opts="$opts $1"
	shift
done
# concatenate additional options

dp=$(basename "$infile")

sppflags=" -nb -s "
# common SLAM ++ flags - no bitmaps, silent

echo "../bin/slam_plus_plus $sppflags -i $infile $opts"
../bin/slam_plus_plus $sppflags -i $infile $opts > slampp.log 2>&1
# run, save to a log

runtime=`cat slampp.log | egrep "done\\. it took [0-9:\\.]+ \([0-9\\.]+ sec\)" | cut -d '(' -f 2 | cut -d ')' -f 1 | tail -n 1`
num_iters=`cat slampp.log | egrep "solver took [0-9]+ iterations" | cut -d ' ' -f 3 | tail -n 1`
chi2=`cat slampp.log | grep "denormalized chi2 error:" | cut -d ' ' -f 4 | tail -n 1`
# get the number of iterations and chi2

chi2_round=`echo $chi2 | gawk '{ print int($0 + .5); }'`

cat slampp.log
# show the log to the user

rm -f slampp.log

echo '----------'
echo 

echo 'datasets[$tn]="'${dp}'"'
echo 'md5sums[$tn]="'`md5sum "${infile}" | cut -d " " -f 1`'"'
echo 'command[$tn]=" '${opts}' "'
echo 'chi2s[$tn]='${chi2_round}' # '${chi2}
echo 'iters[$tn]='$num_iters
echo 'notes[$tn]=""'
echo 'tn=$(( $tn + 1 ))'
# copy paste this to unit tests
