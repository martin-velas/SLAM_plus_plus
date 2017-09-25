#!/bin/bash

#rm -f src/slam_app/CMakeFiles/slam_plus_plus.dir/Solve3DImpl.o src/slam_app/CMakeFiles/slam_plus_plus.dir/Solve3DPoseOnlyImpl.o src/slam_app/CMakeFiles/slam_plus_plus.dir/SolveBAImpl.o src/slam_app/CMakeFiles/slam_plus_plus.dir/SolveBAIntrinsicsImpl.o src/slam_app/CMakeFiles/slam_plus_plus.dir/SolveBAStereoImpl.o src/slam_app/CMakeFiles/slam_plus_plus.dir/SolveSpheronImpl.o
#make slam_plus_plus
# need to rebuild?

verbose=0
robust_timing=0

default_path="../data/"
#default_path="$HOME/my-projects/SLAM_plus_plus/data/"

while [[ $# > 0 ]]; do
	if [[ "$1" == "-v" || "$1" == "--verbose" ]]; then
		verbose=1
	elif [[ "$1" == "-q" || "$1" == "--quiet" ]]; then
		verbose=0
	elif [[ "$1" == "-rt" || "$1" == "--robust-timing" ]]; then
		robust_timing=1
	elif [[ ( "$1" == "-i" || "$1" == "--data-path" ) && $# > 1 ]]; then
		shift
		data_path="$1"
	else
		>&2 echo "error: argument '$1': unkown argument or not enough parameters"
	fi
	#echo arg "'$1' ($# remain)"
	shift
done

if [[ -z "$data_path" ]]; then
	if [[ ! -z "$SLAMPP_DATA" ]]; then
		data_path="$SLAMPP_DATA"
	else
		data_path="$default_path"
	fi
fi

data_path=`echo $data_path | sed 's/\/*$//'`

if [[ $verbose != 0 ]]; then
	echo using "'${data_path}'" as input path
fi

sppflags=" -nb -s "
# common SLAM ++ flags - no bitmaps, silent

tn=0
# dataset / test number

datasets[$tn]="manhattanOlson3500.txt"
md5sums[$tn]="f9e150f35a47bbaf8433efd25eb32c4b"
command[$tn]=" -po "
chi2s[$tn]=146 # 146.08
iters[$tn]=5
notes[$tn]="ICRA13 time 0.0419 sec, chi2 6112.18";
tn=$(( $tn + 1 ))

datasets[$tn]="10K.graph"
md5sums[$tn]="df8fb5bb0f14ed0e0d51090e0a796cca"
command[$tn]=" -po "
chi2s[$tn]=303 # 303.17
iters[$tn]=5
tn=$(( $tn + 1 ))

datasets[$tn]="10KHOG-MAN_g2o.txt"
md5sums[$tn]="50c4b37aef5347f13ddb059a1d9c46df"
command[$tn]=" -po "
iters[$tn]=5
chi2s[$tn]=171545 # 171545.45
notes[$tn]="ICRA13 time 0.4852 sec, chi2 171545.45";
tn=$(( $tn + 1 ))

datasets[$tn]="city10k.txt"
md5sums[$tn]="0083509176db8328ee3b12ce77f3fc05"
command[$tn]=" -po "
chi2s[$tn]=31934 # 31934.00
iters[$tn]=5
notes[$tn]="ICRA13 time 0.4203, chi2 31931.41"
tn=$(( $tn + 1 ))

datasets[$tn]="cityTrees10k.txt"
md5sums[$tn]="144585687bb7e528ef8e4a89b042da95"
command[$tn]="  "
chi2s[$tn]=549 # 548.50
iters[$tn]=5
notes[$tn]="ICRA13 time 0.0916, chi2 548.50"
tn=$(( $tn + 1 ))

datasets[$tn]="intel.txt"
md5sums[$tn]="9fc7f05950e0a5b57ed2663a2c0f3db4"
command[$tn]=" -po "
chi2s[$tn]=17 # 17.06
iters[$tn]=3
notes[$tn]="ICRA13 time 0.0052 sec, chi2 559.05";
tn=$(( $tn + 1 ))

datasets[$tn]="killian-court.txt"
md5sums[$tn]="a1fee4f7afef823c888f0a8b3ea91ce7"
command[$tn]=" -po "
chi2s[$tn]=0 # 0.00
iters[$tn]=2
notes[$tn]="ICRA13 time 0.0070 sec, chi2 0.000005"
tn=$(( $tn + 1 ))

datasets[$tn]="manhattanOlson3500.txt"
md5sums[$tn]="f9e150f35a47bbaf8433efd25eb32c4b"
command[$tn]=" -nsp 1 -po "
chi2s[$tn]=202 # 201.54
iters[$tn]=1600
notes[$tn]="ICRA13 incremental time 10.0038"
tn=$(( $tn + 1 ))

datasets[$tn]="10kHog-man.txt"
md5sums[$tn]="50c4b37aef5347f13ddb059a1d9c46df"
command[$tn]="  -nsp 1 -po "
chi2s[$tn]=173915 # 173914.87
iters[$tn]=7122 # used to be 7124 with -ffast-math
notes[$tn]="ICRA13 incremental time 329.1840"
tn=$(( $tn + 1 ))

datasets[$tn]="city10k.txt"
md5sums[$tn]="0083509176db8328ee3b12ce77f3fc05"
command[$tn]="  -nsp 1 -po "
chi2s[$tn]=334352 # 334351.87
iters[$tn]=6297
notes[$tn]="ICRA13 incremental time 222.5930"
tn=$(( $tn + 1 ))

datasets[$tn]="cityTrees10k.txt"
md5sums[$tn]="144585687bb7e528ef8e4a89b042da95"
command[$tn]="  -nsp 1 "
chi2s[$tn]=382 # 382.43
iters[$tn]=1839
notes[$tn]="ICRA13 incremental time 22.7070"
tn=$(( $tn + 1 ))

datasets[$tn]="intel.txt"
md5sums[$tn]="9fc7f05950e0a5b57ed2663a2c0f3db4"
command[$tn]="  -nsp 1 -po "
chi2s[$tn]=509 # 509.29
iters[$tn]=506
notes[$tn]="ICRA13 incremental time 0.8424"
tn=$(( $tn + 1 ))

datasets[$tn]="killian-court.txt"
md5sums[$tn]="a1fee4f7afef823c888f0a8b3ea91ce7"
command[$tn]="  -nsp 1 -po "
chi2s[$tn]=0 # 0.01
iters[$tn]=728
notes[$tn]="ICRA13 incremental time 2.1485"
tn=$(( $tn + 1 ))

datasets[$tn]="victoria-park.txt"
md5sums[$tn]="b64e9f2eb9f322eea96a1354991e1de0"
command[$tn]=" -nsp 1 -s "
iters[$tn]=3424
chi2s[$tn]=140 # 140.09
notes[$tn]="ICRA13 incremental time 28.0194 sec";
tn=$(( $tn + 1 ))

datasets[$tn]="sphere2500.txt"
md5sums[$tn]="4a78a67f9ce065bde01e55590a3d5fed"
command[$tn]=" -po "
chi2s[$tn]=728 # 727.72
iters[$tn]=5
tn=$(( $tn + 1 ))

datasets[$tn]="parking-garage.txt"
md5sums[$tn]="1e8889f61e4f4e09451ebfdd921eb565"
command[$tn]=" -po "
chi2s[$tn]=1 # 1.46
iters[$tn]=5
tn=$(( $tn + 1 ))

datasets[$tn]="w100K_sort.txt"
md5sums[$tn]="c595eafbaeabe72185aa9ddfa42f471d"
command[$tn]=" -po "
chi2s[$tn]=8685 # 8685.07
iters[$tn]=5
notes[$tn]="ICRA13 time 9.2213 sec, chi2 8685.07";
tn=$(( $tn + 1 ))

datasets[$tn]="venice871.g2o"
md5sums[$tn]="54f3f9dd06e781f0c69abd6911967498"
command[$tn]=" -us "
chi2s[$tn]=233948939 # 233948938.86 # used to be 234013899 (234013899.18) before robust quaternion / axis angle conversions
iters[$tn]=5
tn=$(( $tn + 1 ))

datasets[$tn]="manhattanOlson3500.txt"
md5sums[$tn]="f9e150f35a47bbaf8433efd25eb32c4b"
command[$tn]="  -nsp 1 -po -fL "
chi2s[$tn]=147 # 146.95
iters[$tn]=1600
notes[$tn]="RSS13 incremental time 3.046, chi2 6119.83"
tn=$(( $tn + 1 ))

datasets[$tn]="10kHog-man.txt"
md5sums[$tn]="50c4b37aef5347f13ddb059a1d9c46df"
command[$tn]="  -nsp 1 -po -fL "
chi2s[$tn]=171586 # 171586.24
iters[$tn]=7122
notes[$tn]="RSS13 incremental time 79.651, chi2 171919"
tn=$(( $tn + 1 ))

datasets[$tn]="city10k.txt"
md5sums[$tn]="0083509176db8328ee3b12ce77f3fc05"
command[$tn]="  -nsp 1 -po -fL "
chi2s[$tn]=31948 # 31948.35
iters[$tn]=6297
notes[$tn]="RSS13 incremental time 53.951, chi2 31931.4"
tn=$(( $tn + 1 ))

datasets[$tn]="cityTrees10k.txt"
md5sums[$tn]="144585687bb7e528ef8e4a89b042da95"
command[$tn]="  -nsp 1 -fL "
chi2s[$tn]=318 # 318.43
iters[$tn]=1839
notes[$tn]="RSS13 incremental time 19.308, chi2 12062.6"
tn=$(( $tn + 1 ))

datasets[$tn]="sphere2500.txt"
md5sums[$tn]="4a78a67f9ce065bde01e55590a3d5fed"
command[$tn]="  -nsp 1 -fL -po "
chi2s[$tn]=733 # 732.68 # used to be 731 (730.89) with -ffast-math
iters[$tn]=2552 # used to be 2537 with -ffast-math
notes[$tn]="RSS13 incremental time 9.865, chi2 727.72"
tn=$(( $tn + 1 ))

datasets[$tn]="intel.txt"
md5sums[$tn]="9fc7f05950e0a5b57ed2663a2c0f3db4"
command[$tn]="  -nsp 1 -fL -po "
chi2s[$tn]=17 # 17.46
iters[$tn]=506
notes[$tn]="RSS13 incremental time 0.353, chi2 553.83"
tn=$(( $tn + 1 ))

datasets[$tn]="killian-court.txt"
md5sums[$tn]="a1fee4f7afef823c888f0a8b3ea91ce7"
command[$tn]="  -nsp 1 -fL -po "
chi2s[$tn]=0 # 0.00
iters[$tn]=728
notes[$tn]="RSS13 incremental time 1.045, chi2 0.00005"
tn=$(( $tn + 1 ))

datasets[$tn]="victoria-park.txt"
md5sums[$tn]="b64e9f2eb9f322eea96a1354991e1de0"
command[$tn]="  -nsp 1 -fL "
chi2s[$tn]=104 # 104.11
iters[$tn]=3424
notes[$tn]="RSS13 incremental time 11.202, chi2 144.91"
tn=$(( $tn + 1 ))

datasets[$tn]="parking-garage.txt"
md5sums[$tn]="1e8889f61e4f4e09451ebfdd921eb565"
command[$tn]="  -nsp 1 -fL -po "
chi2s[$tn]=1 # 1.28
iters[$tn]=1586 # used to be 1530 with -ffast-math
notes[$tn]="RSS13 incremental time 3.410, chi2 1.3106"
tn=$(( $tn + 1 ))

processed=0
problems=0
index=0
for ds in "${datasets[@]}"
do
	dsf="$data_path/$ds"
	if [ ! -f $dsf ]; then
		echo "warning: $dsf not found"
		index=$(( $index + 1 ))
		continue
	fi

	processed=$(( $processed + 1 ))

	ds_sum="`md5sum $dsf | cut -d ' ' -f 1`"
	stored_sum="${md5sums[$index]}"
	if [ $ds_sum != $stored_sum ]; then
		echo "$dsf - bad checksum - $ds_sum != $stored_sum"
	fi
	# make sure the dataset is the correct dataset

	rm -f slampp.log

	../bin/slam_plus_plus $sppflags -i $dsf ${command[$index]} > slampp.log 2>&1
	#cat slampp.log
	# run slam++

	runtime=`cat slampp.log | egrep "done\\. it took [0-9:\\.]+ \([0-9\\.]+ sec\)" | cut -d '(' -f 2 | cut -d ')' -f 1 | tail -n 1`
	num_iters=`cat slampp.log | egrep "solver took [0-9]+ iterations" | cut -d ' ' -f 3 | tail -n 1`
	chi2=`cat slampp.log | grep "denormalized chi2 error:" | cut -d ' ' -f 4 | tail -n 1`
	# get the number of iterations and chi2

	if [[ $robust_timing != 0 ]]; then
		times="";
		for i in `seq 1 10`; do
			times="$times `../bin/slam_plus_plus $sppflags -ndt -i $dsf ${command[$index]} 2>&1 | egrep "done\\. it took [0-9:\\.]+ \([0-9\\.]+ sec\)" | cut -d '(' -f 2 | cut -d ' ' -f 1 | tail -n 1`"
			# run SLAM ++ and concatenate the time to the variable, can run with --no-detailed-timing as we dont care right now

			time_sum=`echo $times | gawk '{ sum = 0; for(i = 1; i <= NF; ++ i) sum += $(i); print int(sum); }'`
			#echo $time_sum # debug
			if [[ "$i" -ge 4 && "$time_sum" -ge 10 ]]; then
				break
			fi
			# if there were at least four iterations and the sum of times is over 10 seconds then this is enough
		done
		# robust timing

		runtime=`echo $times | gawk '{ avg = 0; for(i = 1; i <= NF; ++ i) avg += $(i); avg /= NF; sddev = 0; for(i = 1; i <= NF; ++ i) sddev += ($(i) - avg) * ($(i) - avg); sddev = sqrt(sddev / NF); for(;;) { sorted = 1; for(i = 2; i <= NF; ++ i) { if($(i - 1) > $(i)) { sorted = 0; tmp = $(i - 1); $(i - 1) = $(i); $(i) = tmp; } } if(sorted) break; } print "avg " avg ", sd " sddev ", med " $(int((NF + 1) / 2)) " sec (" NF " smp)" }'`
		# digest runtime stats
	fi
	# robust timing

	chi2_round=`echo $chi2 | gawk '{ print int($0 + .5); }'`

	if [[ ( $verbose != 0 || $robust_timing != 0 ) && ! ( -z $num_iters || -z $chi2 ) ]]; then
		if [[ -z "${notes[$index]}" ]]; then
			echo "$ds: $num_iters iterations, chi2: $chi2 (ref ${chi2s[$index]}), time: $runtime"
		else
			echo "$ds: $num_iters iterations, chi2: $chi2 (ref ${chi2s[$index]}), time: $runtime, notes: '${notes[$index]}'"
		fi
	fi
	# verbose

	if [[ -z $num_iters || -z $chi2 ]]; then
		echo "$dsf - SLAM++ did not finish (`cat slampp.log | grep error | wc -l` error(s))"
		problems=$(( $problems + 1 ))
	else
		if [ $num_iters -ne ${iters[$index]} ]; then
			echo "$dsf - bad number of iterations - $num_iters != ${iters[$index]}"
			problems=$(( $problems + 1 ))
		fi
		if [ $chi2_round -ne ${chi2s[$index]} ]; then
			echo "$dsf - bad chi2 - $chi2 ($chi2_round) != ${chi2s[$index]}"
			problems=$(( $problems + 1 ))
		fi
	fi
	# make sure that both chi2 and the number of iterations do match the reference

	#echo "chi2s[$index]=$chi2_round # $chi2"
	#echo "iters[$index]=$num_iters"
	# print reference values

	rm -f slampp.log

	index=$(( $index + 1 ))
done

if [ $problems -eq 0 ]; then
	echo "no problems occurred, all $processed tests finished successfully"
else
	echo "finished; processed $processed datasets, there were $problems problem(s)"
fi
