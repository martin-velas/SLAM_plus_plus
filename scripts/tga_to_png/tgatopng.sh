#!/bin/bash

# t_odo get temp file, store ok/not ok flags, then reduce using awk
#NCONV=0
#NERR=0
LOGFILE=`mktemp`

control_c() # run if user hits control-c
{
	NCONV=`cat "$LOGFILE" | grep -e 1 | wc -l`
	NERR=`cat "$LOGFILE" | grep -e 0 | wc -l`
	rm -f "$LOGFILE"
	echo "'`pwd`': interrupted; converted $NCONV images, had $NERR problems"
	exit $?
}

trap control_c SIGINT

FROMFORMAT=tga
TOFORMAT=png

if [[ $# -eq 1 && "$1" == "-j" ]]; then
	if [[ `uname | grep BSD | wc -w` -ne 0 ]]; then
		NCPUS=`sysctl -a | egrep -i 'hw.ncpu' | cut -d " " -f 2` # BSD
	else
		NCPUS=`cat /proc/cpuinfo | grep processor | wc -l` # linux
	fi
	if [[ $NCPUS -eq 0 ]]; then NCPUS=16; fi # fallback
	echo "running in parallel ${NCPUS}x"

	export LOGFILE
	export TOFORMAT
	function myConvertTga {
		if [[ -f "$1" ]]; then
			mogrify -format $TOFORMAT "$1"
			if [[ $? -eq 0 ]]; then
				echo 1 >> "$LOGFILE"
				rm -f "$1"
				echo "converted '$1' in parallel"
			else
				>&2 echo "error converting '$1'"
				echo 0 >> "$LOGFILE"
			fi
		fi
	}
	export -f myConvertTga

	#MYNAME=`hostname -s`
	#MYIPS=`hostname -I | gawk '{ if(++ n < 2) printf("" $1); else printf("|" $1); }'`
	#IS_ANSELM=`ping -c 1 $MYNAME.anselm.it4i.cz | grep -E "$MYIPS" | wc -l`
	#IS_SALOMON=`ping -c 1 $MYNAME.anselm.it4i.cz | grep -E "$MYIPS" | wc -l`
	# wacky detection of it4i machines

	if [[ ! `which parallel 2> /dev/null` ]]; then
		echo loading module parallel
		module load parallel
	fi
	# try to load the parallel module if it cant be seen

	find `pwd` -name '*.'$FROMFORMAT -print0 | parallel -j$NCPUS -0 myConvertTga {}
	# GNU parallel

else

	find `pwd` -name '*.'$FROMFORMAT -print0 |
		while IFS= read -r -d $'\0' f; do
			if [[ -f "$f" ]]; then
				mogrify -format $TOFORMAT "$f"
				if [[ $? -eq 0 ]]; then
					#NCONV=$(( $NCONV + 1 )) // # can't, runs in a different process
					echo 1 >> "$LOGFILE"
					rm "$f"
					echo "converted '$f'"
				else
					>&2 echo "error converting '$f'"
					#NERR=$(( $NERR + 1 )) // # can't, runs in a different process
					echo 0 >> "$LOGFILE"
				fi
			fi
		done

fi

NCONV=`cat "$LOGFILE" | grep -e 1 | wc -l`
NERR=`cat "$LOGFILE" | grep -e 0 | wc -l`
rm -f "$LOGFILE"
echo "finished with '`pwd`': converted $NCONV images, had $NERR problems"
