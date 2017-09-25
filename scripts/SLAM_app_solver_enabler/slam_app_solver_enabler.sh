#!/bin/bash

FILES=../src/slam_app/Solve*Impl.cpp

GR='\033[0;32m'
RD='\033[0;31m'
NC='\033[0m'

ENABLE_ALL=0
DISABLE_ALL=0
FORCE="-i"
while [[ $# -ge 1 ]]; do
	if [[ "$1" == "--help" || "$1" == "-h" ]]; then
		>&2 echo "use $0 -s|--save -r|--restore -ea|--enable-all or -da|--disable-all"
		>&2 echo "all the changes to the files prompt for user confirmation unless -f or --force is specified"
		>&2 echo "without any argument, this just displays the states of the solvers"
		>&2 echo "note that multiline comments are not properly supported, please don't use them to disable solvers"
		exit
	elif [[ "$1" == "--force" || "$1" == "-f" ]]; then
		>&2 echo -e "${RD}will not prompt for confirmation${NC}"
		FORCE=""
	elif [[ "$1" == "--save" || "$1" == "-s" ]]; then
		>&2 echo -e "${RD}will back-up solver files${NC}"
		mkdir .slam_app_impl_bk
		cp $FORCE ../src/slam_app/Solve*Impl.cpp .slam_app_impl_bk/
	elif [[ "$1" == "-r" || "$1" == "--restore" ]]; then
		>&2 echo -e "${RD}will restore solver files${NC}"
		if [[ ! -d .slam_app_impl_bk ]]; then
			>&2 echo "error: .slam_app_impl_bk not found: you did not run with -save to back-up your files"
			exit
		fi
		cp $FORCE .slam_app_impl_bk/Solve*Impl.cpp ../src/slam_app/
	elif [[ "$1" == "-ea" || "$1" == "--enable-all" ]]; then
		>&2 echo -e "${RD}will enable all solvers${NC}"
		if [[ ! -d .slam_app_impl_bk ]]; then
			>&2 echo "warning: .slam_app_impl_bk not found: run with -save first to back-up your files"
			exit
		fi
		ENABLE_ALL=1
		DISABLE_ALL=0
	elif [[ "$1" == "-da" || "$1" == "--disable-all" ]]; then
		>&2 echo -e "${RD}will disable all solvers${NC}"
		if [[ ! -d .slam_app_impl_bk ]]; then
			>&2 echo "warning: .slam_app_impl_bk not found: run with -save first to back-up your files"
			exit
		fi
		DISABLE_ALL=1
		ENABLE_ALL=0
	else
		>&2 echo "error: unknown commandline flag: $1"
	fi
	shift
done

for f in $FILES; do
	MODE=`echo $f | awk '{ print substr($1, index($1, "Solve") + 5, index($1, "Impl.cpp") - index($1, "Solve") - 5); }'`
	ENABLER=`cat $f | grep -E "#define[[:space:]]+__[A-Za-z0-9_]+_ENABLED"`
	ENABLED=`echo $ENABLER | grep -E "[[:space:]]*(//)|(/\*)[[:space:]]*#define" | wc -l`
	ENABLED=$(( 1 - $ENABLED ))
	echo -e "${GR}'"$f"' ("$MODE" solver) is currently "`echo $ENABLED | awk '{ if($1 == 1) print "ON"; else print "OFF"; }'`"${NC}"
	if [[ $ENABLE_ALL -eq 1 ]]; then
		if [[ $ENABLED -eq 0 ]]; then
			ENABLER_FOUND=`cat $f | sed -n '/\/\/[[:space:]]*#define[[:space:]][[:space:]]*__[A-Za-z0-9_]*_ENABLED/=' | wc -l`
			if [[ $ENABLER_FOUND -ne 1 ]]; then
				>&2 echo "error: failed to find the way to reenable (are you using multiline comments to disable? please don't)"
				continue
			fi
			ENABLER_LINE=`cat $f | sed -n '/\/\/[[:space:]]*#define[[:space:]][[:space:]]*__[A-Za-z0-9_]*_ENABLED/='`
			NEWf=$f".new"
			cat $f | head -n $(( $ENABLER_LINE - 1 )) > "$NEWf"
			cat $f | head -n $ENABLER_LINE | tail -n 1 | sed 's/\/\///' >> "$NEWf"
			cat $f | tail -n $(( `cat $f | wc -l` - $ENABLER_LINE )) >> "$NEWf"
			diff $f $NEWf
			mv $FORCE $NEWf $f
			rm $NEWf 2> /dev/null # in case the user said no
			echo -e ""
		fi
	fi

	if [[ $DISABLE_ALL -eq 1 ]]; then
		if [[ $ENABLED -eq 1 ]]; then
			ENABLER_FOUND=`cat $f | sed -n '/[[:space:]]*#define[[:space:]][[:space:]]*__[A-Za-z0-9_]*_ENABLED/=' | wc -l`
			if [[ $ENABLER_FOUND -ne 1 ]]; then
				>&2 echo "error: failed to find the way to reenable (are you using multiline comments to disable? please don't)"
				continue
			fi
			ENABLER_LINE=`cat $f | sed -n '/[[:space:]]*#define[[:space:]][[:space:]]*__[A-Za-z0-9_]*_ENABLED/='`
			NEWf=$f".new"
			cat $f | head -n $(( $ENABLER_LINE - 1 )) > "$NEWf"
			cat $f | head -n $ENABLER_LINE | tail -n 1 | awk '{ print "//" $0; }' >> "$NEWf"
			cat $f | tail -n $(( `cat $f | wc -l` - $ENABLER_LINE )) >> "$NEWf"
			diff $f $NEWf
			mv $FORCE $NEWf $f
			rm $NEWf 2> /dev/null # in case the user said no
			echo -e ""
		fi
	fi
done
