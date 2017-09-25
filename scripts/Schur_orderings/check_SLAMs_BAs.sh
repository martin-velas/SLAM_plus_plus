#!/bin/bash

FILES="sparse_flops/*"
for f in $FILES; do
	if [[ ! -d "$f" ]]; then continue; fi

	depth_ff=0
	depth_impq=0
	depth_amics=0
	has_mics=0
	BA=0
	if [[ -d "$f/schur_gord" ]]; then
		BA=1
		F="$f/schur_gord"
		while [[ -d "$F" && -f "$F/system.mtx" ]]; do
			F="$F/schur_ff"
			depth_ff=$(( $depth_ff + 1 ))
		done
		F="$f/schur_gord"
		while [[ -d "$F" && -f "$F/system.mtx" ]]; do
			F="$F/schur_impq"
			depth_impq=$(( $depth_impq + 1 ))
		done
		F="$f/schur_gord"
		while [[ -d "$F" && -f "$F/system.mtx" ]]; do
			if [[ -d "$F/schur_MICS" && -f "$F/schur_MICS/system.mtx" ]]; then
				has_mics=$(( $depth_amics + 2 )) # the loop will take another iteration which will increment once more
			fi
			F="$F/schur_AMICS"
			depth_amics=$(( $depth_amics + 1 ))
		done
	else
		F="$f/schur_ff"
		while [[ -d "$F" && -f "$F/system.mtx" ]]; do
			F="$F/schur_ff"
			depth_ff=$(( $depth_ff + 1 ))
		done
		F="$f/schur_impq"
		while [[ -d "$F" && -f "$F/system.mtx" ]]; do
			F="$F/schur_impq"
			depth_impq=$(( $depth_impq + 1 ))
		done
		F="$f/schur_AMICS"
		while [[ -d "$F" && -f "$F/system.mtx" ]]; do
			if [[ -d "$F/schur_MICS" && -f "$F/schur_MICS/system.mtx" ]]; then
				has_mics=$(( $depth_amics + 2 )) # the loop will take another iteration which will increment once more
			fi
			F="$F/schur_AMICS"
			depth_amics=$(( $depth_amics + 1 ))
		done
	fi
	if [[ $has_mics -lt $depth_amics ]]; then
		has_mics=0 # only last level
	fi
	f20=`echo $f | awk '{ printf("%-24s\n", $0); }'`
	echo -e "$BA|$f20 (BA: $BA, ff: $depth_ff, impq: $depth_impq, AMICS: $depth_amics, LL MICS: $has_mics)"
done | sort -n | cut -d "|" -f 2 | grep "/submit" -v
