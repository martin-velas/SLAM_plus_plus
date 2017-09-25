#!/bin/bash

sed "s/[,)]//g" $1 | awk 'BEGIN {
	print "chi2,dx L2 min,dx L2 med,dx L2 1k,dx L2 max,dx Linf min,dx Linf med,dx Linf 1k,dx Linf max,SC margs,SC cams,UDinv,Extra Chol,Rec. Formula,abs error,rel error"; nf = 0;
	} {
		if($1 == "chi2:") {
			output = $2;
			f = 0; nf = 0;
		} else if(index($0, "debug: update L2 norm:") != 0) {
			output = output "," $6 "," $8 "," $10 "," $12;
			f = 1;
		} else if(index($0, "debug: update Linf norm:") != 0) {
			output = output "," $6 "," $8 "," $10 "," $12;
			f = 2;
		} else if(index($0, "Schur margs took") != 0) {
			output = output "," $4;
			f = 3;
		} else if(index($0, "recursive inverse of camera SC took") != 0) {
			output = output "," $7;
			f = 4;
		} else if(index($0, "calculating UDinv took") != 0) {
			output = output "," $4;
			f = 5;
		} else if(index($0, "Cholesky of lambda took") != 0) {
			output = output "," $5;
			f = 6;
		} else if(index($0, "recursive margs took") != 0) {
			output = output "," $4;
			f = 7;
		} else if(index($0, "debug: the marginals error:") != 0) {
			output = output "," $5 "," $7;
			f = 8;
		} else
			f = 666;
		if(f != 666) { /* ignore the unparsed fields */
			if(f != nf)
				nf = 0;
			else
				++ nf;
			if(nf == 9) {
				print output;
				nf = 0;
			}
		}
	}'
