#!/bin/sh
cat $1 | grep -E "(^LUB )|(^CSparse )|(^----)|( nnz,)|(fear and loading)|(number of FLOPs in sparse LU)" | sed "s/[(,\\']//g" | sed 's/\/system\.mtx//g' | awk -v "btype=$2" ' BEGIN {
		btimes = cstimes = 0;
		btime[0] = "";
		cstime[0] = "";
		if(btype != "")
			btype = btype ",";
	} {
	if(index($0, "fear and loading"))
		matname = $4;
	else if(index($0, "elements") && index($0, "nnz")) {
		size = $1;
		nnz = $5;
	} else if(index($0, "number of FLOPs in sparse LU")) {
		flops = $7;
		symflops = $8;
	} else if(index($0, "LUB")) {
		btime[++btimes] = $2;
		bprec = $4;
		bdens = "=" $5 "+" $6;
	} else if(index($0, "CSparse")) {
		cstime[++cstimes] = $2;
		csprec = $4;
		csdens = "=" $5 "+" $6;
	} else if($1 == "----") {
		if(btimes + cstimes > 0)
			print btype matname "," size "," size "," nnz "," cstime[1] "," cstime[2] "," cstime[3] "," csdens "," csprec "," btime[1] "," btime[2] "," btime[3] "," bdens "," bprec "," flops "," symflops;
		matname = size = nnz = csdens = csprec = bdens = bprec = "";
		btimes = cstimes = 0;
		flops = symflops = "";
		delete btime;
		delete cstime;
		btime[0] = "";
		cstime[0] = "";
	}
	} END {
		if(btimes + cstimes > 0)
			print btype matname "," size "," size "," nnz "," cstime[1] "," cstime[2] "," cstime[3] "," csdens "," csprec "," btime[1] "," btime[2] "," btime[3] "," bdens "," bprec "," flops "," symflops;
    }'
