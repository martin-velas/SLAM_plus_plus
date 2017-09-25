#!/bin/sh

FACTIMES=0
EXTCHOLMOD=0
FIVELEVEL=0
while [[ $# -gt 0 ]]; do
	if [[ "$1" == "--" ]]; then
		shift
		break
	elif [[ "$1" == "--help" || "$1" == "-h" ]]; then
		>&2 echo "use [options] [--] <file0> <file1> ... <fileN>"
		>&2 echo "where [options can be]:"
		>&2 echo "--factorize-times - report factorization times rather than"
		>&2 echo "    linear solving times for CSparse / Cholmod / UBlock FBS"
		>&2 echo "--extended-cholmod - reports also simplical and supernodal"
		>&2 echo "    factorization times for Cholmod"
		>&2 echo "--five-level or -5l options shows five levels of Schur (default 3)"
		>&2 echo "-- - separates options from files in case a filename starts"
		>&2 echo "    with a dash"
		exit
	elif [[ "$1" == "--factorize-times" ]]; then
		FACTIMES=1
	elif [[ "$1" == "--five-level" || "$1" == "-5l" ]]; then
		FIVELEVEL=1
	elif [[ "$1" == "--extended-cholmod" ]]; then
		EXTCHOLMOD=1
	else
		break
	fi
	shift
done
# parse commandline

ISFIRST=1 # first file prints header
while [[ $# -gt 0 ]]; do

cat $1 | awk -v fiver=$FIVELEVEL -v want_header=$ISFIRST -v want_factorization_times=$FACTIMES -v want_ech=$EXTCHOLMOD 'function get(fn) {
		if(want_header)
			return fn;
		if(fn in fields)
			return fields[fn];
		else
			return "N/A";
	}
	function set(fn,v) {
		if(fn in fields)
			print("error: field " fn " already in fields");
		fields[fn] = v;
	}
	function flushParsed() {
		if(had_section) {
			t = (want_factorization_times)? "f" : "s";
			print get("short_dataset") "\t" get("mode") "\t" get("size") "\t" get("cut_1") "\t" get("cut_2") "\t" get("cut_3") \
				((fiver)? "\t" get("cut_4") "\t" get("cut_5") : "") "\t" get("csparse_err") "\t" get("schur_err") "\t" get("csparse_"t"time") \
				"\t" get("cholmod_"t"time") ((want_ech)?  "\t" get("cholmod_sim_"t"time")  "\t" get("cholmod_sup_"t"time") : "") \
				"\t" get("ublock_"t"time") "\t" get("sc_ublock") "\t" get("sc_cholmod") "\t" get("sc_eigen") "\t" get("sc_gpu") \
				"\t" get("sc_matmults") "\t" get("sc_dinv") "\t" get("sc_dinvp") "\t" get("sc_dinvgpu") "\t" get("direct_fac_density") "\t" get("schur_density") \
				"\t" get("mem_1") "\t" get("mem_2") "\t" get("mem_3") "\t" get("mem_4") ((fiver)?  "\t" get("mem_5")  "\t" get("mem_6") : "");
			delete fields;
			had_section = 0;
		}
	}
	BEGIN {
		had_section = want_header;
		flushParsed(); /* print the header */
		want_header = 0;
		had_section = 0;
	}
	{
		if(index($0, "fear and loading") != 0 && index($0, "/system.mtx") != 0) {
			flushParsed();
			/* print data so far */

			split($0, a, "/");
			ds = a[length(a) - 1];
			while(length(ds) < 16)
				ds = ds " ";
			set("dataset", ds);

			had_section = 1;
			ncuts = 0; nlevel = 0; /* reset counters */
		} else if($1 == "will" && $2 == "combine" && $3 == "results" && $4 == "from") {
			split($0, a, "/");
			split(a[length(a) - 1], b, "_");
			set("mode", b[2]);
		} else if($1 == "found" && $2 == "base" && $3 == "path:") {
			split($0, a, "/");
			ds = a[length(a)];
			sub("'"'"'", "", ds);
			while(length(ds) < 8)
				ds = ds " ";
			set("short_dataset", ds);
		} else if($1 == "original" && $2 == "size")
			set("size", $3);
		else if($1 == "cut" && $2 == "at")
			set("cut_" ++ ncuts, $3);
		else if($1 == "required" && $2 == "memory:" && $4 == "MB")
			set("mem_" ++ nlevel, $3);
		else if(substr($1, 2, length($1) - 1) == "-level" && $2 == "Schur" && $3 == "would" && $4 == "require" && $6 == "MB")
			set("mem_" ++ nlevel, $5);
		else if($1 == "full" && $2 == "lambda" && $3 == "would" && $4 == "require" && $6 == "MB")
			set("mem_" ++ nlevel, $5);
		else if($1 == "last" && $2 == "level" && $3 == "dense" && $4 == "SC:" && $10 == "nonzeros)")
			set("schur_density", $9);
		else if($1 == "SC" && $2 == "solution" && $3 == "error:")
			set("schur_err", $6);
		else if($1 == "CS" && $2 == "solution" && $3 == "error:")
			set("csparse_err", $6);
		else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "msec" && $6 == "using" && $7 == "CSparse") {
			set("csparse_stime", $4); /* solve */
			sub("\\(", "", $8); set("csparse_ftime", $8); /* factorize */
			set("csparse_otime", $11); /* overhead */
		} else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "msec" && $6 == "using" && $7 == "Cholmod") {
			set("cholmod_stime", $4); /* solve */
			sub("\\(", "", $8); set("cholmod_ftime", $8); /* factorize */
			set("cholmod_otime", $11); /* overhead */
		} else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "msec" && $6 == "using" && $7 == "simplical" && $8 == "Cholmod") {
			set("cholmod_sim_stime", $4); /* solve */
			sub("\\(", "", $9); set("cholmod_sim_ftime", $9); /* factorize */
			set("cholmod_sim_otime", $12); /* overhead */
		} else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "msec" && $6 == "using" && $7 == "supernodal" && $8 == "Cholmod") {
			set("cholmod_sup_stime", $4); /* solve */
			sub("\\(", "", $9); set("cholmod_sup_ftime", $9); /* factorize */
			set("cholmod_sup_otime", $12); /* overhead */
		} else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "msec" && $6 == "using" && $7 == "block" && $8 == "Chol") {
			set("ublock_stime", $4); /* solve */
			sub("\\(", "", $9); set("ublock_ftime", $9); /* factorize */
			set("direct_fac_density", $12);
		} else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "+" && $7 == "msec" && $8 == "using" && $9 == "SC" && $10 == "+" && $11 == "block" && $12 == "Chol") {
			set("sc_ublock", $4);
			set("sc_matmults", $6);
		} else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "+" && $7 == "msec" && $8 == "using" && $9 == "SC" && $10 == "+" && $11 == "Cholmod")
			set("sc_cholmod", $4);
		else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "+" && $7 == "msec" && $8 == "using" && $9 == "SC" && $10 == "+" && $11 == "Eigen" && $12 == "dense")
			set("sc_eigen", $4);
		else if($1 == "the" && $2 == "solution" && $3 == "took" && $5 == "+" && $7 == "msec" && $8 == "using" && $9 == "SC" && $10 == "+" && $11 == "GPU" && $12 == "dense")
			set("sc_gpu", $4);
		else if($1 == "block" && $2 == "diagonal" && $3 == "inverse" && $4 == "took" && $6 == "msec" && $7 == "(in" && $8 == "series)")
			set("sc_dinv", $5);
		else if($1 == "block" && $2 == "diagonal" && $3 == "inverse" && $4 == "took" && $6 == "msec" && $7 == "(parallel)")
			set("sc_dinvp", $5);
		else if($1 == "block" && $2 == "diagonal" && $3 == "inverse" && $4 == "took" && $6 == "msec" && $7 == "(GPU)")
			set("sc_dinvgpu", $5);
	} END { flushParsed(); }'

	shift
	ISFIRST=0

done
