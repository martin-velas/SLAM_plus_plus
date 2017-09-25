@echo off 
setlocal enableDelayedExpansion 

mkdir eigs

for /F %%x in ('dir /B /D *.mtx') do (
	echo %%x
	copy %%x matrix.mtx
	del matlablog.txt 2> nul
	start /wait /min G:\frag\zam_aja_cache\MATLAB\R2008a\bin\matlab -automation -nodisplay -nosplash -nodesktop -wait -logfile matlablog.txt -r "run('%CD%\get_eigs_gt.m'); exit;"
	type matlablog.txt
	del matlablog.txt
	del matrix.mtx
	move eigvecs.mtx eigs\%%x.eigvecs.mtx
	move eigvals.mtx eigs\%%x.eigvals.mtx
)
pause
