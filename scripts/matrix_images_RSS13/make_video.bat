@echo off

dir /B /S *.png > list.txt
set infile=mf://@list.txt -mf fps=25:type=png
set outfile="video.avi"
set globOpts=-noskip -vf softskip,harddup,eq=0:0
rem ,crop=704:544:8:18
set h264opts=bitrate=4000:frameref=6:analyse=all:me=umh:subme=7:trellis=2:bframes=1:subq=7:mixed_refs:weight_b:no_fast_pskip:direct_pred=auto:mixed_refs:nr=200:threads=auto
set audioCodec=copy
set audioOpts=cbr:preset=256

del divx2pass.log.mbtree.temp divx2pass.log.mbtree divx2pass.log.temp divx2pass.log >> nul 2>&1

mencoder %globOpts% %infile% -oac %audioCodec% -lameopts %audioOpts% -ovc x264 -x264encopts pass=1:%h264opts% -o nul
mencoder %globOpts% %infile% -oac %audioCodec% -lameopts %audioOpts% -ovc x264 -x264encopts pass=2:%h264opts% -o %outfile%

del divx2pass.log.mbtree.temp divx2pass.log.mbtree divx2pass.log.temp divx2pass.log >> nul 2>&1

pause
