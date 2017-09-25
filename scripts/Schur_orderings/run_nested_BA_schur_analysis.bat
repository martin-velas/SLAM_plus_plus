@echo off

mkdir BA_flops_gpuinv2
cd BA_flops_gpuinv2

set SPP_BIN=%~dp0%..\bin\slam_schur_orderings.exe
set TEE=%~dp0%wtee

rem echo %SPP_BIN%

set N=0

@echo on

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
rem evaluate gord perf (no images written)

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
rem evaluate nested AMICS perf (no images written)

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_ff/schur_ff/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
rem evaluate nested ff perf (no images written)

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_seq00 -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_41 -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W20 -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/karlsruhe_loop_W6 -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_AMICS/schur_AMICS/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_AMICS/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_ff/schur_ff/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_ff/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

rem exit

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_seq00/schur_gord -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loop/schur_gord -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loop41/schur_gord -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loopw20/schur_gord -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loopw6/schur_gord -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_seq00/schur_gord -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loop/schur_gord -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loop41/schur_gord -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loopw20/schur_gord -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loopw6/schur_gord -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_seq00/schur_gord -nni -nes ../sparse_flops/kr_seq00/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loop/schur_gord -nni -nes ../sparse_flops/kr_loop/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loop41/schur_gord -nni -nes ../sparse_flops/kr_loop41/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loopw20/schur_gord -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loopw6/schur_gord -nni -nes ../sparse_flops/kr_loopw6/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../sparse_flops/kr_loopw20/schur_gord -nni -nes ../sparse_flops/kr_loopw20/schur_gord/schur_AMICS/schur_AMICS/schur_AMICS/schur_MICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/ff6 -nni -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/ff6 -nni -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/schur_MICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/ff6 -nni -nes ../sparse_flops/ff6/schur_gord/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/ff6 -nni -nes ../sparse_flops/ff6/schur_gord/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/ff6 -nni -nes ../sparse_flops/ff6/schur_gord/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/ff6 -nni -nes ../sparse_flops/ff6/schur_gord/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/cathedral -nni -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/cathedral -nni -nes ../sparse_flops/cath/schur_gord/schur_AMICS/schur_MICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/cathedral -nni -nes ../sparse_flops/cath/schur_gord/schur_AMICS/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/cathedral -nni -nes ../sparse_flops/cath/schur_gord/schur_ff/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/cathedral -nni -nes ../sparse_flops/cath/schur_gord/schur_ff/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/cathedral -nni -nes ../sparse_flops/cath/schur_gord/ ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_AMICS/schur_AMICS/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_AMICS/schur_MICS/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_AMICS/schur_AMICS/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_ff/schur_ff/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/schur_ff/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1
(FOR /L %%G IN (1,1,2) DO %SPP_BIN% ../../data/BA_sys/venice871.g2o -nni -nes ../sparse_flops/venice/schur_gord/ -dm 10000 ) 2> log%N%.ER | %TEE% log%N%.OU
set /A N=%N%+1

@pause
