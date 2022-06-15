#!/bin/bash
declare -a workdirs=("/home/lnex/dataset/changed1/02" "/home/lnex/dataset/changed1/01"  "/home/lnex/dataset/changed2/01" "/home/lnex/dataset/changed2/02")
declare -a algos=("prior")
for i in ${!workdirs[@]}; do
    for j in {1..5}; do
        for k in ${!algos[@]}; do
            for l in 5 10 15 20 25 30; do
                tmux new-session -d ./build_global_pc_ot ${workdirs[$i]} $j ${algos[$k]} $l
            done
        done
        sleep 120;
    done
done

# for j in {1..5}; do
#     for k in ${!algos[@]}; do
#         for l in 5 10 15 20 25 30; do
#             ./build_global_pc_ot /home/lnex/dataset/changed1/01 $j ${algos[$k]} $l
#         done
#     done
# done
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 5
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 10
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 15
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 20
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 25
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 30