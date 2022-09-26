#!/bin/bash
declare -a workdirs=("/home/lnex/work/new_dataset/S1_01/small" "/home/lnex/work/new_dataset/S1_01/big"  "/home/lnex/work/new_dataset/S1_02/small" "/home/lnex/work/new_dataset/S1_02/big" "/home/lnex/work/new_dataset/S2_01/small" "/home/lnex/work/new_dataset/S2_01/big"  "/home/lnex/work/new_dataset/S2_02/small" "/home/lnex/work/new_dataset/S2_02/big")
declare -a algos=("prior" "vio")
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
