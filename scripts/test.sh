#!/bin/bash
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 5
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 10
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 15
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 20
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 25
# ./build_global_pc_ot /home/lnex/dataset/changed1/01 1 prior 30

# touch /home/lnex/out.csv

# ./build_confusion_mat /home/lnex/dataset/changed1/01 1 prior 5 2.4 >> /home/lnex/out.csv
# ./build_confusion_mat /home/lnex/dataset/changed1/01 1 prior 10 2.4 >> /home/lnex/out.csv
# ./build_confusion_mat /home/lnex/dataset/changed1/01 1 prior 15 2.4 >> /home/lnex/out.csv
# ./build_confusion_mat /home/lnex/dataset/changed1/01 1 prior 20 2.4 >> /home/lnex/out.csv
# ./build_confusion_mat /home/lnex/dataset/changed1/01 1 prior 25 2.4 >> /home/lnex/out.csv
# ./build_confusion_mat /home/lnex/dataset/changed1/01 1 prior 30 2.4 >> /home/lnex/out.csv


#!/bin/bash
declare -a workdirs=("/home/lnex/dataset/changed1/01" "/home/lnex/dataset/changed1/02" "/home/lnex/dataset/changed2/01" "/home/lnex/dataset/changed2/02")
declare -a algos=("prior" "vio")
for i in ${!workdirs[@]}; do
    for k in ${!algos[@]}; do
        for l in 5 10 15 20 25 30; do
            metrics_file="${workdirs[$i]}/metrics_${algos[$k]}_$l.csv" 
            if [[ -f $metrics_file ]] 
            then
                rm $metrics_file
            fi
            touch $metrics_file
            for j in {1..5}; do
                tmux new-session -d "./build_confusion_mat ${workdirs[$i]} $j ${algos[$k]} $l 2.4 >> $metrics_file"
            done
        done
        sleep 50
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