#!/bin/bash
#x stands for run, y stands for instance, #z stands for algorithm

bditers=(25 50 100)
sims=(500 1000)

for y in {1..4}; do
    for b in "${bditers[@]}"; do
        for i in "${sims[@]}"; do
            touch ~/scratch/gep_newsim_1_${y}_3.sh
            f=~/scratch/gep_newsim_1_${y}_3.sh
            echo "#$ -N gep_newsim">$f
            echo "#$ -j y">>$f
            echo "#$ -p -0">>$f
            echo "#$ -S /bin/bash">>$f
            echo "#$ -pe smp 4">>$f
            echo "#$ -o /home/akul/scratch/gep_finalsim_out_$y.log">>$f
            echo "#$ -l h_rt=12:50:00">>$f
            echo "#$ -l h_vmem=6g">>$f
            echo "julia /home/akul/sddp_comp/gep.jl 1 $y 3 4 3600 3600 $b $i">>$f
            qsub /home/akul/scratch/gep_newsim_1_${y}_3.sh
            sleep 10
        done
    done
done


