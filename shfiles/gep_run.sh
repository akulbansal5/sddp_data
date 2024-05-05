#!/bin/bash
#x stands for run, y stands for instance, #z stands for algorithm

for y in {1..1}; do
    for x in {1..1}; do
        for z in {1..6}; do
            touch ~/scratch/gep_newsim_${x}_${y}_${z}.sh
            f=~/scratch/gep_newsim_${x}_${y}_${z}.sh
            echo "#$ -N gep_newsim">$f
            echo "#$ -j y">>$f
            echo "#$ -p -0">>$f
            echo "#$ -S /bin/bash">>$f
            echo "#$ -pe smp 2">>$f
            echo "#$ -o /home/akul/scratch/gep_finsim_out_$y.log">>$f
            echo "#$ -l h_rt=12:50:00">>$f
            echo "#$ -l h_vmem=6g">>$f
            echo "julia /home/akul/sddp_comp/gep.jl $x $y $z 2 120 120 25 1000">>$f
            qsub /home/akul/scratch/gep_newsim_${x}_${y}_${z}.sh
            sleep 10
        done
    done
done


