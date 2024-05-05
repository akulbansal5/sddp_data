#!/bin/bash
#x stands for run, y stands for instance, #z stands for algorithm

# rm /home/akul/scratch/smkp_detr_nov29.log

for y in {1..1}; do
    for t in {1..2}; do
        for g in {1..2}; do
            touch ~/scratch/smkp_newdetr.sh
            f=~/scratch/smkp_newdetr.sh
            echo "#$ -N SMKP_newdetr">$f
            echo "#$ -j y">>$f
            echo "#$ -p -0">>$f
            echo "#$ -S /bin/bash">>$f
            echo "#$ -pe smp 2">>$f
            echo "#$ -o /home/akul/scratch/smkp_newdetr_$y.log">>$f
            echo "#$ -l h_rt=12:50:00">>$f
            echo "#$ -l h_vmem=6g">>$f
            echo "julia /home/akul/sddp_comp/smkp_DE.jl $y 2 240 $g $t">>$f
            qsub /home/akul/scratch/smkp_newdetr.sh
            sleep 3
        done
    done
done


# duals   = [SDDP.LagrangianDuality(), SDDP.LaporteLouveauxDuality()]
# bpass   = [SDDP.DefaultMultiBackwardPass(), SDDP.AnguloMultiBackwardPass()]
# fpass   = [SDDP.DefaultMultiForwardPass(), SDDP.DefaultNestedForwardPass()]
# spass   = [SDDP.InSampleMonteCarloMultiple(), SDDP.AllSampleMonteCarloMultiple()]


# y               = parse(Int64, ARGS[1])      #instance
# z               = parse(Int64, ARGS[2])      #duality
# x               = parse(Int64, ARGS[3])      #determines the forward pass and sampling scheme
# q               = parse(Int64, ARGS[4])      #determines the backward pass that we are going to use
# M               = parse(Int64, ARGS[5])      #number of scenario paths sampled 
# threads         = parse(Int64, ARGS[6])      #threads in the solver


# y               = parse(Int64, ARGS[1])      #instance
# z               = parse(Int64, ARGS[2])      #duality
# x               = parse(Int64, ARGS[3])      #determines the forward pass and sampling scheme
# q               = parse(Int64, ARGS[4])      #determines the backward pass that we are going to use
# M               = parse(Int64, ARGS[5])      #number of scenario paths sampled 
# threads         = parse(Int64, ARGS[6])      #threads in the solver