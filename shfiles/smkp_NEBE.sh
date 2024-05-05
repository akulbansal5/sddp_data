#!/bin/bash
#x stands for run, y stands for instance, #z stands for algorithm
#this is the file 

for y in {1..1}; do
    for z in {1..1}; do
        for b in {1..1}; do
            for x in {1..1}; do
                for p in {1..1}; do
                    for s in {1..1}; do
                        for m in 2 5; do
                            for g in {1..1}; do
                                touch ~/scratch/smkp_test_${y}_${z}_${b}_${x}_${p}_${s}_${m}_${g}.sh
                                f=~/scratch/smkp_test_${y}_${z}_${b}_${x}_${p}_${s}_${m}_${g}.sh
                                echo "#$ -N SMKP_testsim">$f
                                echo "#$ -j y">>$f
                                echo "#$ -p -0">>$f
                                echo "#$ -S /bin/bash">>$f
                                echo "#$ -pe smp 2">>$f
                                echo "#$ -o /home/akul/scratch/smkp_NEBE_${y}_${z}_${b}_${x}_${p}_${s}_${m}_${g}.log">>$f
                                echo "#$ -l h_rt=12:50:00">>$f
                                echo "#$ -l h_vmem=6g">>$f
                                echo "julia /home/akul/sddp_comp/smkp_NEBE.jl $y $z $x $b $m $g 2 240 0 0 $p $s 10">>$f
                                qsub /home/akul/scratch/smkp_test_${y}_${z}_${b}_${x}_${p}_${s}_${m}_${g}.sh
                                sleep 5
                            done
                        done
                    done
                done
            done
        done
    done
done


# y               = parse(Int64, ARGS[1])      #instance
# z               = parse(Int64, ARGS[2])      #duality
# x               = parse(Int64, ARGS[3])      #determines the forward pass and sampling scheme
# q (or b in above loop)              = parse(Int64, ARGS[4])      #determines the backward pass that we are going to use
# M               = parse(Int64, ARGS[5])      #number of scenario paths sampled
# delta (g in above loop)           = parse(Int64, ARGS[6])      #delta value 
# threads         = parse(Int64, ARGS[7])      #threads in the solver
# time_limit      = parse(Int64, ARGS[8])      #time limit on the algorithm
# iter_limit      = parse(Int64, ARGS[9])      #number of iterations in the problem
# final_run       = parse(Int64, ARGS[10])     #= 1 in sddip algorithm check if the entire scenario tree is traversed to compute deterministic bounds
# prob            = parse(Int64, ARGS[11])     #used in tito-s stoping criterions
# seed            = parse(Int64, ARGS[12])     #determines whether to set the seed or not
# postSim         = 1e-2*parse(Int64, ARGS[13])     #number of simulations to get the upper bound, determined as percentage of scenarios
###NOTE: SOME OF THE ABOVE VALUES ARE NOT RELEVANT FOR NESTED BENDERS RUNS



# rm /home/akul/scratch/smkp_nested.log
# touch ~/scratch/smkp_newsim.sh
# f=~/scratch/smkp_newsim.sh
# echo "#$ -N SMKP_newsim">$f
# echo "#$ -j y">>$f
# echo "#$ -p -0">>$f
# echo "#$ -S /bin/bash">>$f
# echo "#$ -pe smp 10">>$f
# echo "#$ -o /home/akul/scratch/smkp_nested.log">>$f
# echo "#$ -l h_rt=12:50:00">>$f
# echo "#$ -l h_vmem=4g">>$f
# echo "julia /home/akul/sddp_comp/smkp_nested.jl 1 1 10">>$f
# qsub /home/akul/scratch/smkp_newsim.sh
# sleep 3



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
