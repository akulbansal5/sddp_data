#latest file as on Feb 13, 2024

for y in {2..2}; do
    for z in {1..1}; do
        for b in {1..1}; do
            for x in {1..1}; do
                for p in {1..1}; do
                    for s in {1..1}; do
                        for m in 2 5; do
                            for g in 1 2; do
                                touch ~/scratch/gep_test_${y}_${z}_${b}_${x}_${p}_${s}_${m}_${g}.sh
                                f=~/scratch/gep_test_${y}_${z}_${b}_${x}_${p}_${s}_${m}_${g}.sh
                                echo "#$ -N gep_testsim">$f
                                echo "#$ -j y">>$f
                                echo "#$ -p -0">>$f
                                echo "#$ -S /bin/bash">>$f
                                echo "#$ -pe smp 2">>$f
                                echo "#$ -o /home/akul/scratch/gep__${y}_${z}_${b}_${x}_${p}_${s}_${m}_${g}.log">>$f
                                echo "#$ -l h_rt=12:50:00">>$f
                                echo "#$ -l h_vmem=6g">>$f
                                echo "julia /home/akul/sddp_comp/gep.jl $y $z $x $b $m $g 2 240 10 0 $p $s 10">>$f
                                qsub /home/akul/scratch/gep_test_${y}_${z}_${b}_${x}_${p}_${s}_${m}_${g}.sh
                                sleep 3
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
# b               = parse(Int64, ARGS[4])      #determines the backward pass that we are going to use
# M               = parse(Int64, ARGS[5])      #number of scenario paths sampled
# delta           = parse(Int64, ARGS[6])      #delta value 
# threads         = parse(Int64, ARGS[7])      #threads in the solver
# time_limit      = parse(Int64, ARGS[8])      #time limit on the algorithm
# iter_limit      = parse(Int64, ARGS[9])      #number of iterations in the problem
# final_run       = parse(Int64, ARGS[10])     #in sddip algorithm check if the entire scenario tree is traversed to compute deterministic bounds
# prob            = parse(Int64, ARGS[11])     #used in tito-s stoping criterions
# seed            = parse(Int64, ARGS[12])     #determines whether to set the seed or not
# postSim         = parse(Int64, ARGS[13])     #number of simulations to get the upper bound


# gep_method_runs(
#     folder,
#     finalpath, 
#     duals[z], 
#     fpass[x],
#     bpass[q],
#     spass[x],
#     reps,
#     id,
#     st, 
#     scens,
#     time_limit,
#     iter_limit, 
#     mipgap,
#     iter_pass,
#     threads,
#     M,
#     gList[delta],
#     final_run,
#     cdfinv[prob],
#     cdfinv[prob],
#     seed)

