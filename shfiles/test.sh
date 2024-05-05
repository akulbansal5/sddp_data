for y in {1..1}; do
    for z in {1..2}; do
        for b in {1..2}; do
            for x in {1..1}; do
                for p in {1..2}; do
                    for s in {1..1}; do
                        for m in 2 5; do
                            touch ~/scratch/test_${y}_${z}_${b}_${x}_${p}_${s}_${m}.sh
                            f=~/scratch/test_${y}_${z}_${b}_${x}_${p}_${s}_${m}.sh
                            echo "#$ -N test_runs">$f
                            echo "#$ -j y">>$f
                            echo "#$ -p -0">>$f
                            echo "#$ -S /bin/bash">>$f
                            echo "#$ -pe smp 2">>$f
                            echo "#$ -o /home/akul/scratch/test_${y}_${z}_${b}_${x}_${p}_${s}_${m}.log">>$f
                            echo "#$ -l h_rt=12:50:00">>$f
                            echo "#$ -l h_vmem=6g">>$f
                            echo "julia /home/akul/sddp_comp/test.jl $y $z $x $b $m 1 2 1200 1200 1 $p $s">>$f
                            qsub /home/akul/scratch/test_${y}_${z}_${b}_${x}_${p}_${s}_${m}.sh
                            sleep 5
                        done
                    done
                done
            done
        done
    done
done


# y               = parse(Int64, ARGS[1])      #instance based on the list of builders
# z               = parse(Int64, ARGS[2])      #duality
# x               = parse(Int64, ARGS[3])      #determines the forward pass and sampling scheme
# q               = parse(Int64, ARGS[4])      #determines the backward pass that we are going to use
# M               = parse(Int64, ARGS[5])      #number of scenario paths sampled
# delta           = parse(Int64, ARGS[6])      #delta value 
# threads         = parse(Int64, ARGS[7])      #threads in the solver
# time_limit      = parse(Int64, ARGS[8])      #time limit on the algorithm
# iter_limit      = parse(Int64, ARGS[9])      #number of iterations in the problem
# final_run       = parse(Int64, ARGS[10])     #in sddip algorithm check if the entire scenario tree is traversed to compute deterministic bounds
#p                                             #probability values that we are going to try
#s                                             #seeds we are going to try



# if final_run == 1
#     final_run = true
# else
#     final_run = false
# end

# mipgap = 1e-4
# iter_pass = 1
# inst_name = instnames[y]

# filepath  = "/home/akul/sddp_comp/data/"*inst_name
# suffix    = "_Jan2024_$(z)_$(q)_$(x).csv"
# finalpath = filepath*suffix

# test_runs(
#     finalpath,
#     builders[y],
#     duals[z],
#     fpass[x],
#     bpass[q],
#     spass[x],
#     time_limit,
#     iter_limit,
#     mipgap,
#     iter_pass,
#     threads,
#     M,
#     gList[delta],
#     x,
#     final_run)