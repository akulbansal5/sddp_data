"""
create and run instances of the Stochastic multi-knapsack problem (SMKP)
"""





import_time = @elapsed begin
    using SDDP
    using FileIO
    using Gurobi
    using Serialization
    using CSV
    using DataFrames
    using HiGHS
end


function smkp_gen_ver1(st, lb, rows, cols, A, T, c, q, threads)

    """
    Generates the instance of the SMKP class
    
    st (int): number of stages

    A (list of matrices): the coefficient matrix associated with 
                          the state variables, one for each stage
    T (list of matrices): coefficient matrix associated with the
                          recourse, one for each stage
    c (list):             corresponds to the objective coefficients
                          for the first stage.
    q (list of vectors):  each vector within the list represents the support set
                          for that particular stage

    ver1: in version 1 we have both local and state variables
    """

    model = SDDP.LinearPolicyGraph(
        stages = st,
        sense  = :Min,
        lower_bound = lb,
        optimizer = Gurobi.Optimizer,
        solver_threads = threads,
    ) do sp, stage

        @variable(sp, 0 <= x[1:cols], Bin, SDDP.State, initial_value = 0)

        one_vector = ones(Int, cols)
        
        coeff = 1
        if stage == 1
            coeff = 3/4
        end

        h = coeff*A[stage]*one_vector
        

        @constraint(
            sp,
            knapsack[i in 1:rows],
            sum((A[stage][i, j]*x[j].out + T[stage][i, j]*x[j].in) for j in 1:cols) >= h[i]
        )

        if stage == 1
            @stageobjective(
                sp,
                sum(c[j]*x[j].out for j in 1:cols)
            )
        else
            SDDP.parameterize(sp, q[stage]) do qs
                @stageobjective(
                    sp,
                    sum(qs[j]*x[j].out for j in 1:cols)
                )
            end
        end
    end
    return model
end

function smkp_gen_ver2(st, lb, rows, cols, A, T, c, q, threads)

    """
    Generates the instance of the SMKP class
    
    st (int): number of stages

    A (list of matrices): the coefficient matrix associated with 
                          the state variables, one for each stage
    T (list of matrices): coefficient matrix associated with the
                          recourse, one for each stage
    c (list):             corresponds to the objective coefficients
                          for the first stage.
    q (list of vectors):  each vector within the list represents the support set
                          for that particular stage

    """

    model = SDDP.LinearPolicyGraph(
        stages = st,
        sense  = :Min,
        lower_bound = lb,
        optimizer = Gurobi.Optimizer,
        solver_threads = threads,
    ) do sp, stage

        @variable(sp, 0 <= x[1:cols], Bin, SDDP.State, initial_value = 0)
        @variable(sp, 1 <= p[1:rows])

        one_vector = ones(Int, cols)
        
        h = (3/4)*A[stage]*one_vector + (3/4)*T[stage]*one_vector
        penalty = 100*cols*(3/4) + 5

        @constraint(
            sp,
            knapsack[i in 1:rows],
            sum((A[stage][i, j]*x[j].out + T[stage][i, j]*x[j].in) for j in 1:cols) + p[i] >= h[i]
        )

        

        if stage == 1
            @stageobjective(
                sp,
                sum(c[j]*x[j].out for j in 1:cols) + sum(penalty*p[i] for i in 1:rows)
            )
        else
            SDDP.parameterize(sp, q[stage]) do qs
                @stageobjective(
                    sp,
                    sum(qs[j]*x[j].out for j in 1:cols) + sum(penalty*p[i] for i in 1:rows)
                )
            end
        end
    end
    return model
end

function smkp_gen_ver3(st, lb, rows, cols, A, T, c, d, q, threads)

    """
    Generates the instance of the SMKP class
    
    st (int): number of stages

    A (list of matrices): the coefficient matrix associated with 
                          the state variables, one for each stage
    T (list of matrices): coefficient matrix associated with the
                          recourse, one for each stage
    c (list):             corresponds to the objective coefficients
                          for the first stage.
    q (list of vectors):  each vector within the list represents the support set
                          for that particular stage

    """

    model = SDDP.LinearPolicyGraph(
        stages = st,
        sense  = :Min,
        lower_bound = lb,
        optimizer = Gurobi.Optimizer,
        solver_threads = threads,
    ) do sp, stage

        @variable(sp, 0 <= x[1:cols], Bin, SDDP.State, initial_value = 0)

        if stage == 1
            @variable(sp, 0 <= z[1:cols], Bin)
        else
            @variable(sp, 0 <= p[1:rows], Int)
        end


        one_vector = ones(Int, cols)
        
        h = (3/4)*A[stage]*one_vector + (3/4)*T[stage]*one_vector
        penalty = 200.0

        if stage == 1
            @constraint(
                sp,
                knapsack[i in 1:rows],
                sum((A[stage][i, j]*x[j].out + T[stage][i, j]*z[j]) for j in 1:cols) >= h[i]
            )
        else
            @constraint(
                sp,
                knapsack[i in 1:rows],
                sum((A[stage][i, j]*x[j].out + T[stage][i, j]*x[j].in) for j in 1:cols) + p[i] >= h[i]
            )
        end

        

        if stage == 1
            @stageobjective(
                sp,
                sum(c[j]*x[j].out for j in 1:cols) + sum(d[j]*z[j] for j in 1:cols)
            )
        else
            SDDP.parameterize(sp, q[stage]) do qs
                @stageobjective(
                    sp,
                    sum(penalty*p[i] for i in 1:rows) + sum(qs[j]*x[j].out for j in 1:cols)
                )
            end
        end
    end
    return model
end

function smkp_gen_ver4(st, lb, rows, cols, A, T, c, d, q, threads)

    """
    Generates the instance of the SMKP class
    
    st (int): number of stages

    A (list of matrices): the coefficient matrix associated with 
                          the state variables, one for each stage
    T (list of matrices): coefficient matrix associated with the
                          recourse, one for each stage
    c (list):             corresponds to the objective coefficients
                          for the first stage.
    q (list of vectors):  each vector within the list represents the support set
                          for that particular stage

    ver4: builds on top of ver3 and now scales the costs by 1e-1
    """

    model = SDDP.LinearPolicyGraph(
        stages = st,
        sense  = :Min,
        lower_bound = lb,
        optimizer = Gurobi.Optimizer,
        solver_threads = threads,
    ) do sp, stage

        @variable(sp, 0 <= x[1:cols], Bin, SDDP.State, initial_value = 0)

        if stage == 1
            @variable(sp, 0 <= z[1:cols], Bin)
        else
            @variable(sp, 0 <= p[1:rows], Int)
        end

        cost_scale = 1e-1
        one_vector = ones(Int, cols)
        
        h = (3/4)*A[stage]*one_vector + (3/4)*T[stage]*one_vector
        penalty = 200.0

        if stage == 1
            @constraint(
                sp,
                knapsack[i in 1:rows],
                sum((A[stage][i, j]*x[j].out + T[stage][i, j]*z[j]) for j in 1:cols) >= h[i]
            )
        else
            @constraint(
                sp,
                knapsack[i in 1:rows],
                sum((A[stage][i, j]*x[j].out + T[stage][i, j]*x[j].in) for j in 1:cols) + p[i] >= h[i]
            )
        end

        

        
        if stage == 1
            @stageobjective(
                sp,
                sum(cost_scale*c[j]*x[j].out for j in 1:cols) + sum(cost_scale*d[j]*z[j] for j in 1:cols)
            )
        else
            SDDP.parameterize(sp, q[stage]) do qs
                @stageobjective(
                    sp,
                    sum(cost_scale*penalty*p[i] for i in 1:rows) + sum(cost_scale*qs[j]*x[j].out for j in 1:cols)
                )
            end
        end
    end
    return model
end

function smkp_gen_ver5(st, lb, rows, cols, A, T, c, d, q, threads)

    """
    Generates the instance of the SMKP class
    
    st (int): number of stages

    A (list of matrices): the coefficient matrix associated with 
                          the state variables, one for each stage
    T (list of matrices): coefficient matrix associated with the
                          recourse, one for each stage
    c (list):             corresponds to the objective coefficients
                          for the first stage.
    q (list of vectors):  each vector within the list represents the support set
                          for that particular stage

    ver4: builds on top of ver3 and now scales the costs by 1e-1
    ver5: uses HiGHS optimizer instead of the Gurobi optimizers
    """

    model = SDDP.LinearPolicyGraph(
        stages = st,
        sense  = :Min,
        lower_bound = lb,
        optimizer = HiGHS.Optimizer,
        solver_threads = threads,
    ) do sp, stage

        @variable(sp, 0 <= x[1:cols], Bin, SDDP.State, initial_value = 0)

        if stage == 1
            @variable(sp, 0 <= z[1:cols], Bin)
        else
            @variable(sp, 0 <= p[1:rows], Int)
        end

        cost_scale = 1e-1

        one_vector = ones(Int, cols)
        
        h = (3/4)*A[stage]*one_vector + (3/4)*T[stage]*one_vector
        penalty = 200.0

        if stage == 1
            @constraint(
                sp,
                knapsack[i in 1:rows],
                sum((A[stage][i, j]*x[j].out + T[stage][i, j]*z[j]) for j in 1:cols) >= h[i]
            )
        else
            @constraint(
                sp,
                knapsack[i in 1:rows],
                sum((A[stage][i, j]*x[j].out + T[stage][i, j]*x[j].in) for j in 1:cols) + p[i] >= h[i]
            )
        end

        if stage == 1
            @stageobjective(
                sp,
                sum(cost_scale*c[j]*x[j].out for j in 1:cols) + sum(cost_scale*d[j]*z[j] for j in 1:cols)
            )
        else
            SDDP.parameterize(sp, q[stage]) do qs
                @stageobjective(
                    sp,
                    sum(cost_scale*penalty*p[i] for i in 1:rows) + sum(cost_scale*qs[j]*x[j].out for j in 1:cols)
                )
            end
        end
    end
    return model
end

function getSMKPdata(folder, file)

    """
    deserializes the SMKP data
    """

    out = open(folder*file, "r") do f
        deserialize(f)
    end

    return out
end

function getAllSMKPdata(folder, id, inst, st, rows, cols, scens)

    """
    deserialize the entire SMKP data
    """

    #get the A matrix
    AList = []
   
    for t in 1:st
        file = "smkp_A"*"_$(id)_$(inst)_$(t)_$(rows)_$(cols)_$(scens).jls"
        A_t = getSMKPdata(folder, file)
        push!(AList, A_t)
    end

    #get the T matrix
    TList = []

    for t in 1:st
        file = "smkp_T"*"_$(id)_$(inst)_$(t)_$(rows)_$(cols)_$(scens).jls"
        T_t  = getSMKPdata(folder, file)
        push!(TList, T_t)
    end

    #get the c vector
    file = "smkp_c"*"_$(id)_$(inst)_$(1)_$(1)_$(cols).jls"
    c = getSMKPdata(folder, file)


    file = "smkp_d"*"_$(id)_$(inst)_$(1)_$(1)_$(cols).jls"
    d = getSMKPdata(folder, file)

    #get all the scenarios
    #for each stage we need a list of lists
    qDict = Dict()
    for t in 2:st
        qDict[t] = []
        for s in 1:scens
            file = "smkp_q"*"_$(id)_$(inst)_$(t)_$(s)_$(cols).jls"
            q_t_s = getSMKPdata(folder, file)
            push!(qDict[t], q_t_s)
        end
    end
    return AList, TList, c, d, qDict
end

function csv_recording(filepath, header_tuple, row_values, output)
    
    if !isfile(filepath)
        # header_row = ("class", "dual_handle", "fwd_pass", "id", "T", "rows", "cols", "scens", "threads", "maxTime", "iter", "time", "cut-s", "cut-ns", "lb", "stat-ub")
        file = open(filepath, "w")
        CSV.write(file, [header_tuple], append = true)
        close(file)
    end
    
    
    iters = length(output.bb)

    for i in 1:iters
        new_row = (output.time_list[i], output.cs_list[i], output.cns_list[i], output.bb[i], output.cumm_list[i])
        final_row = (row_values..., new_row...)
        file = open(filepath, "a")
        CSV.write(file, [final_row], append=true)
        close(file)    
    end

end


function train_DE(

    model,
    duality_handler,
    forward_pass,
    backward_pass,
    sampling_scheme,
    time_limit,
    mipgap,
    iter_pass,
    M = 1
)


    build_time = @elapsed begin
        det_model = SDDP.deterministic_equivalent(model, Gurobi.Optimizer, solver_threads = threads)
    end

    solve_time = @elapsed begin
        set_optimizer_attribute(det_model, "TimeLimit", max(time_limit - build_time, 60))
        set_optimizer_attribute(det_model, "mip_gap", mipgap)
        JuMP.optimize!(det_model)

    end

    method_objval = JuMP.objective_value(det_model)
    best_bd       = JuMP.objective_bound(det_model)
    gap_at_end    = MOI.get(det_model, MOI.RelativeGap())

    if !isfile(filepath)
        header_row = ("class", "method", "id", "inst", "T", "rows", "cols", "scens", "threads", "maxTime", "b-time(s)", "s-time(s)", "obj", "bound", "gap")
        file = open(filepath, "w")
        CSV.write(file, [header_row], append = true)
        close(file)
    end

    row = ("smkp", "detr", id, inst, st, rows, cols, scens, threads, time_limit, build_time, solve_time, method_objval, best_bd, gap_at_end)
    file = open(filepath, "a")
    CSV.write(file, [row], append=true)
    close(file)


    return build_time, solve_time, best_bd, gap_at_end

end


function train_method(
    model,
    duality_handler,
    forward_pass,
    backward_pass,
    sampling_scheme,
    time_limit,
    iter_limit,
    mipgap,
    iter_pass,
    M = 1,
    delta = 0.05,
    fpass_type = 2,
    final_run = false,
    type1_prob = 1.28,
    type2_prob = 1.28,
    seed = nothing
)

    #fpass_type = 2 means Nested Benders algorithm
    #fpass_type = 1 means SDDiP algorithm
    #iter_pass: 0 is the default implementation 
                #1 is the new implementation for Nested Benders and SDDiP algorithm that allows sampling multiple paths

    outputs = nothing

    if fpass_type == 2
        if iter_limit < 1
            # println("       smkp_NES.jl: train function without the iteration limit ")
            outputs = SDDP.train(
                model;
                duality_handler = duality_handler,
                forward_pass    = forward_pass,
                backward_pass   = backward_pass,
                sampling_scheme = sampling_scheme,
                stopping_rules  = [SDDP.TimeLimit(time_limit), SDDP.NBBoundStalling(delta)],
                mipgap          = mipgap,
                iter_pass       = iter_pass,
                M               = M,
                print_level     = 2,
                final_run       = final_run)
        else
            # println("       smkp_NES.jl: the second train function is choosen")
            outputs = SDDP.train(model,
                duality_handler = duality_handler,
                forward_pass    = forward_pass,
                backward_pass   = backward_pass,
                sampling_scheme = sampling_scheme,
                stopping_rules  = [SDDP.IterationLimit(iter_limit), SDDP.NBBoundStalling(delta), SDDP.TimeLimit(time_limit)],
                mipgap          = mipgap,
                iter_pass       = iter_pass,
                M               = M,
                print_level     = 2,
                final_run       = final_run)
        end
    else
        if iter_limit < 1
            # println("       smkp_NES.jl: train function without the iteration limit ")
            outputs = SDDP.train(
                model;
                duality_handler = duality_handler,
                forward_pass    = forward_pass,
                backward_pass   = backward_pass,
                sampling_scheme = sampling_scheme,
                stopping_rules  = [SDDP.TimeLimit(time_limit), SDDP.TitoStalling(type1_prob, type2_prob, delta)],
                mipgap          = mipgap,
                iter_pass       = iter_pass,
                M               = M,
                print_level     = 2,
                final_run       = final_run,
                seed            = seed)
        else
            # println("       smkp_NES.jl: the second train function is choosen")
            outputs = SDDP.train(model,
                duality_handler = duality_handler,
                forward_pass    = forward_pass,
                backward_pass   = backward_pass,
                sampling_scheme = sampling_scheme,
                stopping_rules  = [SDDP.IterationLimit(iter_limit), SDDP.TitoStalling(type1_prob, type2_prob, delta), SDDP.TimeLimit(time_limit)],
                mipgap          = mipgap,
                iter_pass       = iter_pass,
                M               = M,
                print_level     = 2,
                final_run       = final_run,
                seed            = seed)
        end
    end


        
    
    sddp_bound  = outputs[1].bb 
    sddp_simval = outputs[1].cumm_list[end]

    println("the bound attained from sddp:            $(sddp_bound)")
    println("the simulation value attained from sddp: $(sddp_simval)")
    println("================== end ====================")

    return outputs

end


function smkp_method_runs(
    folder,
    filepath, 
    duality_handler, 
    forward_pass,
    backward_pass,
    sampling_scheme,
    run, 
    id, 
    inst, 
    rows, 
    cols, 
    st, 
    scens, 
    time_limit = 120,
    iter_limit = 2, 
    mipgap = 1e-4,
    iter_pass = 0,
    threads = 1,
    M = 1, 
    delta = 0.05,
    fpass_type = 2,
    final_run = false,
    postSim = 0.05,
    simTime = 3600,
    type1_prob = 1.28,
    type2_prob = 1.28,
    seed = nothing
)


    """
    instances (list): list of instance (has attributes T,s, N, sim, binN)
    fpass_type is same as forward_pass -> just using the integer code for the forward pass
    final_run: for sddip algo, traversing the entire scenario tree to get deterministic bounds 
    
    """
    lb   = 0
    AList, TList, c, d, qDict = getAllSMKPdata(folder, id, inst, st, rows, cols, scens)

    build_time = @elapsed begin
        smkp_model = smkp_gen_ver4(st, lb, rows, cols, AList, TList, c, d, qDict, threads)
    end

    method_time = 0
    method_bound = "nan"
    method_ci_low = "nan"
    method_ci_high = "nan"
    cuts_std = "nan"
    cuts_nonstd = "nan"
    iterations = "nan"
    default_changes = 0
    simulation_time = 0
    sim_lower = "nan"           #the lower value from simulation
    sim_upper = "nan"           #the upper value from simulation
    ub = "nan"

    

    
    outputs = [(iter = iterations, time = method_time, bb = method_bound, ub = ub, low = method_ci_low, high = method_ci_high, cs = cuts_std, cns = cuts_nonstd, changesS = default_changes, ub_final = "nan")]
    total_scens = scens^(st-1)
    simCount = max(10, round(Int64, postSim*total_scens))
    
    try
        method_time = @elapsed begin
            
            
            outputs = train_method(smkp_model,
            duality_handler,
            forward_pass,
            backward_pass,
            sampling_scheme,
            time_limit,
            iter_limit,
            mipgap, 
            iter_pass,
            M,
            delta,
            fpass_type,
            final_run,
            type1_prob,
            type2_prob,
            seed)

        end

        simulation_time = @elapsed begin
            simulations = SDDP.simulate(
                smkp_model,
                simCount,
                simTime,
                set_sim_seed = true,
                sim_seed = 6*seed
            )

            objectives = map(simulations) do simulation
                return sum(stage[:stage_objective] for stage in simulation)
            end
            
            μ, ci = SDDP.confidence_interval(objectives)
            sim_lower = μ - ci
            sim_upper = μ + ci
        end

    catch e
        simulation_time = 0
        method_bound   = "error"
        method_ci_low  = "error"
        method_ci_high = "error"
        cuts_std       = "error"
        cuts_nonstd    = "error"
        println("Caught an exception: $(e)")
    end



    
    if !isfile(filepath)
        header_row = ("class", "dual", "bpass", "fpass", "spass", "run", "id", "seed", "T", "rows", "cols", "scens","delta","M", "type1_prob", "type2_prob", "seed", "mipgap","simCount", "maxTime", "b-time(s)", "t-time(s)", "s-time(s)", "sim-time", "iterations", "changes", "cuts_std", "cuts_nonstd", "l_bound", "ci_low", "ci_high", "sim_low", "sim_high","benders_ub", "ub_final")
        file = open(filepath, "w")
        CSV.write(file, [header_row], append = true)
        close(file)
    end

    if isnothing(seed)
        seed = -1
    end
    
    
    output = outputs[1]
    ub = output.ub
    row = ("smkp", string(duality_handler), string(backward_pass),  string(forward_pass), "InSample", run, id, seed, st, rows, cols, scens, delta, M, type1_prob, type2_prob, seed, mipgap, simCount, time_limit, build_time, method_time, output.time, simulation_time, output.iter, output.changesS, output.cs, output.cns, output.bb, output.low, output.high, sim_lower, sim_upper, ub, output.ub_final)
    file = open(filepath, "a")
    CSV.write(file, [row], append=true)
    close(file)


    
end


function smkp_nebe_runs_ver2(
    folder,
    filepath, 
    duality_handler, 
    forward_pass,
    backward_pass,
    sampling_scheme,
    run, 
    id, 
    inst, 
    rows, 
    cols, 
    st, 
    scens, 
    time_limit = 120,
    iter_limit = 2, 
    mipgap = 1e-4,
    iter_pass = 0,
    threads = 1,
    M = 1, 
    delta = 0.05,
    fpass_type = 2,
    final_run = false,
    postSim = 0.05,
    simTime = 3600,
    type1_prob = 1.28,
    type2_prob = 1.28,
    seed = nothing
)
 
    """
    instances (list): list of instance (has attributes T,s, N, sim, binN)
    fpass_type is same as forward_pass -> just using the integer code for the forward pass
    final_run: for sddip algo, traversing the entire scenario tree to get deterministic bounds 
    ver2: more up to date with the parameters it takes as inputs
          compared with smkp_method_runs, we are not doing simulation post the training step
          becausethis function is for Nested Benders Runs
    """

    lb   = 0

    AList, TList, c, d, qDict = getAllSMKPdata(folder, id, inst, st, rows, cols, scens)

    build_time = @elapsed begin
        smkp_model = smkp_gen_ver4(st, lb, rows, cols, AList, TList, c, d, qDict, threads)
    end

    method_time = 0
    method_bound = "nan"
    method_ci_low = "nan"
    method_ci_high = "nan"
    cuts_std = "nan"
    cuts_nonstd = "nan"
    iterations = "nan"
    default_changes = 0
    simulation_time = 0
    sim_lower = "nan"           #the lower value from simulation
    sim_upper = "nan"           #the upper value from simulation
    ub = "nan"

    

    
    outputs = [(iter = iterations, time = method_time, bb = method_bound, ub = ub, low = method_ci_low, high = method_ci_high, cs = cuts_std, cns = cuts_nonstd, changesS = default_changes, ub_final = "nan")]
    total_scens = scens^(st-1)
    simCount = max(10, round(Int64, postSim*total_scens))
    
    try
        method_time = @elapsed begin
            
            
            outputs = train_method(smkp_model,
            duality_handler,
            forward_pass,
            backward_pass,
            sampling_scheme,
            time_limit,
            iter_limit,
            mipgap, 
            iter_pass,
            M,
            delta,
            fpass_type,
            final_run,
            type1_prob,
            type2_prob,
            seed)

        end

    catch e
        simulation_time = 0
        method_bound   = "error"
        method_ci_low  = "error"
        method_ci_high = "error"
        cuts_std       = "error"
        cuts_nonstd    = "error"
        println("Caught an exception: $(e)")
    end

    #in case of Nested Benders, the relevant column to look at is benders_ub
    if !isfile(filepath)
        header_row = ("class", "dual", "bpass", "fpass", "spass", "run", "id", "T", "rows", "cols", "scens","delta","M", "type1_prob", "type2_prob", "seed", "mipgap","simCount", "maxTime", "b-time(s)", "t-time(s)", "s-time(s)", "sim-time", "iterations", "changes", "cuts_std", "cuts_nonstd", "l_bound", "ci_low", "ci_high", "sim_low", "sim_high","benders_ub", "ub_final")
        file = open(filepath, "w")
        CSV.write(file, [header_row], append = true)
        close(file)
    end
    


    #output.low -> lower bound based on train values
    #output.high -> upper bound based on train values
    #sim_upper -> upper bound based on out-of-sample values
    #sim_lower -> lower bound based on out-of-sample values
    #output.ub_final -> computed from a final forward pass after the training step (traverses the entire scenario tree)
    
    output = outputs[1]
    ub = output.ub
    row = ("smkp", string(duality_handler), string(backward_pass),  string(forward_pass), "InSample", run, id, st, rows, cols, scens, delta, M, type1_prob, type2_prob, seed, mipgap, simCount, time_limit, build_time, method_time, output.time, simulation_time, output.iter, output.changesS, output.cs, output.cns, output.bb, output.low, output.high, sim_lower, sim_upper, ub, output.ub_final)
    file = open(filepath, "a")
    CSV.write(file, [row], append=true)
    close(file)

end


function smkp_detr_runs(filepath, id, inst, rows, cols, st, scens, time_limit, threads, mipgap)

    """
    solution of the extensive form of the SMKP problem
    this function is old: check smkp_DE file for doing DE runs
    do "compare" on smkp_DE and smkp_NEBE file to see the difference
    """
    
    lb   = 0
    
    build_time = 0
    solve_time = 0
    method_objval = "nan"
    best_bd = "nan"
    gap_at_end = "nan"
    var_count = "nan"
    constr_count = "nan"


    AList, TList, c, qDict = getAllSMKPdata(folder, id, inst, st, rows, cols, scens)

    try
        build_time = @elapsed begin
            smkp_model = smkp_gen_ver4(st, lb, rows, cols, AList, TList, c, qDict, threads)
            det_model = SDDP.deterministic_equivalent(smkp_model, Gurobi.Optimizer, solver_threads = threads)
        end

        solve_time = @elapsed begin
            set_optimizer_attribute(det_model, "TimeLimit", max(time_limit - build_time, 60))
            set_optimizer_attribute(det_model, "mip_gap", mipgap)
            JuMP.optimize!(det_model)
        end

        method_objval = JuMP.objective_value(det_model)
        best_bd       = JuMP.objective_bound(det_model)
        gap_at_end    = MOI.get(det_model, MOI.RelativeGap())
        var_count     = JuMP.num_variables(det_model)
        constr_count  = JuMP.num_constraints(det_model; count_variable_in_set_constraints = false)

    catch e
        method_time   = "error"
        gap_at_end    = "error"
        method_objval = "error"
        best_bd       = "error"
        println("Caught an exception: $(e)")
    end

    if !isfile(filepath)
        header_row = ("class", "method", "id", "inst", "T", "rows", "cols", "scens", "vars_detr", "constr_detr", "threads", "mipgap", "maxTime", "b-time(s)", "s-time(s)", "obj", "bound", "gap")
        file = open(filepath, "w")
        CSV.write(file, [header_row], append = true)
        close(file)
    end

    row = ("smkp", "detr", id, inst, st, rows, cols, scens, var_count, constr_count, threads, mipgap, time_limit, build_time, solve_time, method_objval, best_bd, gap_at_end)
    file = open(filepath, "a")
    CSV.write(file, [row], append=true)
    close(file)

end



folder  = "/home/akul/sddp_comp/data/"

# allInst = [[145, 2, 2, 6, 2], [144, 3, 2, 5, 2], [117, 3, 10, 120, 2], [136, 3, 10, 60, 2], [137, 3, 10, 60, 3], [138, 3, 10, 60, 4], [139, 3, 10, 60, 5], [140, 4, 10, 60, 2], [141, 4, 10, 60, 3], [142, 4, 10, 60, 4], [143, 4, 10, 60, 5]]
# allInst = [[146, 3, 10, 120, 3], [147, 3, 10, 120, 3], [148, 3, 10, 120, 3], [149, 3, 10, 120, 5], [150, 3, 10, 120, 5], [151, 3, 10, 120, 5], [152, 3, 10, 60, 3], [153, 3, 10, 60, 3], [154, 3, 10, 60, 3], [155, 3, 10, 60, 5], [156, 3, 10, 60, 5], [157, 3, 10, 60, 5], [158, 3, 40, 120, 3], [159, 3, 40, 120, 3], [160, 3, 40, 120, 3], [161, 3, 40, 120, 5], [162, 3, 40, 120, 5], [163, 3, 40, 120, 5]]
# allInst = [[164, 3, 10, 80, 3], [165, 3, 10, 80, 3], [166, 3, 10, 80, 3], [167, 3, 10, 80, 5], [168, 3, 10, 80, 5], [169, 3, 10, 80, 5], [170, 3, 10, 60, 3], [171, 3, 10, 60, 3], [172, 3, 10, 60, 3], [173, 3, 10, 60, 5], [174, 3, 10, 60, 5], [175, 3, 10, 60, 5], [176, 3, 10, 30, 3], [177, 3, 10, 30, 3], [178, 3, 10, 30, 3], [179, 3, 10, 30, 5], [180, 3, 10, 30, 5], [181, 3, 10, 30, 5]]
# allInst = [[182, 3, 10, 20, 3], [183, 3, 10, 20, 3], [184, 3, 10, 20, 3], [185, 3, 10, 20, 5], [186, 3, 10, 20, 5], [187, 3, 10, 20, 5], [176, 3, 10, 30, 3], [177, 3, 10, 30, 3], [178, 3, 10, 30, 3], [179, 3, 10, 30, 5], [180, 3, 10, 30, 5], [181, 3, 10, 30, 5], [188, 3, 10, 40, 3], [189, 3, 10, 40, 3], [190, 3, 10, 40, 3], [191, 3, 10, 40, 5], [192, 3, 10, 40, 5], [193, 3, 10, 40, 5]]
# allInst =  [[176, 3, 10, 30, 3]]
# allInst = [[226, 2, 2, 5, 2], [144, 3, 2, 5, 2], [227, 4, 2, 5, 2], [228, 5, 2, 5, 2], [229, 2, 2, 5, 3], [230, 3, 2, 5, 3]]
# allInst = [[176, 3, 10, 30, 3], [231, 4, 10, 30, 3], [179, 3, 10, 30, 5]]
# allInst = [[234, 4, 10, 30, 10], [235, 5, 15, 40, 3], [236, 5, 15, 40, 10]]
# allInst = [[237, 3, 10, 30, 3], [238, 3, 10, 30, 5], [239, 4, 10, 30, 3], [240, 4, 10, 30, 10], [241, 5, 15, 40, 3], [242, 5, 15, 40, 10], [243, 3, 10, 30, 3], [244, 3, 10, 30, 5], [245, 4, 10, 30, 3], [246, 4, 10, 30, 10], [247, 5, 15, 40, 3], [248, 5, 15, 40, 10]]
# allInst = [[145, 2, 2, 6, 2]]
# allInst = [[227, 4, 2, 5, 2]]
# allInst = [[228, 5, 2, 5, 2]]

allInst = [[249, 3, 10, 20, 3], [250, 3, 10, 20, 3], [251, 3, 10, 20, 3], [252, 3, 10, 20, 3], [253, 3, 10, 20, 3], [254, 3, 10, 30, 3], [255, 3, 10, 30, 3], [256, 3, 10, 30, 3], [257, 3, 10, 30, 3], [258, 3, 10, 30, 3], [259, 3, 10, 40, 3], [260, 3, 10, 40, 3], [261, 3, 10, 40, 3], [262, 3, 10, 40, 3], [263, 3, 10, 40, 3], [264, 3, 15, 20, 3], [265, 3, 15, 20, 3], [266, 3, 15, 20, 3], [267, 3, 15, 20, 3], [268, 3, 15, 20, 3], [269, 3, 15, 30, 3], [270, 3, 15, 30, 3], [271, 3, 15, 30, 3], [272, 3, 15, 30, 3], [273, 3, 15, 30, 3], [274, 3, 15, 40, 3], [275, 3, 15, 40, 3], [276, 3, 15, 40, 3], [277, 3, 15, 40, 3], [278, 3, 15, 40, 3], [279, 3, 10, 20, 4], [280, 3, 10, 20, 4], [281, 3, 10, 20, 4], [282, 3, 10, 20, 4], [283, 3, 10, 20, 4], [284, 3, 10, 30, 4], [285, 3, 10, 30, 4], [286, 3, 10, 30, 4], [287, 3, 10, 30, 4], [288, 3, 10, 30, 4], [289, 3, 10, 40, 4], [290, 3, 10, 40, 4], [291, 3, 10, 40, 4], [292, 3, 10, 40, 4], [293, 3, 10, 40, 4], [294, 3, 15, 20, 4], [295, 3, 15, 20, 4], [296, 3, 15, 20, 4], [297, 3, 15, 20, 4], [298, 3, 15, 20, 4], [299, 3, 15, 30, 4], [300, 3, 15, 30, 4], [301, 3, 15, 30, 4], [302, 3, 15, 30, 4], [303, 3, 15, 30, 4], [304, 3, 15, 40, 4], [305, 3, 15, 40, 4], [306, 3, 15, 40, 4], [307, 3, 15, 40, 4], [308, 3, 15, 40, 4], [309, 3, 10, 20, 5], [310, 3, 10, 20, 5], [311, 3, 10, 20, 5], [312, 3, 10, 20, 5], [313, 3, 10, 20, 5], [314, 3, 10, 30, 5], [315, 3, 10, 30, 5], [316, 3, 10, 30, 5], [317, 3, 10, 30, 5], [318, 3, 10, 30, 5], [319, 3, 10, 40, 5], [320, 3, 10, 40, 5], [321, 3, 10, 40, 5], [322, 3, 10, 40, 5], [323, 3, 10, 40, 5], [324, 3, 15, 20, 5], [325, 3, 15, 20, 5], [326, 3, 15, 20, 5], [327, 3, 15, 20, 5], [328, 3, 15, 20, 5], [329, 3, 15, 30, 5], [330, 3, 15, 30, 5], [331, 3, 15, 30, 5], [332, 3, 15, 30, 5], [333, 3, 15, 30, 5], [334, 3, 15, 40, 5], [335, 3, 15, 40, 5], [336, 3, 15, 40, 5], [337, 3, 15, 40, 5], [338, 3, 15, 40, 5], [339, 4, 10, 20, 3], [340, 4, 10, 20, 3], [341, 4, 10, 20, 3], [342, 4, 10, 20, 3], [343, 4, 10, 20, 3], [344, 4, 10, 30, 3], [345, 4, 10, 30, 3], [346, 4, 10, 30, 3], [347, 4, 10, 30, 3], [348, 4, 10, 30, 3], [349, 4, 10, 40, 3], [350, 4, 10, 40, 3], [351, 4, 10, 40, 3], [352, 4, 10, 40, 3], [353, 4, 10, 40, 3], [354, 4, 15, 20, 3], [355, 4, 15, 20, 3], [356, 4, 15, 20, 3], [357, 4, 15, 20, 3], [358, 4, 15, 20, 3], [359, 4, 15, 30, 3], [360, 4, 15, 30, 3], [361, 4, 15, 30, 3], [362, 4, 15, 30, 3], [363, 4, 15, 30, 3], [364, 4, 15, 40, 3], [365, 4, 15, 40, 3], [366, 4, 15, 40, 3], [367, 4, 15, 40, 3], [368, 4, 15, 40, 3], [369, 4, 10, 20, 4], [370, 4, 10, 20, 4], [371, 4, 10, 20, 4], [372, 4, 10, 20, 4], [373, 4, 10, 20, 4], [374, 4, 10, 30, 4], [375, 4, 10, 30, 4], [376, 4, 10, 30, 4], [377, 4, 10, 30, 4], [378, 4, 10, 30, 4], [379, 4, 10, 40, 4], [380, 4, 10, 40, 4], [381, 4, 10, 40, 4], [382, 4, 10, 40, 4], [383, 4, 10, 40, 4], [384, 4, 15, 20, 4], [385, 4, 15, 20, 4], [386, 4, 15, 20, 4], [387, 4, 15, 20, 4], [388, 4, 15, 20, 4], [389, 4, 15, 30, 4], [390, 4, 15, 30, 4], [391, 4, 15, 30, 4], [392, 4, 15, 30, 4], [393, 4, 15, 30, 4], [394, 4, 15, 40, 4], [395, 4, 15, 40, 4], [396, 4, 15, 40, 4], [397, 4, 15, 40, 4], [398, 4, 15, 40, 4], [399, 4, 10, 20, 5], [400, 4, 10, 20, 5], [401, 4, 10, 20, 5], [402, 4, 10, 20, 5], [403, 4, 10, 20, 5], [404, 4, 10, 30, 5], [405, 4, 10, 30, 5], [406, 4, 10, 30, 5], [407, 4, 10, 30, 5], [408, 4, 10, 30, 5], [409, 4, 10, 40, 5], [410, 4, 10, 40, 5], [411, 4, 10, 40, 5], [412, 4, 10, 40, 5], [413, 4, 10, 40, 5], [414, 4, 15, 20, 5], [415, 4, 15, 20, 5], [416, 4, 15, 20, 5], [417, 4, 15, 20, 5], [418, 4, 15, 20, 5], [419, 4, 15, 30, 5], [420, 4, 15, 30, 5], [421, 4, 15, 30, 5], [422, 4, 15, 30, 5], [423, 4, 15, 30, 5], [424, 4, 15, 40, 5], [425, 4, 15, 40, 5], [426, 4, 15, 40, 5], [427, 4, 15, 40, 5], [428, 4, 15, 40, 5], [429, 5, 10, 20, 3], [430, 5, 10, 20, 3], [431, 5, 10, 20, 3], [432, 5, 10, 20, 3], [433, 5, 10, 20, 3], [434, 5, 10, 30, 3], [435, 5, 10, 30, 3], [436, 5, 10, 30, 3], [437, 5, 10, 30, 3], [438, 5, 10, 30, 3], [439, 5, 10, 40, 3], [440, 5, 10, 40, 3], [441, 5, 10, 40, 3], [442, 5, 10, 40, 3], [443, 5, 10, 40, 3], [444, 5, 15, 20, 3], [445, 5, 15, 20, 3], [446, 5, 15, 20, 3], [447, 5, 15, 20, 3], [448, 5, 15, 20, 3], [449, 5, 15, 30, 3], [450, 5, 15, 30, 3], [451, 5, 15, 30, 3], [452, 5, 15, 30, 3], [453, 5, 15, 30, 3], [454, 5, 15, 40, 3], [455, 5, 15, 40, 3], [456, 5, 15, 40, 3], [457, 5, 15, 40, 3], [458, 5, 15, 40, 3], [459, 5, 10, 20, 4], [460, 5, 10, 20, 4], [461, 5, 10, 20, 4], [462, 5, 10, 20, 4], [463, 5, 10, 20, 4], [464, 5, 10, 30, 4], [465, 5, 10, 30, 4], [466, 5, 10, 30, 4], [467, 5, 10, 30, 4], [468, 5, 10, 30, 4], [469, 5, 10, 40, 4], [470, 5, 10, 40, 4], [471, 5, 10, 40, 4], [472, 5, 10, 40, 4], [473, 5, 10, 40, 4], [474, 5, 15, 20, 4], [475, 5, 15, 20, 4], [476, 5, 15, 20, 4], [477, 5, 15, 20, 4], [478, 5, 15, 20, 4], [479, 5, 15, 30, 4], [480, 5, 15, 30, 4], [481, 5, 15, 30, 4], [482, 5, 15, 30, 4], [483, 5, 15, 30, 4], [484, 5, 15, 40, 4], [485, 5, 15, 40, 4], [486, 5, 15, 40, 4], [487, 5, 15, 40, 4], [488, 5, 15, 40, 4], [489, 5, 10, 20, 5], [490, 5, 10, 20, 5], [491, 5, 10, 20, 5], [492, 5, 10, 20, 5], [493, 5, 10, 20, 5], [494, 5, 10, 30, 5], [495, 5, 10, 30, 5], [496, 5, 10, 30, 5], [497, 5, 10, 30, 5], [498, 5, 10, 30, 5], [499, 5, 10, 40, 5], [500, 5, 10, 40, 5], [501, 5, 10, 40, 5], [502, 5, 10, 40, 5], [503, 5, 10, 40, 5], [504, 5, 15, 20, 5], [505, 5, 15, 20, 5], [506, 5, 15, 20, 5], [507, 5, 15, 20, 5], [508, 5, 15, 20, 5], [509, 5, 15, 30, 5], [510, 5, 15, 30, 5], [511, 5, 15, 30, 5], [512, 5, 15, 30, 5], [513, 5, 15, 30, 5], [514, 5, 15, 40, 5], [515, 5, 15, 40, 5], [516, 5, 15, 40, 5], [517, 5, 15, 40, 5], [518, 5, 15, 40, 5]]


duals    = [SDDP.LagrangianDuality(), SDDP.LaporteLouveauxDuality()]
bpass    = [SDDP.DefaultMultiBackwardPass(), SDDP.AnguloMultiBackwardPass()]
fpass    = [SDDP.DefaultMultiForwardPass(), SDDP.DefaultNestedForwardPass()]
spass    = [SDDP.InSampleMonteCarloMultiple(), SDDP.AllSampleMonteCarloMultiple()]
gList    = [0.01, 0.05, 0.0001, 0.10]
cdfinv   = [1.03, 1.28, 1.64]

                                                         
y               = parse(Int64, ARGS[1])      #instance
z               = parse(Int64, ARGS[2])      #duality
x               = parse(Int64, ARGS[3])      #determines the forward pass and sampling scheme (x = 2 for Nested Benders)
q               = parse(Int64, ARGS[4])      #determines the backward pass that we are going to use
M               = parse(Int64, ARGS[5])      #number of scenario paths sampled
delta           = parse(Int64, ARGS[6])      #delta value -> corresponds to the gap used for termination
threads         = parse(Int64, ARGS[7])      #threads in the solver
time_limit      = parse(Int64, ARGS[8])      #time limit on the algorithm
iter_limit      = parse(Int64, ARGS[9])      #number of iterations in the problem
final_run       = parse(Int64, ARGS[10])     #if 1 then in sddip algorithm the entire scenario tree is traversed to compute deterministic bounds
prob            = parse(Int64, ARGS[11])     #used in tito's stoping criterions
seed            = parse(Int64, ARGS[12])     #determines whether to set the seed or not (the seed is set to make the algorithms sddip methods comparable)
postSim         = 1e-2*parse(Int64, ARGS[13])#number of simulations to get the upper bound, determined as percentage of scenarios

if final_run == 1
    final_run = true
else
    final_run = false
end

if seed  < 1
    seed = nothing
end

# println("values retrieved")

inst      =  1
run       =  1
id        =  allInst[y][1]
st        =  allInst[y][2]
rows      =  allInst[y][3]
cols      =  allInst[y][4]
scens     =  allInst[y][5]
mipgap    = 1e-4
iter_pass = 1

current_dir = @__DIR__
folder      = current_dir*"/data/"
filename    = "smkp_NEBE_"
suffix      = "_$(id)_$(mipgap_i)_$(mintime).csv"
finalpath   = folder*filename*suffix

#### NOTE: USE THIS FOR SMKP SDDIP RUNS
smkp_method_runs(
    folder,
    finalpath, 
    duals[z], 
    fpass[x],
    bpass[q],
    spass[x],
    run,
    id, 
    inst, 
    rows, 
    cols, 
    st, 
    scens,
    time_limit,
    iter_limit, 
    mipgap,
    iter_pass,
    threads,
    M,
    gList[delta],
    x,
    final_run,
    postSim,
    time_limit,
    cdfinv[prob],
    cdfinv[prob],
    seed)


#### USE THIS FOR SMKP NESTED BENDERS RUNS
# smkp_nebe_runs_ver2(
#     folder,
#     finalpath, 
#     duals[z], 
#     fpass[x],
#     bpass[q],
#     spass[x],
#     run,
#     id, 
#     inst, 
#     rows, 
#     cols, 
#     st, 
#     scens,
#     time_limit,
#     iter_limit, 
#     mipgap,
#     iter_pass,
#     threads,
#     M,
#     gList[delta],
#     x,
#     final_run,
#     postSim,
#     time_limit,
#     cdfinv[prob],
#     cdfinv[prob],
#     seed)


