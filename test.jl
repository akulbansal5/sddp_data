"""

A testbed of small SMIP instances to test if the SDDiP problem is running correctly
for sddip: after a "large" number of iterations the deterministic bounds should have <1% gap

(from SDDP.dev examples)

stochastic_all_blacks.jl 
    We are given N seats and Revenue R_ij for seat i in time period j
    It is uncertain in which period the offer for the seat i will come
    We need to determine if we should accept an offer for a seat or not in a particular scenario

booking_management_model

generation_expansion

Experiment we conduct:

        (a) duals: Lagrangian and Integer L-shaped cut
        (b) bpass: Default vs Angulo
        (c) fpass: Nested vs SDDiP (M = 2)
        (d) final run = 1 if SDDiP and 0 otherwise

"""


println(">>>>>>>>>>> Running test instances <<<<<<<<<<<<<<")
using SDDP, Gurobi, FileIO, CSV, DataFrames




function stochastic_all_blacks(
    threads
)
    # Number of time periods
    T = 3
    # Number of seats
    N = 2
    # R_ij = price of seat i at time j
    R = [3 3 6; 3 3 6]
    # Number of noises
    s = 3
    offers = [
        [[1, 1], [0, 0], [1, 1]],
        [[1, 0], [0, 0], [0, 0]],
        [[0, 1], [1, 0], [1, 1]],
    ]

    model = SDDP.LinearPolicyGraph(
        stages = T,
        sense = :Max,
        upper_bound = 100.0,
        optimizer = Gurobi.Optimizer,
        solver_threads = threads,
    ) do sp, stage
        # Seat remaining?
        @variable(sp, 0 <= x[1:N] <= 1, SDDP.State, Bin, initial_value = 1)
        # Action: accept offer, or don't accept offer
        # We are allowed to accept some of the seats offered but not others
        @variable(sp, accept_offer[1:N], Bin)
        @variable(sp, offers_made[1:N])
        # Balance on seats
        @constraint(
            sp,
            balance[i in 1:N],
            x[i].in - x[i].out == accept_offer[i]
        )
        @stageobjective(sp, sum(R[i, stage] * accept_offer[i] for i in 1:N))
        SDDP.parameterize(sp, offers[stage]) do o
            return JuMP.fix.(offers_made, o)
        end
        @constraint(sp, accept_offer .<= offers_made)
    end

    return model
end


function booking_management_model(
    threads
)

    num_days = 2
    num_rooms = 2
    num_requests = 4

    # maximum revenue that could be accrued.
    max_revenue = (num_rooms + num_requests) * num_days * num_rooms
    # booking_requests is a vector of {0,1} arrays of size
    # (num_days x num_rooms) if the room is requested.
    booking_requests = Array{Int,2}[]
    for room in 1:num_rooms
        for day in 1:num_days
            # note: length_of_stay is 0 indexed to avoid unnecessary +/- 1
            # on the indexing
            for length_of_stay in 0:(num_days-day)
                req = zeros(Int, (num_rooms, num_days))
                req[room:room, day.+(0:length_of_stay)] .= 1
                push!(booking_requests, req)
            end
        end
    end

    return model = SDDP.LinearPolicyGraph(
        stages = num_requests,
        upper_bound = max_revenue,
        sense = :Max,
        optimizer = Gurobi.Optimizer,
        solver_threads = threads
    ) do sp, stage
        @variable(
            sp,
            0 <= vacancy[room = 1:num_rooms, day = 1:num_days] <= 1,
            SDDP.State,
            Bin,
            initial_value = 1
        )
        @variables(
            sp,
            begin
                # Accept request for booking of room for length of time.
                0 <= accept_request <= 1, Bin
                # Accept a booking for an individual room on an individual day.
                0 <= room_request_accepted[1:num_rooms, 1:num_days] <= 1, Bin
                # Helper for JuMP.fix
                req[1:num_rooms, 1:num_days]
            end
        )
        for room in 1:num_rooms, day in 1:num_days
            @constraints(
                sp,
                begin
                    # Update vacancy if we accept a room request
                    vacancy[room, day].out ==
                    vacancy[room, day].in - room_request_accepted[room, day]
                    # Can't accept a request of a filled room
                    room_request_accepted[room, day] <= vacancy[room, day].in
                    # Can't accept invididual room request if entire request is declined
                    room_request_accepted[room, day] <= accept_request
                    # Can't accept request if room not requested
                    room_request_accepted[room, day] <= req[room, day]
                    # Accept all individual rooms is entire request is accepted
                    room_request_accepted[room, day] + (1 - accept_request) >= req[room, day]
                end
            )
        end
        SDDP.parameterize(sp, booking_requests) do request
            return JuMP.fix.(req, request)
        end
        @stageobjective(
            sp,
            sum(
                (room + stage - 1) * room_request_accepted[room, day] for
                room in 1:num_rooms for day in 1:num_days
            )
        )
    end
end

function generation_expansion(
    threads
)
    build_cost  = 1e4
    use_cost    = 4
    num_units   = 5
    capacities  = ones(num_units)
    demand_vals =
        0.5 * [
            5 5 5 5 5 5 5 5
            4 3 1 3 0 9 8 17
            0 9 4 2 19 19 13 7
            25 11 4 14 4 6 15 12
            6 7 5 3 8 4 17 13
        ]

    

    # Cost of unmet demand
    penalty = 5e5
    # Discounting rate
    rho = 0.99
    model = SDDP.LinearPolicyGraph(
        stages = 5,
        lower_bound = 0.0,
        optimizer = Gurobi.Optimizer,
        sense = :Min,
        solver_threads = threads,
    ) do sp, stage
        @variable(
            sp,
            invested[1:num_units],
            SDDP.State,
            Bin,
            initial_value = 0
        )
        @variables(sp, begin
            generation >= 0
            unmet >= 0
            demand
        end)

        @constraints(
            sp,
            begin
                # Can't un-invest
                investment[i in 1:num_units], invested[i].out >= invested[i].in
                # Generation capacity
                sum(capacities[i] * invested[i].out for i in 1:num_units) >=
                generation
                # Meet demand or pay a penalty
                unmet >= demand - sum(generation)
                # For fewer iterations order the units to break symmetry, units are identical (tougher numerically)
                [j in 1:(num_units-1)], invested[j].out <= invested[j+1].out
            end
        )
        # Demand is uncertain
        SDDP.parameterize(ω -> JuMP.fix(demand, ω), sp, demand_vals[stage, :])

        @expression(
            sp,
            investment_cost,
            build_cost *
            sum(invested[i].out - invested[i].in for i in 1:num_units)
        )
        @stageobjective(
            sp,
            (investment_cost + generation * use_cost) * rho^(stage - 1) +
            penalty * unmet
        )
    end

    return model
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

    

        
    # # println(typeof(outputs[1]))
    sddp_bound  = outputs[1].bb 
    sddp_simval = outputs[1].cumm_list[end]

    println("the bound attained from sddp:            $(sddp_bound)")
    println("the simulation value attained from sddp: $(sddp_simval)")
    println("================== end ====================")
    

    return outputs

end

function test_runs(
    filepath,
    model_builder,
    duality_handler, 
    forward_pass,
    backward_pass,
    sampling_scheme,
    time_limit = 120,
    iter_limit = 2, 
    mipgap = 1e-4,
    iter_pass = 0,
    threads = 1,
    M = 1, 
    delta = 0.05,
    fpass_type = 2,
    final_run = false,
    type1_prob = 1.28,
    type2_prob = 1.28,
    seed = nothing
)

    """
    runs all the test problems
    """


    method_time = 0
    method_bound = "nan"
    method_ci_low = "nan"
    method_ci_high = "nan"
    cuts_std = "nan"
    cuts_nonstd = "nan"
    iterations = "nan"
    default_changes = 0
    simulation_time = 0
    ub = "nan"
    
    build_time = @elapsed begin
        model_sab = model_builder(threads)
    end
    outputs = [(iter = iterations, time = method_time, bb = method_bound, ub = ub, low = method_ci_low, high = method_ci_high, cs = cuts_std, cns = cuts_nonstd, changesS = default_changes, ub_final = "nan")]

    try
        method_time = @elapsed begin
        
            outputs = train_method(model_sab,
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
        println("Caught an exception: \n $(e)")
    end


    if seed === nothing
        seed = 0
    end

    if !isfile(filepath)
        header_row = ("class", "dual", "bpass", "fpass", "spass", "delta", "M", "mipgap", "maxTime", "b-time(s)", "t-time(s)", "s-time(s)", "sim-time", "iterations", "changes", "cuts_std", "cuts_nonstd", "type1p", "type2p", "delta", "seed", "l_bound", "ci_low", "ci_high", "benders_ub", "ub_final")
        file = open(filepath, "w")
        CSV.write(file, [header_row], append = true)
        close(file)
    end

    

    output = outputs[1]
    ub = output.ub
    row = (string(model_builder), string(duality_handler), string(backward_pass),  string(forward_pass), "InSample", delta, M, mipgap, time_limit, build_time, method_time, output.time, simulation_time, output.iter, output.changesS, output.cs, output.cns, type1_prob, type2_prob, delta, seed, output.bb, output.low, output.high, ub, output.ub_final)
    file = open(filepath, "a")
    CSV.write(file, [row], append=true)
    close(file)

end

folder  = "/home/akul/sddp_comp/data/"
builders = [stochastic_all_blacks, booking_management_model, generation_expansion]
instnames = ["sab", "bmm", "ge"]

duals   = [SDDP.LagrangianDuality(), SDDP.LaporteLouveauxDuality()]
bpass   = [SDDP.DefaultMultiBackwardPass(), SDDP.AnguloMultiBackwardPass()]
fpass   = [SDDP.DefaultMultiForwardPass(), SDDP.DefaultNestedForwardPass()]
spass   = [SDDP.InSampleMonteCarloMultiple(), SDDP.AllSampleMonteCarloMultiple()]
gList   = [0.01, 0.05, 0.10]
cdfinv   = [0.38, 1.64]


y               = parse(Int64, ARGS[1])      #instance based on the list of builders
z               = parse(Int64, ARGS[2])      #duality
x               = parse(Int64, ARGS[3])      #determines the forward pass and sampling scheme
q               = parse(Int64, ARGS[4])      #determines the backward pass that we are going to use
M               = parse(Int64, ARGS[5])      #number of scenario paths sampled
delta           = parse(Int64, ARGS[6])      #delta value 
threads         = parse(Int64, ARGS[7])      #threads in the solver
time_limit      = parse(Int64, ARGS[8])      #time limit on the algorithm
iter_limit      = parse(Int64, ARGS[9])      #number of iterations in the problem
final_run       = parse(Int64, ARGS[10])     #in sddip algorithm check if the entire scenario tree is traversed to compute deterministic bounds
prob            = parse(Int64, ARGS[11])     #used in tito-s stoping criterions
seed            = parse(Int64, ARGS[12])     #determines whether to set the seed or not


if final_run == 1
    final_run = true
else
    final_run = false
end

if seed  < 1
    seed = nothing
end

mipgap    = 1e-4
iter_pass = 1
inst_name = instnames[y]

filepath  = "/home/akul/sddp_comp/data/"*inst_name
suffix    = "_Jan2024_$(z)_$(q)_$(x)_$(prob)_$(seed)_$(M).csv"
finalpath = filepath*suffix



test_runs(
    finalpath,
    builders[y],
    duals[z],
    fpass[x],
    bpass[q],
    spass[x],
    time_limit,
    iter_limit,
    mipgap,
    iter_pass,
    threads,
    M,
    gList[delta],
    x,
    final_run,
    cdfinv[prob],
    cdfinv[prob],
    seed)




# function test_runs(
#     filepath,
#     model_builder,
#     duality_handler, 
#     forward_pass,
#     backward_pass,
#     sampling_scheme,
#     time_limit = 120,
#     iter_limit = 2, 
#     mipgap = 1e-4,
#     iter_pass = 0,
#     threads = 1,
#     M = 1, 
#     delta = 0.05,
#     fpass_type = 2,
#     final_run = false,
#     type1_prob = 1.28,
#     type2_prob = 1.28,
# )