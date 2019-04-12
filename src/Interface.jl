########################### POMDPModel Implementation #################
"""
Rules
1) All POMDP functions should not change the input parameter(s) except DP object !!!
2) All calls to search functions(eg. findfirst()) should be wrapped by "try...catch" block
"""

function POMDPs.initialstate_distribution(DP::DrivePOMDP)
    #println("Distribution Initialization")
    PD = zeros(length(DP.SSpace))
    weight = 1.5
    ss_init = deepcopy(DP.SsInit)
    for ri in DP.rV
        ss_init.Other.r = ri
        index_init = 0
        try
            index_init = findfirst(x->x==ss_init, DP.SSpace)
        catch
            error("initial state not found in Statespace")
        end
        fk1 = weight*abs(ss_init.Other.v - DP.Routes[ri].Vref[1])
        Pfk1 = cdf(Normal(0.0, 2.0), -fk1)
        PD[index_init] = Pfk1
    end
    normalize!(PD, 1)
    return SparseCat(DP.SSpace, PD)
end

POMDPs.discount(DP::DrivePOMDP) = DP.discount_factor

function POMDPs.isterminal(DP::DrivePOMDP, Ss::Sts)
    result = Ss.Ego.s >= DP.sV[end] || Ss.Other.s >= DP.sV[end]
    #if result
    #    println("enter terminal state")
    #end
    return result
end

function Simulator(DP::DrivePOMDP, up::Updater, policy::Policy, rng::AbstractRNG)
    StsVec = Vector{Sts}()
    ObsVec = Vector{CarOb}()
    ActVec = Vector{Symbol}()
    AccMat = Vector{Vector{Float64}}()
    b = initialize_belief(up, initialstate_distribution(DP))
    BeliefVec = [b,]
    r_total = 0.0
    d = 1.0
    s = deepcopy(initialstate(DP, rng))
    #s = deepcopy(rand(rng, b))
    i = 1
    while !isterminal(DP, s)
        println("-------- Step: $i --------")
        push!(StsVec, s)
        Act_ego = action(policy, b) # b will be changed in every step
        push!(ActVec, Act_ego)
        #sp = rand(rng, transition(DP, s, a))
        sp_cat = transition(DP, s, Act_ego)

        sp_p, spi = findmax(sp_cat.probs)
        sp = sp_cat.vals[spi]
        #sp = rand(rng, sp_cat)
        #acc_ego = (sp.Ego.v^2 - s.Ego.v^2)/(2*(sp.Ego.s - s.Ego.s))
        acc_k = (sp.Other.v^2 - s.Other.v^2)/(2*(sp.Other.s - s.Other.s))
        accVec = IDMtransit(DP, s, Act_ego, acc_k).accs
        push!(AccMat, accVec)

        r = reward(DP, s, Act_ego, sp)
        r_total += d*r

        # sp_real is only for observation generation
        sp_real = deepcopy(sp)
        sp_real.Other.r = DP.SsInit.Other.r
        #o = rand(rng, observation(DP, sp_real))
        ob_cat = observation(DP, sp_real)
        #println("--- show ob distribution ---")
        #show_cat(ob_cat, 0x003)
        #println("--- -------------------- ---")
        ob_p, obi = findmax(ob_cat.probs)
        o = ob_cat.vals[obi]
        push!(ObsVec, o)

        @show s
        @show Act_ego#, acc_ego
        @show o
        println("realtime reward: $r")
        @show r_total

        d *= discount(DP)
        b = deepcopy(update(up, b, Act_ego, o))
        push!(BeliefVec, b)
        probmax, probi = findmax(b.b)
        s = deepcopy(b.state_list[probi])
        #s = deepcopy(rand(rng, b))
        i += 1
    end
    return (StsVec, ObsVec, BeliefVec, ActVec, AccMat)
end


function update(bu::DiscreteUpdater, b::DiscreteBelief, a::Symbol, o::CarOb)
    #println("----- update -----")
    pomdp = b.pomdp
    state_space = b.state_list
    bp = zeros(length(state_space))
    for spi = 1:length(state_space)
        sp = state_space[spi]
        od = observation(pomdp, sp)
        po = pdf(od, o)
        if po > 1e-3
            b_sum = 0.0
            for si = 1:length(state_space)
                if b.b[si] > 1e-2
                     s = state_space[si]
                     td = transition(pomdp, s, a)
                     pp = pdf(td, sp)
                     b_sum += pp * b.b[si]
                 end
             end
             bp[spi] = po * b_sum
         end
    end
    bp_sum = sum(bp)
    if bp_sum == 0.0
        error("""
              Failed discrete belief update: new probabilities sum to zero.

              a = $a
              o = $o

              Failed discrete belief update: new probabilities sum to zero.
              """)
    else
        for i = 1:length(bp); bp[i] /= bp_sum; end
    end
    return DiscreteBelief(pomdp, b.state_list, bp)
end

update(bu::DiscreteUpdater, b::Any, a::Symbol, o::CarOb) = update(bu, initialize_belief(bu, b), a::Symbol, o::CarOb)

function update_info(bu::DiscreteUpdater, b::Any, a::Symbol, o::CarOb)
    return update(bu, b, a, o), nothing
end


function show_cat(cat::SparseCat, num::UInt16)
    cat_dict = Dict(zip(cat.vals, cat.probs))
    println(sort(collect(cat_dict), by = x->x[2])[end-num:end])
end

function show_DB(DB::DiscreteBelief{DrivePOMDP, Sts}, num::UInt16)
    DB_dict = Dict(zip(DB.state_list, DB.b))
    println(sort(collect(DB_dict), by = x->x[2])[end-num:end])
end
