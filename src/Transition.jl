function sv_next!(Car::CarSt, a::Float64, Δt::Float64)
    Car.s += Car.v*Δt + Δt^2*a/2
    Car.v += Δt*a
    return nothing
end

function sv_next(Car::CarSt, a::Float64, Δt::Float64)
    Car_next = deepcopy(Car)
    sv_next!(Car_next, a, Δt)
    return Car_next
end

########## transition ########
# generate the probability of ending in state x' when executing action a in state x
function POMDPs.transition(DP::DrivePOMDP, Ss::Sts, Aego::Symbol)
    acc_k = 0.0
    akV = [e for e in DP.Aset.min:DP.Aset.max]
    acc_distribution = DiscreteND1D(acc_k, DP.Aset.comfort/2, akV, 1.0)
    Ego_next = Egotransit(DP, Ss, Aego).nextSt

    PD = zeros(POMDPs.n_states(DP))
    for (i, ak) in enumerate(akV)
        if acc_distribution[i] > 1.0e-2
            Other_next = sv_next(Ss.Other, ak, DP.Δt) # sign state_next in every loop
            SIndex = POMDPs.stateindex(DP, Sts(Ego_next, Other_next)) # round is made in stateindex()
            PD[SIndex] += acc_distribution[i]
        end
    end
    normalize!(PD, 1)
    #return DiscreteBelief(DP, PD) # toooooo slow
    return SparseCat(DP.SSpace, PD)
end

function Egotransit(DP::DrivePOMDP, Ss::Sts, Aego::Symbol)
    Ego_next = deepcopy(Ss.Ego)
    Other_next = deepcopy(Ss.Other)
    accVec = Vector{Float64}()
    overlap = false
    dist1 = 0.0
    distk = 0.0

    if haskey(DP.Routes[Ss.Ego.r].intersect_Infos, Ss.Other.r)
        PtDistVec1 = DP.Routes[Ss.Ego.r].intersect_Infos[Ss.Other.r]
        if length(PtDistVec1) > 1 && PtDistVec1[2][1] == "overlap"
            dist1 = PtDistVec1[2][3]
            distk = DP.Routes[Ss.Other.r].intersect_Infos[Ss.Ego.r][2][3]
            overlap = true
        end
    end

    for i in 0:Int64(DP.Δt/0.1)-1 # update the state of EgoCar
        acc_ego = 0.0
        if overlap
            Δs1 = Ego_next.s - dist1
            Δs2 = Other_next.s - distk
            if Δs1 > 0 && Δs2 > 0 && Δs2 > Δs1
                acc_ego = Accselect(DP, Ego_next, Other_next, Aego, Δs2 - Δs1)
                acc_ego = Acclimit!(DP, acc_ego)
            else
                acc_ego = Accselect(DP, Ego_next, Aego) # only based on stopline, the state of other car is not relative
                acc_ego = Acclimit!(DP, acc_ego)
            end
        else
            acc_ego = Accselect(DP, Ego_next, Aego) # only based on stopline, the state of other car is not relative
            acc_ego = Acclimit!(DP, acc_ego)
        end
        push!(accVec, acc_ego)
        #@show acc_ego
        sv_next!(Ego_next, acc_ego, 0.1)
        #@show Ego_next
    end
    CSRound!(Ego_next, DP.Δs, DP.Δv)
    return (nextSt=Ego_next, accs=accVec)
end
