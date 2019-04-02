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
    acc_k = 0.0 # we can also use Aref
    akV = [e for e in DP.Aset.min:DP.Aset.max]
    acc_distribution = DiscreteND1D(acc_k, DP.Aset.comfort/2, akV, 1.0)
    PD = zeros(POMDPs.n_states(DP))
    for (i, ak) in enumerate(akV)
        if acc_distribution[i] > 1.0e-2
            result = IDMtransit(DP, Ss, Aego, ak)
            SIndex = 0
            try
                SIndex = POMDPs.stateindex(DP, Sts(result.Ego, result.Other)) # round is made in stateindex()
            catch
                error("Transition Result error: $result")
            end
            PD[SIndex] += acc_distribution[i]
        end
    end
    normalize!(PD, 1)
    #return DiscreteBelief(DP, PD) # toooooo slow
    return SparseCat(DP.SSpace, PD)
end

function IDMtransit(DP::DrivePOMDP, Ss::Sts, Aego::Symbol, acc_k::Float64)

    if Ss.Ego.s <= 0.0 && Ss.Ego.v <= 0.0 && Aego == :giveup # avoid transition of edge states going out of state range
        Other = sv_next(Ss.Other, acc_k, DP.Δt)
        sv_boundry!(DP, Other)
        Ego = deepcopy(Ss.Ego)
        return (Ego=Ego, Other=Other, accs=zeros(Int64(DP.Δt/0.1)))
    end

    Ego = deepcopy(Ss.Ego)
    Other = deepcopy(Ss.Other)
    accVec = Vector{Float64}()
    overlap = false
    dist1 = 0.0
    distk = 0.0

    if haskey(DP.Routes[Ego.r].intersect_Infos, Other.r)
        PtDistVec1 = DP.Routes[Ego.r].intersect_Infos[Other.r]
        if length(PtDistVec1) > 1 && PtDistVec1[2][1] == "overlap"
            dist1 = PtDistVec1[2][3]
            distk = DP.Routes[Other.r].intersect_Infos[Ego.r][2][3]
            overlap = true
        end
    end

    step = Int64(DP.Δt/0.1)
    for i in 1:step+1 # update the state of EgoCar and OtherCar
        acc_ego = 0.0
        if overlap
            if Ego.s < DP.Stopline
                acc_ego = AccCalculate(DP, Ego, Aego) # only based on stopline, the state of other car is not directly used.
            else # Ego.s >= DP.Stopline
                if Aego == :giveup
                    Δs = Other.s - Ego.s + dist1 - distk
                    if Δs < DP.Smin # ego vehicle can not follow other vehicle because too close/ego car is in front of other car.
                        if Ego.v > 0 # vehicle should not go back
                            acc_ego = DP.Aset.min
                        else
                            acc_ego = 0.0
                        end
                    else # Δs >= DP.Smin; ego should follow other
                        acc_ego = AccCalculate(DP, Ego, Other, Aego, Δs)
                    end
                else # Aego == :takeover -> drives freely
                    Vref = DP.Routes[Ego.r].Vref[min(UInt16(floor(Ego.s/DP.Δs)+1), 21)]
                    acc_ego = IDM(Vego=Ego.v, Vfront=Vref, Vref=Vref, Snet=Inf, T=1.0, Amax=DP.Aset.max, Bdec=DP.Aset.comfort, Smin=DP.Smin)
                end
            end
        else # no overlap
            acc_ego = AccCalculate(DP, Ego, Aego)
        end
        acc_ego = Acclimit!(DP, acc_ego)
        push!(accVec, acc_ego)
        #@show acc_ego
        if i <= step
            sv_next!(Ego, acc_ego, 0.1)
            sv_boundry!(DP, Ego)
            sv_next!(Other, acc_k, 0.1)
            sv_boundry!(DP, Other)
        end
        #@show Ego
        #@show Other
    end

    #CSRound!(Ego, DP.Δs, DP.Δv)
    #CSRound!(Other, DP.Δs, DP.Δv)
    return (Ego=Ego, Other=Other, accs=accVec)
end
