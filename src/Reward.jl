########## reward ##########
#POMDPs.reward(DP::DrivePOMDP, Ss::Sts, acc_ego::Float64) = Reward(DP, Ss, acc_ego)
POMDPs.reward(DP::DrivePOMDP, Ss::Sts, Aego::Symbol, Ssnext::Sts) = Reward(DP, Ss, Aego, Ssnext)

# based directly on next state; used in DiscreteValueIteration Solver
function R_crash(DP::DrivePOMDP, Ssnext::Sts, acc_ego::Float64)
    Ego_nextPos = GetGlobalPosition(DP.Routes[Ssnext.Ego.r], min(Ssnext.Ego.s, DP.sV[end]))
    Other_nextPos = GetGlobalPosition(DP.Routes[Ssnext.Other.r], min(Ssnext.Other.s, DP.sV[end]))
    if haskey(DP.Routes[Ssnext.Ego.r].intersect_Infos, Ssnext.Other.r) # if two routes intesect with each other
        if Distance2D(Ego_nextPos, Other_nextPos) < 2 # crash happens
            return -100.0 - 1.5*Ssnext.Ego.v^2 - 2.6*acc_ego*abs(acc_ego)
        end
    end
    return 0.0
end

# include the situation that junction point is far away from every state-point but collision actually happens
function R_crash(DP::DrivePOMDP, Ss::Sts, Ssnext::Sts)
    if haskey(DP.Routes[Ss.Ego.r].intersect_Infos, Ss.Other.r) # if two routes intesect with each other
        for (i, PtDist) in enumerate(DP.Routes[Ss.Ego.r].intersect_Infos[Ss.Other.r])
            InterDistEgo = PtDist[3] # distance from start point to junction point
            InterDistOther = DP.Routes[Ss.Other.r].intersect_Infos[Ss.Ego.r][i][3]
            if Ss.Ego.s < InterDistEgo < Ssnext.Ego.s && Ss.Other.s < InterDistOther < Ssnext.Other.s
                return -150.0
            end
        end
    end
    return 0.0
end

"""
if Vego != Vref, R_v should always be less than R_acc, so that Ego will get a better reward if it chooses to accelerate/decelerate itself.
R_v and R_acc should have a relation between each other.
"""

function R_v(DP::DrivePOMDP, Ss::Sts, acc_ego::Float64)
    Kvp = 1.5
    Kvn = 1.0
    Vref = DP.Routes[Ss.Ego.r].Vref[UInt16(Ss.Ego.s/DP.Δs)+1] # m/s, should be defined as array according to Route/Geometries
    Vego = Ss.Ego.v + acc_ego*DP.Δt
    if Vego > Vref
        #return -Kvp*(Vref - Vego)^2
        return -Kvp*((Vref - Vego)*3.6)^2
    else
        return -Kvn*(Vref - Vego)*3.6
    end
end

function R_acc(DP::DrivePOMDP, Ss::Sts, acc_ego::Float64) #
    Factor = -0.77
    Δa = acc_ego - DP.Routes[Ss.Ego.r].Aref[UInt16(Ss.Ego.s/DP.Δs)+1]
    return Factor*(Δa^2+2.3*abs(Δa))
end


function Reward(DP::DrivePOMDP, Ss::Sts, Aego::Symbol, Ssnext::Sts)
    acc_ego = 0.0
    if Ssnext.Ego.s != Ss.Ego.s
        acc_ego = (Ssnext.Ego.v^2 - Ss.Ego.v^2)/(2*(Ssnext.Ego.s - Ss.Ego.s))
    end
    reward = R_crash(DP, Ss, Ssnext) + R_crash(DP, Ss, acc_ego) + R_crash(DP, Ssnext, acc_ego) + R_v(DP, Ss, acc_ego) + R_acc(DP, Ss, acc_ego)
    reward = trunc(reward; digits=4)
    println("## Realtime Reward:$reward")
    return reward
end
