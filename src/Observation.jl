########## observation ########
# Return the entire observation space.
POMDPs.observations(DP::DrivePOMDP) = DP.OSpace
POMDPs.n_observations(DP::DrivePOMDP) = length(DP.OSpace)
function POMDPs.obsindex(DP::DrivePOMDP, Ob::CarOb)
    result = findfirst(x->x==Ob, DP.OSpace)
    if result == nothing
        error("Observation not found: $Ob")
    else
        return result
    end
end

# generates the probability distribution over observations after taking action a and ending in the new state x'
function POMDPs.observation(DP::DrivePOMDP, newSs::Sts)
    #println("enter observation")
    OtherPos = GetGlobalPosition(DP.Routes[newSs.Other.r], newSs.Other.s)
    #OtherPos = GetGlobalPosition(DP.Routes[DP.SsInit.Other.r], newSs.Other.s)
    Ob_dist = zeros(length(DP.OSpace))
    weight = 1.5 # weight increases -> pfk1 decreases -> the weight of v_diff will decrease.
    N1 = Normal(0.0, DP.Δv)
    N2 = Normal(0.0, DP.Δs)
    for i = 1:length(DP.OSpace)
        Ob = DP.OSpace[i]
        fk1 = weight*abs(Ob.v - newSs.Other.v)
        Pfk1 = cdf(N1, -fk1)
        fk2 = Distance2D(Point2D{Float64}(Ob.x, Ob.y), OtherPos)
        Pfk2 = cdf(N2, -weight*fk2)
        pfk = Pfk1*Pfk2
        Ob_dist[i] = pfk > 1.0e-3 ? pfk : 0.0
    end
    normalize!(Ob_dist, 1)
    return SparseCat(DP.OSpace, Ob_dist)  # return type is a distribution object, but doesn't need to be the same as DiscreteBelief
end
