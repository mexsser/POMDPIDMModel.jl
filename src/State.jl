########### state #############
function CSRound!(CS::CarSt, spre::Float64, vpre::Float64)
    CS.s = round(CS.s, spre)
    CS.v = round(CS.v, vpre)
    return CS
end

function CSRound(CS::CarSt, spre::Float64, vpre::Float64)
    newCS = deepcopy(CS)
    CSRound!(newCS, spre, vpre)
    return newCS
end

function SSRound(SS::Sts, spre::Float64, vpre::Float64)
    newSS = deepcopy(SS) #
    CSRound!(newSS.Ego, spre, vpre)
    CSRound!(newSS.Other, spre, vpre)
    return newSS
end

# helper function to handel the situation when calculated s and v go out of range
function sv_boundry!(DP::DrivePOMDP, CS::CarSt)
    CS.s = min(CS.s, DP.sV[end])
    CS.s = max(CS.s, DP.sV[1])
    CS.v = min(CS.v, DP.vV[end])
    CS.v = max(CS.v, DP.vV[1])
end

POMDPs.states(DP::DrivePOMDP) = DP.SSpace # the state space in this model
POMDPs.n_states(DP::DrivePOMDP) = length(DP.SSpace)
function POMDPs.stateindex(DP::DrivePOMDP, Ss::Sts)
      Ss_R = SSRound(Ss, DP.Δs, DP.Δv)
      sv_boundry!(DP, Ss_R.Ego)
      sv_boundry!(DP, Ss_R.Other)
      try
          return findfirst(x->x==Ss_R, DP.SSpace)
      catch
          error("state not found: $Ss")
      end
end
