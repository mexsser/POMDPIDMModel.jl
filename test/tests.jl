using POMDPIDMModel
using BeliefUpdaters
using Random
import QMDP

"""
Rules
1) Pay attention that the length of all routes should cover sRange, but not the reverse,
   or will result in error like 'attempt to access 1-element Array{Geometry,1} at index [2]'
2) Curvature should be negative if route extends clockwise
"""

function DPObj(style::Symbol, Rother::UInt16, Rlength::Float64, gauge::Float64, θ::Float64, Stopline::Float64, Smin::Float64)
    # Init  Sts
    EgoCar = CarSt(0.0, 4.0, UInt16(1))
    OtherCar = CarSt(0.0, 3.0, Rother) # OtherCar.r is not used later.
    SsInit = Sts(EgoCar, OtherCar)

    # space range
    sRange = [0.0:1.0:Rlength;]
    vRange = [0.0:1.0:6.0;] # max velocity = 72 km/h
    Aset = (min=-2.0, max=2.0, comfort=1.0)
    # other parameters
    discount_factor = 0.9
    Δt = 1.0

    if style == :Crossroad
        Vgeos = [4.0 4.0 4.0; 4.0 2.0 4.0; 4.0 3.0 4.0; 4.0 4.0 4.0]
        Routes = RoutesGenerator(Vgeos, Rlength, gauge, θ, 0.15, 0.7; style=style)
        rRange = Vector{UInt16}([2, 3, 4]) # potential route index for other car
        return DrivePOMDP(discount_factor, Δt, sRange, vRange, rRange, Aset, Routes, SsInit, Stopline, Smin)
    elseif style == :TJunction
        Vgeos = [4.0 4.0 4.0; 4.0 2.0 4.0; 4.0 3.0 4.0]
        Routes = RoutesGenerator(Vgeos, Rlength, gauge, θ, 0.3, 0.35; style=style)
        rRange = Vector{UInt16}([2, 3])
        return DrivePOMDP(discount_factor, Δt, sRange, vRange, rRange, Aset, Routes, SsInit, Stopline, Smin)
    else
        error("Unsupported Style. Please choose either :Crossroad or :TJunction.")
    end
end

#function main()
    Rlength = 20.0
    gauge = 4.0
    θ = 110.0/(180/π)
    halflen = Rlength/2.0+7.0
    style = :TJunction; Stopline = 4.0
    #style = :Crossroad; Stopline = 10.0
    Smin = 2.0

    # init
    DP = DPObj(style, UInt16(2), Rlength, gauge, θ, Stopline, Smin)
    up = DiscreteUpdater{DrivePOMDP}(DP) # updater

    ################################################################################
    #################### Test Solvers ##############################################

    #= #solver FIB
    import FIB
    solver = FIB.FIBSolver(;max_iterations=4, tolerance=1e-4, verbose=false)
    policy = FIB.solve(solver, DP)
    =#

    #=
    # solver SARSOP
    import SARSOP
    solver = SARSOP.SARSOPSolver(;fast=false)
    policy = SARSOP.solve(solver, DP)
    =#

    #=# solver AEMS
    import AEMS
    solver = AEMS.AEMSSolver(max_iterations=4, updater=up)
    policy = solve(solver, DP)
    =#

    # solver QMDP
    solver = QMDP.QMDPSolver(max_iterations=8, tolerance=1e-4, verbose=true)
    policy = QMDP.solve(solver, DP; sparse=true)
    #policy = QMDP.solve(solver, DP)

    StsVec, ObsVec, BeliefVec, ActVec, AccMat = Simulator(DP, up, policy, MersenneTwister(rand(UInt32)))
    Visualisation(DP, StsVec, ObsVec, ActVec, AccMat, halflen, gauge, θ, style, "$style.R$(DP.SsInit.Ego.r)R$(DP.SsInit.Other.r).Passive")

#end

#main()
