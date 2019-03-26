using POMDPIDMModel
using BeliefUpdaters
using Random
using Plots
using Measures
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
    sRange = [0.0:0.1:Rlength;]
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
        Routes = RoutesGenerator(Vgeos, Rlength, gauge, θ, 0.25, 0.35; style=style)
        rRange = Vector{UInt16}([2, 3])
        return DrivePOMDP(discount_factor, Δt, sRange, vRange, rRange, Aset, Routes, SsInit, Stopline, Smin)
    else
        error("Unsupported Style. Please choose either :Crossroad or :TJunction.")
    end
end


Rlength = 24.0
gauge = 4.0
θ = 110.0/(180/π)
halflen = Rlength/2.0+7.0
style = :TJunction; Stopline = 4.0
#style = :Crossroad; Stopline = 10.0
Smin = 2.0

# init
DP = DPObj(style, UInt16(2), Rlength, gauge, θ, Stopline, Smin)
pltJunction = Junction(halflen, gauge, θ; style=style)
pltMap = PlotRoutes(DP, pltJunction)
DrawStopline!(pltMap, DP.Routes[DP.SsInit.Ego.r], DP.Stopline, gauge)
plot(pltMap, size=(600, 400))
#plot(pltMap, size=(600, 500))
savefig("output/Figure4.6a.pdf")


#=
Svec = [i for i in 0.0:0.1:Rlength]
idx_r12 = findfirst(x->x==3.0, DP.Routes[3].Vref)+1
Svec_r1 = Svec[1:idx_r12]
Vref_r1 = DP.Routes[3].Vref[1:idx_r12]
Vref_r23 = DP.Routes[3].Vref[idx_r12:end]
idx_r23 = findlast(x->x==3.0, Vref_r23)+idx_r12-1
Svec_r2 = Svec[idx_r12:idx_r23]
Vref_r2 = DP.Routes[3].Vref[idx_r12:idx_r23]
Svec_r3 = Svec[idx_r23:end]
Vref_r3 = DP.Routes[3].Vref[idx_r23:end]

Aref_r1 = DP.Routes[3].Aref[1:idx_r12]
Aref_r2 = DP.Routes[3].Aref[idx_r12:idx_r23]
Aref_r3 = DP.Routes[3].Aref[idx_r23:end]


#=
plt1 = plot(Svec_r1, Vref_r1, xlim=(0.0, Rlength+1), ylim=(0.0, 5.0), c=:green, size=(460, 310), grid=true, legend=false, ylabel="V(m/s)", left_margin=-2mm, titleposition=:left, showaxis=true, xlabel="S(m)", xticks =0:3.0:Rlength+1, yticks =0:1.0:5.0)
plot!(plt1, Svec_r2, Vref_r2, c=:red)
plot!(plt1, Svec_r3, Vref_r3, c=:blue)
savefig("output/Vref.pdf")
=#
plt2 = plot(Svec_r1, Aref_r1, xlim=(0.0, Rlength+1), ylim=(-2.0, 4.0), c=:green, size=(460, 310), grid=true, legend=false, ylabel="A(m/s²)", left_margin=-2mm, titleposition=:left, showaxis=true, xlabel="S(m)", xticks =0:3.0:Rlength+1, yticks =-2.0:1.0:4.0)
plot!(plt2, Svec_r2, Aref_r2, c=:red)
plot!(plt2, Svec_r3, Aref_r3, c=:blue)
savefig("output/Aref.pdf")
=#
