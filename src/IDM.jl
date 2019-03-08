"""
Intelligent driver model
"""

function IDM(;Vego::Float64, Vfront::Float64, Vref::Float64, Snet::Float64, T::Float64=0.1, Amax::Float64, Bdec::Float64, Smin::Float64=0.1, δ::Float64=3.0)
    """
        Vego:   Velocity of ego car;
        Vfront: Velocity of front car;
        Vref:   the velocity the vehicle would drive at in free traffic;
        Snet:   Distance between ego and front car;
        Smin:   a minimum desired net distance;
        T:      the minimum possible time to the vehicle in front, or Safe time headway;
        Amax:   the maximum vehicle acceleration of ego car;
        Bdec:   comfortable braking deceleration, must be positive;
        δ:      just a exponent, usually set to 4.
    """
    ΔV = Vego - Vfront
    Sstar = Smin + Vego*T + Vego*ΔV/(2*sqrt(Amax*Bdec))
    Aego = Amax*(1 - (Vego/Vref)^δ - (Sstar/Snet)^2)
end

#= # Test
@show IDM(Vego=4.0, Vfront=0.0, Vref=4.0, Snet=4.0, Smin=0.1, T=0.1, Amax=3.0, Bdec=1.0)
=#
