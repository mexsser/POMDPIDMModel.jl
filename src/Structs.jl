###### State and States ######
mutable struct CarSt
    s::Float64
    v::Float64
    r::UInt16
end
CarSt() = CarSt(0.0, 0.0, UInt16(1))
Base.:(==)(a::CarSt, b::CarSt) = a.s == b.s && a.v == b.v && a.r == b.r
Base.hash(St::CarSt, h::UInt) = hash(St.s, hash(St.v, hash(St.r, h)))

struct Sts
    Ego::CarSt
    Other::CarSt
end
Sts() = Sts(CarSt(), CarSt())
Base.:(==)(A::Sts, B::Sts) = A.Ego == B.Ego && A.Other == B.Other
Base.hash(sts::Sts, h::UInt) = hash(sts.Ego, hash(sts.Other, h))

###### Observation ######
mutable struct CarOb
    v::Float64
    x::Float64
    y::Float64
end
CarOb() = CarOb(0.0, 0.0, 0.0)
Base.:(==)(a::CarOb, b::CarOb) = a.v == b.v && a.x == b.x && a.y == b.y
Base.hash(ob::CarOb, h::UInt) = hash(ob.v, hash(ob.x, hash(ob.y, h)))

mutable struct Route
    Geos::Vector{Geometry}
    intersect_Infos::Dict{UInt16, Vector{Tuple{String, Point2D{Float64}, Float64}}} # UInt16: the index of the car whoes route intersects with that of ego car;
    Vref::Vector{Float64}
    Aref::Vector{Float64}
    Length::Float64

    function Route(Geos::Vector{Geometry})  # constructor
        intersect_Infos = Dict{UInt16, Vector{Tuple{String, Point2D{Float64}, Float64}}}()
        Length = round(sum(map(x->x.Length, Geos)), 1.0e-8)
        return new(Geos, intersect_Infos, [], [], Length)
    end
end

# POMDP{State, Action, Observation}; action is the acceleration of ego car.
struct DrivePOMDP <: POMDP{Sts, Symbol, CarOb}
        ##### constants #####
        discount_factor::Float64
        Δt::Float64 # time step
        Δs::Float64
        Δv::Float64
        ##### ranges #####
        sV::Vector{Float64}
        vV::Vector{Float64}
        rV::Vector{UInt16}
        Aset::NamedTuple{(:min, :max, :comfort),Tuple{Float64, Float64, Float64}}

        Routes::Vector{Route}
        SSpace::Vector{Sts}
        OSpace::Vector{CarOb}
        SsInit::Sts

        # used only in IDM model
        Stopline::Float64
        Smin::Float64
end

function round(x::Float64, pre::Union{Float64, Int64}) # pre: precision
    n = floor(Int, x/pre)
    m = x - n*pre
    if m >= pre/2
        return trunc((n+1)*pre; digits= 13)
    else
        return trunc(n*pre; digits= 13)
    end
end

ceil(x::Float64, pre::Union{Float64, Int64}) = sign(x)*pre*ceil(Int, abs(x)/pre)

function Rmat(α::Union{Float64, Irrational})
    return [cos(α) -sin(α); sin(α) cos(α)]
end
