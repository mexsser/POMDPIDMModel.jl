
function DrivePOMDP(discount_factor::Float64, Δt::Float64, sRange::Vector{Float64}, vRange::Vector{Float64},
     rRange::Vector{UInt16}, Aset::NamedTuple{(:min, :max, :comfort),Tuple{Float64, Float64, Float64}},
     Routes::Vector{Route}, SsInit::Sts, Stopline::Float64, Smin::Float64)

     println("########## POMDP Model Initialization ###########")

     Δs = sRange[2] - sRange[1]
     Δv = vRange[2] - vRange[1]

     RoutesInit!(Routes)

     for R in Routes
         RouteAVrefset!(R, Δs, Aset)
     end

     SSpace = Sts[]
     EgoSSpace = CarSt[]
     OtherSSpace = CarSt[]

     for v in vRange, s in sRange
         push!(EgoSSpace, CarSt(s, v, SsInit.Ego.r))
     end
     for v in vRange, s in sRange, r in rRange
         push!(OtherSSpace, CarSt(s, v, r))
     end
     for ego in EgoSSpace, other in OtherSSpace
         push!(SSpace, Sts(ego, other))
     end

     OSpace = Obs[]
     OtherOSpace = CarOb[]

     PosVec = Point2D[]
     s0_shared = Routes[rRange[1]].Geos[1].Length
     StdDevs = (0.1, 0.1)
     rng = MersenneTwister(rand(UInt32))
     """
     avoid generating too many duplicate Positions in OSpace because of R_shared
     """
     for s in sRange[1:end]
         if s <= s0_shared
             Point = GetGlobalPosition(Routes[rRange[1]], s)
             x, y = Sample2D(rng, (Point.x, Point.y), StdDevs)
             x = round(x, 0.01) # should we round (x, y)? Obspace and Stspace must not have the same precision. But for searching easily, the answer is currently "yes"
             y = round(y, 0.01)
             push!(PosVec, Point2D{Float64}(x, y))
         end
     end
     for r in rRange, s in sRange[2:end]
         if s > s0_shared
             Point = GetGlobalPosition(Routes[r], s)
             x, y = Sample2D(rng, (Point.x, Point.y), StdDevs)
             x = round(x, 0.01)
             y = round(y, 0.01)
             push!(PosVec, Point2D{Float64}(x, y))
         end
     end
     for v in vRange, Pos in PosVec
         push!(OtherOSpace, CarOb(v, Pos.x, Pos.y))
     end

     for EgoOb in EgoSSpace, OtherOb in OtherOSpace
         push!(OSpace, Obs(EgoOb, OtherOb))
     end

     DP = DrivePOMDP(discount_factor, Δt, Δs, Δv, sRange, vRange, rRange, Aset, Routes, SSpace, OSpace, SsInit, Stopline, Smin)
     println("###### POMDP Model Initialization Finished ######")
     return DP
 end
