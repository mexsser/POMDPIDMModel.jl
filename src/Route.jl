function GetGlobalPosition(R::Route, Dis::Float64)
    GIndex, GDis = CurrGeo_Index_Dis(R, Dis)
    return Length2XY(R.Geos[GIndex], GDis)
end

function GetGlobalPosAngle(R::Route, Dis::Float64)
    GIndex, GDis = CurrGeo_Index_Dis(R, Dis)
    return Length2XYθ(R.Geos[GIndex], GDis)
end

function CurrGeo_Index_Dis(R::Route, distance::Float64)
    if distance > R.Length
        error("Input distance $distance is longer than route length $(R.Length)")
    end
    dis = distance
    index = 1
    for i in 1:length(R.Geos)
        if dis > round(R.Geos[i].Length, 1.0e-8) + 1.0e-8 # plus precision lose.
            dis -= round(R.Geos[i].Length, 1.0e-8) + 1.0e-8
            index += 1
        else
            break
        end
    end
    return (index, dis)
end

function Dist2Ip(R::Route, GDist::Float64, GIndex::UInt16) # total distance from start point to intersect point
    Dist::Float64 = 0.0
    if GIndex > 1
        for i in 1 : GIndex-1
            Dist += R.Geos[i].Length
        end
    end
    Dist += GDist
    return Dist
end

function Routes_Update!(Rs::Vector{Route}, RIndex::UInt16, newGeo::Geometry)
    push!(Rs[RIndex].Geos, newGeo)
    empty!(Rs.intersect_Infos)
    RoutesInit!(Rs)
end

function RoutesIntersect(R1::Route, R2::Route)
    IntersectInfos = []
    for i in 1:length(R1.Geos)  # what if there are more than one intersect points? what if the car has already passed this point?
        for j in 1:length(R2.Geos)
            Result = GeosIntersect(R1.Geos[i], R2.Geos[j])
            if Result.intersect
                push!(IntersectInfos, (Result..., GIndex1=UInt16(i), GIndex2=UInt16(j))) # if intersect, return the intersection point, distance in Geos and Geo indexes.
            end
        end
    end
    if !isempty(IntersectInfos)
        return IntersectInfos
    else
        return [(intersect=false,)] # if no intersection, just return (intersect=false,) #message="xxx")
    end
end

function RoutesInit!(Rs::Vector{Route})
    for i = 2:length(Rs)
        IntersectInfos = RoutesIntersect(Rs[1], Rs[i])
        PtDistVec1 = Vector{Tuple{String, Point2D{Float64}, Float64}}()
        PtDistVeck = Vector{Tuple{String, Point2D{Float64}, Float64}}()
        for Re in IntersectInfos
            if Re.intersect
                Dist2Ip_1 = Dist2Ip(Rs[1], Re.Dist1, Re.GIndex1)
                Dist2Ip_k = Dist2Ip(Rs[i], Re.Dist2, Re.GIndex2)
                push!(PtDistVec1, (Re.type, Re.Ip, Dist2Ip_1))
                push!(PtDistVeck, (Re.type, Re.Ip, Dist2Ip_k))
            end
        end
        if !isempty(PtDistVec1)
            push!(Rs[1].intersect_Infos, UInt16(i) => PtDistVec1) # used to calculate ΔTTC; ego car should not use this infomation
        end
        if !isempty(PtDistVeck)
            push!(Rs[i].intersect_Infos, UInt16(1) => PtDistVeck)
        end
    end
    return nothing
end

function RouteAVrefset!(R::Route, Sstep::Float64,
    Aset::NamedTuple{(:min, :max, :comfort),Tuple{Float64, Float64, Float64}})
    # first write the funciton of Vref related to s, then sample it with step = Sstep.
    Vvec = Vector{Float64}()
    Avec = Vector{Float64}()
    FVvec = Vector{Function}()
    FAvec = Vector{Function}()
    Geo_num = length(R.Geos)
    if Geo_num < 1
        error("""
              Route has empty Geos.
              Route: $R
              """)
    end
    if Geo_num == 1
        vec_length = 1+UInt16(floor(R.Geos[1].Length/Sstep))
        Vvec = [R.Geos[1].Vstd for i in 1:vec_length]
        Avec = [0.0 for i in 1:vec_length]
    else # Geo_num > 1
        for Gi in 1:Geo_num # one Geo, one Expression
            if R.Geos[Gi].GType == "line" # Vref changes only in straight line; otherwise keeps constant.
                if Gi == 1 # if no preceding Geo exists, calculate Vref with the Vstd of current and next Geo.
                    if  R.Geos[Gi].Vstd == R.Geos[Gi+1].Vstd # a == 0
                        push!(FVvec, s->R.Geos[Gi].Vstd)
                        push!(FAvec, s->0.0)
                    else # a != 0
                        rel_v, rel_a = Exp_approach(R.Geos[Gi].Vstd, R.Geos[Gi+1].Vstd, R.Geos[Gi].Length, Aset)
                        push!(FVvec, rel_v)
                        push!(FAvec, rel_a)
                    end
                else # Gi > 1;
                    # if preceding Geo exists, and has the same Vstd, then calculate Vref with the Vstd of current and next Geo.
                    if Gi != length(R.Geos) && R.Geos[Gi-1].Vstd == R.Geos[Gi].Vstd
                        if  R.Geos[Gi].Vstd == R.Geos[Gi+1].Vstd # a == 0
                            push!(FVvec, s->R.Geos[Gi].Vstd)
                            push!(FAvec, s->0.0)
                        else # a != 0
                            rel_v, rel_a = Exp_approach(R.Geos[Gi].Vstd, R.Geos[Gi+1].Vstd, R.Geos[Gi].Length, Aset)
                            push!(FVvec, rel_v)
                            push!(FAvec, rel_a)
                        end
                    else # Gi == length(R.Geos) || R.Geos[Gi-1].Vstd != R.Geos[Gi].Vstd)
                        # if preceding Geo exists, but has a different Vstd (if pre_Geo != line, Vend == Vstd),
                        # then calculate Vref with the Vend of preceding Geo and Vstd of current Geo // and Vstd of next Geo.
                        if R.Geos[Gi-1].Vstd != R.Geos[Gi].Vstd
                            rel_v, rel_a = Exp_leave(R.Geos[Gi-1].Vstd, R.Geos[Gi].Vstd, R.Geos[Gi].Length, Aset)
                            push!(FVvec, rel_v)
                            push!(FAvec, rel_a)
                        else # R.Geos[Gi-1].Vstd == R.Geos[Gi].Vstd && Gi == length(R.Geos)
                            push!(FVvec, s->R.Geos[Gi].Vstd)
                            push!(FAvec, s->0.0)
                        end
                    end
                end
            else # GType != "line"
                push!(FVvec, s->R.Geos[Gi].Vstd)
                push!(FAvec, s->0.0)
            end
        end
        if length(FVvec) == length(FAvec) == Geo_num
            for i in 0:UInt16(floor(R.Length/Sstep))
                Gi, Δs = CurrGeo_Index_Dis(R, i*Sstep)
                Vstdi = FVvec[Gi](Δs)
                Astdi = FAvec[Gi](Δs)
                push!(Vvec, Vstdi)
                push!(Avec, Astdi)
            end
        else
            error("""
            Expression number is not the same with Geometry number.
            Exp_num: $(length(FVvec))
            Geo_num: $Geo_num
            Related Route: $R
            """)
        end
    end
    R.Vref = Vvec
    R.Aref = Avec
end

function Exp_approach(Vnow::Float64, Vnext::Float64, Length,
    Aset::NamedTuple{(:min, :max, :comfort),Tuple{Float64, Float64, Float64}}) # entering

    a_req = (Vnext^2 - Vnow^2)/(2*Length) # a_req has sign
    if a_req > Aset.max || a_req < Aset.min
        error("""
              This Route is too short to accelerate/brake the car.
              Required a : $a_req
              Length available: $Length.
              """)
    end
    # Vehicle should accelerate at least with a comfort a.
    a = a_req
    if abs(a_req) < Aset.comfort
        a = a_req > 0 ? Aset.comfort : -Aset.comfort
    end
    s_acc = (Vnext^2 - Vnow^2)/(2*a)
    Δs = Length - s_acc
    rel_v(s::Float64) = begin
                            if s <= Δs
                                return Vnow
                            else
                                 return sqrt(Vnow^2 + 2*a*(s-Δs))
                            end
                       end

    rel_a(s::Float64) = begin
                            if s <= Δs
                                return 0.0
                            else
                                 return a
                            end
                       end

    return rel_v, rel_a
end

function Exp_leave(Vpre::Float64, Vnow::Float64, Length,
    Aset::NamedTuple{(:min, :max, :comfort),Tuple{Float64, Float64, Float64}})  # leaving
    a_req = (Vnow^2 - Vpre^2)/(2*Length)
    if a_req > Aset.max || a_req < Aset.min
        error("""
              This Route is too short to accelerate/brake the car.
              Vpre: $Vpre
              Vcurrent: $Vnow
              Required a : $a_req
              Length available: $Length.
              """)
    end
    # Vehicle should accelerate itself as fast as it can.
    a = a_req > 0 ? Aset.max : Aset.min
    s_acc = (Vnow^2 - Vpre^2)/(2*a)
    rel_v(s::Float64) = begin
                            if s < s_acc
                                return sqrt(Vpre^2 + 2*a*s)
                            else
                                 return Vnow
                            end
                       end

    rel_a(s::Float64) = begin
                           if s < s_acc
                               return a
                           else
                                return 0.0
                           end
                      end
    return rel_v, rel_a
end

function RoutesGenerator(Vgeos::Matrix{Float64}, Rlength::Float64, gauge::Float64, θ::Float64, GRatio_Other::Float64, GRatio_Ego::Float64; style::Symbol)

    if style == :Crossroad
        if size(Vgeos) != (4, 3)
            error("The size of Start Velocity Matrix is not correct! Input size is $(size(Vgeos)), while required size is 4x3.")
        end
    elseif style == :TJunction
        if size(Vgeos) != (3, 3)
            error("The size of Start Velocity Matrix is not correct! Input size is $(size(Vgeos)), while required size is 3x3.")
        end
    else
        error("The input style $style is not supported. Please choose either :Crossroad or :TJunciton.")
    end

    Linepts_shared = [0.0 0.0; gauge/sin(θ) gauge/sin(θ)+GRatio_Other*Rlength]
    Linepts_sharedR = Rmat(π/2-θ)*Linepts_shared
    Linepts_sharedR[1,:] .-= gauge/(2*sin(θ))
    Geo_shared = Geometry(Point2D{Float64}(Linepts_sharedR[1,2], Linepts_sharedR[2,2]), Point2D{Float64}(Linepts_sharedR[1,1], Linepts_sharedR[2,1]), Vgeos[2, 1])

    r3 = 1.5*gauge/(1-cos(θ))
    c3_endpt = (x=gauge*(1/sin(θ)+1/(2*tan(θ))), y=-gauge/2)
    Geo32 = Geometry(Point4D{Float64}(Linepts_sharedR[1,1], Linepts_sharedR[2,1], 1/r3, -θ), Point4D{Float64}(c3_endpt.x, c3_endpt.y, 1/r3, 0.0), Vgeos[3, 2])
    if Geo_shared.Length + Geo32.Length >= Rlength
        error("GRatio_Other $GRatio_Other is too big. Please Reduce it and retry.")
    end
    Geo33 = Geometry(Point2D{Float64}(c3_endpt.x, c3_endpt.y), Point2D{Float64}(c3_endpt.x+Rlength-Geo_shared.Length-Geo32.Length, c3_endpt.y), Vgeos[3, 3])
    R3 = Route([Geo_shared, Geo32, Geo33])

    r2 = 0.5*gauge/(1+cos(θ))
    Geo22 = Geometry(Point4D{Float64}(Linepts_sharedR[1,1], Linepts_sharedR[2,1], -1/r2, -θ), Point4D{Float64}(-c3_endpt.x, -c3_endpt.y, -1/r2, -π), Vgeos[2, 2])
    Geo23 = Geometry(Point2D{Float64}(-c3_endpt.x, -c3_endpt.y), Point2D{Float64}(-c3_endpt.x-Rlength+Geo_shared.Length+Geo22.Length, -c3_endpt.y), Vgeos[2, 3])
    R2 = Route([Geo_shared, Geo22, Geo23])

    if style == :Crossroad
        Line1pts = [0.0 0.0; -GRatio_Ego*Rlength (1.0-GRatio_Ego)*Rlength]
        Line1ptsR = Rmat(π/2-θ)*Line1pts
        Line1ptsR[1,:] .+= gauge/(2*sin(θ))
        Geo1 = Geometry(Point2D{Float64}(Line1ptsR[1,1], Line1ptsR[2,1]), Point2D{Float64}(Line1ptsR[1,2], Line1ptsR[2,2]), Vgeos[1, 1])
        R1 = Route([Geo1])

        line4endpt = Rmat(π/2-θ)*[0.0; gauge/sin(θ)-(1.0-GRatio_Other)*Rlength] - [gauge/(2*sin(θ)); 0.0]
        Geo42 = Geometry(Point2D{Float64}(Linepts_sharedR[1,1], Linepts_sharedR[2,1]), Point2D{Float64}(line4endpt[1], line4endpt[2]), Vgeos[4, 2])
        R4 = Route([Geo_shared, Geo42])

        Routes = [R1, R2, R3, R4]
        return Routes
    else # style == :TJunction
        Geo1 = Geometry(Point2D{Float64}(GRatio_Ego*Rlength, gauge/2), Point2D{Float64}((GRatio_Ego-1.0)*Rlength, gauge/2), Vgeos[1, 1])
        R1 = Route([Geo1])
        Routes = [R1, R2, R3]
        return Routes
    end
end
