function State2XY(DP::DrivePOMDP, st::CarSt)
    Pos = GetGlobalPosition(DP.Routes[st.r], st.s)
    return (Pos.x, Pos.y)
end

function Junction(halflen::Float64, gauge::Float64, θ::Float64; style::Symbol) # gauge = Spurweite
    Ycenter=Matrix{Float64}
    if style == :Crossroad
        plot(xlim=(-halflen, halflen), ylim=(-halflen*sin(θ), halflen), size=(650, 650*sin(θ)), grid=false, legend=false, showaxis=false)
        Ycenter = [0.0 0.0; -halflen halflen]
    elseif style == :TJunction
        plot(xlim=(-halflen, halflen), ylim=(-0.3*halflen, halflen), size=(650, 650/2*1.3), grid=false, legend=false, showaxis=false)
        plot!([-halflen, halflen], [-gauge, -gauge], c=:black)
        Ycenter = [0.0 0.0; 0.0 halflen]
    else
        error("style $style not supported, please choose :Crossroad or :TJunction and try again.")
    end
    plot!([-halflen, halflen], [0.0, 0.0], c=:lightgray, linestyle=:dash) # x axis; center line
    α = π/2 - θ
    YcenterR = Rmat(α)*Ycenter
    plot!(YcenterR[1,:], YcenterR[2,:], c=:lightgray, linestyle=:dash) # y axis; center line

    rr = 1.0
    # circular fillet left
    pts_c1 = Plots.partialcircle(-π/2, π/2-θ, 100, rr) # n=100 steps
    x_c1, y_c1 = Plots.unzip(pts_c1)
    x_c1 .-= gauge*(tan(α)+1/sin(θ)) + rr/tan(θ/2)
    y_c1 .+= gauge + rr
    pushfirst!(x_c1, -halflen)
    pushfirst!(y_c1, gauge)
    push!(x_c1, -gauge/sin(θ)-halflen*cos(θ))
    push!(y_c1, halflen*sin(θ))
    plot!(x_c1, y_c1, c=:black)

    # circular fillet right
    pts_c2 = Plots.partialcircle(-π/2-θ, -π/2, 100, rr) # n=100 steps
    x_c2, y_c2 = Plots.unzip(pts_c2)
    x_c2 .+= -(gauge+rr*(1-cos(θ)))/tan(θ) + gauge/sin(θ) + rr*sin(θ)
    y_c2 .+= gauge+rr
    pushfirst!(x_c2, -halflen*cos(θ)+gauge/sin(θ))
    pushfirst!(y_c2, halflen*sin(θ))
    push!(x_c2, halflen)
    push!(y_c2, gauge)
    plt = plot!(x_c2, y_c2, c=:black)

    if style == :Crossroad
        pts_c1_mirror = Rmat(π)*[transpose(x_c1); transpose(y_c1)]
        plt = plot!(plt, pts_c1_mirror[1,:], pts_c1_mirror[2,:], c=:black)
        pts_c2_mirror = Rmat(π)*[transpose(x_c2); transpose(y_c2)]
        plt = plot!(plt, pts_c2_mirror[1,:], pts_c2_mirror[2,:], c=:black)
    end

    return plt
end

function DrawStopline!(plt::Plots.Plot{Plots.GRBackend}, R::Route, StoplineLength::Float64, gauge::Float64)
    StopPoint, θ = GetGlobalPosAngle(R, StoplineLength)
    Δx = gauge/2 * sin(θ)
    Δy = gauge/2 * cos(θ)
    X = [StopPoint.x - Δx, StopPoint.x + Δx]
    Y = [StopPoint.y + Δy, StopPoint.y - Δy]
    plot!(plt, X, Y, c=:blue, linewidth=4)
end

function PlotRoutes(DP::DrivePOMDP, plt::Plots.Plot{Plots.GRBackend})
    for i in 1:length(DP.Routes)
        if i == 1
            for G in DP.Routes[i].Geos
                PlotGeometry(G, plt; color=:blue)
            end
        else
            for G in DP.Routes[i].Geos
                PlotGeometry(G, plt)
            end
        end
    end
    return plt
end

function PlotGeometry(Geo::Geometry, plt::Plots.Plot{Plots.GRBackend}; color::Symbol=:red)
    if Geo.GType == "arc"
        if Geo.Ep.θ > Geo.Sp.θ
            δ = -π/2
        else
            δ = π/2
        end
        pts = Plots.partialcircle(Geo.Sp.θ+δ, Geo.Ep.θ+δ, 100, abs(1/Geo.Sp.cur))
        Center = ArcCenter(Geo)
        x, y = Plots.unzip(pts)
        x .+= Center.x
        y .+= Center.y
        plot!(plt, x, y, c=color, linestyle=:dot)
    elseif Geo.GType == "line"
        plot!(plt, [Geo.Sp.x, Geo.Ep.x], [Geo.Sp.y, Geo.Ep.y], c=color, linestyle=:dot)
    else
        error("Can not print this type of geomatry: $(Geo.GType)")
    end
end

function SVAInit(DP::DrivePOMDP, StsArray::Vector{Sts})
    t = (length(StsArray)-1)*DP.Δt
    Splt = plot(xlim=(0.0, t*1.2), ylim=(0.0, DP.sV[end]*1.2), size=(200, 150), grid=true, legend=false, ylabel="S(m)", left_margin=-2mm, titleposition=:left, showaxis=true, xlabel="t/s", xticks =0:1.0:UInt16(floor(t*1.2)), yticks =0:4*DP.Δs:DP.sV[end])
    Vplt = plot(xlim=(0.0, t*1.2), ylim=(0.0, DP.vV[end]*1.2), size=(200, 150), grid=true, legend=false, ylabel="V(m/s)", left_margin=-2mm, titleposition=:left, showaxis=true, xlabel="t/s", xticks =0:1.0:UInt16(floor(t*1.2)), yticks =DP.vV)
    Aplt = plot(xlim=(0.0, t*1.2), ylim=(DP.Aset.min-0.5, DP.Aset.max+0.5), size=(200, 150), grid=true, legend=false, ylabel="A(m/s²)", left_margin=-2mm, titleposition=:left, showaxis=true, xlabel="t/s", xticks =0:1.0:UInt16(floor(t*1.2)), yticks =DP.Aset.min:1.0:DP.Aset.max)
    return Splt, Vplt, Aplt
end

function PlotSVA!(Plts::NTuple{3, Plots.Plot{Plots.GRBackend}}, DP::DrivePOMDP, Step::Int64, StPair::NTuple{2, CarSt}, AvecPair::NTuple{2, Vector{Float64}}) # should plot standalone and the returned plt should be used as subplot
    plot!(Plts[1], [(Step-1)*DP.Δt, Step*DP.Δt], [StPair[1].s, StPair[2].s], c=:black)
    plot!(Plts[2], [(Step-1)*DP.Δt, Step*DP.Δt], [StPair[1].v, StPair[2].v], c=:black)
    #plot!(Plts[3], [(Step-1)*DP.Δt, (Step-1)*DP.Δt], [APair[1], APair[2]], c=:red, linestyle = :dot)
    Astep = Int64(DP.Δt/0.1)
    for i in 1:Astep+1
        if Step > 1 && i == 1
            plot!(Plts[3], [(Step-1)*DP.Δt+0.1*(i-1), (Step-1)*DP.Δt+0.1*(i-1)], [AvecPair[1][end], AvecPair[2][i]], c=:black)
        elseif i >= 2
            plot!(Plts[3], [(Step-1)*DP.Δt+0.1*(i-2), (Step-1)*DP.Δt+0.1*(i-1)], [AvecPair[2][i-1], AvecPair[2][i]], c=:black)
        end
    end
    return nothing
end

function Visualisation(DP::DrivePOMDP, StsArray::Vector{Sts}, ObsArray::Vector{CarOb}, ActVec::Vector{Symbol}, AccMat::Vector{Vector{Float64}}, halflen::Float64, gauge::Float64, θ::Float64, style::Symbol, picname::String)
    clibrary(:Plots)
    pltJunction = Junction(halflen, gauge, θ; style=style)
    pltMap = PlotRoutes(DP, pltJunction)
    DrawStopline!(pltMap, DP.Routes[DP.SsInit.Ego.r], DP.Stopline, gauge)
    pltSVA_Ego = SVAInit(DP, StsArray)
    plt_Ego = plot(pltSVA_Ego..., layout=(3,1), size=(250, 650))

    anim = @animate for i in 1:length(StsArray)
        if i > 1
            plot!(pltMap, [ObsArray[i-1].x], [ObsArray[i-1].y], seriestype=:scatter, markersize = 4, c=:violet, markerstrokecolor=:black)
        end
        if i == 2
            StPair = (StsArray[i-1].Ego, StsArray[i].Ego)
            PlotSVA!(pltSVA_Ego, DP, i-1, StPair, (AccMat[i-1], AccMat[i-1]))
        end
        if i > 2
            StPair = (StsArray[i-1].Ego, StsArray[i].Ego)
            PlotSVA!(pltSVA_Ego, DP, i-1, StPair, (AccMat[i-2], AccMat[i-1]))
        end

        if ActVec[i] == :takeover
            plot!(pltMap, State2XY(DP, StsArray[i].Ego), seriestype=:scatter, markersize = 5, c=:green, markerstrokecolor=:black)
        else
            plot!(pltMap, State2XY(DP, StsArray[i].Ego), seriestype=:scatter, markersize = 5, c=:orange, markerstrokecolor=:black)
        end
        plot!(pltMap, State2XY(DP, StsArray[i].Other), seriestype=:scatter, markersize = 5, c=:red, markerstrokecolor=:black)

        if style == :TJunction
            column1 = @layout [a{0.2h}; b{0.6h}; c{0.2h}]
            plt_null = plot(grid=false, legend=false, showaxis=false)
            plt_col1 = plot(plt_null, pltMap, plt_null, layout=column1)
            row1 = @layout [a{0.7w} b{0.3w}]
            plot(plt_col1, plt_Ego, layout=row1, size=(900, 650))
        elseif style == :Crossroad
            plot(pltMap, plt_Ego, layout=grid(1, 2, widths=[0.7, 0.3]), size=(900, 650))
        else
            error("Unsupported Style: $style. Please choose either :Crossroad or :TJunction.")
        end
    end
    gif(anim, "output/$picname.gif", fps = Int(floor(1/DP.Δt)))
end
