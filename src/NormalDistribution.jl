function DiscreteND1D(μ::Float64, δ::Float64, x::Vector{Float64}, xstep::Float64) # x should be sorted array with constant step
    if μ < x[1] || μ > x[end]
        error("The mean value is out of input array")
    end
    #xstep = x[2] - x[1]
    CDF = Vector{Float64}()
    D = Normal(μ, δ)
    #for i=1:length(x)
    for xi in x
        #cdfi = cdf(D, x[i]+xstep/2) - cdf(D, x[i]-xstep/2)
        cdfi = cdf(D, xi+xstep/2) - cdf(D, xi-xstep/2)
        if cdfi < 5.0e-3
            cdfi = 0.0
        end
        push!(CDF, cdfi)
    end
    return normalize!(CDF, 1)
    #return map(x-> round(x/Sum; digits=4), CDF)
end

function ContinuousND2D(Means::NTuple{2, Float64}, StdDeviations::NTuple{2, Float64})
    CovarianceMat = [StdDeviations[1]^2 0.0; 0.0 StdDeviations[2]^2]
    D = MvNormal([Means...], CovarianceMat)
    return D
end

function Sample2D(rng::AbstractRNG, Means::NTuple{2, Float64}, StdDeviations::NTuple{2, Float64})
    D = ContinuousND2D(Means, StdDeviations)
    pair = [0.0, 0.0]
    rand!(rng, D, pair)
    return pair
end
