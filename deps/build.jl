# install dependencies

using Pkg
Pkg.add("POMDPs")
import POMDPs
POMDPs.add_registry()
Pkg.add("QMDP")

Pkg.add("Random")
Pkg.add("Distributions")
Pkg.add("LinearAlgebra")
Pkg.add("SymPy")

Pkg.add("POMDPModelTools")
Pkg.add("BeliefUpdaters")
Pkg.add(PackageSpec(url = "https://github.com/mexsser/Geo.jl"))
Pkg.add(PackageSpec(url = "https://github.com/mexsser/DiscreteValueIteration.jl"))

Pkg.add("Plots")
Pkg.add("StatsBase")
Pkg.add("Distributed")
Pkg.add("Measures")
