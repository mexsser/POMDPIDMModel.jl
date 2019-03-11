# install dependencies

using Pkg
Pkg.add("POMDPs")
import POMDPs
POMDPs.add_registry()

Pkg.add("Random")
Pkg.add("StatsBase")
Pkg.add("Distributions")
Pkg.add("LinearAlgebra")
Pkg.add("SymPy")

Pkg.add("POMDPModelTools")
Pkg.add("POMDPPolicies")
Pkg.add("POMDPSimulators")
Pkg.add("BeliefUpdaters")
Pkg.add(PackageSpec(url = "https://github.com/mexsser/DiscreteValueIteration.jl"))
Pkg.add("QMDP")

Pkg.develop(PackageSpec(url = "https://github.com/mexsser/Geo.jl"))
Pkg.add("Plots")
Pkg.add("Distributed")
Pkg.add("Measures")
