# POMDPModel
### Preparation:  
- Please run [**_INSTALL.jl_**](./INSTALL.jl) once to install the needed julia packages.  
- Please include the root folder of this repo in julia search paths in order to use the local package "**Geo**". To do this, add
```julia
push!(LOAD_PATH, "Path/Of/Repo")
```
at the end of the file _**~/.julia/config/startup.jl**_
- in order to use a faster solver **_SparseValueIterationSolver_** in Pkg "**DiscreteValueIteration**", which is a dependency of QMDP Solver, two funcitons should be modified. To locate the package source folder, type the following command in julia REPL:
```julia
dirname(pathof(DiscreteValueIteration))
```
then change the argument type of function **_transition_matrix_a_s_sp(mdp::MDP)_** and **_reward_s_a(mdp::MDP)_** from "**MDP**" to "**Union{MDP,POMDP}**" in **_DiscreteValueIteration/src/sparse.jl_**  

### Dependencies:
- Julia v1.0  
- ffmpeg
- python
- matplotlib
#
![Crossroad](output/Crossroad.R1R3.Passive.gif)
#
![TJunction](output/TJunction.R1R3.Passive.gif)
#
![TJunction](output/TJunction.R1R2.Passive.gif)
