# Decision-Making based on [POMDP](https://en.wikipedia.org/wiki/Partially_observable_Markov_decision_process) and [IDM](https://en.wikipedia.org/wiki/Intelligent_driver_model) Model.
### Installation:  
- Please download and run [**build.jl**](deps/build.jl) in terminal once to install the needed julia packages:
```bash
$ julia build.jl
```
- In Julia-REPL, add this package:
```julia
using Pkg
Pkg.add(PackageSpec(url = "https://github.com/mexsser/POMDPIDMModel.jl"))
```
### Run Test
- in Julia-REPL, type
```julia
import POMDPIDMModel
cd(joinpath(dirname(pathof(POMDPIDMModel)), "..", "test"))
include("runtests.jl")
```
- Or you can simply type the following command in terminal after changing directory to **POMDPIDMModel/test/**
```bash
$ julia --color=yes -i -O -- runtests.jl
```
### Dependencies:
- Julia v1.0.1  
- ffmpeg
- python
- matplotlib
- qt5-default
#
![Crossroad](test/output/Crossroad.R1R3.Passive.gif)
#
![TJunction](test/output/TJunction.R1R3.Passive.gif)
#
![TJunction](test/output/TJunction.R1R2.Passive.gif)
