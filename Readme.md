# AnytimeWeightedAStar.jl

Julia Implementation of [Anytime Weighted A* (AWA*)](https://arxiv.org/abs/1110.2737) algorithm and Randomized Weighted A* (RWA*) algorithm.

RWA* is a variant of AWA* that randomly adjusts the weight at runtime. It typically outperforms AWA* with any static weight.

## Install Instructions

```bash
git clone https://github.com/bhatiaabhinav/AnytimeWeightedAStar.jl
cd AnytimeWeightedAStar.jl/
```

Run Julia REPL as `JULIA_PROJECT=. julia`. Then press `]` to enter package management mode and type `instantiate` and press enter. This will install all the dependencies.

## Running AWA*/RWA*
In julia REPL:
```julia
include("./search_problems/npuzzle.jl");
search_problem = SlidingPuzzle(4:4, 35:45);  # specifies a 4x4 sliding puzzle with the starting state (manhattan) heuristic as a random value 35 and 45.
seed!(search_problem, 1);
reset!(search_problem); # Creates the search problem

awa = awastar_search(search_problem, 2, 10, 10000);  # runs AWA* on the puzzle with a weight=2, a timelimit of 10 seconds and a node-expansions limit of 10000.
println(awa.solution, " ", awa.solution_cost)

rwa = rwastar_search(search_problem, [1,2,3,4,5], 10, 10000, 42); # RWA* with weight set {1,2,3,4,5}, timelimit 10 seconds, node-expansions limit 10000, and RWA*'s random seed = 42.
println(rwa.solution, " ", rwa.solution_cost)
```

See scripts/solve_all.jl for more examples.
