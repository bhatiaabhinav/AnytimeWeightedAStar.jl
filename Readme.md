# AnytimeWeightedAStar.jl

Julia Implementation of [Anytime Weighted A* (AWA*)](https://arxiv.org/abs/1110.2737) algorithm and [Randomized Weighted A* (RWA*)](https://bhatiaabhinav.github.io/publication/BSZsocs21) algorithm (Published at SoCS 2021).

RWA*, a variant of AWA* that randomly adjusts the weight at runtime, usually works better than using any static weight.

## Install Instructions

```julia
using Pkg
Pkg.add("AnytimeWeightedAStar")
```

## Examples
```julia
using AnytimeWeightedAStar
using AnytimeWeightedAStar.SearchProblem
using AnytimeWeightedAStar.ExampleProblems  # Sliding Puzzle, Inverse Sliding Puzzle, Traveling Salesman Problem, City Navigation Problem. See src/example_problems directory for more details.
using Random

search_problem = SlidingPuzzle(4:4, 35:45);  # specifies a 4x4 sliding puzzle (also knowing as 15-Puzzle) with starting state (manhattan) heuristic randomly between 35 and 45.
Random.seed!(search_problem, 1);
reset!(search_problem); # Creates a problem instance.

awa = awastar_search(search_problem, 2, 10, 10000);  # runs AWA* on the puzzle with a weight=2, a timelimit of 10 seconds and node-expansions limit of 10000.
println(awa.solution, " ", awa.solution_cost)

rwa = rwastar_search(search_problem, [1,2,3,4,5], 10, 10000, 42); # RWA* with weight set {1,2,3,4,5}, timelimit 10 seconds, node-expansions limit 10000, and RWA*'s random seed = 42.
println(rwa.solution, " ", rwa.solution_cost)
```

See `scripts/solve_all.jl` for more examples.


## Creating New Search Problems

The AbstractSearchProblem struct and the interface functions for creating new search problems are defined in `src/search_problem/abstract_search_problem.jl`. Look at `src/example_problems/npuzzle.jl` for an example of a search problem.
