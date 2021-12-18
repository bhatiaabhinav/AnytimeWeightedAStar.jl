using Random

abstract type AbstractSearchProblem{S,A} end

Random.seed!(sp::AbstractSearchProblem, seed) = Random.seed!(sp.rng, seed)

obs(sp::AbstractSearchProblem) = []

info(sp::AbstractSearchProblem) = Dict{String,Any}()  # Some nice printatble information about the search-problem instance e.g., number of cities in a Travelling-Salesman-Problem instance, or densitiy of obstacles in a grid-navigation-puzzle instance.

reset!(sp::AbstractSearchProblem) = nothing

function start_state(sp::AbstractSearchProblem{S,A})::S where {S,A}
    error("not implemented")
end

function successors(sp::AbstractSearchProblem{S,A}, state::S) where {S,A}
    error("Not Implemented. Should return a list (or any iterable collection) of Tuple{S, A}. Each tuple is a pair of: next-state and the action that leads to that next-state from the given state.")
end

function cost(sp::AbstractSearchProblem{S}, state::S, action::A, next_state::S)::Float64 where {S,A}
    error("not implemented")
end

function goal_test(sp::AbstractSearchProblem{S,A}, state::S)::Bool where {S,A}
    error("not implemented")
end

function heuristic(sp::AbstractSearchProblem{S,A}, state::S)::Float64 where {S,A}
    0
end
