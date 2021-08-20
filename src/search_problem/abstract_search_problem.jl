using Random

abstract type AbstractSearchProblem end

Random.seed!(sp::AbstractSearchProblem, seed) = Random.seed!(sp.rng, seed)

obs(sp::AbstractSearchProblem) = []

info(sp::AbstractSearchProblem) = Dict{String,Any}()

reset!(sp::AbstractSearchProblem) = nothing

start_state(sp::AbstractSearchProblem) = sp.state

successors(sp::AbstractSearchProblem, state) = nothing

cost(sp::AbstractSearchProblem, state::S, action, next_state::S) where {S} = 0

goal_test(sp::AbstractSearchProblem, state) = false

heuristic(sp::AbstractSearchProblem, state) = 0

key(sp::AbstractSearchProblem, state) = repr(state)