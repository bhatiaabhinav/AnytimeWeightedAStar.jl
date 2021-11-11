using Random

abstract type AbstractSearchProblem{S} end

Random.seed!(sp::AbstractSearchProblem, seed) = Random.seed!(sp.rng, seed)

obs(sp::AbstractSearchProblem) = []

info(sp::AbstractSearchProblem) = Dict{String,Any}()

reset!(sp::AbstractSearchProblem) = nothing

function start_state(sp::AbstractSearchProblem{S})::S where {S}
    error("not implemented")
end

function successors(sp::AbstractSearchProblem{S}, state::S) where {S}
    error("not implemented")
end

function cost(sp::AbstractSearchProblem{S}, state::S, action, next_state::S) where {S}
    error("not implemented")
end

function goal_test(sp::AbstractSearchProblem{S}, state::S)::Bool where {S}
    error("not implemented")
end

function heuristic(sp::AbstractSearchProblem{S}, state::S)::Float64 where {S}
    0
end
