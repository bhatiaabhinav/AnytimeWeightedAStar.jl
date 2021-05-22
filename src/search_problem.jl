export AbstractSearchProblem, seed!, obs, info, reset!, start_state, successors, cost, goal_test, heuristic, key, TreeSearchNode, get_solution, get_children_nodes


using Random

abstract type AbstractSearchProblem end

seed!(sp::AbstractSearchProblem, seed) = Random.seed!(sp.rng, seed)

obs(sp::AbstractSearchProblem) = []

info(sp::AbstractSearchProblem) = Dict{String,Any}()

reset!(sp::AbstractSearchProblem) = nothing

start_state(sp::AbstractSearchProblem) = sp.state

successors(sp::AbstractSearchProblem, state) = nothing

cost(sp::AbstractSearchProblem, state::S, action, next_state::S) where {S} = 0

goal_test(sp::AbstractSearchProblem, state) = false

heuristic(sp::AbstractSearchProblem, state) = 0

key(sp::AbstractSearchProblem, state) = repr(state)

mutable struct TreeSearchNode{S}
    state::S
    path_cost::Float64
    depth::Int
    action::Union{String,Nothing}
    parent::Union{TreeSearchNode{S},Nothing}
    popped_using_w::Float64
    function TreeSearchNode(state::S, path_cost::Real, depth::Integer, action::Union{AbstractString,Nothing}, parent::Union{TreeSearchNode,Nothing}) where {S}
        n = new{S}(state, path_cost, depth, action)
        n.parent = parent
        n.popped_using_w = 0
        return n
    end
end

TreeSearchNode(state) = TreeSearchNode(state, 0, 0, nothing, nothing)

function get_solution(node::TreeSearchNode{S}) where {S}
    if isnothing(node.parent.parent)
        return String[node.action], Float64[node.popped_using_w]
    else
        sol, w_trace = get_solution(node.parent)
        push!(sol, node.action)
        push!(w_trace, node.popped_using_w)
        return sol, w_trace
    end
end

function get_children_nodes(sp::AbstractSearchProblem, node::TreeSearchNode{S}) where {S}
    arr_child_nodes = TreeSearchNode{S}[]
    for (successor_state, successor_action) in successors(sp, node.state)
        path_cost = node.path_cost + cost(sp, node.state, successor_action, successor_state)
        child_node = TreeSearchNode(successor_state, path_cost, node.depth + 1, successor_action, node)
        push!(arr_child_nodes, child_node)
    end
    return arr_child_nodes
end
