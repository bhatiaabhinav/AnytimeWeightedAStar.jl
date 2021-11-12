using ..SearchProblem: AbstractSearchProblem, successors, cost

mutable struct TreeSearchNode{S, A}
    state::S
    path_cost::Float64
    depth::Int
    action::Union{A,Nothing}
    parent::Union{TreeSearchNode{S,A},Nothing}
    popped_using_w::Float64
    function TreeSearchNode{S, A}(state::S, path_cost::Real, depth::Integer, action::Union{A, Nothing}, parent::Union{TreeSearchNode,Nothing}) where {S, A}
        return new{S, A}(state, path_cost, depth, action, parent, 0.0)
    end
end

TreeSearchNode{S,A}(state) where {S,A} = TreeSearchNode{S,A}(state, 0, 0, nothing, nothing)

function get_solution(node::TreeSearchNode{S,A})::Tuple{Vector{A}, Vector{Float64}} where {S,A}
    if node.depth == 1
        return A[node.action], Float64[node.popped_using_w]
    else
        sol, w_trace = get_solution(node.parent)
        push!(sol, node.action)
        push!(w_trace, node.popped_using_w)
        return sol, w_trace
    end
end

function get_children_nodes(sp::AbstractSearchProblem{S, A}, node::TreeSearchNode{S, A}) where {S, A}
    return Iterators.map(successors(sp, node.state)) do successor
        successor_state::S, successor_action::A = successor
        path_cost = node.path_cost + cost(sp, node.state, successor_action, successor_state)
        child_node = TreeSearchNode{S, A}(successor_state, path_cost, node.depth + 1, successor_action, node)
        return child_node
    end
end
