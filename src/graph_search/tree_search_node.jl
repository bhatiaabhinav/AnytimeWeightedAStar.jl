using ..SearchProblem: AbstractSearchProblem, successors, cost

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
