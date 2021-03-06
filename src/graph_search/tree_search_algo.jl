using ..SearchProblem: AbstractSearchProblem

abstract type AbstractTreeSearchAlgorithm{S,A} end

init!(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem) = error("not implemented")
stop_condition(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem) = error("not implemented")
node_expansion_policy(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem) = error("not implemented")
before_node_expansion!(search_algo::AbstractTreeSearchAlgorithm, node::TreeSearchNode,search_prob::AbstractSearchProblem) = error("not implemented")
on_node_generation!(search_algo::AbstractTreeSearchAlgorithm, child_node::TreeSearchNode,search_prob::AbstractSearchProblem) = error("not implemented")
on_node_expansion_finish!(search_algo::AbstractTreeSearchAlgorithm, node::TreeSearchNode,search_prob::AbstractSearchProblem) = error("not implemented")
on_stop!(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem) = error("not implemented")

function graph_search!(search_algo::AbstractTreeSearchAlgorithm{S,A}, search_prob::AbstractSearchProblem{S,A}) where {S,A}
    init!(search_algo, search_prob)
    while !stop_condition(search_algo, search_prob)
        node::TreeSearchNode{S,A} = node_expansion_policy(search_algo, search_prob)
        before_node_expansion!(search_algo, node, search_prob)
        for child_node::TreeSearchNode{S,A} in get_children_nodes(search_prob, node)
            on_node_generation!(search_algo, child_node, search_prob)
        end
        on_node_expansion_finish!(search_algo, node, search_prob)
    end
    on_stop!(search_algo, search_prob)
    return search_algo
end
