using ..SearchProblem: AbstractSearchProblem

abstract type AbstractTreeSearchAlgorithm end

init!(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem) = error("not implemented")
stop_condition(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem) = error("not implemented")
node_expansion_policy(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem) = error("not implemented")
before_node_expansion!(search_algo::AbstractTreeSearchAlgorithm, node::TreeSearchNode,search_prob::AbstractSearchProblem) = error("not implemented")
on_node_generation!(search_algo::AbstractTreeSearchAlgorithm, child_node::TreeSearchNode,search_prob::AbstractSearchProblem) = error("not implemented")
on_node_expansion_finish!(search_algo::AbstractTreeSearchAlgorithm, node::TreeSearchNode,search_prob::AbstractSearchProblem) = error("not implemented")
on_stop!(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem) = error("not implemented")


function graph_search!(search_algo::AbstractTreeSearchAlgorithm, search_prob::AbstractSearchProblem)
    init!(search_algo, search_prob)
    # yield()
    while !stop_condition(search_algo, search_prob)
        node = node_expansion_policy(search_algo, search_prob)  # 0.000007 seconds (42 allocations: 1.531 KiB)
        before_node_expansion!(search_algo, node, search_prob) # 0.000003 seconds (48 allocations: 2.219 KiB)  -> 0.000003 seconds (47 allocations: 2.203 KiB) -> 0 most of the times after switching from String keys to Any keys
        for child_node in get_children_nodes(search_prob, node)  # 0.000010 seconds (99 allocations: 6.234 KiB)
            on_node_generation!(search_algo, child_node, search_prob)  # 0.000023 seconds (244 allocations: 37.469 KiB) -> 0.000021 seconds (200 allocations: 35.406 KiB) -> 0.000006 seconds (13 allocations: 528 bytes) after my custom findvalue function.  # can't optimize further.
        end
        on_node_expansion_finish!(search_algo, node, search_prob) # 0.000000 seconds
        # yield()
    end
    on_stop!(search_algo, search_prob)
    return search_algo
end

