using .SearchProblem: AbstractSearchProblem, start_state, key, heuristic, cost, goal_test
using .GraphSearch
using .GraphSearch: AbstractTreeSearchAlgorithm, graph_search!, get_solution
using Random
using Statistics


abstract type AbstractWeightAdjustmentPolicy end
possible_weights(wap::AbstractWeightAdjustmentPolicy) = [1.0]
(wap::AbstractWeightAdjustmentPolicy)(;kwargs...) = 1.0

struct ConstantWeightPolicy <: AbstractWeightAdjustmentPolicy
    weight::Float64
end
possible_weights(cwp::ConstantWeightPolicy) = [cwp.weight]
(cwp::ConstantWeightPolicy)(;kwargs...) = cwp.weight

mutable struct AWAStar{WAP <: AbstractWeightAdjustmentPolicy} <: AbstractTreeSearchAlgorithm
    weight::Float64
    walltime_limit::Float64
    nodes_budget::Int
    weight_adjustment_policy::WAP

    open_lists::MultipleOpenLists
    closed_set::Set{Any}
    path_costs::Dict{Any, Float64}
    start_time::Float64
    stop_time::Union{Nothing,Float64}
    nodes_expended::Int
    interrupted::Bool

    nodes_expended_to_fist_solution::Union{Int,Nothing}
    num_solutions::Int
    all_solutions_costs::Vector{Float64}
    all_solutions_walltimes::Vector{Float64}
    all_solutions_nodes_expended::Vector{Int}
    all_solutions::Vector{Vector{Any}}
    solution::Union{Vector{Any},Nothing}
    solution_cost::Float64
    solution_walltime::Union{Float64,Nothing}
    solution_nodes_expended::Union{Int,Nothing}
    solution_w_trace::Union{Vector{Float64},Nothing}

    function AWAStar(weight::Real, walltime_limit::Real, nodes_budget::Integer, wap::WAP) where WAP <: AbstractWeightAdjustmentPolicy
        awastar = new{WAP}(weight, walltime_limit, nodes_budget, wap)
        awastar.open_lists = MultipleOpenLists(possible_weights(wap))
        awastar.closed_set = Set{Any}()
        awastar.path_costs = Dict{Any,Float64}()
        awastar.all_solutions_costs = Float64[]
        awastar.all_solutions_walltimes = Float64[]
        awastar.all_solutions_nodes_expended = Int[]
        awastar.all_solutions = Vector{Any}[]
        return awastar
    end
end

simulated_time(awastar::AWAStar, simulated_expansion_rate) = awastar.nodes_expended / simulated_expansion_rate

function wall_time(awastar::AWAStar)
    if isnothing(awastar.stop_time)
        return time() - awastar.start_time
    else
        return awastar.stop_time - awastar.start_time
    end
end

expansion_rate(awastar::AWAStar) = awastar.nodes_expended / wall_time(awastar)

quality(cost, optimal_cost) = optimal_cost / cost

function interrupt!(awastar::AWAStar)
    awastar.interrupted = true
end

function GraphSearch.init!(awastar::AWAStar, sp::AbstractSearchProblem)
    empty!(awastar.open_lists)
    empty!(awastar.closed_set)
    empty!(awastar.path_costs)
    awastar.nodes_expended = 0
    awastar.interrupted = false
    awastar.nodes_expended_to_fist_solution = nothing
    awastar.num_solutions = 0
    empty!(awastar.all_solutions_costs)
    empty!(awastar.all_solutions_walltimes)
    empty!(awastar.all_solutions_nodes_expended)
    empty!(awastar.all_solutions)
    awastar.solution = nothing
    awastar.solution_cost = Inf
    awastar.solution_walltime = nothing
    awastar.solution_nodes_expended = nothing
    awastar.solution_w_trace = nothing

    awastar.start_time = time()
    awastar.stop_time = nothing

    starting_state = start_state(sp)
    start_node = TreeSearchNode(starting_state)
    start_node_key = key(sp, starting_state)
    start_node_g, start_node_h = start_node.path_cost, heuristic(sp, starting_state)
    @debug "AWA* Run" awastar.weight_adjustment_policy starting_state start_node_h
    push!(awastar.open_lists, start_node_key, start_node, start_node_g, start_node_h)
    awastar.path_costs[start_node_key] = start_node_g
end

function GraphSearch.stop_condition(awastar::AWAStar, sp::AbstractSearchProblem)
    timeup = wall_time(awastar) >= awastar.walltime_limit
    budgetup = awastar.nodes_expended >= awastar.nodes_budget
    converged = false
    open_list_empty = false
    if length(awastar.open_lists) > 0
        _, _, g, h = peek(awastar.open_lists, 1)
        cost_lb = g + h
        converged = cost_lb >= awastar.solution_cost
    else
        open_list_empty = true
    end
    stop = open_list_empty || converged || timeup || budgetup || awastar.interrupted
    if stop
        @debug "Stopping" open_list_empty converged timeup budgetup awastar.interrupted
    end
    return stop
end

function GraphSearch.node_expansion_policy(awastar::AWAStar, sp::AbstractSearchProblem)
    node_f = Inf
    node = nothing
    node_key = nothing
    while node_f >= awastar.solution_cost
        @assert length(awastar.open_lists) > 0 "Should not be possible"
        awastar.weight = awastar.weight_adjustment_policy(num_solutions=awastar.num_solutions)
        node_key, node, node_g, node_h = pop!(awastar.open_lists, awastar.weight)
        node.popped_using_w = awastar.weight
        node_f = node_g + node_h
    end
    @debug "Popped node" awastar.weight node.state node_f
    return node
end

function GraphSearch.before_node_expansion!(awastar::AWAStar, node::TreeSearchNode, sp::AbstractSearchProblem)
    push!(awastar.closed_set, key(sp, node.state))
    return nothing
end

function GraphSearch.on_node_generation!(awastar::AWAStar, child_node::TreeSearchNode, sp::AbstractSearchProblem)
    child_node_g, child_node_h = child_node.path_cost, heuristic(sp, child_node.state)
    child_node_f = child_node_g + child_node_h
    @debug "Child node" child_node.state child_node_g child_node_h child_node_f awastar.solution_cost
    child_node_key = key(sp, child_node.state)

    if child_node_f < awastar.solution_cost
        if goal_test(sp, child_node.state)
            awastar.path_costs[child_node_key] = child_node.path_cost
            if awastar.solution_cost == Inf
                awastar.nodes_expended_to_fist_solution = awastar.nodes_expended
            end
            awastar.solution_cost = child_node_f
            awastar.solution_walltime = wall_time(awastar)
            awastar.solution_nodes_expended = awastar.nodes_expended
            # println("sol cost", awastar.solution_cost, " at ", awastar.nodes_expended, " nodes.")
            awastar.solution, awastar.solution_w_trace = get_solution(child_node)
            awastar.num_solutions += 1
            push!(awastar.all_solutions_walltimes, awastar.solution_walltime)
            push!(awastar.all_solutions_costs, awastar.solution_cost)
            push!(awastar.all_solutions_nodes_expended, awastar.solution_nodes_expended)
            push!(awastar.all_solutions, awastar.solution)
        elseif child_node_key in awastar.closed_set || haskey(awastar.open_lists, child_node_key)
            old_node_cost = awastar.path_costs[child_node_key]
            if old_node_cost > child_node.path_cost
                awastar.path_costs[child_node_key] = child_node.path_cost
                if child_node_key in awastar.closed_set
                    @debug "Better duplicate of a node in closed set" child_node.path_cost old_node_cost
                    delete!(awastar.closed_set, child_node_key)
                else
                    @debug "Better duplicate of a node in open list" child_node.path_cost old_node_cost
                    delete!(awastar.open_lists, child_node_key)
                end
                push!(awastar.open_lists, child_node_key, child_node, child_node_g, child_node_h)
            else
                @debug "Worse duplicate" child_node.path_cost old_node_cost
            end
        else
            @debug "Adding to open list"
            awastar.path_costs[child_node_key] = child_node_g
            push!(awastar.open_lists, child_node_key, child_node, child_node_g, child_node_h)
        end
    else
        @debug "Not worth adding to open list"
    end
end

function GraphSearch.on_node_expansion_finish!(awastar::AWAStar, node::TreeSearchNode, sp::AbstractSearchProblem)
    awastar.nodes_expended += 1
    return nothing
end

function GraphSearch.on_stop!(awastar::AWAStar, sp::AbstractSearchProblem)
    awastar.stop_time = time()
end


# TODO: Add comments
function awastar_search_with_dynamic_weights(search_prob::AbstractSearchProblem, weights_adjustment_policy::AbstractWeightAdjustmentPolicy, walltime_limit::Real, nodes_budget::Integer)
    awastar = AWAStar(1, walltime_limit, nodes_budget, weights_adjustment_policy)
    return graph_search!(awastar, search_prob)
end

function awastar_search(search_prob::AbstractSearchProblem, weight::Real, walltime_limit::Real, nodes_budget::Integer)
    return awastar_search_with_dynamic_weights(search_prob, ConstantWeightPolicy(weight), walltime_limit, nodes_budget)
end
