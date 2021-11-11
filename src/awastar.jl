using .SearchProblem: AbstractSearchProblem, start_state, heuristic, cost, goal_test
using .GraphSearch
using .GraphSearch: AbstractTreeSearchAlgorithm, graph_search!, get_solution
using Random
using Statistics


abstract type AbstractWeightAdjustmentPolicy end
possible_weights(wap::AbstractWeightAdjustmentPolicy) = error("not implemented")
(wap::AbstractWeightAdjustmentPolicy)(;kwargs...)::Float64 = error("Not implemented")

struct ConstantWeightPolicy <: AbstractWeightAdjustmentPolicy
    weight::Float64
end
@inline possible_weights(cwp::ConstantWeightPolicy) = [cwp.weight]
@inline (cwp::ConstantWeightPolicy)(;kwargs...)::Float64 = cwp.weight

mutable struct AWAStar{S, WAP <: AbstractWeightAdjustmentPolicy} <: AbstractTreeSearchAlgorithm{S}
    weight::Float64
    walltime_limit::Float64
    nodes_budget::Int
    weight_adjustment_policy::WAP

    open_lists::MultipleOpenLists{S}
    closed_set::Set{S}
    path_costs::Dict{S, Float64}
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

    function AWAStar{S}(weight::Real, walltime_limit::Real, nodes_budget::Integer, wap::WAP) where {S, WAP <: AbstractWeightAdjustmentPolicy}
        awastar = new{S, WAP}(weight, walltime_limit, nodes_budget, wap)
        awastar.open_lists = MultipleOpenLists{S}(possible_weights(wap))
        awastar.closed_set = Set{S}()
        awastar.path_costs = Dict{S,Float64}()
        awastar.all_solutions_costs = Float64[]
        awastar.all_solutions_walltimes = Float64[]
        awastar.all_solutions_nodes_expended = Int[]
        awastar.all_solutions = Vector{Any}[]
        return awastar
    end
end

@inline simulated_time(awastar::AWAStar, simulated_expansion_rate::Real)::Float64 = awastar.nodes_expended / simulated_expansion_rate

@inline function wall_time(awastar::AWAStar)::Float64
    if isnothing(awastar.stop_time)
        return time() - awastar.start_time
    else
        return awastar.stop_time - awastar.start_time
    end
end

@inline expansion_rate(awastar::AWAStar)::Float64 = awastar.nodes_expended / wall_time(awastar)

@inline quality(cost, optimal_cost)::Float64 = optimal_cost / cost

@inline function interrupt!(awastar::AWAStar)::Nothing
    awastar.interrupted = true
    nothing
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
    start_node_g, start_node_h = start_node.path_cost, heuristic(sp, starting_state)
    @debug "AWA* Run" awastar.weight_adjustment_policy starting_state start_node_h
    push!(awastar.open_lists, starting_state, start_node, start_node_g, start_node_h)
    awastar.path_costs[starting_state] = start_node_g
end

function GraphSearch.stop_condition(awastar::AWAStar{S}, sp::AbstractSearchProblem{S})::Bool where S
    timeup::Bool = wall_time(awastar) >= awastar.walltime_limit
    budgetup::Bool = awastar.nodes_expended >= awastar.nodes_budget
    converged::Bool = false
    open_list_empty::Bool = false
    if length(awastar.open_lists) > 0
        _, g, h = peek(awastar.open_lists, 1.0)
        cost_lb::Float64 = g + h
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

function GraphSearch.node_expansion_policy(awastar::AWAStar{S}, sp::AbstractSearchProblem{S})::TreeSearchNode{S} where S
    node_f::Float64 = Inf
    local node::TreeSearchNode{S}
    while node_f >= awastar.solution_cost
        @assert length(awastar.open_lists) > 0 "Should not be possible"
        awastar.weight = awastar.weight_adjustment_policy(num_solutions=awastar.num_solutions)
        node, node_g, node_h = pop!(awastar.open_lists, awastar.weight)
        node.popped_using_w = awastar.weight
        node_f = node_g + node_h
    end
    return node
end

function GraphSearch.before_node_expansion!(awastar::AWAStar{S}, node::TreeSearchNode{S}, sp::AbstractSearchProblem{S}) where S
    push!(awastar.closed_set, node.state)
    return nothing
end

function GraphSearch.on_node_generation!(awastar::AWAStar{S}, child_node::TreeSearchNode{S}, sp::AbstractSearchProblem{S}) where S
    child_node_g, child_node_h = child_node.path_cost, heuristic(sp, child_node.state)
    child_node_f = child_node_g + child_node_h
    if child_node_f < awastar.solution_cost
        if goal_test(sp, child_node.state)
            awastar.path_costs[child_node.state] = child_node.path_cost
            if awastar.solution_cost == Inf
                awastar.nodes_expended_to_fist_solution = awastar.nodes_expended
            end
            awastar.solution_cost = child_node_f
            awastar.solution_walltime = wall_time(awastar)
            awastar.solution_nodes_expended = awastar.nodes_expended
            awastar.solution, awastar.solution_w_trace = get_solution(child_node)
            awastar.num_solutions += 1
            push!(awastar.all_solutions_walltimes, awastar.solution_walltime)
            push!(awastar.all_solutions_costs, awastar.solution_cost)
            push!(awastar.all_solutions_nodes_expended, awastar.solution_nodes_expended)
            push!(awastar.all_solutions, awastar.solution)
        elseif child_node.state in awastar.closed_set || haskey(awastar.open_lists, child_node.state)
            old_node_cost = awastar.path_costs[child_node.state]
            if old_node_cost > child_node.path_cost
                awastar.path_costs[child_node.state] = child_node.path_cost
                if child_node.state in awastar.closed_set
                    delete!(awastar.closed_set, child_node.state)
                else
                    delete!(awastar.open_lists, child_node.state)
                end
                push!(awastar.open_lists, child_node.state, child_node, child_node_g, child_node_h)
            end
        else
            awastar.path_costs[child_node.state] = child_node_g
            push!(awastar.open_lists, child_node.state, child_node, child_node_g, child_node_h)
        end
    end
    return nothing
end

function GraphSearch.on_node_expansion_finish!(awastar::AWAStar{S}, node::TreeSearchNode{S}, sp::AbstractSearchProblem{S}) where S
    awastar.nodes_expended += 1
    return nothing
end

function GraphSearch.on_stop!(awastar::AWAStar{S}, sp::AbstractSearchProblem{S}) where S
    awastar.stop_time = time()
    return nothing
end


function awastar_search_with_dynamic_weights(search_prob::AbstractSearchProblem{S}, weights_adjustment_policy::AbstractWeightAdjustmentPolicy, walltime_limit::Real, nodes_budget::Integer) where S
    awastar = AWAStar{S}(1, walltime_limit, nodes_budget, weights_adjustment_policy)
    return graph_search!(awastar, search_prob)
end

function awastar_search(search_prob::AbstractSearchProblem, weight::Real, walltime_limit::Real, nodes_budget::Integer)
    return awastar_search_with_dynamic_weights(search_prob, ConstantWeightPolicy(weight), walltime_limit, nodes_budget)
end
