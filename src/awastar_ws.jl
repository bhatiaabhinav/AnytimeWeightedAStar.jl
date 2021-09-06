

struct WeightSchedulePolicy <: AbstractWeightAdjustmentPolicy
    weights::Vector{Float64}
end

possible_weights(wsp::WeightSchedulePolicy) = wsp.weights

function (wsp::WeightSchedulePolicy)(;num_solutions, kwargs...)
    w = wsp.weights[min(num_solutions + 1, length(wsp.weights))]
    return w
end

function awastar_search_with_scheduled_weights(search_prob::AbstractSearchProblem, weights::AbstractArray{T}, walltime_limit::Real, nodes_budget::Integer) where T <: Real
    wsp = WeightSchedulePolicy(weights)
    return awastar_search_with_dynamic_weights(search_prob, wsp, walltime_limit, nodes_budget)
end
