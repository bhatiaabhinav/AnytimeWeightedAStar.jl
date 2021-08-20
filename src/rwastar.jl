using Random

struct RandomWeightAdjustmentPolicy <: AbstractWeightAdjustmentPolicy
    weights::Vector{Float64}
    rng::MersenneTwister
end

RandomWeightAdjustmentPolicy(weights::AbstractArray{T}) where T <: Real = RandomWeightAdjustmentPolicy(weights, MersenneTwister())
Random.seed!(rwap::RandomWeightAdjustmentPolicy, seed::Integer) = Random.seed!(rwap.rng, seed)


possible_weights(rwap::RandomWeightAdjustmentPolicy) = rwap.weights

(rwap::RandomWeightAdjustmentPolicy)() = rand(rwap.rng, rwap.weights)


function rwastar_search(search_prob::AbstractSearchProblem, weights::AbstractArray{T}, walltime_limit::Real, nodes_budget::Integer, rwa_seed::Integer) where T <: Real
    rwap = RandomWeightAdjustmentPolicy(weights)
    Random.seed!(rwap, rwa_seed)
    return awastar_search_with_dynamic_weights(search_prob, rwap, walltime_limit, nodes_budget)
end

