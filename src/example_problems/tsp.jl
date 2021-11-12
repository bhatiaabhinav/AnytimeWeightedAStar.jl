using ..SearchProblem
using Random

mutable struct TSP <: SearchProblem.AbstractSearchProblem{Vector{UInt8}, UInt8}
    N_range::AbstractArray{Int}
    sparsity_range::Tuple{Float64,Float64}
    N::Int
    state::Array{UInt8}
    costs::Array{Float64,2}
    heuristic_cache::Dict{Array{UInt8},Float64}
    sparsity::Float64
    rng::MersenneTwister
    function TSP(N_range::AbstractArray{Int}, sparsity_range::Tuple{Real,Real})
        tsp = new(N_range, sparsity_range)
        tsp.rng = MersenneTwister()
        tsp.heuristic_cache = Dict{Array{Int},Float64}()
        tsp.N = rand(tsp.rng, tsp.N_range)
        tsp.costs = fill(Inf32, tsp.N, tsp.N)
        return tsp
    end
end

function SearchProblem.info(tsp::TSP)
    return Dict(
        :sparsity => tsp.sparsity,
        :num_cities => tsp.N
    )
end

function SearchProblem.obs(tsp::TSP)
    return Float64[tsp.sparsity, tsp.N / maximum(tsp.N_range)]
end

function SearchProblem.reset!(tsp::TSP)
    empty!(tsp.heuristic_cache)
    tsp.N = rand(tsp.rng, tsp.N_range)
    target_sparsity = tsp.sparsity_range[1] + (tsp.sparsity_range[2] - tsp.sparsity_range[1]) * rand(tsp.rng)
    tsp.state = UInt8[0x01]
    tsp.costs = fill(Inf32, tsp.N, tsp.N)

    num_connections = 0
    @inline connected(cityid1, cityid2) = tsp.costs[cityid1, cityid2] < Inf
    @inline function connect!(cityid1, cityid2, min_distance=1, max_distance=50)
        if !connected(cityid1, cityid2)
            distance = min_distance + rand(tsp.rng, Float64) *  (max_distance - min_distance)
            tsp.costs[cityid1, cityid2] = tsp.costs[cityid2, cityid1] = distance
            num_connections += 1
        end
    end

    @inbounds for cityid in 1:tsp.N
        tsp.costs[cityid, cityid] = 0
        cityid > 1 ? connect!(cityid, cityid - 1) : nothing
        cityid == tsp.N ? connect!(cityid, 1) : nothing
    end
        for cityid1 in 1:(tsp.N - 1)
        for cityid2 in (cityid1 + 1):tsp.N
            if rand(tsp.rng) > target_sparsity
                connect!(cityid1, cityid2)
            end
        end
    end
    max_connections  = Int(tsp.N * (tsp.N - 1) // 2)
    tsp.sparsity = (max_connections - num_connections) / max_connections
end

@inline function SearchProblem.start_state(tsp::TSP)::Vector{UInt8}
    return tsp.state
end

function SearchProblem.cost(tsp::TSP, state::Array{UInt8}, action::UInt8, next_state::Array{UInt8})::Float64
    return @inbounds tsp.costs[state[end], next_state[end]]
end

@inline function SearchProblem.goal_test(tsp::TSP, state::Array{UInt8})::Bool
    if length(state) == tsp.N + 1
        return true
    else
        return false
    end
end

function mst_prims(costs::Array{Float64,2})::Float64
    N = size(costs)[1]
    mst_nodes = Set{Int}()
    key_vals::Array{Float64} = fill(Inf32, N)
    key_vals[1] = 0
    while length(mst_nodes) < N
        candidates = setdiff(1:N, mst_nodes)
        candidate_values = @inbounds Float64[key_vals[c] for c in candidates]
        best_node = minimum(zip(candidate_values, candidates))[2]
        push!(mst_nodes, best_node)
        adjacent_nodes::Vector{Int} = setdiff(filter(node -> 0 < costs[best_node, node] < Inf,  1:N), mst_nodes)
        @simd for adjacent in adjacent_nodes
            @inbounds key_vals[adjacent] =  ifelse(key_vals[adjacent] < costs[best_node, adjacent], key_vals[adjacent], costs[best_node, adjacent])
        end
    end
    mst_cost = sum(key_vals)
    return mst_cost
end

function SearchProblem.heuristic(tsp::TSP, state::Vector{UInt8})::Float64
    if haskey(tsp.heuristic_cache, state)
        return tsp.heuristic_cache[state]
    else
        h::Float64 = 0
        mst_cities = Set(1:tsp.N)

        if length(state) >= 2
            setdiff!(mst_cities, state[1:end - 1]) # remove all visited except current
        end
        mst_cities = sort(collect(mst_cities))

        if length(mst_cities) > 0
            h = mst_prims(tsp.costs[mst_cities, mst_cities])
        end

        # TODO: Consider keeping starting city always in the mst cities.
        tsp.heuristic_cache[state] = h
        return h
    end
end

function SearchProblem.successors(tsp::TSP, state::Array{UInt8})
    @inbounds cur_city = state[end]
    @inbounds if length(state) == tsp.N
        if tsp.costs[1, cur_city] < Inf
            available_actions = UnitRange{UInt8}(1, 1)
        else
            available_actions = UnitRange{UInt8}(1, 0) # empty range
        end
    else
        available_actions = Iterators.filter(UnitRange{UInt8}(1, tsp.N)) do c
            return @inbounds (0 < tsp.costs[cur_city, c] < Inf) && !(c in state)
        end
    end
    return Iterators.map(available_actions) do action
        next_state = vcat(state, action)
        return next_state, action
    end
end
