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
    lk::ReentrantLock
    rng::MersenneTwister
    function TSP(N_range::AbstractArray{Int}, sparsity_range::Tuple{Real,Real})
        tsp = new(N_range, sparsity_range)
        tsp.rng = MersenneTwister()
        tsp.heuristic_cache = Dict{Array{Int},Float64}()
        tsp.N = rand(tsp.rng, tsp.N_range)
        tsp.costs = fill(Inf32, tsp.N, tsp.N)
        tsp.lk = ReentrantLock()
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
    @inline function connect!(cityid1, cityid2, min_distance=0.001, max_distance=1)
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

function mst_prims(G::AbstractMatrix{Float64})::Float64
    V::Int = size(G)[1]  # num verts
    C::Vector{Float64} = fill(Inf, V)  # cost of mst-vertex to reach vertex v
    Q::Vector{Bool} = fill(false, V)  # verts in mst
    q::Int = 0  # size of Q
    C[1] = 0  # cost to reach first vertex
    while q < V
        v = Iterators.argmin(Iterators.map(v -> Q[v] ? Inf : C[v], 1:V))
        Q[v] = true
        q += 1
        for w in Iterators.filter(w -> (0 < G[v, w] < Inf) && !Q[w], 1:V)
            @inbounds C[w] = min(G[v, w], C[w])
        end
    end
    return sum(C)
end

function SearchProblem.heuristic(tsp::TSP, state::Vector{UInt8})::Float64
    if length(state) == tsp.N + 1
        return 0.0
    end
    lock(tsp.lk) do
        if haskey(tsp.heuristic_cache, state)
            return tsp.heuristic_cache[state]
        end
    end

    mst_cities_mask::Vector{Bool} = fill(true, tsp.N)
    if length(state) >= 2
        mst_cities_mask[state[2:end-1]] .= false
    end

    h::Float64 = mst_prims(@views tsp.costs[mst_cities_mask, mst_cities_mask])

    lock(tsp.lk) do
        tsp.heuristic_cache[state] = h
    end
    return h
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
