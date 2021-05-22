using Random
using AnytimeWeightedAStar


mutable struct TSP <: AbstractSearchProblem
    N_range::AbstractArray{Int}
    sparsity_range::Tuple{Float32,Float32}
    epsilon::Float32
    N::Int
    state::Array{Int}
    costs::Array{Float32,2}
    heuristic_cache::Dict{Array{Int},Float32}
    sparsity::Float32
    rng::MersenneTwister
    function TSP(N_range::AbstractArray{Int}, sparsity_range::Tuple{Real,Real}, epsilon=0.001)
        tsp = new(N_range, sparsity_range, epsilon)
        tsp.rng = MersenneTwister()
        tsp.heuristic_cache = Dict{Array{Int},Float32}()
        tsp.N = rand(tsp.rng, tsp.N_range)
        tsp.costs = fill(Inf32, tsp.N, tsp.N)
        return tsp
    end
end


function AnytimeWeightedAStar.reset!(tsp::TSP)
    empty!(tsp.heuristic_cache)
    tsp.N = rand(tsp.rng, tsp.N_range)
    target_sparsity = tsp.sparsity_range[1] + (tsp.sparsity_range[2] - tsp.sparsity_range[1]) * rand(tsp.rng)
    tsp.state = [1]
    tsp.costs = fill(Inf32, tsp.N, tsp.N)

    num_connections = 0
    connected(cityid1, cityid2) = tsp.costs[cityid1, cityid2] < Inf
    function connect!(cityid1, cityid2, min_distance=1, max_distance=50)
        if !connected(cityid1, cityid2)
            distance = min_distance + rand(tsp.rng, Float32) *  (max_distance - min_distance)
            tsp.costs[cityid1, cityid2] = tsp.costs[cityid2, cityid1] = distance
            num_connections += 1
        end
    end

    for cityid in 1:tsp.N
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

function AnytimeWeightedAStar.start_state(tsp::TSP)
    return tsp.state
end

function AnytimeWeightedAStar.cost(tsp::TSP, state::Array{Int}, action::String, next_state::Array{Int})
    return tsp.costs[state[end], next_state[end]]
end

function AnytimeWeightedAStar.goal_test(tsp::TSP, state::Array{Int})
    if length(state) == tsp.N + 1
        return true
    else
        return false
    end
    end

function mst_prims(costs::Array{Float32,2})
    N = size(costs)[1]
    mst_nodes = Set{Int}()
    key_vals::Array{Float32} = fill(Inf32, N)
    key_vals[1] = 0
    while length(mst_nodes) < N
        candidates = setdiff(1:N, mst_nodes)
        candidate_values = [key_vals[c] for c in candidates]
        best_node = minimum(zip(candidate_values, candidates))[2]
        push!(mst_nodes, best_node)
        adjacent_nodes = setdiff(filter(node -> 0 < costs[best_node, node] < Inf,  1:N), mst_nodes)
        for adjacent in adjacent_nodes
            key_vals[adjacent] = min(key_vals[adjacent], costs[best_node, adjacent])
        end
    end
    mst_cost = sum(key_vals)
    return mst_cost
end

function AnytimeWeightedAStar.heuristic(tsp::TSP, state)
    h = 0
    if haskey(tsp.heuristic_cache, state)
        return tsp.heuristic_cache[state]
    else
        h = 0
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

function AnytimeWeightedAStar.successors(tsp::TSP, state::Array{Int})
    cur_city = state[end]
    available_actions = Set()
    if length(state) == tsp.N
        if tsp.costs[1, cur_city] < Inf
            push!(available_actions, 1)
        end
    else
        adjacent = filter(c -> 0 < tsp.costs[cur_city, c] < Inf,  1:tsp.N)
        union!(available_actions, adjacent)
        setdiff!(available_actions, state)
    end
    succ = Set{Tuple{Array{Int},String}}()
    for a in available_actions
        next_state = vcat(state, [a])
        action = "goto $a"
        push!(succ, (next_state, action))
    end
    return succ
end


