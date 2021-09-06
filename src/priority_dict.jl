using DataStructures
import Base: push!, length, keys, pop!, peek, get, delete!, haskey, empty!

mutable struct PriorityDict
    pq::PriorityQueue
    key_node_map::Dict{Any,Any}
end

PriorityDict() = PriorityDict(PriorityQueue(), Dict{Any,Any}())

length(pd::PriorityDict) = length(pd.pq)

haskey(pd::PriorityDict, key) = key in keys(pd.key_node_map)
keys(pd::PriorityDict) = keys(pd.key_node_map)

function push!(pd::PriorityDict, key, value, priority::Float64)
    if haskey(pd, key)
        error("Key already present")
    end
    entry = (key, value, priority)
    pd.key_node_map[key] = entry
    enqueue!(pd.pq, entry, priority)
    return nothing
end

function pop!(pd::PriorityDict)
    if length(pd.pq) == 0
        error("The queue is empty")
    end
    (key, value, priority) = dequeue!(pd.pq)
    delete!(pd.key_node_map, key)
    return key, value, priority
end

function peek(pd::PriorityDict)
    if length(pd.pq) == 0
        error("The queue is empty")
    end
    entry, _ = peek(pd.pq)
    return entry
end

function get(pd::PriorityDict, key, default=nothing)
    if haskey(pd, key)
        key, value, priority = pd.key_node_map[key]
        return value, priority
    else
        return default
    end
end

function delete!(pd::PriorityDict, key)
    if haskey(pd, key)
        entry = pd.key_node_map[key]
        delete!(pd.pq, entry)
        delete!(pd.key_node_map, key)
        (key, value, priority) = entry
        return value, priority
    else
        throw(KeyError(key))
    end
end

function empty!(pd::PriorityDict)
    empty!(pd.pq)
    empty!(pd.key_node_map)
end
