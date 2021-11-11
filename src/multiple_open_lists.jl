using .GraphSearch: TreeSearchNode
import Base: push!, length, keys, pop!, peek, get, delete!, haskey, values, empty!

mutable struct MultipleOpenLists{S}
    key_to_nodehandle_map::Dict{S, Tuple{TreeSearchNode{S}, Float64, Float64, Int}}
    weights::Vector{Float64}
    pqs::Vector{MutableBinaryMinHeap{Tuple{Float64, Int, S}}}
    stats::Tuple{Float64,Float64,Float64,Float64,Float64,Int}
    counter::Int
    function MultipleOpenLists{S}(weights::AbstractArray{T}) where {S, T <: Real}
        mol = new{S}()
        mol.key_to_nodehandle_map = Dict{S, Tuple{TreeSearchNode{S}, Float64, Float64, Int}}()
        mol.weights =  convert(Vector{Float64}, weights)
        if !(1.0 in mol.weights)
            mol.weights = vcat(1.0, mol.weights)
        end
        mol.pqs = MutableBinaryMinHeap{Tuple{Float64, Int, S}}[MutableBinaryMinHeap{Tuple{Float64, Int, S}}() for w in mol.weights]
        mol.stats = (0.0, 0.0, 0.0, 0.0, 0.0, 0)
        mol.counter = 0
        return mol
    end
end

length(mol::MultipleOpenLists) = length(mol.key_to_nodehandle_map)
keys(mol::MultipleOpenLists) = keys(mol.key_to_nodehandle_map)
values(mol::MultipleOpenLists) = values(mol.key_to_nodehandle_map)
haskey(mol::MultipleOpenLists{S}, key::S) where {S} = key in keys(mol.key_to_nodehandle_map)


function push!(mol::MultipleOpenLists{S}, key::S, node::TreeSearchNode{S}, g::Float64, h::Float64)::Nothing where S
    if haskey(mol, key)
        error("already present")
    end
    mol.counter -= 1  # depth first tie breaking
    handle::Int = 0
    @inbounds for i in 1:length(mol.weights)
        w = mol.weights[i]
        f_w = g + w * h
        handle = push!(mol.pqs[i], (f_w, mol.counter, key))
    end
    entry = (node, g, h, handle)
    mol.key_to_nodehandle_map[key] = entry
    mol.stats = update_mean_std_corr(mol.stats..., g, h)
    return nothing
end

@inline function idxof(weight::Float64, weights::AbstractArray{Float64})::Int
    findfirst(isequal(weight), weights)
end

function pop!(mol::MultipleOpenLists{S}, weight::Float64)::Tuple{TreeSearchNode{S}, Float64, Float64} where S
    @inbounds pq = mol.pqs[idxof(weight, mol.weights)]
    (_, _, key), handle = top_with_handle(pq)
    pop!(pq)
    @inbounds for i in 1:length(mol.weights)
        w = mol.weights[i]
        w ≠ weight && delete!(mol.pqs[i], handle)
    end
    node, g, h, handle = mol.key_to_nodehandle_map[key]
    delete!(mol.key_to_nodehandle_map, key)
    return node, g, h
end

function peek(mol::MultipleOpenLists{S}, w::Float64)::Tuple{TreeSearchNode{S}, Float64, Float64} where S
    @inbounds pq = mol.pqs[idxof(w, mol.weights)]
    (_, _, key), handle = top_with_handle(pq)
    node, g, h, handle = mol.key_to_nodehandle_map[key]
    return node, g, h
end

function get(mol::MultipleOpenLists{S}, key::S)::Tuple{TreeSearchNode{S}, Float64, Float64} where S
    return mol.key_to_nodehandle_map[key]
end

function delete!(mol::MultipleOpenLists{S}, key::S)::Nothing where S
    _, _, _, handle = mol.key_to_nodehandle_map[key]
    @inbounds for i in 1:length(mol.weights)
        delete!(mol.pqs[i], handle)
    end
    delete!(mol.key_to_nodehandle_map, key)
    return nothing
end

function empty!(mol::MultipleOpenLists)::Nothing
    @inbounds for i in 1:length(mol.weights)
        extract_all!(mol.pqs[i])
    end
    empty!(mol.key_to_nodehandle_map)
    # TODO: 
    return nothing
end
