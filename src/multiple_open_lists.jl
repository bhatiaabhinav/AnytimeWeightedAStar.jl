using .GraphSearch: TreeSearchNode
import Base: push!, length, keys, pop!, peek, get, delete!, haskey, values, empty!

mutable struct MultipleOpenLists
    open_lists::Dict{Float64,PriorityDict}
    stats::Tuple{Float64,Float64,Float64,Float64,Float64,Int}
    function MultipleOpenLists(weights::AbstractArray{T}) where T <: Real
        mol = new()
        mol.open_lists = Dict{Float64,PriorityDict}(1 => PriorityDict())
        for w in weights
            mol.open_lists[w] = PriorityDict()
        end
        mol.stats = (0.0, 0.0, 0.0, 0.0, 0.0, 0)
        return mol
    end
end

length(mol::MultipleOpenLists) = length(mol.open_lists[1])
weights(mol::MultipleOpenLists) = keys(mol.open_lists)
keys(mol::MultipleOpenLists) = keys(mol.open_lists[1])
values(mol::MultipleOpenLists) = value(mol.open_lists[1])
haskey(mol::MultipleOpenLists, key) = key in keys(mol.open_lists[1])


function push!(mol::MultipleOpenLists, key::String, node::TreeSearchNode, g::Real, h::Real)
    entry = (node, g, h)
    my_keys = collect(keys(mol.open_lists))
    for w in my_keys
        f_w = g + w * h
        push!(mol.open_lists[w], key, entry, f_w)
    end
    mol.stats = update_mean_std_corr(mol.stats..., g, h)
end

function pop!(mol::MultipleOpenLists, w::Real)
    (key, (node, g, h), _) = pop!(mol.open_lists[w])
    vals = collect(values(mol.open_lists))
    for ol in vals
        if ol ≠ mol.open_lists[w]
            delete!(ol, key)
        end
    end
    return (key, node, g, h)
end

function peek(mol::MultipleOpenLists, w::Real)
    (key, (node, g, h), _) = peek(mol.open_lists[w])
    return key, node, g, h
end

function get(mol::MultipleOpenLists, key::String)
    if key in keys(mol)
        ((node, g, h), _) = get(mol.open_lists[1], key)
        return node, g, h
    else
        throw(KeyError(key))
    end
end

function delete!(mol::MultipleOpenLists, key::String)
    vals = collect(values(mol.open_lists))
    for ol in vals
        delete!(ol, key)
    end
end

function empty!(mol::MultipleOpenLists)
    for ol in values(mol.open_lists)
        empty!(ol)
    end
end