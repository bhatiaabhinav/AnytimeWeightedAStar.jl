using ..SearchProblem
using ..SearchProblem: reset!
using Random

const SP_ACTIONS_MEANINGS = Symbol[:UP, :DOWN, :LEFT, :RIGHT]  # In this order to generate children
const SP_ACTIONS_DIRECTIONS = Tuple{Int, Int}[(-1, 0), (1, 0), (0, -1), (0, 1)]

mutable struct SlidingPuzzle <: SearchProblem.AbstractSearchProblem{Matrix{UInt8}, Symbol}
    side_range::AbstractArray{Int}
    manhat_range::AbstractArray{Int}
    inverse::Bool
    side::Int
    puzzle::Array{UInt8,2}
    blank_loc::Tuple{Int,Int}
    puzzle_manhat::Int
    goal_puzzle::Array{UInt8,2}
    heuritic_cache::Dict{Array{UInt8,2},Float64}
    rng::MersenneTwister
    function SlidingPuzzle(side_range::AbstractArray{Int}, manhat_range::AbstractArray{Int}; inverse::Bool=false)
        sp = new(side_range, manhat_range, inverse)
        sp.rng = MersenneTwister()
        sp.heuritic_cache = Dict{Array{UInt8,2},Float64}()
        reset!(sp)
        return sp
    end
end


function max_possible_manhat(side::Int)::Int
    s = 0
    k = 1
    while n - k >= 0
        s += n - k
        k += 2
    end
    return 4n * s
end

function initial_puzzle!(puzzle::Array{UInt8,2})
    side = size(puzzle)[1]
    for column in 1:side
        @simd for row in 1:side
            @inbounds puzzle[row, column] = (row - 1) * side + column
        end
    end
    puzzle[side, side] = 0
    return puzzle
end

function initial_puzzle(side::Int)::Matrix{UInt8}
    puzzle = Array{UInt8}(undef, side, side)
    return initial_puzzle!(puzzle)
end

@inline is_valid_blank_location(side::Int, blank_location::Tuple{Int,Int})::Bool = all((1, 1) .<= blank_location .<= (side, side))

function puzzle_legal_actions(side::Int, blank_location::Tuple{Int,Int})
    return Iterators.filter(1:4) do action_id
        @inbounds direction::Tuple{Int, Int} = SP_ACTIONS_DIRECTIONS[action_id]
        return is_valid_blank_location(side, blank_location .+ direction)
    end
end

@inline l1_norm(p1, p2) = sum(abs.(p1 .- p2))

function find_tile_location(puzzle::Array{UInt8, 2}, tile::UInt8)::Tuple{Int, Int}
    side = size(puzzle)[1]
    for c in 1:side
        for r in 1:side
            @inbounds puzzle[r, c] == tile  &&  return r, c
        end
    end
    return -1, -1
end

function compute_manhat(puzzle::Array{UInt8,2}, to_puzzle::Array{UInt8,2}, inverse=false)::Int
    side = size(puzzle)[1]
    manhat = 0.0
    for to_c::Int in 1:side
        for to_r::Int in 1:side
            @inbounds value = to_puzzle[to_r, to_c]
            value == 0 && continue
            r, c = find_tile_location(puzzle, value)
            manhat += inverse ? l1_norm((r, c), (to_r, to_c)) / Float64(value) : l1_norm((r, c), (to_r, to_c))
        end
    end
    return manhat
end

function compute_next_puzzle(puzzle::Array{UInt8,2}, blank_loc::Tuple{Int,Int}, action::Int)::Tuple{Matrix{UInt8}, Tuple{Int, Int}}
    new_puzzle::Array{UInt8, 2} = copy(puzzle)
    blank_r, blank_c = blank_loc
    @inbounds new_blank_r, new_blank_c = blank_loc .+ SP_ACTIONS_DIRECTIONS[action]
    @inbounds new_puzzle[blank_r, blank_c] = new_puzzle[new_blank_r, new_blank_c]
    @inbounds new_puzzle[new_blank_r, new_blank_c] = 0
    new_blank_loc = (new_blank_r, new_blank_c)
    return new_puzzle, new_blank_loc
end

function shuffle_puzzle_hillclimb!(sp::SlidingPuzzle, target_manhat::Int, ϵ=0.1)::Int
    cache = sp.inverse ? Dict{Array{UInt8,2},Float64}() : sp.heuritic_cache
    manhat = haskey(cache, sp.puzzle) ? cache[sp.puzzle] : compute_manhat(sp.puzzle, sp.goal_puzzle)
    cache[sp.puzzle] = manhat
    while manhat < target_manhat
        legal_actions = puzzle_legal_actions(sp.side, sp.blank_loc)
        action = rand(sp.rng, collect(legal_actions))
        new_puzzle, new_blank_loc = compute_next_puzzle(sp.puzzle, sp.blank_loc, action)
        new_manhat = haskey(cache, new_puzzle) ? cache[new_puzzle] : compute_manhat(new_puzzle, sp.goal_puzzle)
        cache[new_puzzle] = new_manhat
        if new_manhat > manhat || rand(sp.rng) < ϵ
            sp.puzzle = new_puzzle
            sp.blank_loc = new_blank_loc
            manhat = new_manhat
        end
    end
    return manhat
end

function SearchProblem.info(sp::SlidingPuzzle)
    return Dict(
        :puzzle_manhat => sp.puzzle_manhat,
        :inverse_cost => sp.inverse
    )
end

@inline function SearchProblem.obs(sp::SlidingPuzzle)::Vector{Float64}
    return Float64[sp.puzzle_manhat / maximum(sp.manhat_range)]
end

function SearchProblem.reset!(sp::SlidingPuzzle)
    empty!(sp.heuritic_cache)
    sp.side = rand(sp.rng, sp.side_range)
    sp.goal_puzzle = initial_puzzle(sp.side)
    sp.puzzle = copy(sp.goal_puzzle)
    sp.blank_loc = (sp.side, sp.side)
    target_manhat = rand(sp.rng, sp.manhat_range)
    sp.puzzle_manhat = shuffle_puzzle_hillclimb!(sp, target_manhat)
    return nothing
end


@inline SearchProblem.start_state(sp::SlidingPuzzle)::Matrix{UInt8} = sp.puzzle

function findmovedtile(from_puzzle::Array{UInt8,2}, to_puzzle::Array{UInt8,2})::UInt8
    side = size(from_puzzle)[1]
    for c in 1:side
        @inbounds for r in 1:side
            if from_puzzle[r, c] != to_puzzle[r, c]
                return max(from_puzzle[r, c], to_puzzle[r, c])  # one of them is zero, the other is the moved tile
            end
        end
    end
end

@inline function SearchProblem.cost(sp::SlidingPuzzle, state::Array{UInt8,2}, action::Symbol, next_state::Array{UInt8,2})::Float64
    if sp.inverse
        moved_tile = findmovedtile(state, next_state)
        return 1.0 / moved_tile
    else
        return 1.0
    end
end


@inline SearchProblem.goal_test(sp::SlidingPuzzle, state::Array{UInt8,2})::Bool = state == sp.goal_puzzle

@inline function SearchProblem.heuristic(sp::SlidingPuzzle, state::Array{UInt8,2})::Float64
    if haskey(sp.heuritic_cache, state)
        return sp.heuritic_cache[state]
    else
        h = compute_manhat(state, sp.goal_puzzle, sp.inverse)
        sp.heuritic_cache[state] = h  
        return h
    end
end

function SearchProblem.successors(sp::SlidingPuzzle, state::Array{UInt8,2})
    blank_loc = find_tile_location(state, 0x00)
    return Iterators.map(puzzle_legal_actions(sp.side, blank_loc)) do action_id
        @inbounds action::Symbol = SP_ACTIONS_MEANINGS[action_id]
        next_puzzle, _ = compute_next_puzzle(state, blank_loc, action_id)
        return next_puzzle, action
    end
end
