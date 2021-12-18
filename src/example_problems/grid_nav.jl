using ..SearchProblem
using Random

mutable struct GNP <: SearchProblem.AbstractSearchProblem{Tuple{Int, Int}, Tuple{Int, Int}}
    N_range::AbstractArray{Int}
    density_range::Tuple{Float64,Float64}
    N::Int
    density::Float64
    grid::BitMatrix  #  true = blocked
    rng::MersenneTwister
    function GNP(N_range::AbstractArray{Int}, density_range::Tuple{Real,Real})
        gnp = new(N_range, density_range)
        gnp.rng = MersenneTwister()
        gnp.N = rand(gnp.rng, gnp.N_range)
        gnp.density = rand(gnp.rng, density_range)
        gnp.grid = BitMatrix(undef, gnp.N, gnp.N)
        return gnp
    end
end

function SearchProblem.info(gnp::GNP)
    return Dict(
        :density => gnp.density,
        :grid_side => gnp.N
    )
end

function SearchProblem.obs(gnp::GNP)
    return Float64[gnp.density, gnp.N / maximum(gnp.N_range)]
end

function SearchProblem.reset!(gnp::GNP)
    gnp.N = rand(gnp.rng, gnp.N_range)
    density = gnp.density_range[1] + (gnp.density_range[2] - gnp.density_range[1]) * rand(gnp.rng)
    I = gnp.N ÷ 20
    gnp.grid = rand(gnp.rng, gnp.N, gnp.N) .< (density / I)
    for i in 1:I
        gnp.grid = gnp.grid .| gnp.grid[:, vcat(2:gnp.N, 1)]
    end
    gnp.density = sum(gnp.grid) / (gnp.N^2)
end

@inline function SearchProblem.start_state(::GNP)::Tuple{Int, Int}
    return (1, 1)
end

function SearchProblem.cost(::GNP, ::Tuple{Int, Int}, ::Tuple{Int, Int}, ::Tuple{Int, Int})::Float64
    return 1.0
end

@inline function SearchProblem.goal_test(gnp::GNP, state::Tuple{Int, Int})::Bool
    return state == (gnp.N, gnp.N)
end

function SearchProblem.heuristic(gnp::GNP, state::Tuple{Int, Int})::Float64
    return (state .- gnp.N) .|> abs |> sum
end

function SearchProblem.successors(gnp::GNP, state::Tuple{Int, Int})
    @inbounds cur_city = state[end]
    available_actions = Iterators.filter(((0, 1), (0, -1), (1, 0), (-1, 0))) do a
        return all(1 .<= state .+ a  .<= gnp.N)  &&  !gnp.grid[(state .+ a)...]
    end
    return Iterators.map(available_actions) do action
        next_state = state .+ action
        return next_state, action
    end
end
