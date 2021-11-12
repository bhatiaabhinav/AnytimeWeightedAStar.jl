using ..SearchProblem
using Random

puzzle_actions = Dict{String,Tuple{Int,Int}}(
        "UP" => (-1, 0),
        "DOWN" => (1, 0),
        "LEFT" => (0, -1),
        "RIGHT" => (0, 1))


mutable struct City
    position::Tuple{Float32,Float32}
    num_places::Int
    np::Int
    side::Float32
    places_positions::Array{Tuple{Float32,Float32}}
    places_neighbors::Array{Array{Int}}
    costs::Array{Float32,2}
    function City(position::Tuple{Real,Real}, num_places::Integer, np::Integer, side::Real=1)
        city = new(position, num_places, np, side)
        city.places_positions = Array{Tuple{Int,Int}}(undef, num_places)
        city.places_neighbors = fill(Int[], num_places)
        city.costs = fill(Inf32, num_places, num_places)
        return city
    end
end

@inline euclid_distance(point1, point2) = sqrt(sum((point1 .- point2).^2))

function reset_city!(city::City, position::Tuple{Real,Real}, rng::MersenneTwister)
    @inline connected(place1, place2) = city.costs[place1, place2] < Inf
    function connect!(place1, place2)
        if !connected(place1, place2)
            distance = euclid_distance(city.places_positions[place1], city.places_positions[place2])
            city.costs[place1, place2] = city.costs[place2, place1] = distance + (1 + 0.1 * rand(rng, Float32))
            push!(city.places_neighbors[place1], place2)
            push!(city.places_neighbors[place2], place1)
        end
    end

    city.position = position
    city.places_neighbors = [Int[] for _ in 1:city.num_places]
    fill!(city.costs, Inf32)

    for place in 1:city.num_places
        city.places_positions[place] = place == 1 ? city.position : Tuple(city.position .+ rand(rng, Float32, 2) * city.side)
        city.costs[place, place] = 0
        place > 1 ? connect!(place, place - 1) : nothing
        place == city.num_places ? connect!(place, 1) : nothing
    end
    for place in 1:city.num_places
        nearest = sort(1:city.num_places, by=p -> euclid_distance(city.places_positions[place], city.places_positions[p]))
        for p in nearest[2:city.np + 1]
            connect!(place, p)
        end
    end
end


mutable struct CityNavigation <: SearchProblem.AbstractSearchProblem{Tuple{Int, Int}, String}
    num_cities::Int
    num_places::Int
    nc::Int
    np::Int
    side::Float64
    city_side::Float64
    cities::Array{City,1}
    cities_neighbors::Array{Array{Int}}
    costs::Array{Float32,2}
    rng::MersenneTwister
    start_loc::Tuple{Int,Int}
    end_loc::Tuple{Int,Int}
    function CityNavigation(num_cities::Integer, num_places::Integer, nc::Integer, np::Integer, side::Real, city_side::Real)
        cnp = new(num_cities, num_places, nc, np, side, city_side)
        cnp.rng = MersenneTwister()
        cnp.cities = [City((0, 0), num_places, np, city_side) for _ in 1:num_cities]
        cnp.cities_neighbors = fill(Int[], num_cities)
        cnp.costs = fill(Inf32, num_cities, num_cities)
        cnp.start_loc = cnp.end_loc = (0, 0)
    return cnp
    end
end


function SearchProblem.reset!(cnp::CityNavigation)
    @inline connected(cityid1, cityid2) = cnp.costs[cityid1, cityid2] < Inf
    function connect!(cityid1, cityid2)
        if !connected(cityid1, cityid2)
            distance = euclid_distance(cnp.cities[cityid1].position, cnp.cities[cityid2].position)
            cnp.costs[cityid1, cityid2] = cnp.costs[cityid2, cityid1] = distance + 2
            push!(cnp.cities_neighbors[cityid1], cityid2)
            push!(cnp.cities_neighbors[cityid2], cityid1)
        end
    end

    cnp.cities_neighbors = [Int[] for _ in 1:cnp.num_cities]
    fill!(cnp.costs, Inf32)
    for cityid in 1:cnp.num_cities
        reset_city!(cnp.cities[cityid], Tuple(rand(cnp.rng, Float32, 2) * cnp.side), cnp.rng)
        cnp.costs[cityid, cityid] = 0
        cityid > 1 ? connect!(cityid, cityid - 1) : nothing
        cityid == cnp.num_cities ? connect!(cityid, 1) : nothing
    end
    for cityid in 1:cnp.num_cities
        nearest = sort(1:cnp.num_cities, by=cid -> euclid_distance(cnp.cities[cityid].position, cnp.cities[cid].position))
        for cid in nearest[2:cnp.nc + 1]
            connect!(cityid, cid)
        end
    end

    cnp.start_loc = cnp.end_loc = (1, 1)
    while cnp.start_loc == cnp.end_loc
        cnp.start_loc = rand(cnp.rng, 1:cnp.num_cities), rand(cnp.rng, 1:cnp.num_places)
        cnp.end_loc = rand(cnp.rng, 1:cnp.num_cities), rand(cnp.rng, 1:cnp.num_places)
    end
end


@inline SearchProblem.start_state(cnp::CityNavigation) = cnp.start_loc

@inline function SearchProblem.cost(cnp::CityNavigation, state::Tuple{Integer,Integer}, action::String, next_state::Tuple{Integer,Integer})::Float64
    cityid1, placeid1 = state
    cityid2, placeid2 = next_state
    return cityid1 == cityid2 ? cnp.cities[cityid1].costs[placeid1, placeid2] : cnp.costs[cityid1, cityid2]
end

@inline function SearchProblem.goal_test(cnp::CityNavigation, state::Tuple{Int, Int})::Bool
    return state == cnp.end_loc
end

@inline function SearchProblem.heuristic(cnp::CityNavigation, state::Tuple{Int, Int})::Float64
    cityid, placeid = state
    dest_cityid, dest_placeid = cnp.end_loc
    return euclid_distance(cnp.cities[cityid].places_positions[placeid],
                        cnp.cities[dest_cityid].places_positions[dest_placeid])
end

function SearchProblem.successors(cnp::CityNavigation, state::Tuple{Int, Int})
    cur_city_id, cur_place_id = state
    cur_city = cnp.cities[cur_city_id]
    succ = Set{Tuple{Tuple{Int,Int},String}}()
    for neighbor_place_id in cur_city.places_neighbors[cur_place_id]
        action = "goto ($cur_city_id, $neighbor_place_id)"
        next_state = (cur_city_id, neighbor_place_id)
        push!(succ, (next_state, action))
    end
    if cur_place_id == 1
        # can go to other cities as well
        for neighbor_city_id in cnp.cities_neighbors[cur_city_id]
            action = "goto city $neighbor_city_id"
            next_state = (neighbor_city_id, 1)
            push!(succ, (next_state, action))
        end
    end
    return succ
end


