using AnytimeWeightedAStar
using AnytimeWeightedAStar.SearchProblem: seed!, reset!, heuristic, start_state
using AnytimeWeightedAStar.ExampleProblems

function solve_all(search_problem, num_instances, rwa_weights, awa_weights, nodes_budget; walltime_limit=Inf)
    solution_qualities_per_approach = Dict{Any,Vector{Float64}}()
    for instance_id in 1:num_instances
        seed!(search_problem, instance_id) 
        reset!(search_problem)
        h0 = heuristic(search_problem, start_state(search_problem))
        for w in [rwa_weights, awa_weights...]
            if w == rwa_weights
                awa = rwastar_search(search_problem, w, walltime_limit, nodes_budget, instance_id)
            else
                awa = awastar_search(search_problem, w, walltime_limit, nodes_budget)
            end
            solution_quality = quality(awa.solution_cost, h0)
            @info "Solved instance" instance_id h0 w awa.nodes_expended awa.num_solutions awa.solution_cost solution_quality

            if !haskey(solution_qualities_per_approach, w)
                solution_qualities_per_approach[w] = Float64[]
            end
            push!(solution_qualities_per_approach[w], solution_quality)
        end
    end
    return solution_qualities_per_approach
end

function get_avg_solution_quality_per_approach(solution_qualities_per_approach)
    avg_per_approach = Dict{Any,Float64}()
    for approach in keys(solution_qualities_per_approach)
        avg_per_approach[approach] = sum(solution_qualities_per_approach[approach]) / length(solution_qualities_per_approach[approach])
    end
    return avg_per_approach
end

rwa_weights = [1,1.5,2,3,4,5]  # Run rwa* with these weights
# awa_weights = [1,1.5,2,3,4,5]  # Run awa* with each of these weights
awa_weights = []  # Run awa* with each of these weights
nodes_budget = 6000
num_instances = 100


search_problem = SlidingPuzzle(4:4, 35:45)  # Sliding Puzzle
# search_problem = SlidingPuzzle(4:4, 35:45, inverse=true)  # Inverse Sliding Puzzle
# search_problem = CityNavigation(150, 150, 3, 3, 100, 1) # City Navigation Problem
# search_problem = TSP(15:25, (0, 0.3))  # Sparse Travelling Salesman Problem

qualities_per_approach = solve_all(search_problem, num_instances, rwa_weights, awa_weights, nodes_budget)
avg_quality_per_approach = get_avg_solution_quality_per_approach(qualities_per_approach)
println("Average qualities:\n", avg_quality_per_approach)

