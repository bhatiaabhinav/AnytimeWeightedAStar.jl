function update_mean(μ::Real, n::Integer, x::Real; remove::Bool=false)
    mask = 2 * Int(!remove) - 1
    new_n = n + mask
    if new_n == 0
        return 0.0, 0
    elseif new_n < 0
        error("Cannot remove an element from an empty collection")
    else
        sum_x = μ * n
        new_sum_x = sum_x + mask * x
        new_μ = new_sum_x / new_n
        return new_μ, new_n
    end
end

function update_mean_std(μ::Real, σ::Real, n::Integer, x::Real; remove::Bool=false)
    new_μ, new_n = update_mean(μ, n, x; remove=remove)
    if new_n == 0
        return 0.0, 0.0, 0
    end
    σ² = σ^2
    μ_x² = σ² + μ^2
    new_μ_x², new_n = update_mean(μ_x², n, x^2; remove=remove)
    new_σ² = new_μ_x² - new_μ^2
    # if new_σ² < -0.1
    #     @warn "precision error?" new_σ²
    # end
    new_σ² = max(0, new_σ²)
    new_σ = √(new_σ²)
    return new_μ, new_σ, new_n
end


function update_mean_std_corr(μ_x::Real, μ_y::Real, σ_x::Real, σ_y::Real, ρ::Real, n::Integer, x::Real, y::Real; remove::Bool=false)
    new_μ_x, new_σ_x, new_n = update_mean_std(μ_x, σ_x, n, x; remove=remove)
    if new_n == 0
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0
    end
    new_μ_y, new_σ_y, new_n = update_mean_std(μ_y, σ_y, n, y; remove=remove)
    cov = ρ * σ_x * σ_y
    μ_xy = cov + μ_x * μ_y
    new_μ_xy, new_n = update_mean(μ_xy, n, x * y; remove=remove)
    new_cov = new_μ_xy - new_μ_x * new_μ_y
    if abs(new_cov) < 1e-9
        new_ρ = 0.0
    else
        new_ρ = new_cov / (new_σ_x * new_σ_y)
    end
    # if abs(new_ρ) > 1 + 1e-9
    #     @warn "precision error?" new_ρ
    # end
    new_ρ = min(max(-1, new_ρ), 1)
    return new_μ_x, new_μ_y, new_σ_x, new_σ_y, new_ρ, new_n
end

# stats = (0.0, 0.0, 0.0, 0.0, 0.0, 0)

# stats = update_mean_std_corr(stats..., 1, 1)
# stats = update_mean_std_corr(stats..., 2, 1.4)
# stats = update_mean_std_corr(stats..., 1.4, 1.5)

function get_trace_distribution(w_trace, possible_weights)
    dist = Dict{Float64,Float64}()
    for w in possible_weights
        dist[round(w, digits=2)] = 0
    end
    for w in w_trace
        dist[round(w, digits=2)] = dist[round(w, digits=2)] + 1 / length(w_trace)
    end
    return dist
end