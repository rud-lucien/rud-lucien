### A Pluto.jl notebook ###
# v0.19.29

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
end

# ╔═╡ a1d37aac-69cb-49a3-9ec0-6fcc8f71ab0e
begin
    using StatsBase
    using Colors
    using PlutoUI
    using Distributions
    using CairoMakie
    using StatisticalRethinking
    using KernelDensity
    using Random
end

# ╔═╡ b1a2a660-6644-11ee-2f12-2d062a616500
md"""
# Stitistical Rethinking Chapter 3
"""

# ╔═╡ 4dbe0da7-fab9-40e9-b394-1f81b60db7c4
md"""
code 3.1
"""

# ╔═╡ 8e68fbfe-f2eb-43da-bc7f-ed0ad0cca50d
let
    Pr_Positive_Vampire = 0.95
    Pr_Positive_Mortal = 0.01
    Pr_Vampire = 0.001
    Pr_Positive = Pr_Positive_Vampire * Pr_Vampire + Pr_Positive_Mortal * (1 - Pr_Vampire)
    Pr_Vampire_Positive = Pr_Positive_Vampire * Pr_Vampire / Pr_Positive
end

# ╔═╡ a727d158-533c-4d36-9755-96b7d197b2a9
@bind slider_1 PlutoUI.Slider(collect(range(3, 1000)), show_value = true)

# ╔═╡ 55c209c3-1a18-4b81-a186-fe27e9d78117
md"""
code 3.2
"""

# ╔═╡ b5a2fe3f-0339-42e8-a82a-cc0497fe1993
begin
    size = slider_1

    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    prob_data = pdf.(Binomial.(9, p_grid), 6)

    #compute product of prob_data and prior
    posterior = prob_data .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)
end

# ╔═╡ 8396f44a-ac96-44f1-b817-a0c7fb6359f7
md"""
code 3.3
"""

# ╔═╡ ea9ba18e-45b8-4255-b00b-a2a455b7490a
samples = sample(p_grid, Weights(posterior), 10_000; replace = true)

# ╔═╡ dd01cf89-1ddb-46eb-9a50-6411e78bd0d3
# define the jitter function as is done in R programing language
# this funtion is simply adding noise to data so the points don't overlap when plotted
function jitter(x)
    z = findmax(collect(skipmissing(x)))[1] - findmin(collect(skipmissing(x)))[1]
    a = z / 50
    if a == 0
        x = x .+ rand(length(x))
        return x
    else
        x = x .+ rand(Uniform(-a, a), length(x))
        return x
    end
end

# ╔═╡ 64ded6c5-a60c-4a12-8c0c-915a88e06d79
md"""
code 3.4 and 3.5
"""

# ╔═╡ b2553cd7-cf73-4538-b04c-6f65652f65ce
let
    f = Figure()
    ax = Axis(f[1, 1]; xlabel = "Sample number", ylabel = "Proportion water (p)")
    scatter!(jitter(samples); alpha = 0.2, markersize = 10)
    ax = Axis(f[1, 2]; xlabel = "Proportion water (p)", ylabel = "Density")
    density!(
        jitter(samples);
        color = (:lightblue, 0.3),
        strokecolor = :skyblue,
        strokewidth = 3,
        strokearound = false,
    )
    f

end

# ╔═╡ b730549a-4254-4bfc-8110-b85a92bc6cfb
md"""
code 3.6
"""

# ╔═╡ 11f4797c-5f92-47cc-b9a7-9d3e6bcfffdb
# add up posterior probability where p < 0.5
sum(posterior[p_grid.<0.5])

# ╔═╡ 9d6c011d-4247-4542-b92a-275d310855c3
md"""
code 3.7
"""

# ╔═╡ 1e9f72d9-74c7-42ec-af1b-6e1915ed2422
# proportion of water less than 0.5 using samples
sum(samples .< 0.5) / length(samples)

# ╔═╡ 2a0cfd4b-a89e-4789-af54-acb3c9f439ae
let
    f = Figure()
    x = 0:0.01:1
    d = Normal(mean(samples), std(samples))

    ax = Axis(f[1, 1]; xlabel = "Proportion water (p)", ylabel = "Density")
    lines!(x, pdf.(d, x))
    x1 = range(0, 0.5; length = 100)
    band!(x1, fill(0, length(x1)), pdf.(d, x1); color = (:darkblue, 0.5), label = "Label")

    ax = Axis(f[1, 2]; xlabel = "Sample number", ylabel = "Proportion water (p)")
    lines!(x, pdf.(d, x))
    x1 = range(0.5, 0.75; length = 100)
    band!(x1, fill(0, length(x1)), pdf.(d, x1); color = (:darkblue, 0.5), label = "Label")

    ax = Axis(f[2, 1]; xlabel = "Sample number", ylabel = "Proportion water (p)")
    lines!(x, pdf.(d, x))
    x1 = range(0, 0.75; length = 100)
    band!(x1, fill(0, length(x1)), pdf.(d, x1); color = (:darkblue, 0.5), label = "Label")

    ax = Axis(f[2, 2]; xlabel = "Sample number", ylabel = "Proportion water (p)")
    lines!(x, pdf.(d, x))
    x1 = range(0.45, 0.81; length = 100)
    band!(x1, fill(0, length(x1)), pdf.(d, x1); color = (:darkblue, 0.5), label = "Label")
    f
end

# ╔═╡ 40a41c9a-0a56-4c33-bd1c-0ec4644cbba1
md"""
code 3.8
"""

# ╔═╡ 1b69b3a0-5f90-4332-8621-9f95dea33e49
# asking how much posterior probability lies between 0.5 and 0.75 using samples
sum((samples .> 0.5) .& (samples .< 0.75)) / length(samples)

# ╔═╡ 594dafeb-ffca-4567-b86e-b39005c5b30f
md"""
code 3.9
"""

# ╔═╡ bf429452-a2c3-4302-82ec-ba5c213e10fd
quantile(samples, 0.8)

# ╔═╡ 2f44cf6b-a2e0-4aaa-9ab1-69f94bea2fb2
md"""
code 3.10
"""

# ╔═╡ acf9a20a-3de9-4117-b5e5-07e3c1bf4692
quantile(samples, [0.1, 0.9])

# ╔═╡ c7eadfbf-753e-4cf8-b76b-8c98a64613af
md"""
code 3.11
"""

# ╔═╡ 46e31cca-ad35-4b12-b52f-0d0639cd4b36
let
    size = 1000

    # define grid
    global p_grid2 = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    prob_data = pdf.(Binomial.(3, p_grid2), 3)

    #compute product of prob_data and prior
    global posterior2 = prob_data .* prior

    #standardize the posterior so it sums to 1
    posterior2 = posterior2 / sum(posterior2)

    global samples2 = sample(p_grid2, Weights(posterior2), 10_000; replace = true)

end

# ╔═╡ 2d1c69e5-18f0-4644-a3d0-d8d1e870d6bd
md"""
code 3.12
"""

# ╔═╡ 49ad2981-5e41-4aec-8e07-313fdaa1ac42
PI = percentile(samples2, [25, 75])

# ╔═╡ a442fbad-a7f3-4913-8dc7-73c92d98da5c
md"""
code 3.13
"""

# ╔═╡ b9dd4fde-d817-49af-9df3-11ed654e10ac
HPDI = hpdi(samples2, alpha = 0.5)

# ╔═╡ 3df216ca-3051-4540-96bb-64492867b26a
md"""
code 3.14
"""

# ╔═╡ db85bf7d-d1fc-4e4f-af9c-0a352e4af677
p_grid[argmax(posterior2)]

# ╔═╡ 312f4d1c-7e24-457a-a924-77403cbc1651
md"""
code 3.15
"""

# ╔═╡ 4278d117-1327-4ec4-9591-ebdb2b0784fd
let
    k = kde(samples2, bandwidth = 0.01)
    k.x[argmax(k.density)]
end

# ╔═╡ be8760c4-002a-498a-a277-c3f056022694
md"""
code 3.16
"""

# ╔═╡ e251de97-82f3-4177-9b6f-762420109e9b
mean(samples2), median(samples2)

# ╔═╡ ab9f9a1a-3494-45c6-af56-71429230301e
md"""
code 3.17
"""

# ╔═╡ 956a1e74-b919-4f89-adac-9085c06fc33b
sum(posterior2 .* abs.(0.5 .- p_grid)), sum(@. posterior2 * abs(0.5 - p_grid))

# ╔═╡ 059cd603-f14d-4746-b4e6-68a674ea0df4
md"""
code 3.18
"""

# ╔═╡ 0498478c-e8bc-46af-bbcd-cd7bac5f122e
loss = map(d -> sum(@. posterior2 * abs(d - p_grid2)), p_grid2)

# ╔═╡ 48706b5a-851b-4a36-b391-52f51c7870c4
md"""
code 3.19
"""

# ╔═╡ ce0fe42c-2e59-4afa-94d2-79b4bc117594
p_grid2[argmin(loss)], median(samples2)

# ╔═╡ 7c917b41-290f-4913-83dd-ddc3c7703f64
md"""
code 3.20
"""

# ╔═╡ f59ea9f7-182a-4bab-918c-a339515c0b6c
pdf.(Binomial.(2, 0.7), 0:2)

# ╔═╡ 597c35a6-4be9-48b0-949b-f96e233da32e
md"""
code 3.21
"""

# ╔═╡ b36f7121-733a-48f7-bfb6-70a0bfc8c846
rand(Binomial(2, 0.7))

# ╔═╡ ae9324c0-ea3e-4d77-9e81-9b60fb49f675
md"""
code 3.22
"""

# ╔═╡ fe7e662a-b83f-45d6-80cc-690cedc85a17
rand(Binomial(2, 0.7), 10)

# ╔═╡ dfe46561-f95e-47e4-99c6-facfc9c19092
md"""
code 3.23
"""

# ╔═╡ 2fcdd072-d836-42f2-a455-03e412aac028
let
    dummy_w = rand(Binomial(2, 0.7), 100_000)
    proportions(dummy_w)  # or counts(dummy_w)/100000
end

# ╔═╡ 02278217-cec8-4a4b-8477-d072ebccca14
md"""
code 3.24
"""

# ╔═╡ 10e668e3-ac76-4f9d-b923-7b8f94851dee
let
    dummy_w = rand(Binomial(9, 0.7), 100_000)
    f = Figure()
    ax = Axis(f[1, 1]; xlabel = "dummy water count", ylabel = "Frequency")
    hist!(dummy_w; bins = 30)
    f
end

# ╔═╡ 9e66d2d2-b0e9-4f2b-b106-8a9e8bb33349
md"""
code 3.25
"""

# ╔═╡ eef578ff-0330-4e4a-afcb-53a70d15a65b
let
    w = rand(Binomial(9, 0.6), 10_000)
end

# ╔═╡ b66f9b2f-f81c-4269-9162-593ed88a2251
md"""
code 3.26
"""

# ╔═╡ 0678ff0f-804d-4334-a967-0c5e966fde7f
w = @. rand(Binomial(9, samples2))

# ╔═╡ fa55f9de-deab-4746-89c2-e7b441ccf0b1
md"""
# 3.5 Practice
"""

# ╔═╡ 1ccf5425-7c7a-4b3a-bdc7-2c5caa12feba
md"""
code 3.27
"""

# ╔═╡ f06e9cab-4f28-419a-a9e8-6ab79e3b744a
begin
    # define the grid
    p_grid_1 = range(0, 1, 1000)

    # define the prior
    prior_1 = ones(1000)

    # compute the probability of the data at each value of the grid
    likelihood = @. pdf(Binomial(9, p_grid_1), 6)

    # compute the product of likelihood and prior
    posterior_1 = likelihood .* prior_1

    # standardize the posterior so it sums to 1
    posterior_1 = posterior_1 / sum(posterior_1)

    Random.seed!(100)

    samples_1 = sample(p_grid_1, Weights(posterior_1), 10_000; replace = true)
end

# ╔═╡ 0709734f-8045-4774-b627-df6924cf2e1d
md"""
##### 3E1
"""

# ╔═╡ 9c35d537-f799-4acc-a363-bb04173f79af
sum(samples_1 .< 0.2) / length(samples_1), sum(posterior_1[p_grid_1.<0.2])

# ╔═╡ 838c17ed-3d1c-4478-b188-6ddd4f2c8868
md"""
##### 3E2
"""

# ╔═╡ 459435a3-da56-4a4b-bde1-9d0a26b3e5a0
sum(samples_1 .> 0.8) / length(samples_1), sum(posterior_1[p_grid_1.>0.8])

# ╔═╡ 242bbb4e-3728-49cd-94b5-cd5de457b04e
md"""
##### 3E3
"""

# ╔═╡ 2e207ffd-edd4-4905-a31d-78bb695ab5e0
let
    indices = (p_grid_1 .> 0.2) .& (p_grid_1 .< 0.8)

    sum((samples_1 .> 0.2) .& (samples_1 .< 0.8)) / length(samples_1),
    sum(posterior_1[indices])
end

# ╔═╡ 6a4db54d-8bcf-47ee-96b4-ea008de7ee10
md"""
##### 3E4
"""

# ╔═╡ 2755a82e-b257-455f-9f41-4f9bc54ab297
quantile(samples_1, 0.2)

# ╔═╡ d19a0244-7609-450d-bd3d-4d3148132ea2
# Check
sum(samples_1 .< quantile(samples_1, 0.2)) / length(samples_1)

# ╔═╡ d93962bf-bfdb-4a14-ad5b-9de2e3fbe28c
md"""
##### 3E5
"""

# ╔═╡ 88d8b521-fa0b-4a97-9d08-04963a708c94
quantile(samples_1, 0.8)

# ╔═╡ fefe14d7-bb22-4b39-ae61-a3482704c444
# Check
sum(samples_1 .> quantile(samples_1, 0.8)) / length(samples_1)

# ╔═╡ 1d94b500-af26-4827-8b35-b788d331609a
md"""
##### 3E6
"""

# ╔═╡ 7dc3427e-e31b-4257-a11f-55efed015e53
hpdi(samples_1, alpha = 0.34)

# ╔═╡ 31f52a92-d658-41e9-9112-f856264ab29f
# Check
let
    index_1 = hpdi(samples_1, alpha = 0.34)[1]
    index_2 = hpdi(samples_1, alpha = 0.34)[2]
    sum((index_1 .< samples_1 .< index_2)) / length(samples_1)
end

# ╔═╡ a295ecd2-3556-4806-9c9a-0776504b6418
md"""
##### 3E7
"""

# ╔═╡ 0d56904c-c12c-4968-8564-6d4bff492e08
let
    # Assuming posterior_1 is your posterior probabilities and p_grid_1 is the corresponding p values
    sorted_indices = sortperm(p_grid_1)
    sorted_posterior = posterior_1[sorted_indices]
    sorted_p = p_grid_1[sorted_indices]

    cumulative_posterior = cumsum(sorted_posterior)

    # Normalize the cumulative sum to make it a proper CDF
    cumulative_posterior ./= maximum(cumulative_posterior)

    # Find the indices
    lower_index = findfirst(cumulative_posterior .>= 0.17)
    upper_index = findfirst(cumulative_posterior .>= 0.83)

    # Get the corresponding p values
    lower_p = sorted_p[lower_index]
    upper_p = sorted_p[upper_index]

    (lower_p, upper_p)
end

# ╔═╡ cd9b2d47-4f12-49b0-b1e2-388af388638d
md"""
##### 3M1
"""

# ╔═╡ 4a8304e7-e4e6-4f01-a76d-077700fc46ec
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(15, p_grid), 8)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    f = Figure()
    ax = Axis(f[1, 1], xlabel = "p_grid", ylabel = "posterior probability")

    # Generate random color
    random_color = RGB(rand(), rand(), rand())

    lines!(ax, p_grid, posterior, color = random_color)

    f
end

# ╔═╡ de5e6cc1-3314-45c9-ab7f-afbda265e595
md"""
##### 3M2
"""

# ╔═╡ fd671a3e-9222-4d56-952f-e034b41fe201
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(15, p_grid), 8)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    samples = sample(p_grid, Weights(posterior), 10_000; replace = true)

    hpdi(samples, alpha = 0.10)
end

# ╔═╡ 6f348bd5-ab5b-445b-b9f9-22947046db91
md"""
##### 3M3
"""

# ╔═╡ 3ccd2e6c-1f77-4193-a1d5-05c289e11f4d
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(15, p_grid), 8)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    samples = sample(p_grid, Weights(posterior), 10_000; replace = true)

    w = @. rand(Binomial(15, samples))
    proportions(w)  # or counts(w)/length(w)
    proportions(w)[8] # or sum(w .== 8)/length(w)

    # Generate random color
    random_color = RGB(rand(), rand(), rand())

    f = Figure()
    ax = Axis(f[1, 1]; xlabel = "counts of water", ylabel = "probability")
    hist!(w; bins = 30, normalization = :probability, color = random_color)
    println("Probability of 8 waters is $(round(proportions(w)[8], digits=4))")
    f

end

# ╔═╡ 1f95be8e-2092-4327-b8d7-afe723f3f558
md"""
##### 3M4
"""

# ╔═╡ b9e699ad-446d-4b77-bcab-628e0ed286ca
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(9, p_grid), 6)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    samples = sample(p_grid, Weights(posterior), 10_000; replace = true)

    w = @. rand(Binomial(9, samples))
    proportions(w)  # or counts(w)/length(w)
    proportions(w)[6] # or sum(w .== 8)/length(w)

    # Generate random color
    random_color = RGB(rand(), rand(), rand())

    f = Figure()
    ax = Axis(f[1, 1]; xlabel = "counts of water", ylabel = "probability")
    hist!(w; bins = 30, normalization = :probability, color = random_color)
    println("Probability of 6 waters is $(round(proportions(w)[6], digits=4))")
    f

end

# ╔═╡ 2daa1227-200a-4bab-8e45-116a59f87ce4
md"""
##### 3M5
"""

# ╔═╡ 2b5423c4-1870-443f-adce-4a29e7eb9795
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ifelse.(p_grid .>= 0.5, 1, 0)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(15, p_grid), 8)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    f = Figure()
    ax = Axis(f[1, 1], xlabel = "p_grid", ylabel = "posterior probability")

    # Generate random color
    random_color = RGB(rand(), rand(), rand())

    lines!(ax, p_grid, posterior, color = random_color)

    f
end

# ╔═╡ e0b07136-0fbd-48d8-9865-aede4b7e1d78
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ifelse.(p_grid .>= 0.5, 1, 0)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(15, p_grid), 8)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    samples = sample(p_grid, Weights(posterior), 10_000; replace = true)

    hpdi(samples, alpha = 0.10)
end

# ╔═╡ 2197e493-5b52-44c2-b207-aad2518f9230
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ifelse.(p_grid .>= 0.5, 1, 0)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(15, p_grid), 8)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    samples = sample(p_grid, Weights(posterior), 10_000; replace = true)

    w = @. rand(Binomial(15, samples))
    proportions(w)  # or counts(w)/length(w)
    proportions(w)[8] # or sum(w .== 8)/length(w)

    # Generate random color
    random_color = RGB(rand(), rand(), rand())

    f = Figure()
    ax = Axis(f[1, 1]; xlabel = "counts of water", ylabel = "probability")
    hist!(w; bins = 30, normalization = :probability, color = random_color)
    println("Probability of 8 waters is $(round(proportions(w)[8], digits=4))")
    f

end

# ╔═╡ 07c6c448-4b02-4090-9206-d3c4850ecc38
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ifelse.(p_grid .>= 0.5, 1, 0)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(9, p_grid), 6)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    samples = sample(p_grid, Weights(posterior), 10_000; replace = true)

    w = @. rand(Binomial(9, samples))
    proportions(w)  # or counts(w)/length(w)
    proportions(w)[6] # or sum(w .== 8)/length(w)

    # Generate random color
    random_color = RGB(rand(), rand(), rand())

    f = Figure()
    ax = Axis(f[1, 1]; xlabel = "counts of water", ylabel = "probability")
    hist!(w; bins = 30, normalization = :probability, color = random_color)
    println("Probability of 6 waters is $(round(proportions(w)[6], digits=4))")
    f

end

# ╔═╡ 82808b10-19f7-4900-8fe3-5e16345de4e7
let
    size = 1000
    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)
    prior_2 = ifelse.(p_grid .>= 0.5, 1, 0)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(15, p_grid), 8)

    #compute product of prob_data and prior
    posterior = likelihood .* prior
    posterior_2 = likelihood .* prior_2

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)
    posterior_2 = posterior_2 / sum(posterior_2)

    sample_1 = sample(p_grid, Weights(posterior), 10_000; replace = true)
    sample_2 = sample(p_grid, Weights(posterior_2), 10_000; replace = true)

    mean(sample_1), mean(sample_2)

end

# ╔═╡ ce567663-7d7e-4a92-87e6-d3b2a97d5e39
md"""
##### 3M6
"""

# ╔═╡ 863a6d24-9640-42e4-a9cd-ae40f3c91ac6
let
    size = 1000
    p_grid = range(0, 1; length = size)
    prior = ones(size)

    for tosses = 1:10^5
        waters = rand(Binomial(tosses, 0.7))  # Simulate the number of successful tosses
        likelihood = pdf.(Binomial.(tosses, p_grid), waters)
        posterior = likelihood .* prior
        posterior /= sum(posterior)
        samples = sample(p_grid, Weights(posterior), 10_000; replace = true)
        h = hpdi(samples, alpha = 0.005)  # Calculate the 99% HPDI
        if h[2] - h[1] < 0.05
            println("Need to do $tosses tosses")
            break
        end
    end
end

# ╔═╡ ab098d20-af70-4cb6-9497-32eb5ad972cd
md"""
code 3.28
"""

# ╔═╡ 7d01bccd-c209-4407-a987-2c8ca4058ea3
birth1 = [
    1,
    0,
    0,
    0,
    1,
    1,
    0,
    1,
    0,
    1,
    0,
    0,
    1,
    1,
    0,
    1,
    1,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    1,
    1,
    1,
    0,
    1,
    0,
    1,
    1,
    1,
    0,
    1,
    0,
    1,
    1,
    0,
    1,
    0,
    0,
    1,
    1,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
    1,
    0,
    1,
    0,
    0,
    1,
    0,
    0,
    0,
    1,
    0,
    0,
    1,
    1,
    1,
    1,
    0,
    1,
    0,
    1,
    1,
    1,
    1,
    1,
    0,
    0,
    1,
    0,
    1,
    1,
    0,
    1,
    0,
    1,
    1,
    1,
    0,
    1,
    1,
    1,
    1,
]

# ╔═╡ d16a4de6-2e96-40f5-b769-473c4f0d92c6
birth2 = [
    0,
    1,
    0,
    1,
    0,
    1,
    1,
    1,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    0,
    0,
    1,
    1,
    1,
    0,
    0,
    1,
    1,
    1,
    0,
    1,
    1,
    1,
    0,
    1,
    1,
    1,
    0,
    1,
    0,
    0,
    1,
    1,
    1,
    1,
    0,
    0,
    1,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    0,
    1,
    1,
    0,
    1,
    1,
    0,
    1,
    1,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    1,
    1,
    0,
    0,
    1,
    0,
    0,
    1,
    1,
    0,
    0,
    0,
    1,
    1,
    1,
    0,
    0,
    0,
    0,
]

# ╔═╡ c435e9bd-fdfa-4f99-a082-670cce387a9e
md"""
code 3.30
"""

# ╔═╡ 38206e59-07f9-45d3-beb9-aca653925397
sum(birth1) + sum(birth2)

# ╔═╡ 3abafd5c-9c5f-4044-9e25-fadc1639f5be
md"""
##### 3H1
"""

# ╔═╡ db1bc18b-11d5-4e76-8d17-c2515e33b4af
let
    size = 1000

    count_boys = sum(birth1) + sum(birth2)
    count_total = length(birth1) + length(birth2)

    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(count_total, p_grid), count_boys)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    p_grid[findmax(posterior)[2]]

    f = Figure()
    ax = Axis(f[1, 1], xlabel = "Proportion Boys (p)", ylabel = "Posterior Density")

    # Generate random color
    random_color = RGB(rand(), rand(), rand())

    lines!(ax, p_grid, posterior, color = random_color)
    vlines!(ax, p_grid[findmax(posterior)[2]]; ymin = 0.0, ymax = 1, color = :red)
    f

end

# ╔═╡ e9fe6f18-e801-4134-bd61-55b75fcf0587
md"""
##### 3H2
"""

# ╔═╡ 4e59422c-7c02-441a-8e1f-d3ef2e4c1b14
let
    size = 1000

    count_boys = sum(birth1) + sum(birth2)
    count_total = length(birth1) + length(birth2)

    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(count_total, p_grid), count_boys)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    sample = sample(p_grid, Weights(posterior), 10_000; replace = true)

    HPDI_50 = hpdi(sample, alpha = 0.50)
    HPDI_89 = hpdi(sample, alpha = 0.11)
    HPDI_97 = hpdi(sample, alpha = 0.03)

    HPDI_50, HPDI_89, HPDI_97
end

# ╔═╡ cb3225f7-465a-4d27-a06e-c9ef2436094d
md"""
##### 3H3
"""

# ╔═╡ 689fcccc-91a5-429b-b08b-d871e34e4b91
let
    size = 1000

    count_boys = sum(birth1) + sum(birth2)
    count_total = length(birth1) + length(birth2)

    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(count_total, p_grid), count_boys)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    sample = sample(p_grid, Weights(posterior), 10_000; replace = true)

    w = @. rand(Binomial(200, sample))

    f = Figure()
    Axis(f[1, 1], xlabel = "Number of boys", ylabel = "Density")

    density!(
        w,
        color = (:skyblue, 0.3),
        strokecolor = :lightblue,
        strokewidth = 3,
        strokearound = false,
    )
    vlines!(count_boys; ymin = 0.0, ymax = 1, color = :tomato)

    f

end

# ╔═╡ 28c426fc-303a-431d-98ba-3e013c6a1d2a
md"""
##### 3H4
"""

# ╔═╡ d78eee41-232a-4b6b-a432-0c12e07b134e
let
    size = 1000

    count_boys = sum(birth1)
    count_total = length(birth1)

    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(count_total, p_grid), count_boys)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    sample = sample(p_grid, Weights(posterior), 10_000; replace = true)

    w = @. rand(Binomial(100, sample))

    f = Figure()
    Axis(f[1, 1], xlabel = "Number of boys", ylabel = "Density")

    density!(
        w,
        color = (:skyblue, 0.3),
        strokecolor = :lightblue,
        strokewidth = 3,
        strokearound = false,
    )
    vlines!(count_boys; ymin = 0.0, ymax = 1, color = :tomato)

    f

end

# ╔═╡ 09c665b6-6548-4c57-b130-cc6dc4a7ad22
md"""
##### 3H5
"""

# ╔═╡ 8d30960d-2c95-4331-a523-c1936e6e50c7
let
    size = 1000

    count_boys = sum(birth1) + sum(birth2)
    count_total = length(birth1) + length(birth2)

    girls_first = sum(birth1 .== 0)
    girl_then_boy = sum(birth2[birth1.==0])

    # define grid
    p_grid = range(0, 1, size)

    # define prior
    prior = ones(size)

    # compute probability of data at each value in grid
    likelihood = pdf.(Binomial.(count_total, p_grid), count_boys)

    #compute product of prob_data and prior
    posterior = likelihood .* prior

    #standardize the posterior so it sums to 1
    posterior = posterior / sum(posterior)

    sample = sample(p_grid, Weights(posterior), 10_000; replace = true)

    w = @. rand(Binomial(girls_first, sample))

    f = Figure()
    Axis(f[1, 1], xlabel = "Number of boys", ylabel = "Density")

    density!(
        w,
        color = (:skyblue, 0.3),
        strokecolor = :lightblue,
        strokewidth = 3,
        strokearound = false,
    )
    vlines!(girl_then_boy; ymin = 0.0, ymax = 1, color = :tomato)

    f

end

# ╔═╡ 00000000-0000-0000-0000-000000000001
PLUTO_PROJECT_TOML_CONTENTS = """
[deps]
CairoMakie = "13f3f980-e62b-5c42-98c6-ff1f3baf88f0"
Colors = "5ae59095-9a9b-59fe-a467-6f913c188581"
Distributions = "31c24e10-a181-5473-b8eb-7969acd0382f"
KernelDensity = "5ab0869b-81aa-558d-bb23-cbf5423bbe9b"
PlutoUI = "7f904dfe-b85e-4ff6-b463-dae2292396a8"
Random = "9a3f8284-a2c9-5f02-9a11-845980a1fd5c"
StatisticalRethinking = "2d09df54-9d0f-5258-8220-54c2a3d4fbee"
StatsBase = "2913bbd2-ae8a-5f71-8c99-4fb6c76f3a91"

[compat]
CairoMakie = "~0.10.11"
Colors = "~0.12.10"
Distributions = "~0.25.102"
KernelDensity = "~0.6.7"
PlutoUI = "~0.7.52"
StatisticalRethinking = "~4.7.3"
StatsBase = "~0.34.2"
"""

# ╔═╡ 00000000-0000-0000-0000-000000000002
PLUTO_MANIFEST_TOML_CONTENTS = """
# This file is machine-generated - editing it directly is not advised

julia_version = "1.9.3"
manifest_format = "2.0"
project_hash = "11756ed563a38fd26e13abe8eaeb4ca2d4e0894a"

[[deps.ANSIColoredPrinters]]
git-tree-sha1 = "574baf8110975760d391c710b6341da1afa48d8c"
uuid = "a4c015fc-c6ff-483c-b24f-f7ea428134e9"
version = "0.0.1"

[[deps.AbstractFFTs]]
deps = ["LinearAlgebra"]
git-tree-sha1 = "d92ad398961a3ed262d8bf04a1a2b8340f915fef"
uuid = "621f4979-c628-5d54-868e-fcf4e3e8185c"
version = "1.5.0"
weakdeps = ["ChainRulesCore", "Test"]

    [deps.AbstractFFTs.extensions]
    AbstractFFTsChainRulesCoreExt = "ChainRulesCore"
    AbstractFFTsTestExt = "Test"

[[deps.AbstractLattices]]
git-tree-sha1 = "f35684b7349da49fcc8a9e520e30e45dbb077166"
uuid = "398f06c4-4d28-53ec-89ca-5b2656b7603d"
version = "0.2.1"

[[deps.AbstractMCMC]]
deps = ["BangBang", "ConsoleProgressMonitor", "Distributed", "LogDensityProblems", "Logging", "LoggingExtras", "ProgressLogging", "Random", "StatsBase", "TerminalLoggers", "Transducers"]
git-tree-sha1 = "87e63dcb990029346b091b170252f3c416568afc"
uuid = "80f14c24-f653-4e6a-9b94-39d6b0f70001"
version = "4.4.2"

[[deps.AbstractPlutoDingetjes]]
deps = ["Pkg"]
git-tree-sha1 = "91bd53c39b9cbfb5ef4b015e8b582d344532bd0a"
uuid = "6e696c72-6542-2067-7265-42206c756150"
version = "1.2.0"

[[deps.AbstractTrees]]
git-tree-sha1 = "faa260e4cb5aba097a73fab382dd4b5819d8ec8c"
uuid = "1520ce14-60c1-5f80-bbc7-55ef81b5835c"
version = "0.4.4"

[[deps.Adapt]]
deps = ["LinearAlgebra", "Requires"]
git-tree-sha1 = "76289dc51920fdc6e0013c872ba9551d54961c24"
uuid = "79e6a3ab-5dfb-504d-930d-738a2a938a0e"
version = "3.6.2"
weakdeps = ["StaticArrays"]

    [deps.Adapt.extensions]
    AdaptStaticArraysExt = "StaticArrays"

[[deps.Animations]]
deps = ["Colors"]
git-tree-sha1 = "e81c509d2c8e49592413bfb0bb3b08150056c79d"
uuid = "27a7e980-b3e6-11e9-2bcd-0b925532e340"
version = "0.4.1"

[[deps.ArgCheck]]
git-tree-sha1 = "a3a402a35a2f7e0b87828ccabbd5ebfbebe356b4"
uuid = "dce04be8-c92d-5529-be00-80e4d2c0e197"
version = "2.3.0"

[[deps.ArgTools]]
uuid = "0dad84c5-d112-42e6-8d28-ef12dabb789f"
version = "1.1.1"

[[deps.ArrayInterface]]
deps = ["Adapt", "LinearAlgebra", "Requires", "SparseArrays", "SuiteSparse"]
git-tree-sha1 = "f83ec24f76d4c8f525099b2ac475fc098138ec31"
uuid = "4fba245c-0d91-5ea0-9b3e-6abc04ee57a9"
version = "7.4.11"

    [deps.ArrayInterface.extensions]
    ArrayInterfaceBandedMatricesExt = "BandedMatrices"
    ArrayInterfaceBlockBandedMatricesExt = "BlockBandedMatrices"
    ArrayInterfaceCUDAExt = "CUDA"
    ArrayInterfaceGPUArraysCoreExt = "GPUArraysCore"
    ArrayInterfaceStaticArraysCoreExt = "StaticArraysCore"
    ArrayInterfaceTrackerExt = "Tracker"

    [deps.ArrayInterface.weakdeps]
    BandedMatrices = "aae01518-5342-5314-be14-df237901396f"
    BlockBandedMatrices = "ffab5731-97b5-5995-9138-79e8c1846df0"
    CUDA = "052768ef-5323-5732-b1bb-66c8b64840ba"
    GPUArraysCore = "46192b85-c4d5-4398-a991-12ede77f4527"
    StaticArraysCore = "1e83bf80-4336-4d27-bf5d-d5a4f845583c"
    Tracker = "9f7883ad-71c0-57eb-9f7f-b5c9e6d3789c"

[[deps.Artifacts]]
uuid = "56f22d72-fd6d-98f1-02f0-08ddc0907c33"

[[deps.Automa]]
deps = ["TranscodingStreams"]
git-tree-sha1 = "ef9997b3d5547c48b41c7bd8899e812a917b409d"
uuid = "67c07d97-cdcb-5c2c-af73-a7f9c32a568b"
version = "0.8.4"

[[deps.AxisAlgorithms]]
deps = ["LinearAlgebra", "Random", "SparseArrays", "WoodburyMatrices"]
git-tree-sha1 = "66771c8d21c8ff5e3a93379480a2307ac36863f7"
uuid = "13072b0f-2c55-5437-9ae7-d433b7a33950"
version = "1.0.1"

[[deps.AxisArrays]]
deps = ["Dates", "IntervalSets", "IterTools", "RangeArrays"]
git-tree-sha1 = "16351be62963a67ac4083f748fdb3cca58bfd52f"
uuid = "39de3d68-74b9-583c-8d2d-e117c070f3a9"
version = "0.4.7"

[[deps.BangBang]]
deps = ["Compat", "ConstructionBase", "InitialValues", "LinearAlgebra", "Requires", "Setfield", "Tables"]
git-tree-sha1 = "e28912ce94077686443433c2800104b061a827ed"
uuid = "198e06fe-97b7-11e9-32a5-e1d131e6ad66"
version = "0.3.39"

    [deps.BangBang.extensions]
    BangBangChainRulesCoreExt = "ChainRulesCore"
    BangBangDataFramesExt = "DataFrames"
    BangBangStaticArraysExt = "StaticArrays"
    BangBangStructArraysExt = "StructArrays"
    BangBangTypedTablesExt = "TypedTables"

    [deps.BangBang.weakdeps]
    ChainRulesCore = "d360d2e6-b24c-11e9-a2a3-2a2ae2dbcce4"
    DataFrames = "a93c6f00-e57d-5684-b7b6-d8193f3e46c0"
    StaticArrays = "90137ffa-7385-5640-81b9-e52037218182"
    StructArrays = "09ab397b-f2b6-538f-b94a-2f83cf4a842a"
    TypedTables = "9d95f2ec-7b3d-5a63-8d20-e2491e220bb9"

[[deps.Base64]]
uuid = "2a0f44e3-6c83-55bd-87e4-b1978d98bd5f"

[[deps.Baselet]]
git-tree-sha1 = "aebf55e6d7795e02ca500a689d326ac979aaf89e"
uuid = "9718e550-a3fa-408a-8086-8db961cd8217"
version = "0.1.1"

[[deps.BitTwiddlingConvenienceFunctions]]
deps = ["Static"]
git-tree-sha1 = "0c5f81f47bbbcf4aea7b2959135713459170798b"
uuid = "62783981-4cbd-42fc-bca8-16325de8dc4b"
version = "0.1.5"

[[deps.Bzip2_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "19a35467a82e236ff51bc17a3a44b69ef35185a2"
uuid = "6e34b625-4abd-537c-b88f-471c36dfa7a0"
version = "1.0.8+0"

[[deps.CEnum]]
git-tree-sha1 = "eb4cb44a499229b3b8426dcfb5dd85333951ff90"
uuid = "fa961155-64e5-5f13-b03f-caf6b980ea82"
version = "0.4.2"

[[deps.CPUSummary]]
deps = ["CpuId", "IfElse", "PrecompileTools", "Static"]
git-tree-sha1 = "601f7e7b3d36f18790e2caf83a882d88e9b71ff1"
uuid = "2a0fbf3d-bb9c-48f3-b0a9-814d99fd7ab9"
version = "0.2.4"

[[deps.CRC32c]]
uuid = "8bf52ea8-c179-5cab-976a-9e18b702a9bc"

[[deps.CRlibm]]
deps = ["CRlibm_jll"]
git-tree-sha1 = "32abd86e3c2025db5172aa182b982debed519834"
uuid = "96374032-68de-5a5b-8d9e-752f78720389"
version = "1.0.1"

[[deps.CRlibm_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "e329286945d0cfc04456972ea732551869af1cfc"
uuid = "4e9b3aee-d8a1-5a3d-ad8b-7d824db253f0"
version = "1.0.1+0"

[[deps.CSV]]
deps = ["CodecZlib", "Dates", "FilePathsBase", "InlineStrings", "Mmap", "Parsers", "PooledArrays", "PrecompileTools", "SentinelArrays", "Tables", "Unicode", "WeakRefStrings", "WorkerUtilities"]
git-tree-sha1 = "44dbf560808d49041989b8a96cae4cffbeb7966a"
uuid = "336ed68f-0bac-5ca0-87d4-7b16caf5d00b"
version = "0.10.11"

[[deps.Cairo]]
deps = ["Cairo_jll", "Colors", "Glib_jll", "Graphics", "Libdl", "Pango_jll"]
git-tree-sha1 = "d0b3f8b4ad16cb0a2988c6788646a5e6a17b6b1b"
uuid = "159f3aea-2a34-519c-b102-8c37f9878175"
version = "1.0.5"

[[deps.CairoMakie]]
deps = ["Base64", "Cairo", "Colors", "FFTW", "FileIO", "FreeType", "GeometryBasics", "LinearAlgebra", "Makie", "PrecompileTools", "SHA"]
git-tree-sha1 = "74384dc4aba2b377e22703e849154252930c434d"
uuid = "13f3f980-e62b-5c42-98c6-ff1f3baf88f0"
version = "0.10.11"

[[deps.Cairo_jll]]
deps = ["Artifacts", "Bzip2_jll", "CompilerSupportLibraries_jll", "Fontconfig_jll", "FreeType2_jll", "Glib_jll", "JLLWrappers", "LZO_jll", "Libdl", "Pixman_jll", "Pkg", "Xorg_libXext_jll", "Xorg_libXrender_jll", "Zlib_jll", "libpng_jll"]
git-tree-sha1 = "4b859a208b2397a7a623a03449e4636bdb17bcf2"
uuid = "83423d85-b0ee-5818-9007-b63ccbeb887a"
version = "1.16.1+1"

[[deps.Calculus]]
deps = ["LinearAlgebra"]
git-tree-sha1 = "f641eb0a4f00c343bbc32346e1217b86f3ce9dad"
uuid = "49dc2e85-a5d0-5ad3-a950-438e2897f1b9"
version = "0.5.1"

[[deps.ChainRulesCore]]
deps = ["Compat", "LinearAlgebra", "SparseArrays"]
git-tree-sha1 = "e30f2f4e20f7f186dc36529910beaedc60cfa644"
uuid = "d360d2e6-b24c-11e9-a2a3-2a2ae2dbcce4"
version = "1.16.0"

[[deps.CodecZlib]]
deps = ["TranscodingStreams", "Zlib_jll"]
git-tree-sha1 = "02aa26a4cf76381be7f66e020a3eddeb27b0a092"
uuid = "944b1d66-785c-5afd-91f1-9de20f533193"
version = "0.7.2"

[[deps.ColorBrewer]]
deps = ["Colors", "JSON", "Test"]
git-tree-sha1 = "61c5334f33d91e570e1d0c3eb5465835242582c4"
uuid = "a2cac450-b92f-5266-8821-25eda20663c8"
version = "0.4.0"

[[deps.ColorSchemes]]
deps = ["ColorTypes", "ColorVectorSpace", "Colors", "FixedPointNumbers", "PrecompileTools", "Random"]
git-tree-sha1 = "67c1f244b991cad9b0aa4b7540fb758c2488b129"
uuid = "35d6a980-a343-548e-a6ea-1d62b119f2f4"
version = "3.24.0"

[[deps.ColorTypes]]
deps = ["FixedPointNumbers", "Random"]
git-tree-sha1 = "eb7f0f8307f71fac7c606984ea5fb2817275d6e4"
uuid = "3da002f7-5984-5a60-b8a6-cbb66c0b333f"
version = "0.11.4"

[[deps.ColorVectorSpace]]
deps = ["ColorTypes", "FixedPointNumbers", "LinearAlgebra", "SpecialFunctions", "Statistics", "TensorCore"]
git-tree-sha1 = "600cc5508d66b78aae350f7accdb58763ac18589"
uuid = "c3611d14-8923-5661-9e6a-0046d554d3a4"
version = "0.9.10"

[[deps.Colors]]
deps = ["ColorTypes", "FixedPointNumbers", "Reexport"]
git-tree-sha1 = "fc08e5930ee9a4e03f84bfb5211cb54e7769758a"
uuid = "5ae59095-9a9b-59fe-a467-6f913c188581"
version = "0.12.10"

[[deps.Combinatorics]]
git-tree-sha1 = "08c8b6831dc00bfea825826be0bc8336fc369860"
uuid = "861a8166-3701-5b0c-9a16-15d98fcdc6aa"
version = "1.0.2"

[[deps.CommonSubexpressions]]
deps = ["MacroTools", "Test"]
git-tree-sha1 = "7b8a93dba8af7e3b42fecabf646260105ac373f7"
uuid = "bbf7d656-a473-5ed7-a52c-81e309532950"
version = "0.3.0"

[[deps.Compat]]
deps = ["UUIDs"]
git-tree-sha1 = "8a62af3e248a8c4bad6b32cbbe663ae02275e32c"
uuid = "34da2185-b29b-5c13-b0c7-acf172513d20"
version = "4.10.0"
weakdeps = ["Dates", "LinearAlgebra"]

    [deps.Compat.extensions]
    CompatLinearAlgebraExt = "LinearAlgebra"

[[deps.CompilerSupportLibraries_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "e66e0078-7015-5450-92f7-15fbd957f2ae"
version = "1.0.5+0"

[[deps.CompositionsBase]]
git-tree-sha1 = "802bb88cd69dfd1509f6670416bd4434015693ad"
uuid = "a33af91c-f02d-484b-be07-31d278c5ca2b"
version = "0.1.2"

    [deps.CompositionsBase.extensions]
    CompositionsBaseInverseFunctionsExt = "InverseFunctions"

    [deps.CompositionsBase.weakdeps]
    InverseFunctions = "3587e190-3f89-42d0-90ee-14403ec27112"

[[deps.ConsoleProgressMonitor]]
deps = ["Logging", "ProgressMeter"]
git-tree-sha1 = "3ab7b2136722890b9af903859afcf457fa3059e8"
uuid = "88cd18e8-d9cc-4ea6-8889-5259c0d15c8b"
version = "0.1.2"

[[deps.ConstructionBase]]
deps = ["LinearAlgebra"]
git-tree-sha1 = "c53fc348ca4d40d7b371e71fd52251839080cbc9"
uuid = "187b0558-2788-49d3-abe0-74a17ed4e7c9"
version = "1.5.4"
weakdeps = ["IntervalSets", "StaticArrays"]

    [deps.ConstructionBase.extensions]
    ConstructionBaseIntervalSetsExt = "IntervalSets"
    ConstructionBaseStaticArraysExt = "StaticArrays"

[[deps.Contour]]
git-tree-sha1 = "d05d9e7b7aedff4e5b51a029dced05cfb6125781"
uuid = "d38c429a-6771-53c6-b99e-75d170b6e991"
version = "0.6.2"

[[deps.CpuId]]
deps = ["Markdown"]
git-tree-sha1 = "fcbb72b032692610bfbdb15018ac16a36cf2e406"
uuid = "adafc99b-e345-5852-983c-f28acb93d879"
version = "0.3.1"

[[deps.Crayons]]
git-tree-sha1 = "249fe38abf76d48563e2f4556bebd215aa317e15"
uuid = "a8cc5b0e-0ffa-5ad4-8c14-923d3ee1735f"
version = "4.1.1"

[[deps.DataAPI]]
git-tree-sha1 = "8da84edb865b0b5b0100c0666a9bc9a0b71c553c"
uuid = "9a962f9c-6df0-11e9-0e5d-c546b8b5ee8a"
version = "1.15.0"

[[deps.DataFrames]]
deps = ["Compat", "DataAPI", "DataStructures", "Future", "InlineStrings", "InvertedIndices", "IteratorInterfaceExtensions", "LinearAlgebra", "Markdown", "Missings", "PooledArrays", "PrecompileTools", "PrettyTables", "Printf", "REPL", "Random", "Reexport", "SentinelArrays", "SortingAlgorithms", "Statistics", "TableTraits", "Tables", "Unicode"]
git-tree-sha1 = "04c738083f29f86e62c8afc341f0967d8717bdb8"
uuid = "a93c6f00-e57d-5684-b7b6-d8193f3e46c0"
version = "1.6.1"

[[deps.DataStructures]]
deps = ["Compat", "InteractiveUtils", "OrderedCollections"]
git-tree-sha1 = "3dbd312d370723b6bb43ba9d02fc36abade4518d"
uuid = "864edb3b-99cc-5e75-8d2d-829cb0a9cfe8"
version = "0.18.15"

[[deps.DataValueInterfaces]]
git-tree-sha1 = "bfc1187b79289637fa0ef6d4436ebdfe6905cbd6"
uuid = "e2d170a0-9d28-54be-80f0-106bbe20a464"
version = "1.0.0"

[[deps.Dates]]
deps = ["Printf"]
uuid = "ade2ca70-3891-5945-98fb-dc099432e06a"

[[deps.DefineSingletons]]
git-tree-sha1 = "0fba8b706d0178b4dc7fd44a96a92382c9065c2c"
uuid = "244e2a9f-e319-4986-a169-4d1fe445cd52"
version = "0.1.2"

[[deps.DelaunayTriangulation]]
deps = ["DataStructures", "EnumX", "ExactPredicates", "Random", "SimpleGraphs"]
git-tree-sha1 = "bea7984f7e09aeb28a3b071c420a0186cb4fabad"
uuid = "927a84f5-c5f4-47a5-9785-b46e178433df"
version = "0.8.8"

[[deps.DelimitedFiles]]
deps = ["Mmap"]
git-tree-sha1 = "9e2f36d3c96a820c678f2f1f1782582fcf685bae"
uuid = "8bb1440f-4735-579b-a4ab-409b98df4dab"
version = "1.9.1"

[[deps.DiffResults]]
deps = ["StaticArraysCore"]
git-tree-sha1 = "782dd5f4561f5d267313f23853baaaa4c52ea621"
uuid = "163ba53b-c6d8-5494-b064-1a9d43ac40c5"
version = "1.1.0"

[[deps.DiffRules]]
deps = ["IrrationalConstants", "LogExpFunctions", "NaNMath", "Random", "SpecialFunctions"]
git-tree-sha1 = "23163d55f885173722d1e4cf0f6110cdbaf7e272"
uuid = "b552c78f-8df3-52c6-915a-8e097449b14b"
version = "1.15.1"

[[deps.Distributed]]
deps = ["Random", "Serialization", "Sockets"]
uuid = "8ba89e20-285c-5b6f-9357-94700520ee1b"

[[deps.Distributions]]
deps = ["FillArrays", "LinearAlgebra", "PDMats", "Printf", "QuadGK", "Random", "SpecialFunctions", "Statistics", "StatsAPI", "StatsBase", "StatsFuns", "Test"]
git-tree-sha1 = "3d5873f811f582873bb9871fc9c451784d5dc8c7"
uuid = "31c24e10-a181-5473-b8eb-7969acd0382f"
version = "0.25.102"

    [deps.Distributions.extensions]
    DistributionsChainRulesCoreExt = "ChainRulesCore"
    DistributionsDensityInterfaceExt = "DensityInterface"

    [deps.Distributions.weakdeps]
    ChainRulesCore = "d360d2e6-b24c-11e9-a2a3-2a2ae2dbcce4"
    DensityInterface = "b429d917-457f-4dbc-8f4c-0cc954292b1d"

[[deps.DocStringExtensions]]
deps = ["LibGit2"]
git-tree-sha1 = "2fb1e02f2b635d0845df5d7c167fec4dd739b00d"
uuid = "ffbed154-4ef7-542d-bbb7-c09d3a79fcae"
version = "0.9.3"

[[deps.Documenter]]
deps = ["ANSIColoredPrinters", "AbstractTrees", "Base64", "Dates", "DocStringExtensions", "Downloads", "IOCapture", "InteractiveUtils", "JSON", "LibGit2", "Logging", "Markdown", "MarkdownAST", "Pkg", "PrecompileTools", "REPL", "RegistryInstances", "SHA", "Test", "Unicode"]
git-tree-sha1 = "f667b805e90d643aeb1ca70189827f991a7cc115"
uuid = "e30172f5-a6a5-5a46-863b-614d45cd2de4"
version = "1.1.0"

[[deps.Downloads]]
deps = ["ArgTools", "FileWatching", "LibCURL", "NetworkOptions"]
uuid = "f43a241f-c20a-4ad4-852c-f6b1247861c6"
version = "1.6.0"

[[deps.DualNumbers]]
deps = ["Calculus", "NaNMath", "SpecialFunctions"]
git-tree-sha1 = "5837a837389fccf076445fce071c8ddaea35a566"
uuid = "fa6b7ba4-c1ee-5f82-b5fc-ecf0adba8f74"
version = "0.6.8"

[[deps.EarCut_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "e3290f2d49e661fbd94046d7e3726ffcb2d41053"
uuid = "5ae413db-bbd1-5e63-b57d-d24a61df00f5"
version = "2.2.4+0"

[[deps.EnumX]]
git-tree-sha1 = "bdb1942cd4c45e3c678fd11569d5cccd80976237"
uuid = "4e289a0a-7415-4d19-859d-a7e5c4648b56"
version = "1.0.4"

[[deps.ErrorfreeArithmetic]]
git-tree-sha1 = "d6863c556f1142a061532e79f611aa46be201686"
uuid = "90fa49ef-747e-5e6f-a989-263ba693cf1a"
version = "0.5.2"

[[deps.ExactPredicates]]
deps = ["IntervalArithmetic", "Random", "StaticArraysCore", "Test"]
git-tree-sha1 = "276e83bc8b21589b79303b9985c321024ffdf59c"
uuid = "429591f6-91af-11e9-00e2-59fbe8cec110"
version = "2.2.5"

[[deps.Expat_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "4558ab818dcceaab612d1bb8c19cee87eda2b83c"
uuid = "2e619515-83b5-522b-bb60-26c02a35a201"
version = "2.5.0+0"

[[deps.Extents]]
git-tree-sha1 = "5e1e4c53fa39afe63a7d356e30452249365fba99"
uuid = "411431e0-e8b7-467b-b5e0-f676ba4f2910"
version = "0.1.1"

[[deps.FFMPEG_jll]]
deps = ["Artifacts", "Bzip2_jll", "FreeType2_jll", "FriBidi_jll", "JLLWrappers", "LAME_jll", "Libdl", "Ogg_jll", "OpenSSL_jll", "Opus_jll", "PCRE2_jll", "Zlib_jll", "libaom_jll", "libass_jll", "libfdk_aac_jll", "libvorbis_jll", "x264_jll", "x265_jll"]
git-tree-sha1 = "466d45dc38e15794ec7d5d63ec03d776a9aff36e"
uuid = "b22a6f82-2f65-5046-a5b2-351ab43fb4e5"
version = "4.4.4+1"

[[deps.FFTW]]
deps = ["AbstractFFTs", "FFTW_jll", "LinearAlgebra", "MKL_jll", "Preferences", "Reexport"]
git-tree-sha1 = "b4fbdd20c889804969571cc589900803edda16b7"
uuid = "7a1cc6ca-52ef-59f5-83cd-3a7055c09341"
version = "1.7.1"

[[deps.FFTW_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "c6033cc3892d0ef5bb9cd29b7f2f0331ea5184ea"
uuid = "f5851436-0d7a-5f13-b9de-f02708fd171a"
version = "3.3.10+0"

[[deps.FastRounding]]
deps = ["ErrorfreeArithmetic", "LinearAlgebra"]
git-tree-sha1 = "6344aa18f654196be82e62816935225b3b9abe44"
uuid = "fa42c844-2597-5d31-933b-ebd51ab2693f"
version = "0.3.1"

[[deps.FileIO]]
deps = ["Pkg", "Requires", "UUIDs"]
git-tree-sha1 = "299dc33549f68299137e51e6d49a13b5b1da9673"
uuid = "5789e2e9-d7fb-5bc7-8068-2c6fae9b9549"
version = "1.16.1"

[[deps.FilePathsBase]]
deps = ["Compat", "Dates", "Mmap", "Printf", "Test", "UUIDs"]
git-tree-sha1 = "9f00e42f8d99fdde64d40c8ea5d14269a2e2c1aa"
uuid = "48062228-2e41-5def-b9a4-89aafe57970f"
version = "0.9.21"

[[deps.FileWatching]]
uuid = "7b1f6079-737a-58dc-b8bc-7a2ca5c1b5ee"

[[deps.FillArrays]]
deps = ["LinearAlgebra", "Random"]
git-tree-sha1 = "a20eaa3ad64254c61eeb5f230d9306e937405434"
uuid = "1a297f60-69ca-5386-bcde-b61e274b549b"
version = "1.6.1"
weakdeps = ["SparseArrays", "Statistics"]

    [deps.FillArrays.extensions]
    FillArraysSparseArraysExt = "SparseArrays"
    FillArraysStatisticsExt = "Statistics"

[[deps.FiniteDiff]]
deps = ["ArrayInterface", "LinearAlgebra", "Requires", "Setfield", "SparseArrays"]
git-tree-sha1 = "c6e4a1fbe73b31a3dea94b1da449503b8830c306"
uuid = "6a86dc24-6348-571c-b903-95158fe2bd41"
version = "2.21.1"

    [deps.FiniteDiff.extensions]
    FiniteDiffBandedMatricesExt = "BandedMatrices"
    FiniteDiffBlockBandedMatricesExt = "BlockBandedMatrices"
    FiniteDiffStaticArraysExt = "StaticArrays"

    [deps.FiniteDiff.weakdeps]
    BandedMatrices = "aae01518-5342-5314-be14-df237901396f"
    BlockBandedMatrices = "ffab5731-97b5-5995-9138-79e8c1846df0"
    StaticArrays = "90137ffa-7385-5640-81b9-e52037218182"

[[deps.FixedPointNumbers]]
deps = ["Statistics"]
git-tree-sha1 = "335bfdceacc84c5cdf16aadc768aa5ddfc5383cc"
uuid = "53c48c17-4a7d-5ca2-90c5-79b7896eea93"
version = "0.8.4"

[[deps.Fontconfig_jll]]
deps = ["Artifacts", "Bzip2_jll", "Expat_jll", "FreeType2_jll", "JLLWrappers", "Libdl", "Libuuid_jll", "Pkg", "Zlib_jll"]
git-tree-sha1 = "21efd19106a55620a188615da6d3d06cd7f6ee03"
uuid = "a3f928ae-7b40-5064-980b-68af3947d34b"
version = "2.13.93+0"

[[deps.Formatting]]
deps = ["Printf"]
git-tree-sha1 = "8339d61043228fdd3eb658d86c926cb282ae72a8"
uuid = "59287772-0a20-5a39-b81b-1366585eb4c0"
version = "0.4.2"

[[deps.ForwardDiff]]
deps = ["CommonSubexpressions", "DiffResults", "DiffRules", "LinearAlgebra", "LogExpFunctions", "NaNMath", "Preferences", "Printf", "Random", "SpecialFunctions"]
git-tree-sha1 = "cf0fe81336da9fb90944683b8c41984b08793dad"
uuid = "f6369f11-7733-5829-9624-2563aa707210"
version = "0.10.36"
weakdeps = ["StaticArrays"]

    [deps.ForwardDiff.extensions]
    ForwardDiffStaticArraysExt = "StaticArrays"

[[deps.FreeType]]
deps = ["CEnum", "FreeType2_jll"]
git-tree-sha1 = "50351f83f95282cf903e968d7c6e8d44a5f83d0b"
uuid = "b38be410-82b0-50bf-ab77-7b57e271db43"
version = "4.1.0"

[[deps.FreeType2_jll]]
deps = ["Artifacts", "Bzip2_jll", "JLLWrappers", "Libdl", "Zlib_jll"]
git-tree-sha1 = "d8db6a5a2fe1381c1ea4ef2cab7c69c2de7f9ea0"
uuid = "d7e528f0-a631-5988-bf34-fe36492bcfd7"
version = "2.13.1+0"

[[deps.FreeTypeAbstraction]]
deps = ["ColorVectorSpace", "Colors", "FreeType", "GeometryBasics"]
git-tree-sha1 = "38a92e40157100e796690421e34a11c107205c86"
uuid = "663a7486-cb36-511b-a19d-713bb74d65c9"
version = "0.10.0"

[[deps.FriBidi_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "aa31987c2ba8704e23c6c8ba8a4f769d5d7e4f91"
uuid = "559328eb-81f9-559d-9380-de523a88c83c"
version = "1.0.10+0"

[[deps.Future]]
deps = ["Random"]
uuid = "9fa8497b-333b-5362-9e8d-4d0656e87820"

[[deps.GPUArraysCore]]
deps = ["Adapt"]
git-tree-sha1 = "2d6ca471a6c7b536127afccfa7564b5b39227fe0"
uuid = "46192b85-c4d5-4398-a991-12ede77f4527"
version = "0.1.5"

[[deps.GenericSchur]]
deps = ["LinearAlgebra", "Printf"]
git-tree-sha1 = "fb69b2a645fa69ba5f474af09221b9308b160ce6"
uuid = "c145ed77-6b09-5dd9-b285-bf645a82121e"
version = "0.5.3"

[[deps.GeoInterface]]
deps = ["Extents"]
git-tree-sha1 = "d53480c0793b13341c40199190f92c611aa2e93c"
uuid = "cf35fbd7-0cd7-5166-be24-54bfbe79505f"
version = "1.3.2"

[[deps.GeometryBasics]]
deps = ["EarCut_jll", "Extents", "GeoInterface", "IterTools", "LinearAlgebra", "StaticArrays", "StructArrays", "Tables"]
git-tree-sha1 = "424a5a6ce7c5d97cca7bcc4eac551b97294c54af"
uuid = "5c1252a2-5f33-56bf-86c9-59e7332b4326"
version = "0.4.9"

[[deps.Gettext_jll]]
deps = ["Artifacts", "CompilerSupportLibraries_jll", "JLLWrappers", "Libdl", "Libiconv_jll", "Pkg", "XML2_jll"]
git-tree-sha1 = "9b02998aba7bf074d14de89f9d37ca24a1a0b046"
uuid = "78b55507-aeef-58d4-861c-77aaff3498b1"
version = "0.21.0+0"

[[deps.Glib_jll]]
deps = ["Artifacts", "Gettext_jll", "JLLWrappers", "Libdl", "Libffi_jll", "Libiconv_jll", "Libmount_jll", "PCRE2_jll", "Zlib_jll"]
git-tree-sha1 = "e94c92c7bf4819685eb80186d51c43e71d4afa17"
uuid = "7746bdde-850d-59dc-9ae8-88ece973131d"
version = "2.76.5+0"

[[deps.Graphics]]
deps = ["Colors", "LinearAlgebra", "NaNMath"]
git-tree-sha1 = "d61890399bc535850c4bf08e4e0d3a7ad0f21cbd"
uuid = "a2bd30eb-e257-5431-a919-1863eab51364"
version = "1.1.2"

[[deps.Graphite2_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "344bf40dcab1073aca04aa0df4fb092f920e4011"
uuid = "3b182d85-2403-5c21-9c21-1e1f0cc25472"
version = "1.3.14+0"

[[deps.GridLayoutBase]]
deps = ["GeometryBasics", "InteractiveUtils", "Observables"]
git-tree-sha1 = "f57a64794b336d4990d90f80b147474b869b1bc4"
uuid = "3955a311-db13-416c-9275-1d80ed98e5e9"
version = "0.9.2"

[[deps.Grisu]]
git-tree-sha1 = "53bb909d1151e57e2484c3d1b53e19552b887fb2"
uuid = "42e2da0e-8278-4e71-bc24-59509adca0fe"
version = "1.0.2"

[[deps.HarfBuzz_jll]]
deps = ["Artifacts", "Cairo_jll", "Fontconfig_jll", "FreeType2_jll", "Glib_jll", "Graphite2_jll", "JLLWrappers", "Libdl", "Libffi_jll", "Pkg"]
git-tree-sha1 = "129acf094d168394e80ee1dc4bc06ec835e510a3"
uuid = "2e76f6c2-a576-52d4-95c1-20adfe4de566"
version = "2.8.1+1"

[[deps.HostCPUFeatures]]
deps = ["BitTwiddlingConvenienceFunctions", "IfElse", "Libdl", "Static"]
git-tree-sha1 = "eb8fed28f4994600e29beef49744639d985a04b2"
uuid = "3e5b6fbb-0976-4d2c-9146-d79de83f2fb0"
version = "0.1.16"

[[deps.HypergeometricFunctions]]
deps = ["DualNumbers", "LinearAlgebra", "OpenLibm_jll", "SpecialFunctions"]
git-tree-sha1 = "f218fe3736ddf977e0e772bc9a586b2383da2685"
uuid = "34004b35-14d8-5ef3-9330-4cdb6864b03a"
version = "0.3.23"

[[deps.Hyperscript]]
deps = ["Test"]
git-tree-sha1 = "8d511d5b81240fc8e6802386302675bdf47737b9"
uuid = "47d2ed2b-36de-50cf-bf87-49c2cf4b8b91"
version = "0.0.4"

[[deps.HypertextLiteral]]
deps = ["Tricks"]
git-tree-sha1 = "c47c5fa4c5308f27ccaac35504858d8914e102f9"
uuid = "ac1192a8-f4b3-4bfe-ba22-af5b92cd3ab2"
version = "0.9.4"

[[deps.IOCapture]]
deps = ["Logging", "Random"]
git-tree-sha1 = "d75853a0bdbfb1ac815478bacd89cd27b550ace6"
uuid = "b5f81e59-6552-4d32-b1f0-c071b021bf89"
version = "0.2.3"

[[deps.IfElse]]
git-tree-sha1 = "debdd00ffef04665ccbb3e150747a77560e8fad1"
uuid = "615f187c-cbe4-4ef1-ba3b-2fcf58d6d173"
version = "0.1.1"

[[deps.ImageAxes]]
deps = ["AxisArrays", "ImageBase", "ImageCore", "Reexport", "SimpleTraits"]
git-tree-sha1 = "2e4520d67b0cef90865b3ef727594d2a58e0e1f8"
uuid = "2803e5a7-5153-5ecf-9a86-9b4c37f5f5ac"
version = "0.6.11"

[[deps.ImageBase]]
deps = ["ImageCore", "Reexport"]
git-tree-sha1 = "b51bb8cae22c66d0f6357e3bcb6363145ef20835"
uuid = "c817782e-172a-44cc-b673-b171935fbb9e"
version = "0.1.5"

[[deps.ImageCore]]
deps = ["AbstractFFTs", "ColorVectorSpace", "Colors", "FixedPointNumbers", "Graphics", "MappedArrays", "MosaicViews", "OffsetArrays", "PaddedViews", "Reexport"]
git-tree-sha1 = "acf614720ef026d38400b3817614c45882d75500"
uuid = "a09fc81d-aa75-5fe9-8630-4744c3626534"
version = "0.9.4"

[[deps.ImageIO]]
deps = ["FileIO", "IndirectArrays", "JpegTurbo", "LazyModules", "Netpbm", "OpenEXR", "PNGFiles", "QOI", "Sixel", "TiffImages", "UUIDs"]
git-tree-sha1 = "bca20b2f5d00c4fbc192c3212da8fa79f4688009"
uuid = "82e4d734-157c-48bb-816b-45c225c6df19"
version = "0.6.7"

[[deps.ImageMetadata]]
deps = ["AxisArrays", "ImageAxes", "ImageBase", "ImageCore"]
git-tree-sha1 = "355e2b974f2e3212a75dfb60519de21361ad3cb7"
uuid = "bc367c6b-8a6b-528e-b4bd-a4b897500b49"
version = "0.9.9"

[[deps.Imath_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "3d09a9f60edf77f8a4d99f9e015e8fbf9989605d"
uuid = "905a6f67-0a94-5f89-b386-d35d92009cd1"
version = "3.1.7+0"

[[deps.IndirectArrays]]
git-tree-sha1 = "012e604e1c7458645cb8b436f8fba789a51b257f"
uuid = "9b13fd28-a010-5f03-acff-a1bbcff69959"
version = "1.0.0"

[[deps.Inflate]]
git-tree-sha1 = "ea8031dea4aff6bd41f1df8f2fdfb25b33626381"
uuid = "d25df0c9-e2be-5dd7-82c8-3ad0b3e990b9"
version = "0.1.4"

[[deps.InitialValues]]
git-tree-sha1 = "4da0f88e9a39111c2fa3add390ab15f3a44f3ca3"
uuid = "22cec73e-a1b8-11e9-2c92-598750a2cf9c"
version = "0.3.1"

[[deps.InlineStrings]]
deps = ["Parsers"]
git-tree-sha1 = "9cc2baf75c6d09f9da536ddf58eb2f29dedaf461"
uuid = "842dd82b-1e85-43dc-bf29-5d0ee9dffc48"
version = "1.4.0"

[[deps.IntegerMathUtils]]
git-tree-sha1 = "b8ffb903da9f7b8cf695a8bead8e01814aa24b30"
uuid = "18e54dd8-cb9d-406c-a71d-865a43cbb235"
version = "0.1.2"

[[deps.IntelOpenMP_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "ad37c091f7d7daf900963171600d7c1c5c3ede32"
uuid = "1d5cc7b8-4909-519e-a0f8-d0f5ad9712d0"
version = "2023.2.0+0"

[[deps.InteractiveUtils]]
deps = ["Markdown"]
uuid = "b77e0a4c-d291-57a0-90e8-8db25a27a240"

[[deps.Interpolations]]
deps = ["Adapt", "AxisAlgorithms", "ChainRulesCore", "LinearAlgebra", "OffsetArrays", "Random", "Ratios", "Requires", "SharedArrays", "SparseArrays", "StaticArrays", "WoodburyMatrices"]
git-tree-sha1 = "721ec2cf720536ad005cb38f50dbba7b02419a15"
uuid = "a98d9a8b-a2ab-59e6-89dd-64a1c18fca59"
version = "0.14.7"

[[deps.IntervalArithmetic]]
deps = ["CRlibm", "FastRounding", "LinearAlgebra", "Markdown", "Random", "RecipesBase", "RoundingEmulator", "SetRounding", "StaticArrays"]
git-tree-sha1 = "5ab7744289be503d76a944784bac3f2df7b809af"
uuid = "d1acc4aa-44c8-5952-acd4-ba5d80a2a253"
version = "0.20.9"

[[deps.IntervalSets]]
deps = ["Dates", "Random"]
git-tree-sha1 = "8e59ea773deee525c99a8018409f64f19fb719e6"
uuid = "8197267c-284f-5f27-9208-e0e47529a953"
version = "0.7.7"
weakdeps = ["Statistics"]

    [deps.IntervalSets.extensions]
    IntervalSetsStatisticsExt = "Statistics"

[[deps.InvertedIndices]]
git-tree-sha1 = "0dc7b50b8d436461be01300fd8cd45aa0274b038"
uuid = "41ab1584-1d38-5bbf-9106-f11c6c58b48f"
version = "1.3.0"

[[deps.IrrationalConstants]]
git-tree-sha1 = "630b497eafcc20001bba38a4651b327dcfc491d2"
uuid = "92d709cd-6900-40b7-9082-c6be49f344b6"
version = "0.2.2"

[[deps.Isoband]]
deps = ["isoband_jll"]
git-tree-sha1 = "f9b6d97355599074dc867318950adaa6f9946137"
uuid = "f1662d9f-8043-43de-a69a-05efc1cc6ff4"
version = "0.1.1"

[[deps.IterTools]]
git-tree-sha1 = "4ced6667f9974fc5c5943fa5e2ef1ca43ea9e450"
uuid = "c8e1da08-722c-5040-9ed9-7db0dc04731e"
version = "1.8.0"

[[deps.IteratorInterfaceExtensions]]
git-tree-sha1 = "a3f24677c21f5bbe9d2a714f95dcd58337fb2856"
uuid = "82899510-4779-5014-852e-03e436cf321d"
version = "1.0.0"

[[deps.JLLWrappers]]
deps = ["Artifacts", "Preferences"]
git-tree-sha1 = "7e5d6779a1e09a36db2a7b6cff50942a0a7d0fca"
uuid = "692b3bcd-3c85-4b1f-b108-f13ce0eb3210"
version = "1.5.0"

[[deps.JSON]]
deps = ["Dates", "Mmap", "Parsers", "Unicode"]
git-tree-sha1 = "31e996f0a15c7b280ba9f76636b3ff9e2ae58c9a"
uuid = "682c06a0-de6a-54ab-a142-c8b1cf79cde6"
version = "0.21.4"

[[deps.JpegTurbo]]
deps = ["CEnum", "FileIO", "ImageCore", "JpegTurbo_jll", "TOML"]
git-tree-sha1 = "d65930fa2bc96b07d7691c652d701dcbe7d9cf0b"
uuid = "b835a17e-a41a-41e7-81f0-2f016b05efe0"
version = "0.1.4"

[[deps.JpegTurbo_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "6f2675ef130a300a112286de91973805fcc5ffbc"
uuid = "aacddb02-875f-59d6-b918-886e6ef4fbf8"
version = "2.1.91+0"

[[deps.KernelDensity]]
deps = ["Distributions", "DocStringExtensions", "FFTW", "Interpolations", "StatsBase"]
git-tree-sha1 = "90442c50e202a5cdf21a7899c66b240fdef14035"
uuid = "5ab0869b-81aa-558d-bb23-cbf5423bbe9b"
version = "0.6.7"

[[deps.LAME_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "f6250b16881adf048549549fba48b1161acdac8c"
uuid = "c1c5ebd0-6772-5130-a774-d5fcae4a789d"
version = "3.100.1+0"

[[deps.LLVMOpenMP_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "f689897ccbe049adb19a065c495e75f372ecd42b"
uuid = "1d63c593-3942-5779-bab2-d838dc0a180e"
version = "15.0.4+0"

[[deps.LZO_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "e5b909bcf985c5e2605737d2ce278ed791b89be6"
uuid = "dd4b983a-f0e5-5f8d-a1b7-129d4a5fb1ac"
version = "2.10.1+0"

[[deps.LaTeXStrings]]
git-tree-sha1 = "f2355693d6778a178ade15952b7ac47a4ff97996"
uuid = "b964fa9f-0449-5b57-a5c2-d3ea65f4040f"
version = "1.3.0"

[[deps.LayoutPointers]]
deps = ["ArrayInterface", "LinearAlgebra", "ManualMemory", "SIMDTypes", "Static", "StaticArrayInterface"]
git-tree-sha1 = "88b8f66b604da079a627b6fb2860d3704a6729a1"
uuid = "10f19ff3-798f-405d-979b-55457f8fc047"
version = "0.1.14"

[[deps.LazilyInitializedFields]]
git-tree-sha1 = "410fe4739a4b092f2ffe36fcb0dcc3ab12648ce1"
uuid = "0e77f7df-68c5-4e49-93ce-4cd80f5598bf"
version = "1.2.1"

[[deps.LazyArtifacts]]
deps = ["Artifacts", "Pkg"]
uuid = "4af54fe1-eca0-43a8-85a7-787d91b784e3"

[[deps.LazyModules]]
git-tree-sha1 = "a560dd966b386ac9ae60bdd3a3d3a326062d3c3e"
uuid = "8cdb02fc-e678-4876-92c5-9defec4f444e"
version = "0.3.1"

[[deps.LeftChildRightSiblingTrees]]
deps = ["AbstractTrees"]
git-tree-sha1 = "fb6803dafae4a5d62ea5cab204b1e657d9737e7f"
uuid = "1d6d02ad-be62-4b6b-8a6d-2f90e265016e"
version = "0.2.0"

[[deps.LibCURL]]
deps = ["LibCURL_jll", "MozillaCACerts_jll"]
uuid = "b27032c2-a3e7-50c8-80cd-2d36dbcbfd21"
version = "0.6.3"

[[deps.LibCURL_jll]]
deps = ["Artifacts", "LibSSH2_jll", "Libdl", "MbedTLS_jll", "Zlib_jll", "nghttp2_jll"]
uuid = "deac9b47-8bc7-5906-a0fe-35ac56dc84c0"
version = "7.84.0+0"

[[deps.LibGit2]]
deps = ["Base64", "NetworkOptions", "Printf", "SHA"]
uuid = "76f85450-5226-5b5a-8eaa-529ad045b433"

[[deps.LibSSH2_jll]]
deps = ["Artifacts", "Libdl", "MbedTLS_jll"]
uuid = "29816b5a-b9ab-546f-933c-edad1886dfa8"
version = "1.10.2+0"

[[deps.Libdl]]
uuid = "8f399da3-3557-5675-b5ff-fb832c97cbdb"

[[deps.Libffi_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "0b4a5d71f3e5200a7dff793393e09dfc2d874290"
uuid = "e9f186c6-92d2-5b65-8a66-fee21dc1b490"
version = "3.2.2+1"

[[deps.Libgcrypt_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Libgpg_error_jll", "Pkg"]
git-tree-sha1 = "64613c82a59c120435c067c2b809fc61cf5166ae"
uuid = "d4300ac3-e22c-5743-9152-c294e39db1e4"
version = "1.8.7+0"

[[deps.Libgpg_error_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "c333716e46366857753e273ce6a69ee0945a6db9"
uuid = "7add5ba3-2f88-524e-9cd5-f83b8a55f7b8"
version = "1.42.0+0"

[[deps.Libiconv_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "f9557a255370125b405568f9767d6d195822a175"
uuid = "94ce4f54-9a6c-5748-9c1c-f9c7231a4531"
version = "1.17.0+0"

[[deps.Libmount_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "9c30530bf0effd46e15e0fdcf2b8636e78cbbd73"
uuid = "4b2f31a3-9ecc-558c-b454-b3730dcb73e9"
version = "2.35.0+0"

[[deps.Libuuid_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "7f3efec06033682db852f8b3bc3c1d2b0a0ab066"
uuid = "38a345b3-de98-5d2b-a5d3-14cd9215e700"
version = "2.36.0+0"

[[deps.LightXML]]
deps = ["Libdl", "XML2_jll"]
git-tree-sha1 = "e129d9391168c677cd4800f5c0abb1ed8cb3794f"
uuid = "9c8b4983-aa76-5018-a973-4c85ecc9e179"
version = "0.9.0"

[[deps.LineSearches]]
deps = ["LinearAlgebra", "NLSolversBase", "NaNMath", "Parameters", "Printf"]
git-tree-sha1 = "7bbea35cec17305fc70a0e5b4641477dc0789d9d"
uuid = "d3d80556-e9d4-5f37-9878-2ab0fcc64255"
version = "7.2.0"

[[deps.LinearAlgebra]]
deps = ["Libdl", "OpenBLAS_jll", "libblastrampoline_jll"]
uuid = "37e2e46d-f89d-539d-b4ee-838fcccc9c8e"

[[deps.LinearAlgebraX]]
deps = ["LinearAlgebra", "Mods", "Permutations", "Primes", "SimplePolynomials"]
git-tree-sha1 = "558a338f1eeabe933f9c2d4052aa7c2c707c3d52"
uuid = "9b3f67b0-2d00-526e-9884-9e4938f8fb88"
version = "0.1.12"

[[deps.LogDensityProblems]]
deps = ["ArgCheck", "DocStringExtensions", "Random"]
git-tree-sha1 = "f9a11237204bc137617194d79d813069838fcf61"
uuid = "6fdf6af0-433a-55f7-b3ed-c6c6e0b8df7c"
version = "2.1.1"

[[deps.LogExpFunctions]]
deps = ["DocStringExtensions", "IrrationalConstants", "LinearAlgebra"]
git-tree-sha1 = "7d6dd4e9212aebaeed356de34ccf262a3cd415aa"
uuid = "2ab3a3ac-af41-5b50-aa03-7779005ae688"
version = "0.3.26"

    [deps.LogExpFunctions.extensions]
    LogExpFunctionsChainRulesCoreExt = "ChainRulesCore"
    LogExpFunctionsChangesOfVariablesExt = "ChangesOfVariables"
    LogExpFunctionsInverseFunctionsExt = "InverseFunctions"

    [deps.LogExpFunctions.weakdeps]
    ChainRulesCore = "d360d2e6-b24c-11e9-a2a3-2a2ae2dbcce4"
    ChangesOfVariables = "9e997f8a-9a97-42d5-a9f1-ce6bfc15e2c0"
    InverseFunctions = "3587e190-3f89-42d0-90ee-14403ec27112"

[[deps.Logging]]
uuid = "56ddb016-857b-54e1-b83d-db4d58db5568"

[[deps.LoggingExtras]]
deps = ["Dates", "Logging"]
git-tree-sha1 = "c1dd6d7978c12545b4179fb6153b9250c96b0075"
uuid = "e6f89c97-d47a-5376-807f-9c37f3926c36"
version = "1.0.3"

[[deps.MCMCChains]]
deps = ["AbstractMCMC", "AxisArrays", "Dates", "Distributions", "Formatting", "IteratorInterfaceExtensions", "KernelDensity", "LinearAlgebra", "MCMCDiagnosticTools", "MLJModelInterface", "NaturalSort", "OrderedCollections", "PrettyTables", "Random", "RecipesBase", "Statistics", "StatsBase", "StatsFuns", "TableTraits", "Tables"]
git-tree-sha1 = "8778ea7283a0bf0d7e507a0235adfff38071493b"
uuid = "c7f686f2-ff18-58e9-bc7b-31028e88f75d"
version = "6.0.3"

[[deps.MCMCDiagnosticTools]]
deps = ["AbstractFFTs", "DataAPI", "DataStructures", "Distributions", "LinearAlgebra", "MLJModelInterface", "Random", "SpecialFunctions", "Statistics", "StatsBase", "StatsFuns", "Tables"]
git-tree-sha1 = "56c61163890d15d45901de423be0ad91855a2cd7"
uuid = "be115224-59cd-429b-ad48-344e309966f0"
version = "0.3.7"

[[deps.MIMEs]]
git-tree-sha1 = "65f28ad4b594aebe22157d6fac869786a255b7eb"
uuid = "6c6e2e6c-3030-632d-7369-2d6c69616d65"
version = "0.1.4"

[[deps.MKL_jll]]
deps = ["Artifacts", "IntelOpenMP_jll", "JLLWrappers", "LazyArtifacts", "Libdl", "Pkg"]
git-tree-sha1 = "eb006abbd7041c28e0d16260e50a24f8f9104913"
uuid = "856f044c-d86e-5d09-b602-aeab76dc8ba7"
version = "2023.2.0+0"

[[deps.MLJModelInterface]]
deps = ["Random", "ScientificTypesBase", "StatisticalTraits"]
git-tree-sha1 = "03ae109be87f460fe3c96b8a0dbbf9c7bf840bd5"
uuid = "e80e1ace-859a-464e-9ed9-23947d8ae3ea"
version = "1.9.2"

[[deps.MacroTools]]
deps = ["Markdown", "Random"]
git-tree-sha1 = "9ee1618cbf5240e6d4e0371d6f24065083f60c48"
uuid = "1914dd2f-81c6-5fcd-8719-6d5c9610ff09"
version = "0.5.11"

[[deps.Makie]]
deps = ["Animations", "Base64", "CRC32c", "ColorBrewer", "ColorSchemes", "ColorTypes", "Colors", "Contour", "DelaunayTriangulation", "Distributions", "DocStringExtensions", "Downloads", "FFMPEG_jll", "FileIO", "FixedPointNumbers", "Formatting", "FreeType", "FreeTypeAbstraction", "GeometryBasics", "GridLayoutBase", "ImageIO", "InteractiveUtils", "IntervalSets", "Isoband", "KernelDensity", "LaTeXStrings", "LinearAlgebra", "MacroTools", "MakieCore", "Markdown", "Match", "MathTeXEngine", "Observables", "OffsetArrays", "Packing", "PlotUtils", "PolygonOps", "PrecompileTools", "Printf", "REPL", "Random", "RelocatableFolders", "Setfield", "ShaderAbstractions", "Showoff", "SignedDistanceFields", "SparseArrays", "StableHashTraits", "Statistics", "StatsBase", "StatsFuns", "StructArrays", "TriplotBase", "UnicodeFun"]
git-tree-sha1 = "1d16d20279a145119899b4205258332f0fbeaa94"
uuid = "ee78f7c6-11fb-53f2-987a-cfe4a2b5a57a"
version = "0.19.11"

[[deps.MakieCore]]
deps = ["Observables", "REPL"]
git-tree-sha1 = "a94bf3fef9c690a2a4ac1d09d86a59ab89c7f8e4"
uuid = "20f20a25-4f0e-4fdf-b5d1-57303727442b"
version = "0.6.8"

[[deps.ManualMemory]]
git-tree-sha1 = "bcaef4fc7a0cfe2cba636d84cda54b5e4e4ca3cd"
uuid = "d125e4d3-2237-4719-b19c-fa641b8a4667"
version = "0.1.8"

[[deps.MappedArrays]]
git-tree-sha1 = "2dab0221fe2b0f2cb6754eaa743cc266339f527e"
uuid = "dbb5928d-eab1-5f90-85c2-b9b0edb7c900"
version = "0.4.2"

[[deps.Markdown]]
deps = ["Base64"]
uuid = "d6f4376e-aef5-505a-96c1-9c027394607a"

[[deps.MarkdownAST]]
deps = ["AbstractTrees", "Markdown"]
git-tree-sha1 = "465a70f0fc7d443a00dcdc3267a497397b8a3899"
uuid = "d0879d2d-cac2-40c8-9cee-1863dc0c7391"
version = "0.1.2"

[[deps.Match]]
git-tree-sha1 = "1d9bc5c1a6e7ee24effb93f175c9342f9154d97f"
uuid = "7eb4fadd-790c-5f42-8a69-bfa0b872bfbf"
version = "1.2.0"

[[deps.MathTeXEngine]]
deps = ["AbstractTrees", "Automa", "DataStructures", "FreeTypeAbstraction", "GeometryBasics", "LaTeXStrings", "REPL", "RelocatableFolders", "Test", "UnicodeFun"]
git-tree-sha1 = "8f52dbaa1351ce4cb847d95568cb29e62a307d93"
uuid = "0a4f8689-d25c-4efe-a92b-7142dfc1aa53"
version = "0.5.6"

[[deps.MbedTLS_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "c8ffd9c3-330d-5841-b78e-0817d7145fa1"
version = "2.28.2+0"

[[deps.MicroCollections]]
deps = ["BangBang", "InitialValues", "Setfield"]
git-tree-sha1 = "629afd7d10dbc6935ec59b32daeb33bc4460a42e"
uuid = "128add7d-3638-4c79-886c-908ea0c25c34"
version = "0.1.4"

[[deps.Missings]]
deps = ["DataAPI"]
git-tree-sha1 = "f66bdc5de519e8f8ae43bdc598782d35a25b1272"
uuid = "e1d29d7a-bbdc-5cf2-9ac0-f12de2c33e28"
version = "1.1.0"

[[deps.Mmap]]
uuid = "a63ad114-7e13-5084-954f-fe012c677804"

[[deps.Mods]]
git-tree-sha1 = "61be59e4daffff43a8cec04b5e0dc773cbb5db3a"
uuid = "7475f97c-0381-53b1-977b-4c60186c8d62"
version = "1.3.3"

[[deps.MonteCarloMeasurements]]
deps = ["Distributed", "Distributions", "ForwardDiff", "GenericSchur", "LinearAlgebra", "MacroTools", "Random", "RecipesBase", "Requires", "SLEEFPirates", "StaticArrays", "Statistics", "StatsBase", "Test"]
git-tree-sha1 = "19d4a73e20ca54f0f0e8a4ed349ee0dfd6e997b7"
uuid = "0987c9cc-fe09-11e8-30f0-b96dd679fdca"
version = "1.1.6"

[[deps.MosaicViews]]
deps = ["MappedArrays", "OffsetArrays", "PaddedViews", "StackViews"]
git-tree-sha1 = "7b86a5d4d70a9f5cdf2dacb3cbe6d251d1a61dbe"
uuid = "e94cdb99-869f-56ef-bcf0-1ae2bcbe0389"
version = "0.3.4"

[[deps.MozillaCACerts_jll]]
uuid = "14a3606d-f60d-562e-9121-12d972cd8159"
version = "2022.10.11"

[[deps.Multisets]]
git-tree-sha1 = "8d852646862c96e226367ad10c8af56099b4047e"
uuid = "3b2b4ff1-bcff-5658-a3ee-dbcf1ce5ac09"
version = "0.4.4"

[[deps.NLSolversBase]]
deps = ["DiffResults", "Distributed", "FiniteDiff", "ForwardDiff"]
git-tree-sha1 = "a0b464d183da839699f4c79e7606d9d186ec172c"
uuid = "d41bc354-129a-5804-8e4c-c37616107c6c"
version = "7.8.3"

[[deps.NaNMath]]
deps = ["OpenLibm_jll"]
git-tree-sha1 = "0877504529a3e5c3343c6f8b4c0381e57e4387e4"
uuid = "77ba4419-2d1f-58cd-9bb1-8ffee604a2e3"
version = "1.0.2"

[[deps.NamedArrays]]
deps = ["Combinatorics", "DataStructures", "DelimitedFiles", "InvertedIndices", "LinearAlgebra", "Random", "Requires", "SparseArrays", "Statistics"]
git-tree-sha1 = "b84e17976a40cb2bfe3ae7edb3673a8c630d4f95"
uuid = "86f7a689-2022-50b4-a561-43c23ac3c673"
version = "0.9.8"

[[deps.NamedTupleTools]]
git-tree-sha1 = "90914795fc59df44120fe3fff6742bb0d7adb1d0"
uuid = "d9ec5142-1e00-5aa0-9d6a-321866360f50"
version = "0.14.3"

[[deps.NaturalSort]]
git-tree-sha1 = "eda490d06b9f7c00752ee81cfa451efe55521e21"
uuid = "c020b1a1-e9b0-503a-9c33-f039bfc54a85"
version = "1.0.0"

[[deps.Netpbm]]
deps = ["FileIO", "ImageCore", "ImageMetadata"]
git-tree-sha1 = "d92b107dbb887293622df7697a2223f9f8176fcd"
uuid = "f09324ee-3d7c-5217-9330-fc30815ba969"
version = "1.1.1"

[[deps.NetworkOptions]]
uuid = "ca575930-c2e3-43a9-ace4-1e988b2c1908"
version = "1.2.0"

[[deps.Observables]]
git-tree-sha1 = "6862738f9796b3edc1c09d0890afce4eca9e7e93"
uuid = "510215fc-4207-5dde-b226-833fc4488ee2"
version = "0.5.4"

[[deps.OffsetArrays]]
deps = ["Adapt"]
git-tree-sha1 = "2ac17d29c523ce1cd38e27785a7d23024853a4bb"
uuid = "6fe1bfb0-de20-5000-8ca7-80f57d26f881"
version = "1.12.10"

[[deps.Ogg_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "887579a3eb005446d514ab7aeac5d1d027658b8f"
uuid = "e7412a2a-1a6e-54c0-be00-318e2571c051"
version = "1.3.5+1"

[[deps.OpenBLAS_jll]]
deps = ["Artifacts", "CompilerSupportLibraries_jll", "Libdl"]
uuid = "4536629a-c528-5b80-bd46-f80d51c5b363"
version = "0.3.21+4"

[[deps.OpenEXR]]
deps = ["Colors", "FileIO", "OpenEXR_jll"]
git-tree-sha1 = "327f53360fdb54df7ecd01e96ef1983536d1e633"
uuid = "52e1d378-f018-4a11-a4be-720524705ac7"
version = "0.3.2"

[[deps.OpenEXR_jll]]
deps = ["Artifacts", "Imath_jll", "JLLWrappers", "Libdl", "Zlib_jll"]
git-tree-sha1 = "a4ca623df1ae99d09bc9868b008262d0c0ac1e4f"
uuid = "18a262bb-aa17-5467-a713-aee519bc75cb"
version = "3.1.4+0"

[[deps.OpenLibm_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "05823500-19ac-5b8b-9628-191a04bc5112"
version = "0.8.1+0"

[[deps.OpenSSL_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "ceeda72c9fd6bbebc4f4f598560789145a8b6c4c"
uuid = "458c3c95-2e84-50aa-8efc-19380b2a3a95"
version = "3.0.11+0"

[[deps.OpenSpecFun_jll]]
deps = ["Artifacts", "CompilerSupportLibraries_jll", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "13652491f6856acfd2db29360e1bbcd4565d04f1"
uuid = "efe28fd5-8261-553b-a9e1-b2916fc3738e"
version = "0.5.5+0"

[[deps.Optim]]
deps = ["Compat", "FillArrays", "ForwardDiff", "LineSearches", "LinearAlgebra", "NLSolversBase", "NaNMath", "Parameters", "PositiveFactorizations", "Printf", "SparseArrays", "StatsBase"]
git-tree-sha1 = "01f85d9269b13fedc61e63cc72ee2213565f7a72"
uuid = "429524aa-4258-5aef-a3af-852621145aeb"
version = "1.7.8"

[[deps.Opus_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "51a08fb14ec28da2ec7a927c4337e4332c2a4720"
uuid = "91d4177d-7536-5919-b921-800302f37372"
version = "1.3.2+0"

[[deps.OrderedCollections]]
git-tree-sha1 = "2e73fe17cac3c62ad1aebe70d44c963c3cfdc3e3"
uuid = "bac558e1-5e72-5ebc-8fee-abe8a469f55d"
version = "1.6.2"

[[deps.PCRE2_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "efcefdf7-47ab-520b-bdef-62a2eaa19f15"
version = "10.42.0+0"

[[deps.PDMats]]
deps = ["LinearAlgebra", "SparseArrays", "SuiteSparse"]
git-tree-sha1 = "fcf8fd477bd7f33cb8dbb1243653fb0d415c256c"
uuid = "90014a1f-27ba-587c-ab20-58faa44d9150"
version = "0.11.25"

[[deps.PNGFiles]]
deps = ["Base64", "CEnum", "ImageCore", "IndirectArrays", "OffsetArrays", "libpng_jll"]
git-tree-sha1 = "9b02b27ac477cad98114584ff964e3052f656a0f"
uuid = "f57f5aa1-a3ce-4bc8-8ab9-96f992907883"
version = "0.4.0"

[[deps.Packing]]
deps = ["GeometryBasics"]
git-tree-sha1 = "ec3edfe723df33528e085e632414499f26650501"
uuid = "19eb6ba3-879d-56ad-ad62-d5c202156566"
version = "0.5.0"

[[deps.PaddedViews]]
deps = ["OffsetArrays"]
git-tree-sha1 = "0fac6313486baae819364c52b4f483450a9d793f"
uuid = "5432bcbf-9aad-5242-b902-cca2824c8663"
version = "0.5.12"

[[deps.Pango_jll]]
deps = ["Artifacts", "Cairo_jll", "Fontconfig_jll", "FreeType2_jll", "FriBidi_jll", "Glib_jll", "HarfBuzz_jll", "JLLWrappers", "Libdl"]
git-tree-sha1 = "4745216e94f71cb768d58330b059c9b76f32cb66"
uuid = "36c8627f-9965-5494-a995-c6b170f724f3"
version = "1.50.14+0"

[[deps.Parameters]]
deps = ["OrderedCollections", "UnPack"]
git-tree-sha1 = "34c0e9ad262e5f7fc75b10a9952ca7692cfc5fbe"
uuid = "d96e819e-fc66-5662-9728-84c9c7592b0a"
version = "0.12.3"

[[deps.ParetoSmoothedImportanceSampling]]
deps = ["CSV", "DataFrames", "Distributions", "JSON", "Printf", "Random", "Statistics", "StatsFuns", "Test"]
git-tree-sha1 = "ab01cfa3648326833ce3bd225117d2c0cacc5d9d"
uuid = "98f080ec-61e2-11eb-1c7b-31ea1097256f"
version = "1.5.2"

[[deps.Parsers]]
deps = ["Dates", "PrecompileTools", "UUIDs"]
git-tree-sha1 = "716e24b21538abc91f6205fd1d8363f39b442851"
uuid = "69de0a69-1ddd-5017-9359-2bf0b02dc9f0"
version = "2.7.2"

[[deps.Permutations]]
deps = ["Combinatorics", "LinearAlgebra", "Random"]
git-tree-sha1 = "25e2bb0973689836bf164ecb960762f1bb8794dd"
uuid = "2ae35dd2-176d-5d53-8349-f30d82d94d4f"
version = "0.4.17"

[[deps.Pixman_jll]]
deps = ["Artifacts", "CompilerSupportLibraries_jll", "JLLWrappers", "LLVMOpenMP_jll", "Libdl"]
git-tree-sha1 = "64779bc4c9784fee475689a1752ef4d5747c5e87"
uuid = "30392449-352a-5448-841d-b1acce4e97dc"
version = "0.42.2+0"

[[deps.Pkg]]
deps = ["Artifacts", "Dates", "Downloads", "FileWatching", "LibGit2", "Libdl", "Logging", "Markdown", "Printf", "REPL", "Random", "SHA", "Serialization", "TOML", "Tar", "UUIDs", "p7zip_jll"]
uuid = "44cfe95a-1eb2-52ea-b672-e2afdf69b78f"
version = "1.9.2"

[[deps.PkgVersion]]
deps = ["Pkg"]
git-tree-sha1 = "f9501cc0430a26bc3d156ae1b5b0c1b47af4d6da"
uuid = "eebad327-c553-4316-9ea0-9fa01ccd7688"
version = "0.3.3"

[[deps.PlotUtils]]
deps = ["ColorSchemes", "Colors", "Dates", "PrecompileTools", "Printf", "Random", "Reexport", "Statistics"]
git-tree-sha1 = "f92e1315dadf8c46561fb9396e525f7200cdc227"
uuid = "995b91a9-d308-5afd-9ec6-746e21dbc043"
version = "1.3.5"

[[deps.PlutoUI]]
deps = ["AbstractPlutoDingetjes", "Base64", "ColorTypes", "Dates", "FixedPointNumbers", "Hyperscript", "HypertextLiteral", "IOCapture", "InteractiveUtils", "JSON", "Logging", "MIMEs", "Markdown", "Random", "Reexport", "URIs", "UUIDs"]
git-tree-sha1 = "e47cd150dbe0443c3a3651bc5b9cbd5576ab75b7"
uuid = "7f904dfe-b85e-4ff6-b463-dae2292396a8"
version = "0.7.52"

[[deps.PolygonOps]]
git-tree-sha1 = "77b3d3605fc1cd0b42d95eba87dfcd2bf67d5ff6"
uuid = "647866c9-e3ac-4575-94e7-e3d426903924"
version = "0.1.2"

[[deps.Polynomials]]
deps = ["LinearAlgebra", "RecipesBase", "Setfield", "SparseArrays"]
git-tree-sha1 = "ea78a2764f31715093de7ab495e12c0187f231d1"
uuid = "f27b6e38-b328-58d1-80ce-0feddd5e7a45"
version = "4.0.4"

    [deps.Polynomials.extensions]
    PolynomialsChainRulesCoreExt = "ChainRulesCore"
    PolynomialsFFTWExt = "FFTW"
    PolynomialsMakieCoreExt = "MakieCore"
    PolynomialsMutableArithmeticsExt = "MutableArithmetics"

    [deps.Polynomials.weakdeps]
    ChainRulesCore = "d360d2e6-b24c-11e9-a2a3-2a2ae2dbcce4"
    FFTW = "7a1cc6ca-52ef-59f5-83cd-3a7055c09341"
    MakieCore = "20f20a25-4f0e-4fdf-b5d1-57303727442b"
    MutableArithmetics = "d8a4904e-b15c-11e9-3269-09a3773c0cb0"

[[deps.PooledArrays]]
deps = ["DataAPI", "Future"]
git-tree-sha1 = "36d8b4b899628fb92c2749eb488d884a926614d3"
uuid = "2dfb63ee-cc39-5dd5-95bd-886bf059d720"
version = "1.4.3"

[[deps.PositiveFactorizations]]
deps = ["LinearAlgebra"]
git-tree-sha1 = "17275485f373e6673f7e7f97051f703ed5b15b20"
uuid = "85a6dd25-e78a-55b7-8502-1745935b8125"
version = "0.2.4"

[[deps.PrecompileTools]]
deps = ["Preferences"]
git-tree-sha1 = "03b4c25b43cb84cee5c90aa9b5ea0a78fd848d2f"
uuid = "aea7be01-6a6a-4083-8856-8a6e6704d82a"
version = "1.2.0"

[[deps.Preferences]]
deps = ["TOML"]
git-tree-sha1 = "00805cd429dcb4870060ff49ef443486c262e38e"
uuid = "21216c6a-2e73-6563-6e65-726566657250"
version = "1.4.1"

[[deps.PrettyTables]]
deps = ["Crayons", "LaTeXStrings", "Markdown", "Printf", "Reexport", "StringManipulation", "Tables"]
git-tree-sha1 = "ee094908d720185ddbdc58dbe0c1cbe35453ec7a"
uuid = "08abe8d2-0d0c-5749-adfa-8a2ac140af0d"
version = "2.2.7"

[[deps.Primes]]
deps = ["IntegerMathUtils"]
git-tree-sha1 = "4c9f306e5d6603ae203c2000dd460d81a5251489"
uuid = "27ebfcd6-29c5-5fa9-bf4b-fb8fc14df3ae"
version = "0.5.4"

[[deps.Printf]]
deps = ["Unicode"]
uuid = "de0858da-6303-5e67-8744-51eddeeeb8d7"

[[deps.ProgressLogging]]
deps = ["Logging", "SHA", "UUIDs"]
git-tree-sha1 = "80d919dee55b9c50e8d9e2da5eeafff3fe58b539"
uuid = "33c8b6b6-d38a-422a-b730-caa89a2f386c"
version = "0.1.4"

[[deps.ProgressMeter]]
deps = ["Distributed", "Printf"]
git-tree-sha1 = "00099623ffee15972c16111bcf84c58a0051257c"
uuid = "92933f4c-e287-5a05-a399-4b506db050ca"
version = "1.9.0"

[[deps.QOI]]
deps = ["ColorTypes", "FileIO", "FixedPointNumbers"]
git-tree-sha1 = "18e8f4d1426e965c7b532ddd260599e1510d26ce"
uuid = "4b34888f-f399-49d4-9bb3-47ed5cae4e65"
version = "1.0.0"

[[deps.QuadGK]]
deps = ["DataStructures", "LinearAlgebra"]
git-tree-sha1 = "9ebcd48c498668c7fa0e97a9cae873fbee7bfee1"
uuid = "1fd47b50-473d-5c70-9696-f719f8f3bcdc"
version = "2.9.1"

[[deps.REPL]]
deps = ["InteractiveUtils", "Markdown", "Sockets", "Unicode"]
uuid = "3fa0cd96-eef1-5676-8a61-b3b8758bbffb"

[[deps.Random]]
deps = ["SHA", "Serialization"]
uuid = "9a3f8284-a2c9-5f02-9a11-845980a1fd5c"

[[deps.RangeArrays]]
git-tree-sha1 = "b9039e93773ddcfc828f12aadf7115b4b4d225f5"
uuid = "b3c3ace0-ae52-54e7-9d0b-2c1406fd6b9d"
version = "0.3.2"

[[deps.Ratios]]
deps = ["Requires"]
git-tree-sha1 = "1342a47bf3260ee108163042310d26f2be5ec90b"
uuid = "c84ed2f1-dad5-54f0-aa8e-dbefe2724439"
version = "0.4.5"
weakdeps = ["FixedPointNumbers"]

    [deps.Ratios.extensions]
    RatiosFixedPointNumbersExt = "FixedPointNumbers"

[[deps.RecipesBase]]
deps = ["PrecompileTools"]
git-tree-sha1 = "5c3d09cc4f31f5fc6af001c250bf1278733100ff"
uuid = "3cdcf5f2-1ef4-517c-9805-6587b60abb01"
version = "1.3.4"

[[deps.Reexport]]
git-tree-sha1 = "45e428421666073eab6f2da5c9d310d99bb12f9b"
uuid = "189a3867-3050-52da-a836-e630ba90ab69"
version = "1.2.2"

[[deps.RegistryInstances]]
deps = ["LazilyInitializedFields", "Pkg", "TOML", "Tar"]
git-tree-sha1 = "ffd19052caf598b8653b99404058fce14828be51"
uuid = "2792f1a3-b283-48e8-9a74-f99dce5104f3"
version = "0.1.0"

[[deps.RelocatableFolders]]
deps = ["SHA", "Scratch"]
git-tree-sha1 = "ffdaf70d81cf6ff22c2b6e733c900c3321cab864"
uuid = "05181044-ff0b-4ac5-8273-598c1e38db00"
version = "1.0.1"

[[deps.Requires]]
deps = ["UUIDs"]
git-tree-sha1 = "838a3a4188e2ded87a4f9f184b4b0d78a1e91cb7"
uuid = "ae029012-a4dd-5104-9daa-d747884805df"
version = "1.3.0"

[[deps.RingLists]]
deps = ["Random"]
git-tree-sha1 = "f39da63aa6d2d88e0c1bd20ed6a3ff9ea7171ada"
uuid = "286e9d63-9694-5540-9e3c-4e6708fa07b2"
version = "0.2.8"

[[deps.Rmath]]
deps = ["Random", "Rmath_jll"]
git-tree-sha1 = "f65dcb5fa46aee0cf9ed6274ccbd597adc49aa7b"
uuid = "79098fc4-a85e-5d69-aa6a-4863f24498fa"
version = "0.7.1"

[[deps.Rmath_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "6ed52fdd3382cf21947b15e8870ac0ddbff736da"
uuid = "f50d1b31-88e8-58de-be2c-1cc44531875f"
version = "0.4.0+0"

[[deps.RoundingEmulator]]
git-tree-sha1 = "40b9edad2e5287e05bd413a38f61a8ff55b9557b"
uuid = "5eaf0fd0-dfba-4ccb-bf02-d820a40db705"
version = "0.2.1"

[[deps.SHA]]
uuid = "ea8e919c-243c-51af-8825-aaa63cd721ce"
version = "0.7.0"

[[deps.SIMDTypes]]
git-tree-sha1 = "330289636fb8107c5f32088d2741e9fd7a061a5c"
uuid = "94e857df-77ce-4151-89e5-788b33177be4"
version = "0.1.0"

[[deps.SLEEFPirates]]
deps = ["IfElse", "Static", "VectorizationBase"]
git-tree-sha1 = "4b8586aece42bee682399c4c4aee95446aa5cd19"
uuid = "476501e8-09a2-5ece-8869-fb82de89a1fa"
version = "0.6.39"

[[deps.ScientificTypesBase]]
git-tree-sha1 = "a8e18eb383b5ecf1b5e6fc237eb39255044fd92b"
uuid = "30f210dd-8aff-4c5f-94ba-8e64358c1161"
version = "3.0.0"

[[deps.Scratch]]
deps = ["Dates"]
git-tree-sha1 = "30449ee12237627992a99d5e30ae63e4d78cd24a"
uuid = "6c6a2e73-6563-6170-7368-637461726353"
version = "1.2.0"

[[deps.SentinelArrays]]
deps = ["Dates", "Random"]
git-tree-sha1 = "04bdff0b09c65ff3e06a05e3eb7b120223da3d39"
uuid = "91c51154-3ec4-41a3-a24f-3f23e20d615c"
version = "1.4.0"

[[deps.Serialization]]
uuid = "9e88b42a-f829-5b0c-bbe9-9e923198166b"

[[deps.SetRounding]]
git-tree-sha1 = "d7a25e439d07a17b7cdf97eecee504c50fedf5f6"
uuid = "3cc68bcd-71a2-5612-b932-767ffbe40ab0"
version = "0.2.1"

[[deps.Setfield]]
deps = ["ConstructionBase", "Future", "MacroTools", "StaticArraysCore"]
git-tree-sha1 = "e2cc6d8c88613c05e1defb55170bf5ff211fbeac"
uuid = "efcf1570-3423-57d1-acb7-fd33fddbac46"
version = "1.1.1"

[[deps.ShaderAbstractions]]
deps = ["ColorTypes", "FixedPointNumbers", "GeometryBasics", "LinearAlgebra", "Observables", "StaticArrays", "StructArrays", "Tables"]
git-tree-sha1 = "db0219befe4507878b1a90e07820fed3e62c289d"
uuid = "65257c39-d410-5151-9873-9b3e5be5013e"
version = "0.4.0"

[[deps.SharedArrays]]
deps = ["Distributed", "Mmap", "Random", "Serialization"]
uuid = "1a1011a3-84de-559e-8e89-a11a2f7dc383"

[[deps.Showoff]]
deps = ["Dates", "Grisu"]
git-tree-sha1 = "91eddf657aca81df9ae6ceb20b959ae5653ad1de"
uuid = "992d4aef-0814-514b-bc4d-f2e9a6c4116f"
version = "1.0.3"

[[deps.SignedDistanceFields]]
deps = ["Random", "Statistics", "Test"]
git-tree-sha1 = "d263a08ec505853a5ff1c1ebde2070419e3f28e9"
uuid = "73760f76-fbc4-59ce-8f25-708e95d2df96"
version = "0.4.0"

[[deps.SimpleGraphs]]
deps = ["AbstractLattices", "Combinatorics", "DataStructures", "IterTools", "LightXML", "LinearAlgebra", "LinearAlgebraX", "Optim", "Primes", "Random", "RingLists", "SimplePartitions", "SimplePolynomials", "SimpleRandom", "SparseArrays", "Statistics"]
git-tree-sha1 = "b608903049d11cc557c45e03b3a53e9260579c19"
uuid = "55797a34-41de-5266-9ec1-32ac4eb504d3"
version = "0.8.4"

[[deps.SimplePartitions]]
deps = ["AbstractLattices", "DataStructures", "Permutations"]
git-tree-sha1 = "dcc02923a53f316ab97da8ef3136e80b4543dbf1"
uuid = "ec83eff0-a5b5-5643-ae32-5cbf6eedec9d"
version = "0.3.0"

[[deps.SimplePolynomials]]
deps = ["Mods", "Multisets", "Polynomials", "Primes"]
git-tree-sha1 = "d537c31cf9995236166e3e9afc424a5a1c59ff9d"
uuid = "cc47b68c-3164-5771-a705-2bc0097375a0"
version = "0.2.14"

[[deps.SimpleRandom]]
deps = ["Distributions", "LinearAlgebra", "Random"]
git-tree-sha1 = "3a6fb395e37afab81aeea85bae48a4db5cd7244a"
uuid = "a6525b86-64cd-54fa-8f65-62fc48bdc0e8"
version = "0.3.1"

[[deps.SimpleTraits]]
deps = ["InteractiveUtils", "MacroTools"]
git-tree-sha1 = "5d7e3f4e11935503d3ecaf7186eac40602e7d231"
uuid = "699a6c99-e7fa-54fc-8d76-47d257e15c1d"
version = "0.9.4"

[[deps.Sixel]]
deps = ["Dates", "FileIO", "ImageCore", "IndirectArrays", "OffsetArrays", "REPL", "libsixel_jll"]
git-tree-sha1 = "2da10356e31327c7096832eb9cd86307a50b1eb6"
uuid = "45858cf5-a6b0-47a3-bbea-62219f50df47"
version = "0.1.3"

[[deps.Sockets]]
uuid = "6462fe0b-24de-5631-8697-dd941f90decc"

[[deps.SortingAlgorithms]]
deps = ["DataStructures"]
git-tree-sha1 = "c60ec5c62180f27efea3ba2908480f8055e17cee"
uuid = "a2af1166-a08f-5f64-846c-94a0d3cef48c"
version = "1.1.1"

[[deps.SparseArrays]]
deps = ["Libdl", "LinearAlgebra", "Random", "Serialization", "SuiteSparse_jll"]
uuid = "2f01184e-e22b-5df5-ae63-d93ebab69eaf"

[[deps.SpecialFunctions]]
deps = ["IrrationalConstants", "LogExpFunctions", "OpenLibm_jll", "OpenSpecFun_jll"]
git-tree-sha1 = "e2cfc4012a19088254b3950b85c3c1d8882d864d"
uuid = "276daf66-3868-5448-9aa4-cd146d93841b"
version = "2.3.1"
weakdeps = ["ChainRulesCore"]

    [deps.SpecialFunctions.extensions]
    SpecialFunctionsChainRulesCoreExt = "ChainRulesCore"

[[deps.SplittablesBase]]
deps = ["Setfield", "Test"]
git-tree-sha1 = "e08a62abc517eb79667d0a29dc08a3b589516bb5"
uuid = "171d559e-b47b-412a-8079-5efa626c420e"
version = "0.1.15"

[[deps.StableHashTraits]]
deps = ["Compat", "SHA", "Tables", "TupleTools"]
git-tree-sha1 = "19df33ca14f24a3ad2df9e89124bd5f5cc8467a2"
uuid = "c5dd0088-6c3f-4803-b00e-f31a60c170fa"
version = "1.0.1"

[[deps.StackViews]]
deps = ["OffsetArrays"]
git-tree-sha1 = "46e589465204cd0c08b4bd97385e4fa79a0c770c"
uuid = "cae243ae-269e-4f55-b966-ac2d0dc13c15"
version = "0.1.1"

[[deps.Static]]
deps = ["IfElse"]
git-tree-sha1 = "f295e0a1da4ca425659c57441bcb59abb035a4bc"
uuid = "aedffcd0-7271-4cad-89d0-dc628f76c6d3"
version = "0.8.8"

[[deps.StaticArrayInterface]]
deps = ["ArrayInterface", "Compat", "IfElse", "LinearAlgebra", "PrecompileTools", "Requires", "SparseArrays", "Static", "SuiteSparse"]
git-tree-sha1 = "03fec6800a986d191f64f5c0996b59ed526eda25"
uuid = "0d7ed370-da01-4f52-bd93-41d350b8b718"
version = "1.4.1"
weakdeps = ["OffsetArrays", "StaticArrays"]

    [deps.StaticArrayInterface.extensions]
    StaticArrayInterfaceOffsetArraysExt = "OffsetArrays"
    StaticArrayInterfaceStaticArraysExt = "StaticArrays"

[[deps.StaticArrays]]
deps = ["LinearAlgebra", "Random", "StaticArraysCore"]
git-tree-sha1 = "0adf069a2a490c47273727e029371b31d44b72b2"
uuid = "90137ffa-7385-5640-81b9-e52037218182"
version = "1.6.5"
weakdeps = ["Statistics"]

    [deps.StaticArrays.extensions]
    StaticArraysStatisticsExt = "Statistics"

[[deps.StaticArraysCore]]
git-tree-sha1 = "36b3d696ce6366023a0ea192b4cd442268995a0d"
uuid = "1e83bf80-4336-4d27-bf5d-d5a4f845583c"
version = "1.4.2"

[[deps.StatisticalRethinking]]
deps = ["CSV", "DataFrames", "Dates", "Distributions", "DocStringExtensions", "Documenter", "Formatting", "KernelDensity", "LinearAlgebra", "MCMCChains", "MonteCarloMeasurements", "NamedArrays", "NamedTupleTools", "Optim", "OrderedCollections", "Parameters", "ParetoSmoothedImportanceSampling", "PrettyTables", "Random", "Reexport", "Requires", "Statistics", "StatsBase", "StatsFuns", "StructuralCausalModels", "Tables", "Test", "Unicode"]
git-tree-sha1 = "08a543e968a379b3a5d04c2acc25a3f8310e9961"
uuid = "2d09df54-9d0f-5258-8220-54c2a3d4fbee"
version = "4.7.3"

[[deps.StatisticalTraits]]
deps = ["ScientificTypesBase"]
git-tree-sha1 = "30b9236691858e13f167ce829490a68e1a597782"
uuid = "64bff920-2084-43da-a3e6-9bb72801c0c9"
version = "3.2.0"

[[deps.Statistics]]
deps = ["LinearAlgebra", "SparseArrays"]
uuid = "10745b16-79ce-11e8-11f9-7d13ad32a3b2"
version = "1.9.0"

[[deps.StatsAPI]]
deps = ["LinearAlgebra"]
git-tree-sha1 = "1ff449ad350c9c4cbc756624d6f8a8c3ef56d3ed"
uuid = "82ae8749-77ed-4fe6-ae5f-f523153014b0"
version = "1.7.0"

[[deps.StatsBase]]
deps = ["DataAPI", "DataStructures", "LinearAlgebra", "LogExpFunctions", "Missings", "Printf", "Random", "SortingAlgorithms", "SparseArrays", "Statistics", "StatsAPI"]
git-tree-sha1 = "1d77abd07f617c4868c33d4f5b9e1dbb2643c9cf"
uuid = "2913bbd2-ae8a-5f71-8c99-4fb6c76f3a91"
version = "0.34.2"

[[deps.StatsFuns]]
deps = ["HypergeometricFunctions", "IrrationalConstants", "LogExpFunctions", "Reexport", "Rmath", "SpecialFunctions"]
git-tree-sha1 = "f625d686d5a88bcd2b15cd81f18f98186fdc0c9a"
uuid = "4c63d2b9-4356-54db-8cca-17b64c39e42c"
version = "1.3.0"

    [deps.StatsFuns.extensions]
    StatsFunsChainRulesCoreExt = "ChainRulesCore"
    StatsFunsInverseFunctionsExt = "InverseFunctions"

    [deps.StatsFuns.weakdeps]
    ChainRulesCore = "d360d2e6-b24c-11e9-a2a3-2a2ae2dbcce4"
    InverseFunctions = "3587e190-3f89-42d0-90ee-14403ec27112"

[[deps.StringManipulation]]
deps = ["PrecompileTools"]
git-tree-sha1 = "a04cabe79c5f01f4d723cc6704070ada0b9d46d5"
uuid = "892a3eda-7b42-436c-8928-eab12a02cf0e"
version = "0.3.4"

[[deps.StructArrays]]
deps = ["Adapt", "ConstructionBase", "DataAPI", "GPUArraysCore", "StaticArraysCore", "Tables"]
git-tree-sha1 = "0a3db38e4cce3c54fe7a71f831cd7b6194a54213"
uuid = "09ab397b-f2b6-538f-b94a-2f83cf4a842a"
version = "0.6.16"

[[deps.StructuralCausalModels]]
deps = ["CSV", "Combinatorics", "DataFrames", "DataStructures", "Distributions", "DocStringExtensions", "LinearAlgebra", "NamedArrays", "Reexport", "Statistics"]
git-tree-sha1 = "01c838be8d7119708b839aa16d413088a1076ee8"
uuid = "a41e6734-49ce-4065-8b83-aff084c01dfd"
version = "1.4.1"

[[deps.SuiteSparse]]
deps = ["Libdl", "LinearAlgebra", "Serialization", "SparseArrays"]
uuid = "4607b0f0-06f3-5cda-b6b1-a6196a1729e9"

[[deps.SuiteSparse_jll]]
deps = ["Artifacts", "Libdl", "Pkg", "libblastrampoline_jll"]
uuid = "bea87d4a-7f5b-5778-9afe-8cc45184846c"
version = "5.10.1+6"

[[deps.TOML]]
deps = ["Dates"]
uuid = "fa267f1f-6049-4f14-aa54-33bafae1ed76"
version = "1.0.3"

[[deps.TableTraits]]
deps = ["IteratorInterfaceExtensions"]
git-tree-sha1 = "c06b2f539df1c6efa794486abfb6ed2022561a39"
uuid = "3783bdb8-4a98-5b6b-af9a-565f29a5fe9c"
version = "1.0.1"

[[deps.Tables]]
deps = ["DataAPI", "DataValueInterfaces", "IteratorInterfaceExtensions", "LinearAlgebra", "OrderedCollections", "TableTraits"]
git-tree-sha1 = "a1f34829d5ac0ef499f6d84428bd6b4c71f02ead"
uuid = "bd369af6-aec1-5ad0-b16a-f7cc5008161c"
version = "1.11.0"

[[deps.Tar]]
deps = ["ArgTools", "SHA"]
uuid = "a4e569a6-e804-4fa4-b0f3-eef7a1d5b13e"
version = "1.10.0"

[[deps.TensorCore]]
deps = ["LinearAlgebra"]
git-tree-sha1 = "1feb45f88d133a655e001435632f019a9a1bcdb6"
uuid = "62fd8b95-f654-4bbd-a8a5-9c27f68ccd50"
version = "0.1.1"

[[deps.TerminalLoggers]]
deps = ["LeftChildRightSiblingTrees", "Logging", "Markdown", "Printf", "ProgressLogging", "UUIDs"]
git-tree-sha1 = "f133fab380933d042f6796eda4e130272ba520ca"
uuid = "5d786b92-1e48-4d6f-9151-6b4477ca9bed"
version = "0.1.7"

[[deps.Test]]
deps = ["InteractiveUtils", "Logging", "Random", "Serialization"]
uuid = "8dfed614-e22c-5e08-85e1-65c5234f0b40"

[[deps.TiffImages]]
deps = ["ColorTypes", "DataStructures", "DocStringExtensions", "FileIO", "FixedPointNumbers", "IndirectArrays", "Inflate", "Mmap", "OffsetArrays", "PkgVersion", "ProgressMeter", "UUIDs"]
git-tree-sha1 = "34cc045dd0aaa59b8bbe86c644679bc57f1d5bd0"
uuid = "731e570b-9d59-4bfa-96dc-6df516fadf69"
version = "0.6.8"

[[deps.TranscodingStreams]]
deps = ["Random", "Test"]
git-tree-sha1 = "9a6ae7ed916312b41236fcef7e0af564ef934769"
uuid = "3bb67fe8-82b1-5028-8e26-92a6c54297fa"
version = "0.9.13"

[[deps.Transducers]]
deps = ["Adapt", "ArgCheck", "BangBang", "Baselet", "CompositionsBase", "ConstructionBase", "DefineSingletons", "Distributed", "InitialValues", "Logging", "Markdown", "MicroCollections", "Requires", "Setfield", "SplittablesBase", "Tables"]
git-tree-sha1 = "53bd5978b182fa7c57577bdb452c35e5b4fb73a5"
uuid = "28d57a85-8fef-5791-bfe6-a80928e7c999"
version = "0.4.78"

    [deps.Transducers.extensions]
    TransducersBlockArraysExt = "BlockArrays"
    TransducersDataFramesExt = "DataFrames"
    TransducersLazyArraysExt = "LazyArrays"
    TransducersOnlineStatsBaseExt = "OnlineStatsBase"
    TransducersReferenceablesExt = "Referenceables"

    [deps.Transducers.weakdeps]
    BlockArrays = "8e7c35d0-a365-5155-bbbb-fb81a777f24e"
    DataFrames = "a93c6f00-e57d-5684-b7b6-d8193f3e46c0"
    LazyArrays = "5078a376-72f3-5289-bfd5-ec5146d43c02"
    OnlineStatsBase = "925886fa-5bf2-5e8e-b522-a9147a512338"
    Referenceables = "42d2dcc6-99eb-4e98-b66c-637b7d73030e"

[[deps.Tricks]]
git-tree-sha1 = "eae1bb484cd63b36999ee58be2de6c178105112f"
uuid = "410a4b4d-49e4-4fbc-ab6d-cb71b17b3775"
version = "0.1.8"

[[deps.TriplotBase]]
git-tree-sha1 = "4d4ed7f294cda19382ff7de4c137d24d16adc89b"
uuid = "981d1d27-644d-49a2-9326-4793e63143c3"
version = "0.1.0"

[[deps.TupleTools]]
git-tree-sha1 = "155515ed4c4236db30049ac1495e2969cc06be9d"
uuid = "9d95972d-f1c8-5527-a6e0-b4b365fa01f6"
version = "1.4.3"

[[deps.URIs]]
git-tree-sha1 = "b7a5e99f24892b6824a954199a45e9ffcc1c70f0"
uuid = "5c2747f8-b7ea-4ff2-ba2e-563bfd36b1d4"
version = "1.5.0"

[[deps.UUIDs]]
deps = ["Random", "SHA"]
uuid = "cf7118a7-6976-5b1a-9a39-7adc72f591a4"

[[deps.UnPack]]
git-tree-sha1 = "387c1f73762231e86e0c9c5443ce3b4a0a9a0c2b"
uuid = "3a884ed6-31ef-47d7-9d2a-63182c4928ed"
version = "1.0.2"

[[deps.Unicode]]
uuid = "4ec0a83e-493e-50e2-b9ac-8f72acf5a8f5"

[[deps.UnicodeFun]]
deps = ["REPL"]
git-tree-sha1 = "53915e50200959667e78a92a418594b428dffddf"
uuid = "1cfade01-22cf-5700-b092-accc4b62d6e1"
version = "0.4.1"

[[deps.VectorizationBase]]
deps = ["ArrayInterface", "CPUSummary", "HostCPUFeatures", "IfElse", "LayoutPointers", "Libdl", "LinearAlgebra", "SIMDTypes", "Static", "StaticArrayInterface"]
git-tree-sha1 = "b182207d4af54ac64cbc71797765068fdeff475d"
uuid = "3d5dd08c-fd9d-11e8-17fa-ed2836048c2f"
version = "0.21.64"

[[deps.WeakRefStrings]]
deps = ["DataAPI", "InlineStrings", "Parsers"]
git-tree-sha1 = "b1be2855ed9ed8eac54e5caff2afcdb442d52c23"
uuid = "ea10d353-3f73-51f8-a26c-33c1cb351aa5"
version = "1.4.2"

[[deps.WoodburyMatrices]]
deps = ["LinearAlgebra", "SparseArrays"]
git-tree-sha1 = "de67fa59e33ad156a590055375a30b23c40299d3"
uuid = "efce3f68-66dc-5838-9240-27a6d6f5f9b6"
version = "0.5.5"

[[deps.WorkerUtilities]]
git-tree-sha1 = "cd1659ba0d57b71a464a29e64dbc67cfe83d54e7"
uuid = "76eceee3-57b5-4d4a-8e66-0e911cebbf60"
version = "1.6.1"

[[deps.XML2_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Libiconv_jll", "Zlib_jll"]
git-tree-sha1 = "24b81b59bd35b3c42ab84fa589086e19be919916"
uuid = "02c8fc9c-b97f-50b9-bbe4-9be30ff0a78a"
version = "2.11.5+0"

[[deps.XSLT_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Libgcrypt_jll", "Libgpg_error_jll", "Libiconv_jll", "Pkg", "XML2_jll", "Zlib_jll"]
git-tree-sha1 = "91844873c4085240b95e795f692c4cec4d805f8a"
uuid = "aed1982a-8fda-507f-9586-7b0439959a61"
version = "1.1.34+0"

[[deps.Xorg_libX11_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Xorg_libxcb_jll", "Xorg_xtrans_jll"]
git-tree-sha1 = "afead5aba5aa507ad5a3bf01f58f82c8d1403495"
uuid = "4f6342f7-b3d2-589e-9d20-edeb45f2b2bc"
version = "1.8.6+0"

[[deps.Xorg_libXau_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "6035850dcc70518ca32f012e46015b9beeda49d8"
uuid = "0c0b7dd1-d40b-584c-a123-a41640f87eec"
version = "1.0.11+0"

[[deps.Xorg_libXdmcp_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "34d526d318358a859d7de23da945578e8e8727b7"
uuid = "a3789734-cfe1-5b06-b2d0-1dd0d9d62d05"
version = "1.1.4+0"

[[deps.Xorg_libXext_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg", "Xorg_libX11_jll"]
git-tree-sha1 = "b7c0aa8c376b31e4852b360222848637f481f8c3"
uuid = "1082639a-0dae-5f34-9b06-72781eeb8cb3"
version = "1.3.4+4"

[[deps.Xorg_libXrender_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg", "Xorg_libX11_jll"]
git-tree-sha1 = "19560f30fd49f4d4efbe7002a1037f8c43d43b96"
uuid = "ea2f1a96-1ddc-540d-b46f-429655e07cfa"
version = "0.9.10+4"

[[deps.Xorg_libpthread_stubs_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "8fdda4c692503d44d04a0603d9ac0982054635f9"
uuid = "14d82f49-176c-5ed1-bb49-ad3f5cbd8c74"
version = "0.1.1+0"

[[deps.Xorg_libxcb_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "XSLT_jll", "Xorg_libXau_jll", "Xorg_libXdmcp_jll", "Xorg_libpthread_stubs_jll"]
git-tree-sha1 = "b4bfde5d5b652e22b9c790ad00af08b6d042b97d"
uuid = "c7cfdc94-dc32-55de-ac96-5a1b8d977c5b"
version = "1.15.0+0"

[[deps.Xorg_xtrans_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl"]
git-tree-sha1 = "e92a1a012a10506618f10b7047e478403a046c77"
uuid = "c5fb5394-a638-5e4d-96e5-b29de1b5cf10"
version = "1.5.0+0"

[[deps.Zlib_jll]]
deps = ["Libdl"]
uuid = "83775a58-1f1d-513f-b197-d71354ab007a"
version = "1.2.13+0"

[[deps.isoband_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "51b5eeb3f98367157a7a12a1fb0aa5328946c03c"
uuid = "9a68df92-36a6-505f-a73e-abb412b6bfb4"
version = "0.2.3+0"

[[deps.libaom_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "3a2ea60308f0996d26f1e5354e10c24e9ef905d4"
uuid = "a4ae2306-e953-59d6-aa16-d00cac43593b"
version = "3.4.0+0"

[[deps.libass_jll]]
deps = ["Artifacts", "Bzip2_jll", "FreeType2_jll", "FriBidi_jll", "HarfBuzz_jll", "JLLWrappers", "Libdl", "Pkg", "Zlib_jll"]
git-tree-sha1 = "5982a94fcba20f02f42ace44b9894ee2b140fe47"
uuid = "0ac62f75-1d6f-5e53-bd7c-93b484bb37c0"
version = "0.15.1+0"

[[deps.libblastrampoline_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "8e850b90-86db-534c-a0d3-1478176c7d93"
version = "5.8.0+0"

[[deps.libfdk_aac_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "daacc84a041563f965be61859a36e17c4e4fcd55"
uuid = "f638f0a6-7fb0-5443-88ba-1cc74229b280"
version = "2.0.2+0"

[[deps.libpng_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg", "Zlib_jll"]
git-tree-sha1 = "94d180a6d2b5e55e447e2d27a29ed04fe79eb30c"
uuid = "b53b4c65-9356-5827-b1ea-8c7a1a84506f"
version = "1.6.38+0"

[[deps.libsixel_jll]]
deps = ["Artifacts", "JLLWrappers", "JpegTurbo_jll", "Libdl", "Pkg", "libpng_jll"]
git-tree-sha1 = "d4f63314c8aa1e48cd22aa0c17ed76cd1ae48c3c"
uuid = "075b6546-f08a-558a-be8f-8157d0f608a5"
version = "1.10.3+0"

[[deps.libvorbis_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Ogg_jll", "Pkg"]
git-tree-sha1 = "b910cb81ef3fe6e78bf6acee440bda86fd6ae00c"
uuid = "f27f6e37-5d2b-51aa-960f-b287f2bc3b7a"
version = "1.3.7+1"

[[deps.nghttp2_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "8e850ede-7688-5339-a07c-302acd2aaf8d"
version = "1.48.0+0"

[[deps.p7zip_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "3f19e933-33d8-53b3-aaab-bd5110c3b7a0"
version = "17.4.0+0"

[[deps.x264_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "4fea590b89e6ec504593146bf8b988b2c00922b2"
uuid = "1270edf5-f2f9-52d2-97e9-ab00b5d0237a"
version = "2021.5.5+0"

[[deps.x265_jll]]
deps = ["Artifacts", "JLLWrappers", "Libdl", "Pkg"]
git-tree-sha1 = "ee567a171cce03570d77ad3a43e90218e38937a9"
uuid = "dfaa095f-4041-5dcd-9319-2fabd8486b76"
version = "3.5.0+0"
"""

# ╔═╡ Cell order:
# ╟─b1a2a660-6644-11ee-2f12-2d062a616500
# ╠═a1d37aac-69cb-49a3-9ec0-6fcc8f71ab0e
# ╟─4dbe0da7-fab9-40e9-b394-1f81b60db7c4
# ╠═8e68fbfe-f2eb-43da-bc7f-ed0ad0cca50d
# ╠═a727d158-533c-4d36-9755-96b7d197b2a9
# ╟─55c209c3-1a18-4b81-a186-fe27e9d78117
# ╠═b5a2fe3f-0339-42e8-a82a-cc0497fe1993
# ╟─8396f44a-ac96-44f1-b817-a0c7fb6359f7
# ╠═ea9ba18e-45b8-4255-b00b-a2a455b7490a
# ╠═dd01cf89-1ddb-46eb-9a50-6411e78bd0d3
# ╟─64ded6c5-a60c-4a12-8c0c-915a88e06d79
# ╠═b2553cd7-cf73-4538-b04c-6f65652f65ce
# ╟─b730549a-4254-4bfc-8110-b85a92bc6cfb
# ╠═11f4797c-5f92-47cc-b9a7-9d3e6bcfffdb
# ╟─9d6c011d-4247-4542-b92a-275d310855c3
# ╠═1e9f72d9-74c7-42ec-af1b-6e1915ed2422
# ╠═2a0cfd4b-a89e-4789-af54-acb3c9f439ae
# ╟─40a41c9a-0a56-4c33-bd1c-0ec4644cbba1
# ╠═1b69b3a0-5f90-4332-8621-9f95dea33e49
# ╟─594dafeb-ffca-4567-b86e-b39005c5b30f
# ╠═bf429452-a2c3-4302-82ec-ba5c213e10fd
# ╟─2f44cf6b-a2e0-4aaa-9ab1-69f94bea2fb2
# ╠═acf9a20a-3de9-4117-b5e5-07e3c1bf4692
# ╟─c7eadfbf-753e-4cf8-b76b-8c98a64613af
# ╠═46e31cca-ad35-4b12-b52f-0d0639cd4b36
# ╟─2d1c69e5-18f0-4644-a3d0-d8d1e870d6bd
# ╠═49ad2981-5e41-4aec-8e07-313fdaa1ac42
# ╟─a442fbad-a7f3-4913-8dc7-73c92d98da5c
# ╠═b9dd4fde-d817-49af-9df3-11ed654e10ac
# ╟─3df216ca-3051-4540-96bb-64492867b26a
# ╠═db85bf7d-d1fc-4e4f-af9c-0a352e4af677
# ╟─312f4d1c-7e24-457a-a924-77403cbc1651
# ╠═4278d117-1327-4ec4-9591-ebdb2b0784fd
# ╟─be8760c4-002a-498a-a277-c3f056022694
# ╠═e251de97-82f3-4177-9b6f-762420109e9b
# ╟─ab9f9a1a-3494-45c6-af56-71429230301e
# ╠═956a1e74-b919-4f89-adac-9085c06fc33b
# ╟─059cd603-f14d-4746-b4e6-68a674ea0df4
# ╠═0498478c-e8bc-46af-bbcd-cd7bac5f122e
# ╟─48706b5a-851b-4a36-b391-52f51c7870c4
# ╠═ce0fe42c-2e59-4afa-94d2-79b4bc117594
# ╟─7c917b41-290f-4913-83dd-ddc3c7703f64
# ╠═f59ea9f7-182a-4bab-918c-a339515c0b6c
# ╟─597c35a6-4be9-48b0-949b-f96e233da32e
# ╠═b36f7121-733a-48f7-bfb6-70a0bfc8c846
# ╟─ae9324c0-ea3e-4d77-9e81-9b60fb49f675
# ╠═fe7e662a-b83f-45d6-80cc-690cedc85a17
# ╟─dfe46561-f95e-47e4-99c6-facfc9c19092
# ╠═2fcdd072-d836-42f2-a455-03e412aac028
# ╟─02278217-cec8-4a4b-8477-d072ebccca14
# ╠═10e668e3-ac76-4f9d-b923-7b8f94851dee
# ╟─9e66d2d2-b0e9-4f2b-b106-8a9e8bb33349
# ╠═eef578ff-0330-4e4a-afcb-53a70d15a65b
# ╟─b66f9b2f-f81c-4269-9162-593ed88a2251
# ╠═0678ff0f-804d-4334-a967-0c5e966fde7f
# ╟─fa55f9de-deab-4746-89c2-e7b441ccf0b1
# ╟─1ccf5425-7c7a-4b3a-bdc7-2c5caa12feba
# ╠═f06e9cab-4f28-419a-a9e8-6ab79e3b744a
# ╟─0709734f-8045-4774-b627-df6924cf2e1d
# ╠═9c35d537-f799-4acc-a363-bb04173f79af
# ╠═838c17ed-3d1c-4478-b188-6ddd4f2c8868
# ╠═459435a3-da56-4a4b-bde1-9d0a26b3e5a0
# ╟─242bbb4e-3728-49cd-94b5-cd5de457b04e
# ╠═2e207ffd-edd4-4905-a31d-78bb695ab5e0
# ╟─6a4db54d-8bcf-47ee-96b4-ea008de7ee10
# ╠═2755a82e-b257-455f-9f41-4f9bc54ab297
# ╠═d19a0244-7609-450d-bd3d-4d3148132ea2
# ╟─d93962bf-bfdb-4a14-ad5b-9de2e3fbe28c
# ╠═88d8b521-fa0b-4a97-9d08-04963a708c94
# ╠═fefe14d7-bb22-4b39-ae61-a3482704c444
# ╟─1d94b500-af26-4827-8b35-b788d331609a
# ╠═7dc3427e-e31b-4257-a11f-55efed015e53
# ╠═31f52a92-d658-41e9-9112-f856264ab29f
# ╠═a295ecd2-3556-4806-9c9a-0776504b6418
# ╠═0d56904c-c12c-4968-8564-6d4bff492e08
# ╟─cd9b2d47-4f12-49b0-b1e2-388af388638d
# ╠═4a8304e7-e4e6-4f01-a76d-077700fc46ec
# ╠═de5e6cc1-3314-45c9-ab7f-afbda265e595
# ╠═fd671a3e-9222-4d56-952f-e034b41fe201
# ╟─6f348bd5-ab5b-445b-b9f9-22947046db91
# ╠═3ccd2e6c-1f77-4193-a1d5-05c289e11f4d
# ╟─1f95be8e-2092-4327-b8d7-afe723f3f558
# ╠═b9e699ad-446d-4b77-bcab-628e0ed286ca
# ╟─2daa1227-200a-4bab-8e45-116a59f87ce4
# ╠═2b5423c4-1870-443f-adce-4a29e7eb9795
# ╠═e0b07136-0fbd-48d8-9865-aede4b7e1d78
# ╠═2197e493-5b52-44c2-b207-aad2518f9230
# ╠═07c6c448-4b02-4090-9206-d3c4850ecc38
# ╠═82808b10-19f7-4900-8fe3-5e16345de4e7
# ╟─ce567663-7d7e-4a92-87e6-d3b2a97d5e39
# ╠═863a6d24-9640-42e4-a9cd-ae40f3c91ac6
# ╟─ab098d20-af70-4cb6-9497-32eb5ad972cd
# ╟─7d01bccd-c209-4407-a987-2c8ca4058ea3
# ╟─d16a4de6-2e96-40f5-b769-473c4f0d92c6
# ╟─c435e9bd-fdfa-4f99-a082-670cce387a9e
# ╠═38206e59-07f9-45d3-beb9-aca653925397
# ╟─3abafd5c-9c5f-4044-9e25-fadc1639f5be
# ╠═db1bc18b-11d5-4e76-8d17-c2515e33b4af
# ╟─e9fe6f18-e801-4134-bd61-55b75fcf0587
# ╠═4e59422c-7c02-441a-8e1f-d3ef2e4c1b14
# ╠═cb3225f7-465a-4d27-a06e-c9ef2436094d
# ╠═689fcccc-91a5-429b-b08b-d871e34e4b91
# ╟─28c426fc-303a-431d-98ba-3e013c6a1d2a
# ╠═d78eee41-232a-4b6b-a432-0c12e07b134e
# ╟─09c665b6-6548-4c57-b130-cc6dc4a7ad22
# ╠═8d30960d-2c95-4331-a523-c1936e6e50c7
# ╟─00000000-0000-0000-0000-000000000001
# ╟─00000000-0000-0000-0000-000000000002
