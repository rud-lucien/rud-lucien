# julia_specific_cheatsheet

# Methods for timing Julia code
@time sum(rand(1000))
@elapsed sum(rand(1000))
@timev sum(rand(1000))

# Benchmarking
using BenchmarkTools
@benchmark sum(rand(1000))
@btime sum(rand(1000))

# Identifying type stability
@code_warntype sum(rand(1000))


# not type-stable code
function simdsum(x)
    s = 0
    @simd for v in x
        s += v
    end
    return s
end

# type-stable code
function simdsum_fixed(x)
    s = zero(eltype(x))
    @simd for v in A
        s += v
    end
    return s
end

# Pre-allocating arrays
Vector{Float64}(undef, n)
Vector{Int64}(undef, n)
Matrix{Float64}(undef, m, n)
Matrix{Int64}(undef, m, n)
Array{Float64}(undef, m, n)
Array{Int64}(undef, m, n)

# Pre-allocating arrays with zeros
zeros(Float64, m, n)
zeros(Int64, m, n)

# Pre-allocating arrays with ones
ones(Float64, m, n)
ones(Int64, m, n)

# Preallocate a DataFrame with 1000 rows and 3 columns
df = DataFrame(A = fill(undef, 1000), B = fill(undef, 1000), C = fill(undef, 1000))
df = DataFrame(A = Vector{Int}(undef, 1000), B = Vector{Float64}(undef, 1000), C = Vector{String}(undef, 1000))

# Use @allocated to check how much memory is allocated by a function (this is useful for seeing if you are copying arrays or dataframes unnecessarily)
@allocated sum(rand(1000))

# Creating a range of numbers
range_example = range(0, stop=1, length=20)
println("range_example: ", range_example)

# Using broadcasting with a comparison operator to create a BitVector
bit_vector = range_example .>= 0.5
println("bit_vector: ", bit_vector)

# Converting BitVector to Vector of AbstractFloat
float_vector = convert(Vector{AbstractFloat}, bit_vector)
println("float_vector: ", float_vector)

# Using broadcasting with an anonymous function and ternary operator
ternary_vector = (x -> x >= 0.5 ? 1 : 0).(range_example)
println("ternary_vector: ", ternary_vector)

# Using broadcasting with ifelse function
ifelse_vector = ifelse.(range_example .>= 0.5, 1, 0)
println("ifelse_vector: ", ifelse_vector)

# Using map function with an anonymous function and ternary operator
map_vector = map(x -> x >= 0.5 ? 1 : 0, range_example)
println("map_vector: ", map_vector)