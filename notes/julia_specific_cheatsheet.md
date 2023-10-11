# Julia Specific Cheatsheet

## Julia performance tips
###  More dots: Fuse vectorized operations, vectrizing is faster than loops
```julia  
f(x) = 3x .^ 2 + 4x + 7x .^ 3;
fdot(x) = @. 3x^2 + 4x + 7x^3; # equivalent to 3 .* x.^2 .+ 4 .* x .+ 7 .* x.^3 but faster

x = rand(10^6);

@time f(x); #slower
@time fdot(x); #faster

@time @. f(x); #just as fast
@time f.(x); #just as fast
```

### Consider using views for slices 
n array "slice" expression like array[1:5, :] creates a copy of that data
An alternative is to create a "view" of the array, which is an array object (a SubArray) that actually references the data of the original array in-place, without making a copy.
```julia
fcopy(x) = sum(x[2:end-1]);
@views fview(x) = sum(x[2:end-1]);

@time fcopy(x); # slower
@time fview(x); # faster
```

## Methods for Timing Julia Code

```julia
@time sum(rand(1000))
@elapsed sum(rand(1000))
@timev sum(rand(1000))
```

## Methods for Benchmarking julia
```julia
using BenchmarkTools
@benchmark sum(rand(1000))
@btime sum(rand(1000))
```

## Methods for Identifying Type Stability in julia
```julia
@code_warntype sum(rand(1000))
Type-Stable vs Not Type-Stable Code
Not type-stable code:
```

## Julia code that is not type-stable
```julia
function simdsum(x)
    s = 0
    @simd for v in x
        s += v
    end
    return s
end
```

## Julia code that is Type-stable
```julia
function simdsum_fixed(x)
    s = zero(eltype(x))
    @simd for v in A
        s += v
    end
    return s
end
```

## Pre-allocating Arrays in Julia
```julia
Vector{Float64}(undef, n)
Vector{Int64}(undef, n)
Matrix{Float64}(undef, m, n)
Matrix{Int64}(undef, m, n)
Array{Float64}(undef, m, n)
Array{Int64}(undef, m, n)
```

## Pre-allocating Arrays with Zeros or Ones
### With zeros:
```julia
zeros(Float64, m, n)
zeros(Int64, m, n)
```

### With ones:
```julia
ones(Float64, m, n)
ones(Int64, m, n)
```

## Pre-allocating a DataFrame
```julia
df = DataFrame(A = fill(undef, 1000), B = fill(undef, 1000), C = fill(undef, 1000))
df = DataFrame(A = Vector{Int}(undef, 1000), B = Vector{Float64}(undef, 1000), C = Vector{String}(undef, 1000))
```

## Checking Memory Allocation
Use @allocated to check how much memory is allocated by a function (this is useful for seeing if you are copying arrays or dataframes unnecessarily)
```julia
@allocated sum(rand(1000))
```

## Creating a Range of Numbers
```julia
range_example = range(0, stop=1, length=20)
println("range_example: ", range_example)
```

## Using Broadcasting with Comparison Operator to Create a BitVector
```julia
bit_vector = range_example .>= 0.5
println("bit_vector: ", bit_vector)
```

## Converting BitVector to Vector of AbstractFloat
```julia
float_vector = convert(Vector{AbstractFloat}, bit_vector)
println("float_vector: ", float_vector)
```

## Using Broadcasting with an Anonymous Function and Ternary Operator
```julia
ternary_vector = (x -> x >= 0.5 ? 1 : 0).(range_example)
println("ternary_vector: ", ternary_vector)
```

## Using Broadcasting with ifelse Function
```julia
ifelse_vector = ifelse.(range_example .>= 0.5, 1, 0)
println("ifelse_vector: ", ifelse_vector)
```

## Using Map Function with an Anonymous Function and Ternary Operator
```julia
map_vector = map(x -> x >= 0.5 ? 1 : 0, range_example)
println("map_vector: ", map_vector)
```

## Identifying Package of a Function
How to find out what package a function is from i.e. The @which macro in Julia returns the method that would be called for a given set of arguments

```julia
@which sample # this should return the source code for the sample function which is StatsBase
```

## Finding Out What Functions a Package Exports
```julia
using ExamplePackage
names(ExamplePackage)
```

## Finding Methods that Accept a Specific Type as an Argument
For instance, if you have a DataFrame and you want to see what functions you can call on it 
(i.e., what functions accept a DataFrame as an argument), you can use methodswith(DataFrame) 
to get a list of such functions. This can be a quick way to explore the functionality available for a type.

```julia
using DataFrames
methodswith(DataFrame)
```

# Generator Expressions in Julia
Generator expressions are a high-performance, memory-efficient way to create and transform sequences.
They are similar to array comprehensions, but don't allocate an intermediate array, making them more memory-efficient.
Use them when working with large datasets or when performing simple transformations on a sequence.

## Example: Creating an array with the squares of numbers 1 to 5
```julia
squares = collect(x^2 for x in 1:5)
```

## Example: Summing the squares of numbers 1 to 5 without creating an intermediate array
```julia
sum_of_squares = sum(x^2 for x in 1:5)
```

## Using Generator Expressions to Create Arrays
### 1D array:
```julia
j = collect(2x for x = 1:5)
println("1D array: ", j)
```

### 2D array:
```julia
u = collect(x + 2y for x in 1:5, y in 0:1)
println("2D array: ", u)
```

### 3D array:
```julia
p = collect(x + 2y + 3z for x in 1:4, y in 0:1, z in 1:3)
println("3D array: ", p)
```

## How to Multiply Vectors Element-Wise in Julia

### Define your vectors:

```julia
vector1 = [1, 2, 3]
vector2 = [4, 5, 6]
vector3 = [7, 8, 9]
```

### Method 1: Using the .*
The .* operator performs element-wise multiplication on arrays

```julia
result1 = vector1 .* vector2 .* vector3
println("Method 1 result: ", result1)
```

### Method 2: Using map()
The map() function applies a function to every element in the arrays

```julia
result2 = map(*, vector1, vector2, vector3)
println("Method 2 result: ", result2)
```

### Method 3: Using list comprehension
List comprehension is a compact way to create arrays based on existing arrays

```julia
result3 = [a*b*c for (a, b, c) in zip(vector1, vector2, vector3)]
println("Method 3 result: ", result3)
```

## How to Multiply Matrices Element-Wise in Julia

## Define your matrix:
```julia
matrix = [1 2 3; 4 5 6; 7 8 9]
```

### Method 1: Using the .*
The .* operator performs element-wise multiplication on arrays

```julia
result1 = prod(matrix, dims=2)
println("Method 1 result: ", result1)
```

### Method 2: Using map() and reduce()
The map() function applies a function to every element in the arrays
The reduce() function applies a binary operation to all elements in an array

```julia
result2 = mapreduce(*, matrix, dims=2)
println("Method 2 result: ", result2)
```

### Method 3: Using list comprehension
List comprehension is a compact way to create arrays based on existing arrays

```julia
result3 = [prod(row) for row in eachrow(matrix)]
println("Method 3 result: ", result3)
```

### Method 4: Using mapslices()
The mapslices() function applies a function to slices of an array along a specified dimension

```julia
result4 = mapslices(prod, matrix, dims=2)
println("Method 4 result: ", result4)
```

## Countmap in Julia
The countmap() function returns a dictionary mapping each unique value in an array to the number of times it appears in the array.

```julia
using StatsBase
flips = rand(["Heads", "Tails"], 100)
counts = countmap(flips)
println("counts: ", counts)
```
returns
```julia
counts: Dict{String,Int64} with 2 entries:
  "Tails" => 51
  "Heads" => 49
```

## Piping options in Julia
The pipe operator |> is used to pipe the result of an expression into the first argument of a function call.
This can be useful for chaining together multiple function calls.

```julia
(sqrt ∘ +)(2, 3) # equivalent to sqrt(+(2, 3))

2 |> +(3) |> sqrt # equivalent to sqrt(+(2, 3))

1:10 |> sum |> sqrt # equivalent to sqrt(sum(1:10))
```
## Piping with broadcasting

```julia
["a", "list", "of", "strings"] .|> [uppercase, reverse, titlecase, length]
```
returns
```julia
4-element Vector{Any}:
 "A"
 "tsil"
 "Of"
 7
```
When combining pipes with anonymous functions, parentheses must be used if subsequent
pipes are not to parsed as part of the anonymous function's body. Compare:
```julia
1:3 .|> (x -> x^2) |> sum |> sqrt
```
returns
```julia
3.7416573867739413
```

```julia
1:3 .|> x -> x^2 |> sum |> sqrt
```
returns
```julia
3-element Vector{Float64}:
    1.0
    2.0
    3.0
```

# How to create a basic plot using CairoMakie or GLMakie in Julia
```julia
using CairoMakie
f = Figure()
ax = Axis(f[1, 1],
    xlabel = "x",
    ylabel = "y",
    title = "Plot Title")

x = 1:10
y = rand(10)

lines!(ax, x, y, color = :red)
scatter!(ax, x, y, color = :blue)
```

# How to add jitter(random noise to numbers) for a scatter plot in Makie
## Create a jitter function
```julia
# define the jitter function as is done in R programing language
# this funtion is simply adding noise to numbers
function jitter(x)
    z = findmax(collect(skipmissing(x)))[1] - findmin(collect(skipmissing(x)))[1]
    a = z/50
    if a == 0
        x = x .+ rand(length(x))
        return x
    else
        x = x .+ rand(Uniform(-a, a), length(x))
        return x
    end
end
```
## Use the jitter function with a vector of data in Makie
```julia
# define a vector of data
sample = rand(1000)

# using the jitter function in a scatter plot with the vector of data
f = Figure()
ax = Axis(f[1, 1]; xlabel="x", ylabel="y")
scatter!(jitter(sample); alpha=0.2, markersize=10)
ax = Axis(f[1, 2]; xlabel="x", ylabel="y")
density!(jitter(sample); color=(:lightblue, 0.3), strokecolor=:skyblue, strokewidth = 3, strokearound = false)
f
```





