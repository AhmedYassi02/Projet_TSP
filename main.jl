# lire les instance depuis"Projet/instances/instance_6_1.txt"

using JuMP, Gurobi
include("dfj_function.jl")
include("mtz_function.jl")
include("utils.jl")


# map between filepaths and instance names
intances_to_paths = Dict(
    "instance_6_1" => "Instances/instance_6_1.txt",
    "instance_20_1" => "Instances/instance_20_1.txt",
    "instance_20_2" => "Instances/instance_20_2.txt",
    "instance_20_3" => "Instances/instance_20_3.txt",
    "instance_30_1" => "Instances/instance_30_1.txt",
    "instance_40_1" => "Instances/instance_40_1.txt",
    "instance_50_1" => "Instances/instance_50_1.txt",
    "instance_70_1" => "Instances/instance_70_1.txt",
    "instance_80_1" => "Instances/instance_80_1.txt",
    "instance_80_2" => "Instances/instance_80_2.txt",
    "instance_100_1" => "Instances/instance_100_1.txt",
)


instance = "instance_80_1"
println("Reading instance from ", intances_to_paths[instance])

nombre_aerodromes, depart, arrivee, nombre_min_aerodromes, nombre_regions, regions, rayon, coordonnees = read_instance(intances_to_paths[instance])

println("nombre_aerodromes = ", nombre_aerodromes)
println("aerodrome depart = ", depart)
println("aerodrome arrivee = ", arrivee)
println("nombre_min_aerodromes = ", nombre_min_aerodromes)
println("nombre_regions = ", nombre_regions)
println("regions de chaque aerodrome = ", regions)
println("rayon max = ", rayon)
println("coordonnees de chaque aerodrome = ", coordonnees)


# calcul des distances entre les aerodromes
distances = zeros(nombre_aerodromes, nombre_aerodromes)
for i in 1:nombre_aerodromes
    for j in 1:nombre_aerodromes
        distances[i, j] = distance(coordonnees[i], coordonnees[j])
    end
end


x = solve_MTZ(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
# print("Sous-tours : ", detect_subtour(x))

# stocker la solution dans un fichier dans "Instances/solutions/solution_"instance".txt"
path_solution = "Instances/solutions/solution_" * instance * ".txt"
f = open(path_solution, "w")
for i in 1:nombre_aerodromes-1
    for j in 1:nombre_aerodromes-1
        write(f, string(x[i, j]) * " ")
    end
    write(f, string(x[i, nombre_aerodromes]) * ";")
end
for j in 1:nombre_aerodromes-1
    write(f, string(x[nombre_aerodromes, j]) * " ")
end
write(f, string(x[nombre_aerodromes, nombre_aerodromes]))
