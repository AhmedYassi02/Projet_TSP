# lire les instance depuis"Projet/instances/instance_6_1.txt"

using JuMP, Gurobi
include("dfj_function.jl")
include("mtz_function.jl")
include("rtl_function.jl")
include("utils.jl")


# map between filepaths and instance names



instance = "instance_6_1"
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


x = solve_RTL(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
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
