# lire les instance depuis"Projet/instances/instance_6_1.txt"

using JuMP, Gurobi
include("dfj_function.jl")
include("mtz_function.jl")
include("rlt_function.jl")
include("gcs_function.jl")
include("sf_function.jl")
include("utils.jl")

relax = false
# choisir une instance
instance = "instance_100_1"
println("Reading instance from ", intances_to_paths[instance])

#  choisir une Fonction de résolution
methode = "RLT"
println("Using ", methode, " method")
formulation = methode == "RLT" ? solve_RTL : methode == "MTZ" ? solve_MTZ : methode == "DFJ" ? solve_DFJ : methode == "GCS" ? solve_GCS : methode == "SF" ? solve_SF : error("Methode inconnue")


nombre_aerodromes, depart, arrivee, nombre_min_aerodromes, nombre_regions, regions, rayon, coordonnees = read_instance(intances_to_paths[instance])

println("nombre_aerodromes = ", nombre_aerodromes)
println("aerodrome depart = ", depart)
println("aerodrome arrivee = ", arrivee)
println("nombre_min_aerodromes = ", nombre_min_aerodromes)
println("nombre_regions = ", nombre_regions)
println("regions de chaque aerodrome = ", regions)
println("rayon max = ", rayon)
println("coordonnees de chaque aerodrome = ", coordonnees)


# calcul des distances entre les aerodromes (matrice des distances)
distances = zeros(nombre_aerodromes, nombre_aerodromes)
for i in 1:nombre_aerodromes
    for j in 1:nombre_aerodromes
        distances[i, j] = distance(coordonnees[i], coordonnees[j])
    end
end


model = formulation(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon, relax)

x = value.(model[:x])
objective = objective_value(model)
# mip_gap = MOI.get(model, MOI.MIPGap())
solving_time = solve_time(model)

# stocker la solution dans un fichier
path_solution = "Instances/solutions/solution_" * instance * "_" * methode * ".txt"
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

# write model performance, time, gap to relaxed solution, ...
write(f, "\n")
write(f, "Objective value: " * string(objective) * "\n")
# write(f, "MIP gap: " * string(mip_gap) * "\n")
write(f, "Solving time: " * string(solving_time) * "s\n")


close(f)
