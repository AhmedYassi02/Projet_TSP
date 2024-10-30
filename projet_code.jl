# lire les instance depuis"Projet/instances/instance_6_1.txt"

using JuMP, Gurobi
# include("dfj_function.jl")
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


instance = "instance_6_1"
println("Reading instance from ", intances_to_paths[instance])

function lire_ligne_non_vide(f)
    ligne = readline(f)
    while isempty(strip(ligne)) && !eof(f)
        ligne = readline(f)
    end
    return ligne
end

function read_instance(file_name)
    f = open(file_name)

    nombre_aerodromes = parse(Int, lire_ligne_non_vide(f))

    depart = parse(Int, lire_ligne_non_vide(f))

    arrivee = parse(Int, lire_ligne_non_vide(f))

    nombre_min_aerodromes = parse(Int, lire_ligne_non_vide(f))

    nombre_regions = parse(Int, lire_ligne_non_vide(f))

    regions_line = lire_ligne_non_vide(f)
    regions = [parse(Int, x) for x in split(regions_line)]

    rayon = parse(Int, lire_ligne_non_vide(f))

    coordonnees = [[parse(Int, x) for x in split(lire_ligne_non_vide(f))] for i in 1:nombre_aerodromes]

    close(f)
    return nombre_aerodromes, depart, arrivee, nombre_min_aerodromes, nombre_regions, regions, rayon, coordonnees
end


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

function distance(a, b)
    return sqrt((a[1] - b[1])^2 + (a[2] - b[2])^2)
end

distances = zeros(nombre_aerodromes, nombre_aerodromes)

for i in 1:nombre_aerodromes
    for j in 1:nombre_aerodromes
        distances[i, j] = distance(coordonnees[i], coordonnees[j])
    end
end


function find_subtours(sol)
    """
    Function to find subtours in a solution of the Traveling Salesman Problem (TSP).

    Parameters:
        sol (Dict{Tuple{Int, Int}, Float64}): Solution dictionary representing the TSP solution.
            The keys are tuples (i, j) representing edges, and the values are binary variables
            indicating if the edge is selected in the solution.

    Returns:
        subtours (Vector{Vector{Int}}): List of subtours found in the solution.
    """

    subtours = []  # List to store subtours
    remaining_nodes = collect(1:length(sol))  # List of remaining nodes to be visited

    while !isempty(remaining_nodes)
        subtour = []  # List to store a subtour
        push!(subtours, subtour)  # Add the subtour to the list of subtours

        start = remaining_nodes[1]  # Start the subtour from the first remaining node

        while true
            push!(subtour, start)  # Add the current node to the subtour

            if start in remaining_nodes
                deleteat!(remaining_nodes, findfirst(==(start), remaining_nodes))  # Remove the visited node
            end

            # Find the next node in the solution that is connected to the current node and has not been visited yet
            next_node = findfirst(x -> x[1] == start && sol[x] > 0.5, keys(sol))
            next_node = next_node !== nothing ? next_node[2] : nothing

            if next_node === nothing || !(next_node in remaining_nodes)
                break  # If there is no next node or the next node has already been visited, end the subtour
            end

            start = next_node  # Move to the next node in the subtour
        end
    end

    return subtours
end

function solve_DFJ(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
    model = Model(Gurobi.Optimizer)
    set_silent(model)
    # Variables : x[i,j] = 1 si l'arc (i, j) est dans le chemin, 0 sinon
    @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)
    # Fonction objectif : minimiser la distance totale
    @objective(model, Min, sum(distances[i, j] * x[i, j] for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes))

    # Contrainte de rayon maximal
    for i in 1:nombre_aerodromes
        for j in 1:nombre_aerodromes
            if distances[i, j] > rayon
                @constraint(model, x[i, j] == 0)
            end
        end
    end

    # conservation du nombre d'arcs entrants et sortants sauf pour le depart et l'arrivee
    for i in 1:nombre_aerodromes
        if i != depart && i != arrivee
            @constraint(model, sum(x[j, i] for j in 1:nombre_aerodromes if j != i) - sum(x[i, j] for j in 1:nombre_aerodromes if j != i) == 0)
        elseif i == depart
            @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) - sum(x[j, i] for j in 1:nombre_aerodromes if j != i) == 1)
        elseif i == arrivee
            @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) - sum(x[j, i] for j in 1:nombre_aerodromes if j != i) == -1)
        end

    end

    # Contrainte de nombre minimal d'aérodromes à visiter
    @constraint(model, sum(sum(x[i, j] for j in 1:nombre_aerodromes if j != i) for i in 1:nombre_aerodromes) >= nombre_min_aerodromes)

    # Contrainte de régions
    for r in 0:nombre_regions
        @constraint(model, sum(x[i, j] for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes if regions[i] == r || regions[j] == r) >= 1)
    end

    for i in 1:nombre_aerodromes
        @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) <= 1)
    end

    # on peut pas aller de i a i
    for i in 1:nombre_aerodromes
        @constraint(model, x[i, i] == 0)
    end
    # on peut pas aller de arrivee a depart
    @constraint(model, x[arrivee, depart] == 0)
    @constraint(model, x[depart, arrivee] == 0)



    # Résolution initiale sans contrainte d'élimination de sous-tours
    optimize!(model)
    while true
        # Obtenez les valeurs actuelles de la solution
        x_values = value.(x)
        # Détecte un sous-tour dans la solution actuelle
        subtour = find_subtours(x_values)
        if subtour === nothing
            println("Solution optimale trouvée sans sous-tours !")
            break
        else
            println("Sous-tour détecté : ", subtour)
            # Ajoute une contrainte pour éliminer ce sous-tour
            @constraint(model, sum(x[i, j] for i in subtour, j in subtour if i != j) <= length(subtour) - 1)
            # Réoptimise avec la nouvelle contrainte
            optimize!(model)
        end
    end
    # Affiche la solution finale
    println("Distance minimale parcourue : ", objective_value(model))
    # renvoie la solution
    return value.(x)
end
function solve_MTZ(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
    model = Model(Gurobi.Optimizer)
    set_silent(model)

    # Variables : x[i,j] = 1 si l'arc (i, j) est dans le chemin, 0 sinon
    @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)

    # MTZ auxiliary variables: u[i] represents the order of each node i in the path
    @variable(model, u[1:nombre_aerodromes] >= 0, Int)

    # Objective function: minimize the total distance
    @objective(model, Min, sum(distances[i, j] * x[i, j] for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes))

    # conservation du nombre d'arcs entrants et sortants sauf pour le depart et l'arrivee
    for i in 1:nombre_aerodromes
        if i != depart && i != arrivee
            @constraint(model, sum(x[j, i] for j in 1:nombre_aerodromes if j != i) - sum(x[i, j] for j in 1:nombre_aerodromes if j != i) == 0)
        elseif i == depart
            @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) - sum(x[j, i] for j in 1:nombre_aerodromes if j != i) == 1)
        elseif i == arrivee
            @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) - sum(x[j, i] for j in 1:nombre_aerodromes if j != i) == -1)
        end
    end


    # on peut pas aller de arrivee a depart
    @constraint(model, x[arrivee, depart] == 0)
    @constraint(model, x[depart, arrivee] == 0)

    # on peut pas aller de i a i
    for i in 1:nombre_aerodromes
        @constraint(model, x[i, i] == 0)
    end

    # Contrainte de nombre minimal d'aérodromes à visiter
    @constraint(model, sum(sum(x[i, j] for j in 1:nombre_aerodromes if j != i) for i in 1:nombre_aerodromes) >= nombre_min_aerodromes)


    # Contrainte des régions
    for r in 0:nombre_regions
        @constraint(model, sum(x[i, j] for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes if regions[i] == r || regions[j] == r) >= 1)
    end

    # Contrainte de rayon maximal
    for i in 1:nombre_aerodromes
        for j in 1:nombre_aerodromes
            if distances[i, j] > rayon
                @constraint(model, x[i, j] == 0)
            end
        end
    end

    # MTZ methode pour eliminer les sous-tours
    for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes
        if i != j && i != depart && j != depart && i != arrivee && j != arrivee
            @constraint(model, u[i] - u[j] + 1 <= (nombre_aerodromes - 1) * (1 - x[i, j]))
        end
    end
    # on visite un aerodrome une seule fois
    for i in 1:nombre_aerodromes
        @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) <= 1)
    end

    optimize!(model)
    println("Minimum distance traveled: ", objective_value(model))

    return value.(x)
end

x = solve_DFJ(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
# print("Sous-tours : ", detect_subtour(x))

# stocker la solution dans un fichier dans "Instances/solutions/solution_"instance".txt"
path_solution = "Instances/solutions/solution_" * instance * ".txt"
f = open(path_solution, "w")
for i in 1:nombre_aerodromes
    for j in 1:nombre_aerodromes
        write(f, string(x[i, j]) * " ")

    end
end
