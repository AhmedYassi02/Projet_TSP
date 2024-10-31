
using JuMP, Gurobi

function find_subtours(sol)

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


function detect_subtour(x_values)
    visited = falses(nombre_aerodromes)
    for i in 1:nombre_aerodromes
        if !visited[i]
            current = i
            path = []
            while !visited[current]
                push!(path, current)
                visited[current] = true
                # Trouver le prochain noeud
                current = findfirst(x_values[current, :] .== 1)
                if current === nothing || current == depart || current == arrivee
                    break
                end
            end
            # Retourner le sous-tour si trouvé
            if length(path) > 1 && current != arrivee
                return path
            end
        end
    end
    return nothing
end

function solve_DFJ(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
    model = Model(Gurobi.Optimizer)
    set_silent(model)
    # Variables : x[i,j] = 1 si l'arc (i, j) est dans le chemin, 0 sinon
    @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)
    # Fonction objectif : minimiser la distance totale
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

    # on ne peut pas visiter un aerodrome plus d'une fois
    for i in 1:nombre_aerodromes
        @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) <= 1)
    end

    for j in 1:nombre_aerodromes
        @constraint(model, sum(x[i, j] for i in 1:nombre_aerodromes if i != j) <= 1)
    end

    # on peut pas aller de i a i
    for i in 1:nombre_aerodromes
        @constraint(model, x[i, i] == 0)
    end

    #### Contraintes supplémentaires du problème ####

    # Contrainte de nombre minimal d'aérodromes à visiter
    @constraint(model, sum(sum(x[i, j] for j in 1:nombre_aerodromes if j != i) for i in 1:nombre_aerodromes) >= nombre_min_aerodromes - 1)


    # Contrainte des régions
    for r in 0:nombre_regions
        @constraint(model, sum(x[i, j] for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes if regions[i] == r || regions[j] == r) >= 1)
    end

    # Contrainte de rayon maximal
    for i in 1:nombre_aerodromes
        for j in 1:nombre_aerodromes
            @constraint(model, x[i, j] * distances[i, j] <= rayon)
        end
    end



    # Résolution initiale sans contrainte d'élimination de sous-tours
    optimize!(model)
    while true
        # Obtenez les valeurs actuelles de la solution
        x_values = value.(x)
        # Détecte un sous-tour dans la solution actuelle
        subtour = detect_subtour(x_values)
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

function solve_DFJ_(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
    model = Model(Gurobi.Optimizer)
    set_silent(model)
    # Variables : x[i,j] = 1 si l'arc (i, j) est dans le chemin, 0 sinon
    @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)
    # Fonction objectif : minimiser la distance totale
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

    # on ne peut pas visiter un aerodrome plus d'une fois
    for i in 1:nombre_aerodromes
        @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) <= 1)
    end

    for j in 1:nombre_aerodromes
        @constraint(model, sum(x[i, j] for i in 1:nombre_aerodromes if i != j) <= 1)
    end

    # on peut pas aller de i a i
    for i in 1:nombre_aerodromes
        @constraint(model, x[i, i] == 0)
    end

    #### Contraintes supplémentaires du problème ####

    # Contrainte de nombre minimal d'aérodromes à visiter
    @constraint(model, sum(sum(x[i, j] for j in 1:nombre_aerodromes if j != i) for i in 1:nombre_aerodromes) >= nombre_min_aerodromes - 1)


    # Contrainte des régions
    for r in 0:nombre_regions
        @constraint(model, sum(x[i, j] for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes if regions[i] == r || regions[j] == r) >= 1)
    end

    # Contrainte de rayon maximal
    for i in 1:nombre_aerodromes
        for j in 1:nombre_aerodromes
            @constraint(model, x[i, j] * distances[i, j] <= rayon)
        end
    end


    # Résolution initiale sans contrainte d'élimination de sous-tours
    optimize!(model)
    # Function to convert the solution into a dictionary format
    function solution_to_dict(x_values)
        sol = Dict{Tuple{Int,Int},Float64}()
        for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes
            if i != j && value(x[i, j]) > 0.5  # Consider only active edges
                sol[(i, j)] = value(x[i, j])
            end
        end
        return sol
    end
    # Loop to detect and eliminate subtours
    while true
        # Convert the solution to a dictionary format
        sol = solution_to_dict(x)

        # Find subtours in the current solution
        subtours = find_subtours(sol)

        # If no subtours are found, break the loop
        if length(subtours) == 1
            println("Optimal solution found with no subtours!")
            break
        end

        # Add a constraint for each detected subtour to eliminate it
        for subtour in subtours
            if length(subtour) > 1
                @constraint(model, sum(x[i, j] for i in subtour, j in subtour if i != j) <= length(subtour) - 1)
            end
        end

        # Re-optimize with the new subtour elimination constraints
        optimize!(model)
    end

    # Print the final solution
    println("Minimum distance traveled: ", objective_value(model))
    return value.(x)
end