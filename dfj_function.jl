
using JuMP, Gurobi

# another function to detect subtours
function detect_subtours_(x_values, nombre_aerodromes, depart, arrivee)
    # return all subtours instead of just one
    visited = falses(nombre_aerodromes)
    subtours = []
    for i in 1:nombre_aerodromes
        if !visited[i]
            current = i
            path = []
            while !visited[current]
                push!(path, current)
                visited[current] = true
                # Trouver le prochain noeud
                current = findfirst(x_values[current, :] .> 0.5)
                if current === nothing || current == depart || current == arrivee
                    break
                end
            end
            # Retourner le sous-tour si trouvé
            if length(path) > 1 && current != arrivee
                push!(subtours, path)
            end
        end
    end
    return subtours
end

function detect_subtour(x_values, nombre_aerodromes, depart, arrivee)
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

function solve_DFJ(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon, relax=false)
    model = Model(Gurobi.Optimizer)
    set_silent(model)
    # set time limit
    set_optimizer_attribute(model, "TimeLimit", 60)


    if relax
        println("Relaxation linéaire activée")
        @variable(model, 0 <= x[1:nombre_aerodromes, 1:nombre_aerodromes] <= 1)
    elseif !relax
        println("Relaxation linéaire désactivée")
        @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)
    end
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


    last_sous_tour = []
    # Résolution initiale sans contrainte d'élimination de sous-tours
    optimize!(model)
    while true
        # Obtenez les valeurs actuelles de la solution
        x_values = value.(x)
        # Détecte un sous-tour dans la solution actuelle
        subtour = detect_subtour(x_values, nombre_aerodromes, depart, arrivee)
        if subtour === nothing
            println("Solution optimale trouvée sans sous-tours !")
            break
        elseif subtour == last_sous_tour
            println("Sous-tour détecté mais déjà éliminé : ", subtour)
            continue
        else
            println("Sous-tour détecté : ", subtour)
            last_sous_tour = subtour
            # Ajoute une contrainte pour éliminer ce sous-tour
            @constraint(model, sum(x[i, j] for i in subtour, j in subtour if i != j) <= length(subtour) - 1)
            # Réoptimise avec la nouvelle contrainte
            optimize!(model)
        end
    end
    # Affiche la solution finale
    println("Distance minimale parcourue : ", objective_value(model))
    # renvoie la solution
    return model
end



function solve_DFJ_(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon, relax=false)
    model = Model(Gurobi.Optimizer)
    set_silent(model)
    # set time limit
    set_optimizer_attribute(model, "TimeLimit", 60)


    if relax
        println("Relaxation linéaire activée")
        @variable(model, 0 <= x[1:nombre_aerodromes, 1:nombre_aerodromes] <= 1)
    elseif !relax
        println("Relaxation linéaire désactivée")
        @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)
    end
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


    eliminated_sous_tour = []
    # Résolution initiale sans contrainte d'élimination de sous-tours
    optimize!(model)
    # une boucle qui prend tout les sous tours et les elimine
    loop = true
    while loop
        # Obtenez les valeurs actuelles de la solution
        x_values = value.(x)
        # Détecte tous les sous-tour dans la solution actuelle
        subtours = detect_subtours_(x_values, nombre_aerodromes, depart, arrivee)
        append!(eliminated_sous_tour, subtours)
        if length(subtours) == 0
            println("Solution optimale trouvée sans sous-tours !")
            break
        else
            println("Sous-tours détectés : ", subtours)
            for subtour in subtours
                if subtour in eliminated_sous_tour
                    continue
                end
                # Ajoute une contrainte pour éliminer ce sous-tour
                @constraint(model, sum(x[i, j] for i in subtour, j in subtour if i != j) <= length(subtour) - 1)
            end
            # Réoptimise avec la nouvelle contrainte
            optimize!(model)
        end
    end
    # Affiche la solution finale
    println("Distance minimale parcourue : ", objective_value(model))
    # renvoie la solution
    return model
end