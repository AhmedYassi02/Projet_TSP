
using JuMP, Gurobi

function solve_GCS(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
    model = Model(Gurobi.Optimizer)
    set_silent(model)

    #### Contraintes de base ####

    # Variables : x[i,j] = 1 si l'arc (i, j) est dans le chemin, 0 sinon
    @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)

    # RLT auxiliary variables:
    @variable(model, alpha[1:nombre_aerodromes, 1:nombre_aerodromes] >= 0)
    @variable(model, beta[1:nombre_aerodromes, 1:nombre_aerodromes] >= 0)

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


    # on ne peut pas visiter un aerodrome plus d'une fois
    for i in 1:nombre_aerodromes
        @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) <= 1)
    end

    # on peut pas aller de arrivee a depart
    @constraint(model, x[arrivee, depart] == 0)
    @constraint(model, x[depart, arrivee] == 0)

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

    # Function to add cutset constraints based on the current solution
    function add_cutset_constraints(model, x_values)
        cut_added = false
        for i in 1:nombre_aerodromes
            if i != depart && i != arrivee
                subset = [j for j in 1:nombre_aerodromes if j != depart && j != arrivee && j != i]
                lhs = sum(x[j, k] for j in subset, k in 1:nombre_aerodromes if k ∉ subset)
                if lhs <= 0.5  # Check if there is no flow in/out of subset
                    @constraint(model, lhs >= 1)  # Add cutset constraint
                    cut_added = true
                end
            end
        end
        return cut_added
    end

    # Initial solve without GCS constraints
    optimize!(model)

    # Loop to detect and add GCS constraints until no more subtours are found
    while true
        x_values = value.(x)  # Get current solution values

        # Add GCS constraints based on the solution
        cut_added = add_cutset_constraints(model, x_values)

        # If no more cuts are added, the solution is optimal
        if !cut_added
            println("Optimal solution found with no subtours!")
            break
        end

        # Re-optimize with the new GCS constraints
        optimize!(model)
    end

    # Print the final solution
    println("Minimum distance traveled: ", objective_value(model))
    return model
end