
using JuMP, Gurobi

function solve_RTL(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon, relax=false)
    model = Model(Gurobi.Optimizer)
    set_silent(model)
    set_optimizer_attribute(model, "TimeLimit", 180)

    #### Contraintes de base ####

    if relax
        @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes])
        for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes
            @constraint(model, x[i, j] >= 0)
            @constraint(model, x[i, j] <= 1)
        end
    else
        @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)
    end

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
        @constraint(model, x[i, i] == 0)
        @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) <= 1)
        @constraint(model, sum(x[j, i] for j in 1:nombre_aerodromes if j != i) <= 1)
    end

    # on peut pas aller de arrivee a depart
    @constraint(model, x[arrivee, depart] == 0)
    @constraint(model, x[depart, arrivee] == 0)

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

    ##### Contraintes de la méthode RLT  = Reformulation-linearization based formulation ####
    # contraintes RLT 
    for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes
        if i != j && i != depart
            # Linearization of bilinear terms
            @constraint(model, alpha[i, j] <= (nombre_aerodromes - 1) * x[i, j])
            @constraint(model, alpha[i, j] == beta[i, j] + x[i, j])
            @constraint(model, x[i, j] <= beta[i, j])
        end
    end

    for j in 1:nombre_aerodromes
        if j != depart && j != arrivee
            @constraint(model, sum(alpha[i, j] for i in 1:nombre_aerodromes if i != j) - sum(beta[j, k] for k in 1:nombre_aerodromes if k != j) == 0)
        end
    end

    # pour tout j != arrivee
    for j in 1:nombre_aerodromes
        if j != arrivee
            @constraint(model, x[depart, j] + sum(alpha[i, j] for i in 1:nombre_aerodromes if i != j && i != depart) - sum(beta[j, k] for k in 1:nombre_aerodromes if k != j) == 0)
        end
    end



    optimize!(model)
    println("Minimum distance traveled: ", objective_value(model))

    return model
end