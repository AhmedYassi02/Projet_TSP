
using JuMP, Gurobi

function solve_MTZ(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon, relax=false)
    model = Model(Gurobi.Optimizer)
    # set_silent(model)
    set_optimizer_attribute(model, "TimeLimit", 180)

    # Variables : x[i,j] = 1 si l'arc (i, j) est dans le chemin, 0 sinon
    if relax
        @variable(model, 0 <= x[1:nombre_aerodromes, 1:nombre_aerodromes] <= 1)

        @variable(model, u[1:nombre_aerodromes] >= 0)
    else
        @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)
        @variable(model, u[1:nombre_aerodromes] >= 0, Int)
    end

    # Objective function: minimize the total distance
    @objective(model, Min, sum(distances[i, j] * x[i, j] for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes if i != j))

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
    # @constraint(model, x[arrivee, depart] == 0)
    # @constraint(model, x[depart, arrivee] == 0)
    # on ne peut pas visiter un aerodrome plus d'une fois
    for i in 1:nombre_aerodromes
        @constraint(model, sum(x[i, j] for j in 1:nombre_aerodromes if j != i) <= 1)
        @constraint(model, sum(x[j, i] for j in 1:nombre_aerodromes if j != i) <= 1)
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
        @constraint(model, x[i, i] == 0)
        for j in 1:nombre_aerodromes
            @constraint(model, x[i, j] * distances[i, j] <= rayon)
        end
    end

    # MTZ methode pour eliminer les sous-tours
    for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes
        if i != j && i != depart && j != depart && i != arrivee && j != arrivee
            @constraint(model, u[i] - u[j] + 1 <= (nombre_aerodromes - 1) * (1 - x[i, j]))
        end
    end




    optimize!(model)
    println("Minimum distance traveled: ", objective_value(model))

    return model
end