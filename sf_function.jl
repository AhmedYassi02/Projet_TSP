
using JuMP, Gurobi

function solve_SF(nombre_aerodromes, depart, arrivee, distances, nombre_min_aerodromes, nombre_regions, regions, rayon)
    model = Model(Gurobi.Optimizer)
    set_silent(model)

    #### Contraintes de base ####

    # Variables : x[i,j] = 1 si l'arc (i, j) est dans le chemin, 0 sinon
    @variable(model, x[1:nombre_aerodromes, 1:nombre_aerodromes], Bin)

    # SF auxiliary variables:
    @variable(model, q[1:nombre_aerodromes, 1:nombre_aerodromes] >= 0)
    @variable(model, z[1:nombre_aerodromes], Bin)

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

    ### Contraintes de la méthode SF ###

    for i in 1:nombre_aerodromes, j in 1:nombre_aerodromes
        if i != j
            @constraint(model, q[i, j] <= (nombre_aerodromes - 1) * x[i, j])
        end
    end

    @constraint(model, sum(q[depart, j] for j in 1:nombre_aerodromes) == sum(z[j] for j in 1:nombre_aerodromes if j != depart))

    for k in 1:nombre_aerodromes
        if k != depart
            @constraint(model, sum(q[i, k] for i in 1:nombre_aerodromes if i != k) - sum(q[k, j] for j in 1:nombre_aerodromes if j != k) == z[k])
            @constraint(model, sum(x[i, k] for i in 1:nombre_aerodromes if i != k) == z[k])
            @constraint(model, z[k] >= 0)
        end
    end





    optimize!(model)
    println("Minimum distance traveled: ", objective_value(model))


    return value.(x)
end