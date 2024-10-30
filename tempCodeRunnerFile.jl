    for i in 1:nombre_aerodromes
        for j in 1:nombre_aerodromes
            if distances[i, j] > rayon
                @constraint(model, x[i, j] == 0)
            end
        end
    end