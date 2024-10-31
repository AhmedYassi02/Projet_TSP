
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


function distance(a, b)
    return sqrt((a[1] - b[1])^2 + (a[2] - b[2])^2)
end