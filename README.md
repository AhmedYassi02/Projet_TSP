Projet Course d'avions -  Cours Optimisation Discrète - 2024/2025.


## Introduction : 
Le problème consiste à trouver le chemin le plus court pour un avion qui doit traverser un nombre minimum d'aerodromes, tout en visitant des aerodromes dans des regions spécifiques.

## Objectif
Comparer les performances de différents formulations du problème Voyageur de Commerce pour résoudre le problème de la course d'avions.

Les méthodes comparées :
- Formulation d'elimination des sous-tours de Dantzig-Fulkerson-Johnson ( DFJ )
- Formulation de Miller-Tucker-Zemlin ( MTZ )
- Formulation Single Flow ( SF )
- Reformulation-linearization based formulation ( RLT )

Les formulation sont presentées dans [Integer programming formulations for the elementary shortest path problem](https://www.sciencedirect.com/science/article/abs/pii/S0377221716000084?via%3Dihub)

## Utilisation

Pour utiliser le programme, il suffit d'ouvrir le fichier `main.jl`, modifier les paramètres `method` , `ìnstance` et `relax` et de lancer le programme.

