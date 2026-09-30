# Mécanique et fabrication

## Architecture visible

La photo du prototype montre un convergent et un diffuseur blancs autour d'une section d'essai transparente. Les panneaux supérieur et inférieur paraissent en bois clair, le fond de la veine est sombre, et des éléments de liaison violets ferment les extrémités. Une maquette est portée dans la veine ; une caméra et un éclairage permettent l'observation. Les fils et le boîtier électronique restent apparents.

L'[assemblage STEP](../cad/assemblage-onshape.step) est la meilleure source disponible pour l'agencement CAO. Les [23 pièces STEP](../cad/step/) servent à examiner les formes séparément. Les modèles ne fournissent pas, à eux seuls, des plans de fabrication cotés avec tolérances ou une nomenclature validée.

## Repères de composants

Une nomenclature de l'assemblage historique cite un cône de concentration, des alvéoles, une section d'essai, deux colliers, un diffuseur, une fenêtre en polycarbonate, un pied et de la visserie M2, M3 et M5. Cet état historique peut différer de l'export V2, qui comporte par exemple plusieurs plaques, supports et jonctions. Il sert de repère pour le tri, **pas de liste de commande vérifiée**.

| Sous-ensemble | Fichiers ou éléments à examiner | Vérification nécessaire |
| --- | --- | --- |
| Entrée et sortie | Cône de concentration, alvéoles, diffuseur, pied diffuseur | Sens du flux, raccords, fixation au ventilateur |
| Veine d'essai | Plaques, fenêtre, jonctions, colliers | Dimensions internes, étanchéité, accès à la maquette |
| Instrumentation | Rail, glissière, support de cellule de charge, support et maintien caméra | Alignement, absence de contact parasite avec la veine |
| Composants achetés | Ventilateur, caméra, cellule de charge | Références exactes et encombrement réel |

## Ordre de montage proposé à vérifier sur le banc

1. Identifier les pièces réellement fabriquées et les pièces commerciales dans l'assemblage CAO.
2. Assembler à blanc les panneaux et la fenêtre de la veine ; contrôler l'alignement et les interfaces avec le convergent et le diffuseur.
3. Installer les supports, rail, maquette et cellule de charge sans contrainte latérale ni contact non prévu avec les parois.
4. Poser caméra et éclairage en conservant l'accès à la section d'essai.
5. Fixer ventilateur et conduits, puis contrôler le sens du flux, les dégagements des pales et l'absence de pièces libres.
6. Faire contrôler le câblage et les alimentations avant la mise sous tension.

Cet ordre décrit une méthode de préparation ; ce n'est pas un protocole d'atelier validé. Il manque encore les cotes critiques, tolérances, visserie par emplacement, paramètres d'impression et détails d'étanchéité.
