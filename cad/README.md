# CAO de la soufflerie

## Deux jeux de fichiers distincts

- [`assemblage-onshape.step`](assemblage-onshape.step) : export AP242 de l'assemblage Onshape « Assemblage V2 ». C'est la référence disponible pour les positions relatives des composants.
- [`step/`](step/) : 23 fichiers STEP individuels de l'export « Assemblage V2 soufflerei ». Ces fichiers décrivent des pièces isolées ; les ouvrir ensemble ne reconstruit pas automatiquement l'assemblage ni les contraintes Onshape.

Les STEP individuels et l'assemblage entier sont conservés car ils répondent à deux usages différents : inspection ou adaptation d'une pièce, et compréhension de la disposition d'ensemble. L'archive ZIP d'origine n'est pas copiée, puisqu'elle redouble les fichiers individuels. Les originaux restent inchangés.

Le [rendu Blender](../assets/rendu-assemblage.png) et sa [scène éditable](rendu-assemblage.blend) sont une **illustration** issue du STEP d'assemblage après conversion en maillages. Les couleurs et matériaux ont été attribués à partir de la photographie du prototype ; ce rendu ne certifie ni la matière de chaque pièce, ni l'état exact du banc fabriqué, ni l'absence d'interférences.

## Pièces individuelles disponibles

Les noms de l'export ont été conservés, y compris les accents et suffixes `(1)`. Ils identifient notamment le cône de concentration, ses alvéoles, les deux diffuseurs, la fenêtre, les plaques de la veine, les pieds, les jonctions, les supports de caméra et de cellule de charge, un rail, une glissière et les volumes de représentation du ventilateur, de la caméra et de la cellule. Les suffixes ne prouvent pas qu'il s'agit de variantes ou de pièces à fabriquer en double.

Certains modèles représentent des composants achetés ou des volumes d'encombrement. **Ne pas imprimer ou usiner l'ensemble des 23 fichiers sans tri.** La [nomenclature historique](../docs/mecanique-fabrication.md) n'est pas parfaitement alignée avec cet export V2 ; quantité, matériau et procédé doivent être confirmés sur l'assemblage réel.

## Pour fabriquer ou modifier

1. Ouvrir l'assemblage entier dans un logiciel CAO prenant en charge STEP AP242 et vérifier l'échelle, l'orientation, les corps et les interférences.
2. Comparer les zones de fixation et la section d'essai avec la photo du banc réel. L'export ne documente ni tolérances, ni paramétrage d'impression, ni ordre de montage certifié.
3. Séparer les pièces fabriquées des composants commerciaux et contrôler chaque cote utile avant l'usinage ou l'impression.
4. Si une modification est faite dans Onshape, conserver la révision et réexporter à la fois l'assemblage et les pièces concernées.

Le lien Onshape éditable, la révision exacte, les matériaux pièce par pièce et les plans cotés restent à ajouter après vérification.
