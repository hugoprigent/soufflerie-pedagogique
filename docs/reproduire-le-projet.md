# Reproduire la soufflerie

Ce guide rassemble les fichiers et les contrôles nécessaires pour refaire un banc de même architecture. La [photo du prototype](../assets/banc-reel.jpg) et l'[assemblage STEP](../cad/assemblage-onshape.step) montrent l'état documenté ; ils ne donnent pas encore un plan de fabrication entièrement coté ni un câblage certifié. Conserver la traçabilité des choix faits lors d'une reproduction.

## Dossier de départ

| Besoin | Fichier | Usage |
| --- | --- | --- |
| Position des pièces | [Assemblage STEP](../cad/assemblage-onshape.step) | Ouvrir dans un logiciel CAO, inspecter les interfaces et mesurer les cotes utiles. |
| Géométrie de chaque pièce | [23 STEP séparés](../cad/step/) | Préparer les plans, l'impression ou l'usinage selon la pièce. |
| Aspect et vue d'ensemble | [Photo](../assets/banc-reel.jpg) et [rendu 3D](../assets/rendu-assemblage.png) | Comparer prototype et CAO ; le rendu est une interprétation visuelle. |
| Logiciel | [Tableau de bord et modules](../software/) | Reprendre les fonctions GPIO, caméra, affichage et export. |
| Câblage | [Schéma de reprise](schema-cablage.svg) et [notes électriques](electronique-cablage.md) | Relever les connexions réelles puis établir un schéma final. |

## Nomenclature de reprise

Les quantités des pièces CAO sont à extraire de l'assemblage positionné, et les références achetées à lire sur le banc. Les éléments ci-dessous forment une **liste de familles**, pas un panier d'achat validé.

| Famille | Éléments identifiés | À renseigner avant achat ou fabrication |
| --- | --- | --- |
| Conduits et veine | Cône de concentration, alvéoles, diffuseur, jonctions, plaques dessus/dessous/fond, fenêtre, colliers | Dimensions internes, matière, épaisseur, procédé, état de surface, étanchéité |
| Structure et mesure | Pieds, rail, glissière, support de cellule de charge, support et maintien caméra | Positions, tolérances d'ajustement, visserie par liaison, reprise des efforts |
| Ventilation | Ventilateur représenté comme « 120x12x38 » dans le nom d'un STEP | Référence réelle, dimensions vérifiées, tension, courant, nombre de fils, brochage |
| Acquisition | Raspberry Pi, caméra, cellule de charge, carte HX711 | Référence des cartes et connecteurs, plage de mesure, tension et niveaux logiques |
| Éléments éventuels | Capteur BMP280/BME280, encodeur Modulino, LED | Présence réelle et nécessité pour la version à refaire |
| Alimentation et liaisons | Alimentations adaptées, fils, connecteurs, protection, fixations | Schéma électrique final, courant disponible, masse, isolation et cheminement des câbles |

Une nomenclature historique cite une fenêtre en polycarbonate et de la visserie M2, M3 et M5. Ces indications doivent être recoupées avec la version réellement fabriquée ; le dépôt ne donne pas encore les longueurs ni les quantités de vis.

## Séquence de reproduction

1. **Définir la version.** Comparer le [STEP d'assemblage](../cad/assemblage-onshape.step) à la photo. Lister les pièces à fabriquer, les pièces achetées et les écarts observés.
2. **Extraire les plans.** Mesurer dans la CAO les sections d'entrée et d'essai, les interfaces des conduits, l'épaisseur de fenêtre, les trous, les positions de rail et les dégagements autour du ventilateur et de la maquette. Ajouter tolérances et matériaux à des plans d'atelier avant fabrication.
3. **Préparer les pièces.** Utiliser les STEP séparés selon le procédé retenu. Faire un montage à blanc des plaques, de la fenêtre, des jonctions et des conduits ; vérifier alignement, étanchéité et accès à la section d'essai.
4. **Installer l'instrumentation.** Monter la cellule de charge sans effort parasite, puis caméra et éclairage. Vérifier que les fils ne traversent pas le flux ou ne touchent pas les pièces mobiles.
5. **Relever l'électricité.** Photographier les références et sérigraphies des cartes. Hors tension, suivre chaque fil du ventilateur, du HX711, de la cellule, de la caméra et du GPIO. Compléter le [schéma de reprise](schema-cablage.svg) en un schéma validé pour les composants choisis.
6. **Vérifier avant mise sous tension.** Confirmer polarités, alimentation séparée du moteur, masse de référence, niveaux logiques, protections, continuité et absence de court-circuit. Faire contrôler ce relevé par une personne compétente.
7. **Installer le logiciel.** Suivre le [guide Raspberry Pi](installation-utilisation.md), adapter les GPIO et les références dans `software/config.py`, puis tester les périphériques un par un avant de lancer l'application complète.
8. **Étalonner et documenter.** Tare et masses connues pour la chaîne de force ; référence indépendante pour la vitesse. Conserver les valeurs d'étalonnage, le montage, la date et les incertitudes avec les [données](donnees-et-limites.md).

## Informations à compléter pour une réplique fidèle

- Cotes critiques, tolérances, matières, procédés et fichiers de plans cotés.
- Nomenclature avec quantités, références commerciales, visserie et photos de chaque connexion.
- Schéma électrique vérifié du banc, surtout tension du HX711, niveau du tachymètre et mode de commande du ventilateur.
- Procédure d'étalonnage avec étalons, résultats et incertitudes ; essais sur Raspberry Pi et sur le banc assemblé.

Ces lacunes sont indiquées pour éviter qu'une hypothèse issue du logiciel ou d'une ancienne note soit prise pour une mesure sur le prototype.
