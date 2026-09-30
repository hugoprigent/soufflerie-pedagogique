# Soufflerie pédagogique — ENSAM, 2026

Une soufflerie de table conçue pour observer l'écoulement autour d'une maquette et explorer une chaîne de mesure : commande du ventilateur, cellule de charge, caméra et interface d'acquisition sur Raspberry Pi. Ce dépôt rassemble le modèle mécanique, le logiciel et les indications nécessaires pour comprendre le banc et préparer sa reproduction.

![Photographie du banc réel : veine transparente, conduits blancs et électronique visible](assets/banc-reel.jpg)

*Banc réel photographié en mai 2026. La photo montre l'état du prototype, avec câblage encore apparent ; elle ne démontre aucune performance de mesure.*

![Rendu Blender de l'assemblage CAO avec couleurs inspirées du prototype](assets/rendu-assemblage.png)

*Rendu du STEP d'assemblage, avec matériaux et couleurs interprétés d'après la photo. Il illustre la conception et ne remplace pas une vérification de fabrication.*

## Le projet en bref

| Domaine | Ce qui est disponible |
| --- | --- |
| Mécanique | [Assemblage Onshape exporté en STEP](cad/assemblage-onshape.step), [23 modèles STEP de pièces](cad/step/), [scène Blender](cad/rendu-assemblage.blend) et [guide CAO](cad/README.md) |
| Électronique | Architecture Raspberry Pi, ventilateur, cellule de charge et caméra ; [schéma de reprise](docs/schema-cablage.svg) et [câblage à confirmer sur le banc](docs/electronique-cablage.md) |
| Logiciel | [Supervision Python](software/) : PWM et tachymètre, acquisition de force, caméra et traitement d'image, interface HTTP et export CSV |
| Données | [Un export de session](examples/session-2026-05-28.csv) avec [explication des colonnes et limites](docs/donnees-et-limites.md) |

Le banc comporte une section d'essai visible entre un convergent et un diffuseur. La maquette se place dans la veine ; la chaîne de force passe par une cellule de charge et le convertisseur HX711. Le tableau de bord affiche les signaux et permet la commande et l'enregistrement. La caméra sert à la visualisation et à un traitement de flux optique. Les valeurs de vitesse et les coefficients dérivés exigent une calibration indépendante avant toute interprétation quantitative.

![Capture du tableau de bord avec retour caméra](assets/dashboard.png)

*Capture du 28 mai 2026 : le ventilateur est à l'arrêt sur cette image.*

## Contribution

| Prénom | Contribution |
| --- | --- |
| Hugo Prigent | Électronique, logiciel et acquisition de données |

Projet pédagogique réalisé à l'ENSAM en 2026.

## Parcourir le dépôt

- [Reproduire le projet](docs/reproduire-le-projet.md) : fichiers 3D, nomenclature de reprise, ordre de fabrication, montage et points à relever sur le prototype.
- [CAO et fabrication](cad/README.md) : différence entre l'assemblage positionné et les pièces séparées, contenu et précautions d'export.
- [Mécanique et assemblage](docs/mecanique-fabrication.md) : sous-ensembles visibles, ordre de montage proposé et vérifications avant fabrication.
- [Électronique et câblage](docs/electronique-cablage.md) : schéma annoté, affectation des broches selon le code, divergences des notes historiques et contrôles à effectuer.
- [Installation et utilisation](docs/installation-utilisation.md) : environnement Raspberry Pi, lancement, accès local et sécurité réseau.
- [Données et limites](docs/donnees-et-limites.md) : ce que l'export permet d'examiner et ce qu'il ne valide pas.
- [Points à lever avant publication](docs/publication.md) : licences, câblage réel et validation matérielle.

## État de vérification

La syntaxe Python, la structure des fichiers et les en-têtes STEP ont été contrôlés sur une copie locale. Aucun nouvel essai du banc, de la caméra, des GPIO ou de l'installation Raspberry Pi n'a été réalisé pour cette préparation. Les documents historiques présentent des configurations électriques et des résultats contradictoires ; ce dépôt les signale au lieu de les reprendre comme des mesures validées.

Le serveur écoute par défaut sur `127.0.0.1:8080`. L'accès depuis un autre appareil doit être activé explicitement et limité à un réseau maîtrisé : l'application ne comporte pas d'authentification. Voir le [guide d'utilisation](docs/installation-utilisation.md).

## Réutilisation

Le choix d'une licence pour le code, la documentation, les photos et les modèles 3D reste à confirmer avant publication publique. L'absence de fichier de licence ne vaut pas autorisation de réutilisation.
