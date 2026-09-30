# Données et limites des essais

Le fichier [`examples/session-2026-05-28.csv`](../examples/session-2026-05-28.csv) est un export de session existant. Les lignes `#` enregistrent des paramètres au moment de l'export ; la ligne suivante nomme les colonnes. Il permet de voir la structure des acquisitions, sans constituer à lui seul une validation métrologique.

| Colonne | Sens dans l'export |
| --- | --- |
| `t_s` | Temps enregistré par l'acquisition, en secondes |
| `duty_pct` | Consigne PWM du ventilateur |
| `fan_rpm` | Régime déduit du tachymètre, sous réserve d'une lecture correcte |
| `delta_adu` | Signal HX711 après retrait de l'offset, en unités numériques |
| `force_g`, `force_N` | Force calculée avec la tare et l'échelle enregistrées |
| `flow_mag_px` | Grandeur issue du traitement d'image |
| `airspeed_flow_ms` | Estimation de vitesse issue de l'image et de sa calibration |
| `airspeed_fan_ms` | Estimation de vitesse issue du modèle ventilateur |

Dans les premières lignes de cet export, `duty_pct` vaut 60 tandis que `fan_rpm` et les deux estimations de vitesse valent zéro. Cette divergence impose de contrôler le tachymètre, l'état du ventilateur et les temps de transition avant d'utiliser ces colonnes pour conclure sur un essai. Les métadonnées de calibration ne prouvent pas que la chaîne a été étalonnée avec une référence traçable.

Le régime du ventilateur ne donne pas directement la vitesse locale dans la veine. Le flux optique dépend de l'échelle en pixels, de l'éclairage, du traceur visible et du plan observé. La mesure de force dépend notamment de la tare, de la rigidité du montage, des contacts parasites et des perturbations électriques. Aucune précision, plage utile, répétabilité ni coefficient aérodynamique validé n'est revendiqué ici.

Des documents historiques mêlent données enregistrées, valeurs théoriques, corrections et simulations. Ils n'ont pas été repris comme preuves expérimentales. Une suite de validation devrait conserver séparément les lectures brutes, les références de calibration, les transformations, les incertitudes et les conditions de montage, puis comparer plusieurs essais reproductibles.
