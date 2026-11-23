[English](README.md) · **Français**

# Cellule robotisée FANUC + UR5e avec passage de main en l'air, dans Webots

Un **FANUC LR Mate 200iD** et un **Universal Robots UR5e**, équipés chacun d'une pince **Robotiq 2F-85**,
se passent un cube de 50 g **directement de pince à pince, en l'air**. Un convoyeur amène les cubes un par
un contre une butée équipée d'une barrière optique ; un automate (contrôleur superviseur du convoyeur)
lance le cycle des deux robots quand un cube est présent. L'UR5e retourne le cube en l'air et le lâche
dans le bac de sortie. Cadence : **un cube toutes les 14,4 s**. Construit dans
[Webots](https://cyberbotics.com) R2025a.

![La cellule en fonctionnement](docs/cell.png)

Vidéo d'un cycle : [`docs/robot_cell.mp4`](docs/robot_cell.mp4). La première version de la cellule, avec
nos propres modèles de SCARA et de bras 6 axes, est étiquetée [`v1.0`](../../tree/v1.0).

## L'ouvrir

- **Dans le navigateur :** [`site/`](site/) contient une animation 3D interactive (HTML5) d'un cycle,
  exportée par Webots (on peut tourner autour, zoomer et parcourir la ligne de temps), publiée sur GitHub
  Pages depuis la branche `gh-pages` (`git subtree push --prefix site origin gh-pages`) :
  [nicolasbrochen.github.io/robot-cell-webots](https://nicolasbrochen.github.io/robot-cell-webots/).
- **Dans Webots :** ce dépôt *est* le dossier du projet Webots. Ouvrir
  [`worlds/robot_cell.wbt`](worlds/robot_cell.wbt) et lancer la simulation : la cellule démarre seule. Les
  PROTO de l'UR5e et de la Robotiq sont téléchargés depuis le dépôt GitHub de Webots à la première
  ouverture (accès Internet nécessaire une fois, ensuite gardés en cache par Webots).

## Ce que contient la cellule

| Élément | Origine | Rôle |
|---|---|---|
| FANUC LR Mate 200iD | [`protos/FanucLrMate200iD.proto`](protos/FanucLrMate200iD.proto), converti depuis [ros-industrial/fanuc](https://github.com/ros-industrial/fanuc) avec urdf2webots | prend le cube en butée et le présente au point de passage |
| Universal Robots UR5e | PROTO officiel de Webots, sur une platine de 2 cm | prend le cube par en dessous, le retourne, le lâche dans le bac |
| 2 × Robotiq 2F-85 | PROTO officiel de Webots, monté dans les toolSlots | pinces adaptatives à deux doigts (course 85 mm) |
| Convoyeur d'entrée | [`protos/BeltConveyor.proto`](protos/BeltConveyor.proto) | bande de 1,12 m à 0,15 m/s, guides, butée, barrière optique 7 mm au-dessus de la bande |
| Cubes | monde | 40 mm, 50 g, 3 en circulation (recyclés du bac vers le convoyeur) |
| Cellule mobile sur roulettes | [`protos/MobilePlexiCell.proto`](protos/MobilePlexiCell.proto) | poste de travail mobile industriel en profilés d'aluminium rainurés (45×45 mm / 40×40 mm avec rainures en T et équerres d'assemblage), 4 roulettes pivotantes avec freins, étagère inférieure technique accueillant les blocs d'alimentation/contrôleurs (FANUC R-30iB Mate Plus et UR5e Control Box) ainsi que les boîtiers d'apprentissage Teach Pendants (iPendant FANUC et tablette PolyScope UR), plateau supérieur outillé, carter intégral en plexiglas transparent (polycarbonate 6 mm), porte frontale avec sécurité et pupitre arrêt d'urgence |
| Bac de sortie | [`protos/OutputBin.proto`](protos/OutputBin.proto) | bac récepteur des cubes |

### Le passage de main

![Passage de main : pince du FANUC en haut, pince de l'UR5e en bas](docs/handover.png)

- Le FANUC tient le cube par le haut, patins sur les faces ±x, sur ses 30 mm supérieurs.
- L'UR5e arrive **par en dessous**, pince vers le haut, patins sur les faces ±y, sur les 30 mm inférieurs.
  Les patins de 22 mm des deux pinces sont sur des faces différentes et ne se touchent jamais.
- L'UR5e serre, le FANUC ouvre et remonte de 5 cm, puis l'UR5e retourne le cube de 180° en l'air et le
  lâche dans le bac.
- Profondeur de prise calibrée dans Webots (`tools/calibrate.sh`) : avec le centre du cube à 150 mm de
  l'origine de la pince, les doigts s'arrêtent à 0,45 rad et le cube tient pendant une levée de 10 cm.

### Cinématique

[`tools/kinematics_lib.py`](tools/kinematics_lib.py) modélise les deux bras par la chaîne d'articulations
Webots lue dans leurs PROTO (axe, point d'ancrage, pose du segment), plus le montage de la Robotiq, avec une
cinématique inverse numérique (moindres carrés amortis sur la position **et** l'orientation).
`tools/calibrate.sh` fait parcourir aux deux robots 31 configurations aléatoires dans Webots : le modèle
colle au simulateur à **0,21 mm / 0,002°** (UR5e) et **0,23 mm / 0,002°** (FANUC). Les cycles restent dans
les limites articulaires (axe J5 du poignet du FANUC : 0,31 rad de marge) et sous 80 % des vitesses des
moteurs. L'UR5e utilise la configuration de poignet avec l'axe J4 (`wrist_1_joint`) tourné de 180° par
rapport à la première solution trouvée.

### Remarques sur la physique (modèle Robotiq de Webots)

- Les moteurs des doigts de la Robotiq2f85Gripper de Webots sont limités à 0,1 N·m (valeur fixée dans le
  PROTO officiel), bien en dessous du serrage de 20 à 235 N de la vraie pince : le frottement patin/cube
  est porté à 5 pour le reproduire.
- Par défaut, le moteur physique ODE ne freine pas la rotation du cube autour de la normale aux patins :
  pendant le retournement, le cube pivotait hors de la pince de l'UR5e. Un frottement de rotation
  (`rollingFriction 0 0 0.01`) est ajouté aux contacts patin/cube.
- Les deux doigts du modèle Webots sont deux moteurs indépendants limités en couple (sur la vraie 2F-85 ils
  sont couplés mécaniquement) : ils ne recentrent pas un cube déjà tenu par l'autre pince. Le convoyeur
  maintient donc chaque cube plaqué contre la butée pour que le FANUC le prenne centré.

### L'automate (`controllers/conveyor`) et les robots (`controllers/cell_robot`)

| Message | Canal | Sens | Signification |
|---|---|---|---|
| `READY fanuc` / `READY ur5e` | 2 | robot → automate | le robot a atteint sa pose d'attente |
| `START` | 1 | automate → robots | un cube est en butée depuis 0,3 s et les deux robots sont prêts |

- Les deux robots reçoivent `START` au même pas de simulation et jouent leur fichier
  `motions/*_cell.motion` (361 poses, une toutes les 40 ms ; 6 axes + 2 doigts Robotiq). Émetteurs et
  récepteurs sont placés dans les toolSlots.
- Au démarrage, les robots quittent leur pose zéro par une pose intermédiaire sûre (bras de l'UR5e à la
  verticale) et n'annoncent `READY` qu'une fois la pose d'attente réellement atteinte.
- Si l'alimentation s'arrête, les robots restent immobiles en attente (vérifié par les tests).

## Tests

```sh
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # numpy, pour les outils
.venv/bin/python tools/check_kin.py      # fermeture IK -> FK, passage des mouvements aux points clés
.venv/bin/python tools/gen_motions.py    # régénère les cycles + limites et vitesses articulaires
tools/calibrate.sh                       # cinématique vs Webots (31 poses aléatoires par robot) + banc de prise Robotiq
tools/run_tests.sh                       # scénarios tests/*.test.json dans Webots sans affichage
tools/perturb.sh                         # les tests doivent ÉCHOUER sur 5 versions volontairement cassées
tools/run_examples.sh                    # vérifie les mondes d'exemple
```

| Scénario | Vérifie |
|---|---|
| `transfer` | ≥ 5 cubes livrés en 75 s, aucun cube au sol, aucun contact d'un robot avec autre chose qu'un cube |
| `handover` | cube centré à 3 mm près entre les doigts et à 8 mm près le long des patins, glissement ≤ 3 mm dans chaque pince |
| `presence-interlock` | aucun `START` sans cube en butée ; robots immobiles quand l'alimentation est coupée |
| `endurance` | 20 cycles d'affilée (5 min de production) sans prise ratée, chute ni collision |

| Mutation (`tools/perturb.sh`) | Détectée par |
|---|---|
| l'automate ignore la barrière optique | `presence-interlock` |
| l'UR5e ne se ferme jamais au passage de main | `transfer` |
| pas de frottement de rotation sur les patins Robotiq | `transfer` |
| le convoyeur ne maintient plus le cube contre la butée | `handover` |
| les pinces ne serrent pas | `transfer` |

## Reconstruire

| Script | Rôle |
|---|---|
| `tools/cell_layout.py` | cotes de la cellule, profondeur de prise et commandes des pinces (source unique) |
| `tools/kinematics_lib.py` | cinématique directe / inverse de l'UR5e et du FANUC avec leur Robotiq |
| `tools/gen_motions.py` | calcule les deux cycles synchronisés → `motions/fanuc_cell.motion`, `motions/ur5e_cell.motion` |
| `tools/build_world.py` | écrit `worlds/robot_cell.wbt` |
| `tools/calibrate.sh` | valide la cinématique face à Webots et mesure la prise de la Robotiq |
| `tools/build_site.sh` | réenregistre l'animation HTML5 de `site/` (un cycle, Webots sans interface) |
| `tools/build_examples.py` | écrit les mondes d'exemple 02 à 04 |

## Exemples

Les étapes qui ont mené à la cellule, avec nos propres robots procéduraux (`protos/Scara4Axis.proto`,
`protos/Arm6Dof.proto`) : l'exemple 01 utilise le contrôleur `scara_trajectory`, les exemples 02 à 04 le
contrôleur `motion_player`.

| Monde | | |
|---|---|---|
| `example_01_scara_trajectory.wbt` | SCARA qui suit un « 8 » en 3D par cinématique inverse ; trajectoire prévue en vert, trajectoire réelle tracée en direct en rouge | |
| `example_02_scara_pick_place.wbt` | SCARA : prise et dépose d'un cube de 50 g de A vers B puis retour | ![](docs/examples/02_scara_pick_place.png) |
| `example_03_arm6_pick_place.wbt` | même cycle avec le bras 6 axes, pince maintenue verticale | ![](docs/examples/03_arm6_pick_place.png) |
| `example_04_arm6_obstacles.wbt` | contournement d'un pilier de 30 cm et d'un muret : A* avec le modèle de collision du bras entier, 32 mm de dégagement mini | ![](docs/examples/04_arm6_obstacles.png) |

## Réalisé avec

Construit et mis au point en pilotant Webots depuis Claude Code via le serveur MCP
[webotMCP](https://github.com/MHasaan/webotMCP). Le dépôt n'en dépend pas.

## Licences et avertissement

- Modèle FANUC : dérivé des paquets ROS-Industrial `fanuc`, licence BSD-3-Clause, voir
  [`licenses/`](licenses/) (aussi fournie avec les maillages de `site/`).
- UR5e et Robotiq 2F-85 : PROTO de Webots (Apache 2.0), non copiés dans ce dépôt.
- SCARA et bras 6 axes des exemples, convoyeur, bac et clôture : modèles procéduraux maison, aucune CAO
  constructeur.
- FANUC, Universal Robots et Robotiq sont des marques de leurs propriétaires respectifs ; ce projet n'y est
  pas affilié. Simulation à but pédagogique : masses, efforts et serrage ne sont pas ceux des équipements
  réels.
