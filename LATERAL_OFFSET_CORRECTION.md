# Lateral Offset Correction — récapitulatif complet de la session

Fonctionnalité : décaler le robot de quelques cm à gauche/droite en temps réel pendant la
navigation (réalignement sur les rangs), sans arrêter le robot ni annuler le goal de
navigation. Pilotable via un simple topic ROS2.

## Utilisation

```bash
# Décaler de 2 cm à gauche
ros2 topic pub /row_alignment/lateral_offset std_msgs/msg/Float32 "{data: 0.02}" --once

# Décaler de 3 cm à droite
ros2 topic pub /row_alignment/lateral_offset std_msgs/msg/Float32 "{data: -0.03}" --once

# Annuler le décalage
ros2 topic pub /row_alignment/lateral_offset std_msgs/msg/Float32 "{data: 0.0}" --once
```

- Convention de signe : **positif = gauche**, **négatif = droite**, par rapport au sens de
  déplacement du robot (même convention que `SwathFactory::generateOffsetSwaths` dans
  `row_swath_generator.cpp`).
- Le décalage est **persistant** : il reste appliqué (y compris sur les rangs suivants) tant
  qu'on ne republie pas une nouvelle valeur (ex: `0.0` pour annuler).
- **Clamp de sécurité : ±0.5 m.** Toute valeur reçue au-delà est bornée automatiquement, avec
  un `RCLCPP_WARN` loggé au moment du clamp (valeur demandée vs. valeur réellement appliquée).

## Fichiers modifiés/créés

Tout est dans ce workspace (`/home/eterry_autonomy_stack`), pas de dépendance à un autre repo
— contrairement à la note du brief initial qui parlait d'un `eterry_simulation` séparé : sur
cette machine il n'y a que `eterry_autonomy_stack`, donc tout est câblé directement ici.

### 1. Nouveau nœud BT — package `eterry_navigation_custom_plugins`

**Créé** : `src/eterry_navigation_custom_plugins/src/lateral_offset_corrector.cpp`

Nœud BT.CPP (`BT::SyncActionNode`) nommé `LateralOffsetCorrector` :
- S'abonne à `/row_alignment/lateral_offset` (`std_msgs/Float32`), avec clamp ±0.5 m à la
  réception (voir `kMaxLateralOffset`).
- Port d'entrée `nav_path` (le chemin courant), port de sortie `offset_path`.
- À chaque tick : décale chaque pose du chemin perpendiculairement à la direction locale du
  chemin (tangente calculée par différence centrée entre poses voisines), de la valeur
  d'offset courante.
- **Optimisation critique (voir bug #3 ci-dessous)** : ne retouche `nav_path`/`offset_path`
  que lorsque la valeur d'offset a réellement changé depuis le tick précédent (`offset_ ==
  last_applied_offset_` → retour immédiat, sans lire/écrire le blackboard). Sur un chemin de
  100k+ poses re-tické à 10Hz par un `ReactiveSequence`, un simple copier-coller à chaque
  tick suffisait à dépasser le budget de tick.

**Modifié** : `src/eterry_navigation_custom_plugins/CMakeLists.txt`
- Ajout de la cible `lateral_offset_corrector` (même pattern que les autres nœuds BT du
  package : `ament_target_dependencies` sur `rclcpp`, `geometry_msgs`, `nav_msgs`,
  `std_msgs`, `behaviortree_cpp_v3`, + `BT_PLUGIN_EXPORT`).
- Ajout de cette cible à `install(TARGETS ...)`.
- `package.xml` n'a pas eu besoin de modif : toutes les dépendances (`std_msgs`, `nav_msgs`,
  `behaviortree_cpp_v3`, `rclcpp`) étaient déjà déclarées.

### 2. Câblage dans l'arbre de comportement — package `eterry_opennav_coverage`

C'est **ce package-ci**, pas `eterry_simulation`, qui est réellement chargé au runtime — vérifié
en traçant `resume_row_coverage.py` → il charge bien
`eterry_opennav_coverage/behavior_trees/resume_coverage.xml` via
`eterry_opennav_coverage/launch/resume_coverage_launch.py` →
`eterry_opennav_coverage/launch/row_bringup_launch.py` (qui démarre un `component_container_isolated`
nommé `nav2_container`, avec un `backported_bt_navigator::BtNavigator` composable node dedans —
pas le `nav2_bt_navigator` standard).

**Modifié** : `src/eterry_opennav_coverage/params/params.yaml`
- Ajout de `lateral_offset_corrector` à `bt_navigator.plugin_lib_names` (ligne ~92).
- **Important** : cette liste attend le **nom de la librairie CMake** (`lateral_offset_corrector`,
  minuscule), pas le nom de la classe BT enregistrée (`LateralOffsetCorrector`). Voir bug #1.
- `controller_patience: 3.0` ajouté sous `controller_server.ros__parameters` (n'existait pas
  avant → valeur par défaut nav2 ~1.0s, trop courte). Voir bug #4.
- `max_robot_pose_search_dist` du controller `FollowPath` remonté de `1.0` à `3.0`. Voir bug #4.

**Modifié** : `src/eterry_opennav_coverage/behavior_trees/resume_coverage.xml`

Avant :
```xml
<ResumePathTrimmer nav_path="{path}" trimmed_path="{trimmed_path}"/>
<FollowPath
  path="{trimmed_path}"
  controller_id="FollowPath"
  goal_checker_id="stopped_goal_checker"/>
```

Après :
```xml
<ResumePathTrimmer nav_path="{path}" trimmed_path="{trimmed_path}"/>
<ReactiveSequence name="FollowWithLateralAlignment">
  <LateralOffsetCorrector nav_path="{trimmed_path}" offset_path="{offset_path}"/>
  <FollowPath
    path="{offset_path}"
    controller_id="FollowPath"
    goal_checker_id="stopped_goal_checker"/>
</ReactiveSequence>
```

Le `<ReactiveSequence>` (au lieu d'un `<Sequence>` simple) fait que `LateralOffsetCorrector` est
ré-évalué à chaque tick du BT (10Hz, via le `<RateController>` déjà en place plus haut dans
l'arbre) **pendant que `FollowPath` continue de tourner**. Nav2 détecte que le path passé à
`FollowPath` a changé et met à jour le goal en cours (`goal_updated_`) au lieu de
l'annuler/relancer — c'est ce qui permet le décalage à chaud sans arrêter le robot.

## Chronologie des bugs rencontrés pendant le test sur le vrai robot

Le robot de test est le **vrai matériel** (socketcan bridge, capteurs, lidar odometry,
localisation tous actifs), pas un simulateur — donc chaque relance a un impact réel.

### Bug #1 — `Node not recognized: LateralOffsetCorrector`

Premier essai : erreur au chargement du BT, le nœud n'est pas reconnu.

**Cause réelle** : le `nav2_container` tournait déjà *avant* que le build avec le nouveau
plugin ne soit terminé / que `params.yaml` soit mis à jour. `plugin_lib_names` n'est lu
**qu'une fois**, à la construction du `BehaviorTreeEngine` (donc au démarrage du composable
node `bt_navigator`), jamais rechargé à chaud. Renvoyer un nouveau goal au même container ne
suffit pas — il faut un **restart complet du container** (`row_bringup_launch.py` /
`resume_coverage_launch.py`), pas juste un nouveau goal.

Le `.so` lui-même était correct (symbole `BT_RegisterNodesFromPlugin` bien exporté, vérifié
avec `nm -D`), donc ce n'était pas un problème de build.

### Bug #2 — `Could not load library: libLateralOffsetCorrector.so: cannot open shared object file`

En essayant de corriger le bug #1, l'entrée dans `plugin_lib_names` avait été changée (dans
l'IDE, pas par moi) en `LateralOffsetCorrector` (nom de la classe, capitalisé) au lieu de
`lateral_offset_corrector` (nom de la lib). Ça a cassé le chargement différemment : au lieu
d'un warning "not recognized", c'est devenu une exception **fatale** qui a fait échouer tout
le bringup (`lifecycle_manager_navigation: Failed to bring up all requested nodes`).

**Fix** : revenu à l'entrée en minuscule `lateral_offset_corrector`, qui correspond au nom du
`.so` (convention confirmée en comparant avec les autres entrées de la liste : ex.
`resume_path_trimmer` (lib) enregistre la classe `"ResumePathTrimmer"`, `is_first_run_condition_bt_node`
(lib) enregistre `"IsFirstRun"` — la liste `plugin_lib_names` prend toujours le nom de lib,
jamais le nom de classe BT).

Au passage : deux entrées préexistantes dans la liste (`opennav_coverage_navigator`,
`is_path_empty_bt_node`) déclenchent déjà un `ERROR loading library: can't find symbol
[BT_RegisterNodesFromPlugin]` **non-fatal** — comportement préexistant du système, sans
rapport avec notre changement, à ne pas confondre avec une vraie panne.

### Bug #3 — Timeout / `Behavior Tree tick rate 10.00 was exceeded!`

Une fois le plugin bien chargé, le premier lancement échouait quand même : le champ de test
est énorme (100 swaths, ~22.5 km, `ResumePathTrimmer` a produit **178 182 poses**). Le
`ReactiveSequence` re-tick `LateralOffsetCorrector` à 10Hz en continu tant que `FollowPath`
tourne — et mon implémentation initiale faisait une copie complète du `nav_msgs::Path` à
**chaque tick**, même quand l'offset restait à `0.0` (via `getInput` + `setOutput`, chacun
copiant ~178k `PoseStamped`, chacun avec son propre `std_msgs/Header` → string `frame_id`
copiée séparément à chaque fois). Sur le Jetson, ça suffisait à dépasser le budget de 100ms
par tick, ce qui a fait timeout la demande de goal vers `FollowPath` dès le premier envoi.

**Fix** (voir le commentaire dans `lateral_offset_corrector.cpp`) : ne recalculer/republier
`offset_path` que lorsque la valeur d'offset a changé depuis le dernier tick ; sinon, retour
immédiat sans toucher au blackboard.

### Digression — retour temporaire à l'ancien BT

Le temps de valider le fix du bug #3, `resume_coverage.xml` a été remis dans son état
d'origine (sans `ReactiveSequence`/`LateralOffsetCorrector`) sur demande, pour débloquer les
tests en cours, puis re-appliqué une fois le fix #3 prêt.

### Bug #4 — `Resulting plan has 0 poses in it` → `Controller patience exceeded` → goal annulé

Une fois le fix #3 en place, le BT charge bien et `FollowPath` tourne normalement — jusqu'au
moment où un offset est réellement publié/appliqué en cours de route. À ce moment précis :

```
[WARN] extrapolation into the future ... looking up transform from frame [odom] to frame [map]
[WARN] controller_server: Unable to transform robot pose into global plan's frame
[INFO] controller_server: Passing new path to controller.
[WARN] controller_server: Resulting plan has 0 poses in it.   (répété ~17 fois sur ~500ms)
[ERROR] controller_server: Controller patience exceeded
[WARN] [follow_path] [ActionServer] Aborting handle.
[ERROR] bt_navigator: Goal failed
```

**Analyse** : c'est exactement la limite "pas de lissage" identifiée dès le départ dans le
brief initial — le décalage est appliqué comme un **saut instantané** sur tout le chemin
restant, pas une transition progressive. Le controller (`nav2_regulated_pure_pursuit_controller`)
cherche le point du plan le plus proche de la pose robot dans un rayon `max_robot_pose_search_dist`
(qui était à `1.0` m) ; combiné à un léger souci de timing TF (extrapolation de ~386µs,
probablement un symptôme du même instant plutôt qu'une cause indépendante) et au traitement
d'un plan de 178k poses fraîchement remplacé, le controller n'a pas retrouvé de point valide
assez vite, et `controller_patience` (pas explicitement réglé → défaut nav2 ~1.0s) a expiré
avant que ça se stabilise → tout le goal de navigation a été annulé (pas juste un warning
cosmétique cette fois, contrairement à ce qui était noté dans le brief d'origine).

**Fix appliqué (band-aid choisi explicitement, pas la vraie correction)** :
- `controller_patience: 3.0` (nouveau, dans `controller_server.ros__parameters`)
- `max_robot_pose_search_dist: 3.0` (remonté depuis `1.0`, dans `FollowPath`)

Ça donne plus de marge au controller pour absorber le saut sans abandonner le goal, mais **ça
ne corrige pas la cause racine** (le saut instantané reste un saut instantané). L'alternative
proposée et **non retenue pour l'instant** : lisser/étaler l'offset sur plusieurs ticks au lieu
de le sauter d'un coup dans `LateralOffsetCorrector` — à reprendre si le band-aid ne suffit
pas en pratique.

## État actuel / ce qui reste à valider sur le terrain

- Plugin chargé correctement, path énorme géré sans dépasser le budget de tick (bug #3 réglé).
- Le comportement une fois un offset **réellement appliqué en cours de route** n'a **pas encore
  été re-testé** depuis l'ajustement du bug #4 (`controller_patience`/`max_robot_pose_search_dist`)
  — c'est le prochain test à faire sur le terrain.
- Champ de test utilisé jusqu'ici : ~22.5 km / 178k poses (100 swaths) — probablement un champ
  de test non représentatif de la taille réelle d'un champ en usage normal ; si le comportement
  reste fragile, vaut le coup de retester avec un champ plus réaliste pour isoler "trop de
  data" de "vrai bug logique".

## Limites connues (pas encore adressées)

1. **Pas de lissage** : le décalage est appliqué comme un saut instantané, pas une transition
   progressive (voir bug #4). Le clamp ±0.5m limite l'ampleur du saut mais ne le rend pas
   progressif.
2. Le band-aid du bug #4 (`controller_patience`, `max_robot_pose_search_dist`) est un
   assouplissement global du controller, pas spécifique au moment d'un changement d'offset —
   à garder en tête si ça a un effet de bord ailleurs dans la navigation normale (sans offset).

## Fichiers touchés par cette session (résumé, pas les paths pré-existants d'avant)

- `eterry_navigation_custom_plugins/src/lateral_offset_corrector.cpp` (nouveau)
- `eterry_navigation_custom_plugins/CMakeLists.txt` (target + install ajoutés)
- `eterry_opennav_coverage/behavior_trees/resume_coverage.xml` (ReactiveSequence + LateralOffsetCorrector)
- `eterry_opennav_coverage/params/params.yaml` (plugin_lib_names, controller_patience, max_robot_pose_search_dist)

Note : les deux repos (`eterry_navigation_custom_plugins`, `eterry_opennav_coverage`) ont aussi
pas mal d'autres fichiers modifiés/non-trackés qui **ne viennent pas de cette session** (vu via
`git status` — `package.xml`, `coverage_path_trimmer.cpp`, plusieurs `launch/*.py`, des fichiers
`... copy.py/.xml`, etc.). Rien de tout ça n'a été touché ici — à garder à l'œil séparément si
besoin de faire un commit propre un jour.
