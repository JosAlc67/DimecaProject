# DIMECA — Celda robótica simulada IRB 2600 (ROS 2 + MoveIt 2)

Implementación del prototipo digital descrito en *Progress Report 1:
Conceptual Design and Simulation of a Robotic Cell for Coating Application
with Trajectory Replanning* (Grupo 1, MCTG1013). Puerto a ROS 2 Humble +
MoveIt 2 del robot ABB IRB 2600, reutilizando URDF/mallas de
[RAMEL-ESPOL/IRB2600-ABB](https://github.com/RAMEL-ESPOL/IRB2600-ABB)
(`external/IRB2600-ABB` en la raíz del repo, ROS 1 Noetic) como base
geométrica.

**Estado:** desarrollado originalmente en un entorno sin ROS instalado (solo
verificado sintácticamente), y desde entonces **validado de punta a punta en
una VM con Ubuntu 22.04 + ROS 2 Humble** (2026-07-26). Ver la Sección 6 para
el detalle de qué se confirmó funcionando y qué sigue pendiente de revisión
más rigurosa.

### Resultados de validación en VM (2026-07-26)

- **Caso 1** (Tabla XVII — sin obstáculo bloqueando): `fraction=1.000`,
  cobertura completa del panel, trayectoria ejecutada con éxito.
- **Caso 2** (Tabla XVII — obstáculo bloquea parcialmente): `fraction=0.133`,
  el robot se detiene antes de tocar el obstáculo (confirmado numérica y
  visualmente en RViz). El sistema reporta la colisión en vez de ejecutar a
  ciegas, tal como especifica la Tabla VI ("Check collisions").
- **Caso 3** (obstáculo dinámico + replanificación automática): **validado en
  VM**. Corrida real con `replanning_executor_node` sobre 6 filas: Fila 1 y
  Fila 2 quedaron bloqueadas y el sistema replanificó y las ejecutó con éxito
  (Tabla VI "Replan the trajectory"); Fila 3 pasó directa sin bloqueo (como
  el Caso 1); Fila 4 quedó completamente bloqueada (`fraction=0.000`, sin
  margen para replanificar) y el sistema se detuvo de forma segura en vez de
  intentar algo a ciegas — esto es el **Caso 4** de la Tabla XVII
  ("Failed-trajectory report and safe robot stop"), también confirmado en la
  misma corrida.
- **Caso 4** (obstáculo bloquea el acceso por completo): confirmado junto
  con el Caso 3 arriba (Fila 4).

## 1. Alcance de esta fase

**Fase 1** (validada en VM): escena base con obstáculo estático (Tabla XVII,
Casos 1 y 2 del reporte): robot + panel objetivo + un obstáculo fijo en la
planning scene, percepción simulada, señal `spray_on`, y generación/chequeo
de colisión de una trayectoria inicial tipo raster sobre el panel
(`trajectory_planner_node`).

**Fase 2** (implementada, pendiente de validar en VM): replanificación
reactiva cuando el obstáculo cambia de posición durante la ejecución (Tabla
VI, fila "Replan the trajectory"; Tabla XVII, Caso 3), en
`replanning_executor_node` — ejecuta la trayectoria fila por fila,
revalidando cada una contra la escena actual antes de moverse, y si una fila
queda bloqueada intenta una ruta alternativa (IK + planificación OMPL en
espacio de articulaciones) antes de reportar fallo seguro (Caso 4).

## 2. Paquetes

| Paquete | Contenido | Tabla del reporte |
|---|---|---|
| `irb2600_description` | URDF/xacro del IRB 2600 + pedestal + boquilla de spray, `ros2_control` con hardware mock (sin Gazebo) | Tabla XIV: robot, boquilla |
| `irb2600_moveit_config` | SRDF, kinematics, joint_limits, controllers, launch (`demo.launch.py`) | Tabla XIV: ROS 2 + MoveIt 2 |
| `irb2600_coating_cell` | `scene_setup_node` (escena), `perception_sim_node` (cámara RGB-D simulada), `spray_controller_node` (señal `spray_on`), `trajectory_planner_node` (trayectoria inicial + chequeo de colisión, Fase 1), `replanning_executor_node` (ejecución por filas + replanificación reactiva, Fase 2) | Tabla VI: funciones del sistema |

## 3. Instalación de dependencias (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  python3-colcon-common-extensions python3-rosdep

sudo rosdep init 2>/dev/null || true
rosdep update
```

Todo el software usado (ROS 2, MoveIt 2, RViz) es gratuito y de código
abierto — no hay licencias ni costos involucrados.

## 4. Compilar

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 5. Cómo correrlo (en orden)

1. **Smoke test del URDF** (sin MoveIt, confirma que el xacro y las mallas cargan):
   ```bash
   ros2 launch irb2600_description display.launch.py
   ```
2. **MoveIt solo** (RViz con el robot + planning scene, hardware mock; confirma
   que `move_group` y `ros2_control` levantan bien):
   ```bash
   ros2 launch irb2600_moveit_config demo.launch.py
   ```
   Nota: el panel interactivo `moveit_rviz_plugin/MotionPlanning` (el de
   arrastrar-y-planificar) y el display `moveit_rviz_plugin/PlanningScene`
   tienen bugs conocidos y no resueltos en esta instalación de ROS 2
   Humble/MoveIt 2 (fallan al cargar el modelo del robot / nunca se
   suscriben al topic de la escena; ver
   [moveit2#1596](https://github.com/moveit/moveit2/issues/1596),
   [ros2/rviz#808](https://github.com/ros2/rviz/issues/808)), así que no se
   usan en `config/moveit.rviz` — el robot se ve vía `RobotModel` normal, y
   los objetos de la escena vía un `MarkerArray` propio publicado por
   `scene_setup_node` (ver Sección 6). La planificación/ejecución real de
   trayectorias se hace por código con `trajectory_planner_node` (paso 4),
   que nunca dependió de ninguno de esos plugins.
3. **Celda completa** (MoveIt + panel + obstáculo + percepción simulada + `spray_on`):
   ```bash
   ros2 launch irb2600_coating_cell coating_cell_bringup.launch.py
   ```
4. **Generar la trayectoria inicial** (en otra terminal, con (3) corriendo):
   ```bash
   # Solo calcula y reporta fracción de cobertura / longitud L:
   ros2 run irb2600_coating_cell trajectory_planner_node

   # Calcula y además la ejecuta en el hardware mock (mueve el robot en RViz):
   ros2 run irb2600_coating_cell trajectory_planner_node --ros-args -p execute:=true
   ```

Para mover el obstáculo (probar manualmente el Caso 2 con otra posición),
`scene_setup_node` está suscrito a `/obstacle_pose` de `perception_sim_node`
y vuelve a aplicar la escena automáticamente en cuanto cambia — basta con:
```bash
ros2 param set /perception_sim_node obstacle.position "[0.5, 0.1, 1.0]"
```
(Si corres `scene_setup_node` sin `perception_sim_node`, usa el método
manual como respaldo: `ros2 param set /scene_setup_node obstacle.position
"[...]"` seguido de `ros2 service call /scene_setup_node/refresh_scene
std_srvs/srv/Trigger`.)

## 5b. Probar el Caso 3 (obstáculo dinámico + replanificación)

Con la celda completa corriendo (paso 3), y el obstáculo en una posición que
bloquee alguna fila del panel (por ejemplo la posición por defecto
`[0.5, 0.3, 1.0]`), corre:

```bash
ros2 run irb2600_coating_cell replanning_executor_node --ros-args -p execute:=true
```

El nodo ejecuta el panel fila por fila, imprimiendo por cada una si el paso
fue directo o si tuvo que replanificar, y pausa unos segundos entre filas
(`segment_pause_s`, por defecto 3 s) para dar tiempo a mover el obstáculo a
mano y ver cómo reacciona la siguiente fila:

```bash
# En otra terminal, durante la pausa entre filas:
ros2 param set /perception_sim_node obstacle.position "[0.79, 0.0, 1.0]"
```

Al final imprime un resumen (`Summary: N row(s) direct, M row(s) replanned,
...`). Si una fila queda genuinamente inalcanzable ni replanificando, el nodo
la reporta como fallo seguro (Caso 4 de la Tabla XVII) y se detiene ahí en
vez de seguir a ciegas.

## 6. Qué se validó en VM y qué sigue pendiente

**Confirmado funcionando (Ubuntu 22.04 + ROS 2 Humble, 2026-07-26):**

- `colcon build` compila los 3 paquetes sin errores de dependencias.
- `display.launch.py`, `demo.launch.py` y `coating_cell_bringup.launch.py`
  levantan correctamente (robot, `move_group`, `ros2_control`, escena).
- **Alcanzabilidad del panel**: confirmada — Caso 1 cubre el 100% del panel
  (`fraction=1.000`) con la posición actual en
  `irb2600_coating_cell/config/scene_objects.yaml`.
- **Orientación de la boquilla**: corregida (el vector de aproximación apunta
  hacia la superficie, antiparalelo a la normal saliente `n̂s`; si se calcula
  `theta_error` de la ec. 9 como métrica más adelante, debe medirse contra
  `-n̂s`).
- **Visualización de la escena**: `moveit_rviz_plugin/PlanningScene` nunca se
  suscribe a `/monitored_planning_scene` en esta instalación (confirmado con
  `ros2 topic info`: 1 publisher, 0 subscribers) — se reemplazó por un
  `MarkerArray` propio publicado por `scene_setup_node`
  (`~/scene_markers`), que sí funciona de forma confiable.
- **Caso 3 y Caso 4** (`replanning_executor_node`, Fase 2): confirmados en
  una misma corrida — ver el resumen al inicio de este documento. La pieza
  clave que lo hizo funcionar
  fue sembrar el intento de IK de replanificación con la última
  configuración articular que sí alcanzó el camino cartesiano parcial (no
  con la posición "de reposo"), y apuntar a un punto interpolado un poco
  antes del extremo original de la fila en vez del punto exacto — ver los
  comentarios en `_replan_row` para el detalle completo del diagnóstico.
- **Sensor simulado conectado a la escena** (no probado en VM todavía,
  implementado justo después de la corrida de arriba): `scene_setup_node`
  ahora se suscribe a `/obstacle_pose` y `/workspace_clear` de
  `perception_sim_node` y vuelve a aplicar la escena automáticamente cuando
  la posición del obstáculo cambia — mover el obstáculo ya no requiere el
  paso manual de `refresh_scene`, solo cambiar el parámetro en
  `perception_sim_node` (ver Sección 5). Repórtame errores si esto falla en
  tu próxima corrida, igual que con todo lo anterior.

**Pendiente / no verificado rigurosamente:**

- **Matriz de colisiones permitidas (ACM)** en `irb2600_moveit_config/config/irb2600.srdf`:
  se completó a mano solo con pares de eslabones adyacentes, siguiendo el
  mismo patrón que las configuraciones ROS 1 originales de RAMEL. No es el
  muestreo automático que hace el MoveIt Setup Assistant. Correr el Setup
  Assistant localmente (pestaña "Self-Collisions") para regenerarla de forma
  más rigurosa es recomendable antes de reportar resultados finales, aunque
  no ha causado problemas en las pruebas de los Casos 1 a 4.
- **Detección de colisión discreta, no continua**: tanto `trajectory_planner_node`
  como `replanning_executor_node` revisan la escena una sola vez por fila,
  justo antes de empezar a moverla (coincide con el diseño explícito del
  reporte, Tabla IX: "Replanning type: Discrete/reactive"). Si el obstáculo
  se moviera **a mitad de una fila** en ejecución, no se detectaría hasta
  que esa fila termine. Un sistema con monitoreo continuo necesitaría
  revisar la escena durante la ejecución, no solo antes de cada segmento.
  Esto sigue pendiente — lo de abajo solo conecta el sensor a la escena, no
  cambia cuándo se revisa.
- Errores cosméticos que persisten en los logs sin afectar el funcionamiento:
  advertencia "No 3D sensor plugin(s) defined for octomap updates" (no
  usamos octomap) y "unrealistic inertia" por eslabón (RViz avisando que no
  puede dibujar una caja de inercia auxiliar; no afecta física ni planificación).

## 7. Créditos

URDF, mallas y kinemática del IRB 2600 y de la boquilla (portados desde
`ee_marker.xacro`) provienen de RAMEL-ESPOL/IRB2600-ABB (© 2022, Francisco
Yumbla, Javier Pagalo), incluido en este repositorio como referencia en
`external/IRB2600-ABB`.
