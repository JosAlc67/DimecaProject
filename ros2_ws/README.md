# DIMECA — Celda robótica simulada IRB 2600 (ROS 2 + MoveIt 2)

Implementación del prototipo digital descrito en *Progress Report 1:
Conceptual Design and Simulation of a Robotic Cell for Coating Application
with Trajectory Replanning* (Grupo 1, MCTG1013). Puerto a ROS 2 Humble +
MoveIt 2 del robot ABB IRB 2600, reutilizando URDF/mallas de
[RAMEL-ESPOL/IRB2600-ABB](https://github.com/RAMEL-ESPOL/IRB2600-ABB)
(`external/IRB2600-ABB` en la raíz del repo, ROS 1 Noetic) como base
geométrica.

**Importante:** este workspace fue escrito y verificado sintácticamente en un
entorno sin ROS instalado (no hay `/opt/ros`, `colcon` ni `catkin_make`
disponibles aquí). Todo el xacro/YAML/Python sigue las convenciones estándar
de ROS 2 Humble + MoveIt 2 (Setup Assistant / `moveit_configs_utils`), pero
**no se pudo compilar ni lanzar realmente** — la primera vez que lo corras en
una máquina con ROS 2 es esperable tener que ajustar algo. Ver "Qué falta
validar" más abajo.

## 1. Alcance de esta fase

Cubre la escena base con obstáculo estático (Tabla XVII, Casos 1 y 2 del
reporte): robot + panel objetivo + un obstáculo fijo en la planning scene,
percepción simulada, señal `spray_on`, y generación/chequeo de colisión de
una trayectoria inicial tipo raster sobre el panel.

**No implementado todavía** (queda para la siguiente fase, cuando el
obstáculo se vuelva dinámico): recalcular automáticamente una trayectoria
alternativa cuando la inicial queda bloqueada (Tabla VI, fila "Replan the
trajectory"; Tabla XVII, Caso 3). Por ahora, si el path queda bloqueado,
`trajectory_planner_node` solo lo reporta (fracción de cobertura < 1.0), no
lo recalcula.

## 2. Paquetes

| Paquete | Contenido | Tabla del reporte |
|---|---|---|
| `irb2600_description` | URDF/xacro del IRB 2600 + pedestal + boquilla de spray, `ros2_control` con hardware mock (sin Gazebo) | Tabla XIV: robot, boquilla |
| `irb2600_moveit_config` | SRDF, kinematics, joint_limits, controllers, launch (`demo.launch.py`) | Tabla XIV: ROS 2 + MoveIt 2 |
| `irb2600_coating_cell` | `scene_setup_node` (escena), `perception_sim_node` (cámara RGB-D simulada), `spray_controller_node` (señal `spray_on`), `trajectory_planner_node` (trayectoria inicial + chequeo de colisión) | Tabla VI: funciones del sistema |

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
   arrastrar-y-planificar) tiene un bug conocido y no resuelto en varias
   instalaciones de ROS 2 Humble/MoveIt 2 (falla al cargar el modelo del robot
   con un error de tipos en `joint_limits`; ver
   [moveit2#1596](https://github.com/moveit/moveit2/issues/1596),
   [ros2/rviz#808](https://github.com/ros2/rviz/issues/808)), así que no se
   incluye en `config/moveit.rviz` — solo verás el robot y la escena, sin ese
   panel. La planificación/ejecución real de trayectorias se hace por código
   con `trajectory_planner_node` (paso 4), que nunca dependió de ese panel.
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

Para mover el obstáculo y volver a aplicar la escena (probar manualmente el
Caso 2 con otra posición):
```bash
ros2 param set /scene_setup_node obstacle.position "[0.5, 0.1, 1.0]"
ros2 service call /scene_setup_node/refresh_scene std_srvs/srv/Trigger
```

## 6. Qué falta validar (no se pudo probar en este entorno)

- **Build de `colcon`**: nombres de paquetes/dependencias en los `package.xml`
  no se verificaron contra `rosdep`. Si `rosdep install` o `colcon build`
  fallan por un nombre de paquete, es lo primero a revisar.
- **Alcanzabilidad del panel**: la posición del panel objetivo y del
  obstáculo en `irb2600_coating_cell/config/scene_objects.yaml` son valores
  de primera aproximación (no verificados contra el volumen de trabajo real
  del IRB 2600, hasta 1.85 m según la Tabla VII). Si MoveIt no encuentra
  solución IK, ajustar `target_structure.position`/`obstacle.position` ahí.
- **Matriz de colisiones permitidas (ACM)** en `irb2600_moveit_config/config/irb2600.srdf`:
  se completó a mano solo con pares de eslabones adyacentes, siguiendo el
  mismo patrón que las configuraciones ROS 1 originales de RAMEL. No es el
  muestreo automático que hace el MoveIt Setup Assistant. Correr el Setup
  Assistant localmente (pestaña "Self-Collisions") para regenerarla de forma
  más rigurosa es recomendable antes de reportar resultados finales.
- **Convención de orientación de la boquilla** (`ẑe` paralelo a `n̂s`, ec. 9
  del reporte): implementada literalmente como "eje Z local de la boquilla
  apunta en la misma dirección que la normal saliente de la superficie". Es
  una elección de convención razonable pero no verificada visualmente —
  confirmar en RViz que la boquilla efectivamente mira hacia el panel al
  correr `trajectory_planner_node`.

## 7. Créditos

URDF, mallas y kinemática del IRB 2600 y de la boquilla (portados desde
`ee_marker.xacro`) provienen de RAMEL-ESPOL/IRB2600-ABB (© 2022, Francisco
Yumbla, Javier Pagalo), incluido en este repositorio como referencia en
`external/IRB2600-ABB`.
