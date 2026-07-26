# Guía de ejecución paso a paso — Celda robótica IRB 2600 (ROS 2 + MoveIt 2)

Esta guía asume una VM **limpia** con Ubuntu 22.04 y te lleva, en orden,
desde clonar el repositorio hasta tener la simulación completa corriendo
con el panel de control de botones. Es la versión "solo los pasos, sin
explicaciones" — para el detalle técnico de cada pieza ver `README.md` en
esta misma carpeta.

Convención de terminales usada abajo: cada bloque indica en qué terminal
va (**A**, **B**, **C**...). Terminales marcadas como "queda abierta" no se
cierran hasta el final de la sesión de trabajo.

---

## 0. Requisitos previos

- Ubuntu 22.04 LTS (verificado; no usar 20.04 — ROS 2 Humble es para 22.04).
- Conexión a internet para instalar paquetes ROS 2 y clonar el repo.
- Acceso de administrador (`sudo`) en la VM.

---

## 1. Instalar ROS 2 Humble y dependencias del sistema

**Terminal A** (queda abierta, la usarás para el bringup más adelante):

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  python3-colcon-common-extensions python3-rosdep \
  python3-tk

sudo rosdep init 2>/dev/null || true
rosdep update
```

`python3-tk` es necesario para el panel de control con botones
(`gui_control_node`); si ya lo instalaste antes, `apt` simplemente dirá que
ya está instalado.

Todo lo anterior es software libre y gratuito (ROS 2, MoveIt 2, RViz) — no
hay licencias ni pagos involucrados.

---

## 2. Clonar el repositorio y ubicarse en la rama de trabajo

**Terminal A**:

```bash
cd ~
git clone https://github.com/JosAlc67/DimecaProject.git
cd DimecaProject
git checkout claude/clone-ramel-repository-gb8hxe
git pull origin claude/clone-ramel-repository-gb8hxe
```

Si ya tienes el repo clonado de una sesión anterior, en vez de `git clone`
simplemente:

```bash
cd ~/DimecaProject
git checkout claude/clone-ramel-repository-gb8hxe
git pull origin claude/clone-ramel-repository-gb8hxe
```

---

## 3. Compilar el workspace

**Terminal A**:

```bash
cd ~/DimecaProject/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`source install/setup.bash` solo aplica a **esta** terminal. Cada terminal
nueva que abras para correr algo de este workspace necesita su propio
`source` (paso 4 de cada bloque de abajo).

---

## 4. Verificación rápida (opcional pero recomendada la primera vez)

**Terminal A** (con el workspace ya compilado y sourceado):

```bash
ros2 launch irb2600_description display.launch.py
```

Debe abrirse RViz con el robot IRB 2600 visible, sin errores en la
terminal. Ciérralo (`Ctrl+C` en la terminal, o cerrar la ventana de RViz)
antes de seguir.

---

## 5. Levantar la celda completa

**Terminal A**:

```bash
cd ~/DimecaProject/ros2_ws
source install/setup.bash
ros2 launch irb2600_coating_cell coating_cell_bringup.launch.py 2>&1 | tee /tmp/bringup_log.txt
```

Esto levanta en un solo comando: MoveIt 2 + RViz + hardware simulado
(`ros2_control` mock) + la escena (panel objetivo + 3 obstáculos:
`scaffold_pole`, `tool_cart`, `cable_reel`) + la percepción simulada +
el servicio de `spray_on`.

**Espera hasta que RViz esté completamente abierto** (robot visible, panel
amarillo/violeta/naranja de obstáculos visible) antes de pasar al
siguiente paso. **Deja esta terminal y esta ventana de RViz abiertas** el
resto de la sesión — es tu "servidor" de simulación.

Diagnóstico si algo no aparece:
```bash
grep -iE "error|exception|fail|traceback" /tmp/bringup_log.txt
```

---

## 6. Abrir el panel de control con botones

**Terminal B** (nueva terminal):

```bash
cd ~/DimecaProject/ros2_ws
source install/setup.bash
ros2 run irb2600_coating_cell gui_control_node 2>&1 | tee /tmp/gui_control_log.txt
```

Se abre una ventana con tres botones:

- **Go Home**: regresa el brazo a la posición de reposo (todas las
  articulaciones en 0).
- **Start Route**: ejecuta la trayectoria de recubrimiento completa
  (6 filas) sobre el panel, fila por fila, con replanificación automática
  si algún obstáculo bloquea una fila. Mueve el robot de verdad.
- **Stop**: solo se habilita mientras "Go Home" o "Start Route" están
  corriendo. Cancela el movimiento en curso de inmediato (no espera a que
  termine) y apaga el spray si estaba activo.

Deja esta terminal y ventana abiertas mientras uses el panel.

**Uso recomendado (primera corrida):**
1. Clic en **Go Home** y espera a que la etiqueta diga "Ready." (el brazo
   debe quedar en la posición de reposo).
2. Clic en **Start Route** y observa en RViz cómo el robot recorre el
   panel fila por fila.
3. Si quieres probar la replanificación reactiva (Caso 3 del reporte),
   mueve un obstáculo desde la Terminal C (paso 7) **durante una de las
   pausas entre filas** (verás en la Terminal A el mensaje "Pausing Xs
   before the next row").

Diagnóstico:
```bash
grep -iE "error|exception|fail|traceback|rejected" /tmp/gui_control_log.txt
```

---

## 7. (Opcional) Mover un obstáculo a mano durante la ejecución

**Terminal C** (nueva terminal, solo si quieres provocar una
replanificación manualmente):

```bash
cd ~/DimecaProject/ros2_ws
source install/setup.bash
ros2 param set /perception_sim_node scaffold_pole.position "[0.79, 0.0, 1.0]"
```

Puedes usar `scaffold_pole`, `tool_cart` o `cable_reel` (los tres nombres
de obstáculos definidos en
`src/irb2600_coating_cell/config/scene_objects.yaml`). El obstáculo se
mueve en RViz de inmediato y `scene_setup_node` actualiza la escena de
planificación automáticamente — no hace falta ningún otro comando.

Para regresarlo a su posición original:
```bash
ros2 param set /perception_sim_node scaffold_pole.position "[1.25, 0.55, 1.05]"
```

---

## 8. (Opcional) Correr los nodos por CLI en vez del panel

Si prefieres no usar la GUI, los mismos comportamientos están disponibles
como nodos individuales (con la celda del paso 5 corriendo):

```bash
# Volver a home:
ros2 run irb2600_coating_cell go_home_node

# Ejecutar la ruta completa con replanificación (por defecto NO mueve el
# robot, solo planifica y reporta; agregar -p execute:=true para que sí
# lo mueva):
ros2 run irb2600_coating_cell replanning_executor_node --ros-args -p execute:=true

# Fase 1: trayectoria inicial contra un solo obstáculo estático (sin
# replanificación reactiva):
ros2 run irb2600_coating_cell trajectory_planner_node --ros-args -p execute:=true
```

---

## 9. Cómo cerrar todo al terminar

En orden inverso a como se abrió:

1. **Terminal C** (si la usaste): `Ctrl+C`, cerrar.
2. **Terminal B** (GUI): cerrar la ventana del panel o `Ctrl+C` en la
   terminal.
3. **Terminal A** (bringup): `Ctrl+C` en la terminal. Espera a que
   terminen de cerrarse todos los procesos (`move_group`, `ros2_control`,
   RViz) antes de cerrar la ventana de la terminal.

No dejes procesos de una corrida anterior corriendo en segundo plano antes
de volver a lanzar el paso 5 — si ves errores tipo "controller already
loaded" al relanzar, casi siempre es por una terminal anterior que quedó
abierta con el bringup todavía corriendo.

---

## Problemas comunes

| Síntoma | Causa / solución |
|---|---|
| `ModuleNotFoundError` o `Package not found` al correr `ros2 run`/`ros2 launch` | Falta `source install/setup.bash` en esa terminal (cada terminal nueva lo necesita). |
| GUI no abre, error de Tkinter | Falta `sudo apt install -y python3-tk` (paso 1). |
| "Controller already loaded" al lanzar el bringup | Hay una terminal anterior con el bringup todavía corriendo; ciérrala primero (paso 9). |
| El robot no se mueve al clic en "Start Route" desde CLI (`replanning_executor_node` sin GUI) | Falta el flag `--ros-args -p execute:=true` — por defecto ese nodo solo planifica. La GUI ya lo fuerza automáticamente. |
| Una fila se reporta como fallo seguro ("Table XVII Case 4") | Comportamiento esperado si el obstáculo bloquea demasiado esa fila incluso con los márgenes de reintento — no es un bug, es el sistema deteniéndose de forma segura en vez de intentar algo a ciegas. |

Para el detalle técnico completo (arquitectura, bugs resueltos durante el
desarrollo, qué se validó y qué queda pendiente de validación más
rigurosa) ver `README.md` en esta misma carpeta.
