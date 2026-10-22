# taller_omni — Taller de robot omnidireccional

Taller individual: cada persona programa el **modelo cinemático** del Clearpath Dingo-O y hace que el robot recorra **su propia trayectoria** en el simulador. El robot y sus medidas son los mismos para todos; lo que cambia es la trayectoria.

La guía paso a paso, con la teoría y pruebas automáticas, está en el notebook [`notebook/Taller_Robot_Omnidireccional.ipynb`](notebook/Taller_Robot_Omnidireccional.ipynb). Este README es el resumen.

## Qué se programa

Solo se edita **`taller_omni/modelo.py`**. Los nodos de ROS 2 ya están escritos y registrados en `setup.py`.

| Paso | Función en `modelo.py` | Qué recibe el robot |
|---|---|---|
| 0 | `trayectoria(t)`: tu trayectoria asignada y sus derivadas | — |
| 1 | `mundo_a_robot()`: velocidades del mundo → marco del robot | `cmd_vel` (vx, vy, ω) |
| 2 | `cinematica_inversa()`: (vx, vy, ω) → velocidad de cada rueda | velocidad de cada rueda |
| 3 | `controlador()`: corrección proporcional con la pose real | velocidad de cada rueda |

Medidas del robot: `R = 0.05 m` (radio de rueda), `LX = 0.285 m`, `LY = 0.1854 m`.

## Trayectoria asignada

Según tu número de lista *n*: **forma** = ((n − 1) mod 6) + 1 y **modo** = A, B o C según ⌊(n − 1)/6⌋ mod 3.

| n | Forma | Modo |
|---|---|---|
| 1 – 6 | 1 a 6 | A |
| 7 – 12 | 1 a 6 | B |
| 13 – 18 | 1 a 6 | C |

| Forma | Descripción (empieza en el origen, sentido antihorario) |
|---|---|
| 1 | Círculo de radio 0.8 m con centro en (0, 0.8) |
| 2 | Elipse de centro (0, 0.7), semiejes 1.2 m (x) y 0.7 m (y) |
| 3 | Ocho vertical: x oscila 2 veces con amplitud 0.6 m mientras y va de 0 a 1.8 m y vuelve |
| 4 | Cuadrado: (0,0) → (1.2,0) → (1.2,1.2) → (0,1.2) → (0,0) |
| 5 | Triángulo equilátero: (0,0) → (1.4,0) → (0.7,1.212) → (0,0) |
| 6 | Rombo: (0,0) → (0.8,0.9) → (0,1.8) → (−0.8,0.9) → (0,0) |

| Modo | Orientación θ(t) |
|---|---|
| A | fija: θ = 0 |
| B | una vuelta antihoraria: θ = 2πt/T |
| C | oscila ±45°: θ = (π/4)·sin(2πt/T) |

Todas duran T = 30 s, empiezan y terminan en el origen, y en los polígonos la velocidad es constante.

## Cómo ejecutar

Tres terminales, cada una con `cd ~/practica_03 && source install/setup.bash`:

```bash
# Terminal 1: simulador (nivel:=1 para el paso 1; nivel:=2 para los pasos 2 y 3)
ros2 launch taller_omni taller.launch.py nivel:=1

# Terminal 2: graba y guarda el gráfico (deseada vs real) en ~/taller_resultados/
ros2 run taller_omni graficar

# Terminal 3: tu código
ros2 run taller_omni nivel1          # o nivel2 / nivel3
```

- Antes de cada intento: `ros2 run taller_omni reiniciar` (devuelve el robot al inicio, dentro de la jaula de drones).
- Para probar sin haber escrito tu trayectoria: `ros2 run taller_omni nivel1 --ros-args -p ejemplo:=true` (círculo de ejemplo).
- Después de editar `modelo.py` no hace falta recompilar: detén el nodo y vuelve a ejecutarlo.
- Con `nivel:=2` el controlador de Clearpath se reemplaza por uno que recibe la velocidad de cada rueda (`/do100_0000/wheel_velocity_controller/commands`). En ese modo el panel de teleoperación no mueve el robot.

## Archivos del paquete

| Archivo | Para qué |
|---|---|
| `taller_omni/modelo.py` | **plantilla del alumno** |
| `taller_omni/nivel1.py`, `nivel2.py`, `nivel3.py` | nodos de cada nivel (llaman a `modelo.py`) |
| `taller_omni/nodo_base.py` | lógica común: reloj, marco inicial, referencia |
| `taller_omni/pose_real.py` | publica la pose real del robot desde Gazebo en `/taller/pose_real` |
| `taller_omni/graficar.py` | gráfico y error RMS de cada intento |
| `taller_omni/reiniciar.py` | devuelve el robot al punto de inicio |
| `launch/taller.launch.py` | simulador + pose real + cambio de controlador según el nivel |
| `config/ruedas_controller.yaml` | controlador por rueda de los niveles 2 y 3 |
| `notebook/` | guía del taller |

## Problemas frecuentes

| Síntoma | Causa probable |
|---|---|
| `Package 'taller_omni' not found` | falta `source install/setup.bash` en esa terminal |
| El nodo se queda en *"Esperando la pose del robot"* | el simulador no está corriendo |
| `Paso 0 / Nivel X: completa ... en modelo.py` | esa función todavía tiene el `raise NotImplementedError` |
| Niveles 2/3: el robot no se mueve | el simulador se lanzó con `nivel:=1`; relanzarlo con `nivel:=2` |
| El robot choca con la red de la jaula | no se reinició antes del intento: `ros2 run taller_omni reiniciar` |
