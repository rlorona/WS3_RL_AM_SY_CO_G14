# Guion del video — Workshop 3 (Grupo 14)

Duración objetivo: **6–8 minutos**. Todo con **pantalla dividida**: Gazebo a la
izquierda, RViz a la derecha (y una terminal pequeña abajo si quieres mostrar los
comandos).

## Preparación (antes de grabar)

1. Cierra simulaciones viejas:
   ```bash
   pkill -f "gz sim"; pkill -f "parameter_bridge"; pkill -f "rviz2"
   ```
2. Ten 4 terminales listas, todas con:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/workshop_03/install/setup.bash
   ```
3. Acomoda las ventanas: Gazebo izquierda, RViz derecha, terminales abajo.
4. Sube un poco el zoom de la letra de la terminal (`Ctrl + Shift + +`).

---

## Toma 1 — Presentación (30 s)

- Muestra la pantalla con el repo y di: grupo 14, integrantes **Ricardo, Alexa,
  Steven y Cristhian**, iniciales **R A S C**.
- Menciona: Dingo-O, 4 ruedas mecanum a 45°, ROS 2 Jazzy + Gazebo Harmonic.

---

## Toma 2 — Modelo y cinemática (60 s)

- Abre `src/omni_workshop/omni_workshop/self_controller.py` y muestra
  `world_to_robot()` e `inverse_kinematics()`.
- Explica en una frase: *"paso la velocidad del mundo al frame del robot y luego
  reparto esa velocidad entre las cuatro ruedas con la matriz de 45°"*.
- Muestra la terminal con:
  ```bash
  python3 tools/test_kinematics.py
  ```
  y el mensaje **All kinematic checks passed**.

---

## Toma 3 — Figura propia (`grupo14`) (2 min)

**Terminal 1**
```bash
ros2 launch omni_workshop workshop.launch.py rviz:=true
```
Espera `Wheel controller active`.

**Terminal 2**
```bash
ros2 run omni_workshop self_controller --ros-args -p figure:=grupo14
```

Mientras corre:
- Señala en RViz las **tres rutas**: blanca/negra = deseada, azul = real en
  Gazebo, naranja = odometría.
- Di: *"la odometría se calcula solo con las ruedas; si una patina, no se entera,
  por eso se separa de la real"*.
- Al terminar, abre el PNG generado:
  ```bash
  eog ~/workshop_results/trajectory_*.png
  ```
  y lee el **RMS** de `actual` y de `odometry`.

---

## Toma 4 — Iniciales R A S C (2 min)

**Terminal 1** (relanza, ahora con la pose desplazada):
```bash
ros2 launch omni_workshop workshop.launch.py rviz:=true x:=-1.3 y:=-0.8
```

**Terminal 2**:
```bash
ros2 run omni_workshop self_controller --ros-args -p figure:=RASC
```

- Pon RViz en vista superior antes de arrancar (`T` y arrastra, o usa el botón
  *TopDownOrtho*).
- Comenta que el robot **no gira**: se traslada en x e y manteniendo la
  orientación, y así dibuja las cuatro letras.
- Al final, abre el PNG de la corrida.

---

## Toma 5 — Mapeo entorno pequeño (`office_0`) (1.5 min)

**Terminal 1**
```bash
ros2 launch omni_workshop simulation.launch.py world:=office_0
```
**Terminal 2**
```bash
ros2 launch omni_workshop mapping.launch.py
```

- Conduce con el panel **Omnidirectional teleop** (o `W/A/S/D/Q/E`).
- Muestra cómo el mapa se va formando en RViz.
- Guarda:
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_0 \
    --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
  ```

---

## Toma 6 — Mapeo entorno grande (`office_4`) (1.5 min)

Igual que la toma 5 con `world:=office_4`. Si el equipo va lento, usa
`office_1` y dilo: *"este es el entorno grande; si el equipo no da abasto se usa
office_1, de 19 × 11 m"*.

Cierra mostrando los dos `.pgm`:

```bash
ls -lh ~/workshop_results/maps/
eog ~/workshop_results/maps/office_0.pgm
```

---

## Toma 7 — Cierre (30 s)

- Muestra la tabla de errores del informe y los dos mapas.
- Menciona las respuestas del Markdown: diferencias, ventajas, propuesta
  (invernaderos de flores en Cayambe) e implementación en Ecuador.

---

## Consejos de grabación

- Graba por tomas y únelas después; no hace falta una sola toma continua.
- Si algo falla, corta y repite la toma: no dejes el error en el video.
- Antes de cada toma, espera a que Gazebo esté en `RTF ≈ 1` y RViz haya cargado
  el robot.
- No grabes la contraseña ni tokens si aparece una terminal con historial.
