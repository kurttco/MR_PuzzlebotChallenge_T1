# Guía de sintonización del controlador

Este documento acompaña a `config/controller_params.yaml`. Describe el proceso
empírico para subir ganancias sobre el robot físico sin Gazebo, minimizando el
riesgo de movimientos erráticos.

## Principios

1. **Un parámetro a la vez.** Nunca subir Kp y Ki en el mismo run.
2. **Empezar en zona segura.** Las ganancias de arranque (ver más abajo)
   están deliberadamente bajas: el robot se moverá lento, pero nunca
   inestable.
3. **Goals chicos antes de goals grandes.** 30 cm antes de 1 m, 1 m
   antes de 2 m. Si algo sale mal a 30 cm, no hay tiempo de empotrarse.
4. **Terminal de pánico siempre lista.** Mantener abierta en otra pestaña:
   ```bash
   ros2 topic pub --rate 20 /VelocitySetL std_msgs/msg/Float32 "{data: 0.0}" &
   ros2 topic pub --rate 20 /VelocitySetR std_msgs/msg/Float32 "{data: 0.0}" &
   ```
   Un Ctrl-C al nodo controller ya publica ceros por el `finally`, pero esto
   sobreescribe cualquier setpoint residual aunque muera todo lo demás.
5. **Cada cambio de ganancia = un run documentado.** Anotar en una hoja:
   Kp, Ki, Kd, qué se observó (overshoot, tiempo de asentamiento, oscila?).

## Ganancias de arranque (ya escritas en el YAML)

| Parámetro | Valor inicial | Justificación |
|---|---:|---|
| `kp_ang` | 0.8 | Con error de π/2 rad produce ω ≈ 1.26, saturado a 0.8. Respuesta ágil sin romper. |
| `ki_ang` | 0.0 | No se añade hasta que el P esté tuneado. |
| `kd_ang` | 0.0 | Sólo si hay oscilación residual. |
| `kp_lin` | 0.4 | Con error de 1 m produce v = 0.4, saturado a v_max = 0.08. Sube lento y seguro. |
| `ki_lin` | 0.0 | Igual, lo último que se toca. |
| `v_max`  | 0.08 m/s | ~mitad del rango lineal (0.074 era el cmd=1 puro). |
| `omega_max` | 0.8 rad/s | Por debajo del límite físico observado (~1.0). |

## Proceso paso a paso

### Paso 1 — Validar odometría antes de mover el robot

Con el robot **en el aire** (ruedas libres), no ejecutar el controller,
sólo mandar cmd=1 manualmente y ver que `/robot_pose` refleje algo coherente:

```bash
# Terminal 1: agente + odometría sola
ros2 launch puzzlebot_mc2 mc2_part1_square.launch.py &
# (matar el controller y path_generator para que no publiquen)
pkill -f controller.py
pkill -f path_generator.py

# Terminal 2: forzar ruedas a 1.0
ros2 topic pub -1 /VelocitySetL std_msgs/msg/Float32 "{data: 1.0}"
ros2 topic pub -1 /VelocitySetR std_msgs/msg/Float32 "{data: 1.0}"

# Terminal 3: monitorear pose
ros2 topic echo /robot_pose
```

Las ruedas girando libres a cmd=1 deberían reportar `ω_rueda ≈ 1.64 rad/s`
en `/VelocityEncL` y `/VelocityEncR`. La pose debería avanzar en X a
~0.074 m/s. Si la escala es muy diferente (ej: está reportando RPM en
lugar de rad/s), hay que añadir un `encoder_scale` al nodo odometry.

### Paso 2 — Tunear el PID angular (solo)

Mandar un goal que requiera SÓLO rotación: un punto detrás del robot, o
justo al lado. Por ejemplo, dejar el robot en `(0, 0, 0°)` y mandar
`(0.01, 0.01)` — un punto casi en el origen pero ligeramente rotado.

Con `control_mode: sequential`, el controller se queda en
`ROTATE_TO_GOAL` hasta alinearse. Observar en consola:
- Si gira demasiado y vuelve (overshoot) → Kp muy alto.
- Si tarda eternamente en alcanzar la tolerancia → Kp muy bajo.
- Si nunca llega a cero (gira, se detiene, error residual) → falta Ki.

**Subir `kp_ang` en pasos de 0.3 hasta ver un overshoot leve, luego bajar
al 60% de ese valor.**

Si después hay error residual estacionario (ej: error = 0.03 rad pero el
robot ya no se mueve por la deadzone), añadir Ki:

```yaml
ki_ang: 0.05    # regla de arranque: kp/15 a kp/10
```

### Paso 3 — Tunear el PID lineal (sobre el angular ya tuneado)

Goal puramente recto enfrente del robot, ej: `(0.30, 0)` con el robot
apuntando a X+. Verás el controller saltar directo a `MOVE_TO_GOAL`.

- Si vibra al llegar → Kp muy alto.
- Si llega pero se queda corto (error residual de 10-20 cm) → falta Ki.
- Si llega y se pasa (overshoot traslacional) → bajar Kp.

**Subir `kp_lin` en pasos de 0.2 hasta ver overshoot, bajar al 60%.**

Añadir Ki si hay error residual por fricción estática:

```yaml
ki_lin: 0.05
```

### Paso 4 — Probar goal que combina rotación + avance

Un goal a 45° adelante, ej: `(0.50, 0.50)`. Verás la FSM transicionar
`ROTATE → MOVE → HOLD`. Si en alguno de los estados algo se descompone,
se tunea esa parte aislada regresando al Paso 2 o 3.

### Paso 5 — Cuadrado de 2 m

Cuando un solo goal funcione bien, lanzar la Parte 1. Observar los 4
segmentos completos. Los problemas típicos en este punto:

- **Error acumulativo** entre waypoints. La odometría tiene drift; el PID
  compensa, pero si el drift es grande (>10% por metro recorrido), vale
  la pena revisar las dimensiones del robot o calibrar mejor los encoders.
- **Sobregiro en las esquinas.** El robot gira de más, luego corrige
  hacia atrás. Típicamente es `kp_ang` muy alto en combinación con poco
  `hold_time`. Subir `hold_time` a 0.5 s.

### Paso 6 — Probar el modo simultáneo

Cambiar en el YAML:

```yaml
control_mode: "simultaneous"
```

Correr el mismo cuadrado. Van a pasar dos cosas:
1. Los `sim_kp_lin` y `sim_kp_ang` están aún más bajos, así que el primer
   run va a ser lento.
2. Como v y ω actúan simultáneamente, las transiciones entre segmentos
   son más suaves — pero el error transversal puede ser mayor durante
   el avance (el robot avanza mientras corrige el ángulo, lo que traza
   un arco en vez de dos segmentos rectos).

Subir `sim_kp_lin` y `sim_kp_ang` con la misma metodología del Paso 2-3.

### Paso 7 — Comparar los dos modos

Con `log_csv: true` ya están quedando los CSVs en `/tmp/puzzlebot_logs/`.
Después de un run en cada modo:

```bash
python3 ~/ros2_ws/src/puzzlebot_mc2/scripts/analyze.py \
    /tmp/puzzlebot_logs/run_sequential_*.csv \
    /tmp/puzzlebot_logs/run_simultaneous_*.csv
```

La tabla comparativa te va a decir cuál tuvo:
- Menor error medio y máximo de posición (precisión).
- Menor tiempo total (velocidad).
- Menor `smoothness` (qué tan bruscamente cambia ω — más bajo es más suave).
- Menor `control_energy` (qué tanto esfuerzo gastó el actuador).

**Tendencia esperada** en el PuzzleBot real (no es ley, es hipótesis):
el secuencial va a tener menor error final pero mayor tiempo total y
smoothness (movimiento más "cuadrado"). El simultáneo va a ser más
rápido y suave pero con error medio ligeramente mayor por el trazado
en arco.

## Síntomas comunes y qué tocar

| Síntoma | Probable causa | Acción |
|---|---|---|
| Se queda quieto y marca error = 0.04 m | deadzone + Ki=0 | subir `ki_lin` |
| Oscila alrededor del goal | Kp muy alto en fase HOLD | subir `hold_time` o bajar Kp |
| Gira bien pero avanza torcido | drift de odometría | revisar `wheel_radius`/`wheel_base`, validar encoders |
| Sobregira siempre en la misma dirección | asimetría de motores | añadir factor de corrección por rueda en el bridge |
| Llega, declara alcanzado, pero después se mueve solo | ruido en `/robot_pose` excede `epsilon_pos` | subir `epsilon_pos` a 0.07 m |
| `/goal_reached` llega pero el path generator no publica el siguiente | race condition, id no coincide | revisar logs de path_generator |

## Perturbaciones que compensa la ley de control

Esto sirve para el video. El controlador activamente compensa:

1. **No linealidad de deadzone.** El bridge traduce a comando de rueda y
   el PID sube la señal hasta salir de la zona muerta.
2. **Fricción variable según el piso.** El término integral acumula y
   empuja más comando cuando el robot se "pega".
3. **Asimetría entre motores.** Una rueda puede girar un poco más
   rápido que la otra con el mismo cmd. El P angular detecta la
   desviación y corrige.
4. **Ruido de encoders.** El filtro EMA en el término D atenúa las
   fluctuaciones muestra a muestra sin perder la tendencia.
5. **Saturación del actuador.** El clamping de `v_max`/`omega_max` más
   el anti-windup evita que el integral explote cuando el motor no puede
   responder más rápido.

## Criterios de robustez en orden de impacto

Si solo podés mencionar 3 en el video, estos son los que más pesan:

1. **Anti-windup del integral** — evita que, tras saturación, el robot
   sobrepase brutalmente al desaturarse.
2. **Histéresis de `hold_time`** — evita declarar "meta alcanzada" por
   un spike de ruido, lo que pararía prematuramente.
3. **Saturación consciente del actuador** — garantiza que los comandos
   publicados son físicamente realizables, y que el PID "sabe" el
   techo al que trabaja.
