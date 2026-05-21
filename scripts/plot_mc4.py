#!/usr/bin/env python3
"""
plot_mc4.py
-----------
Genera las graficas del reporte a partir de un CSV del controller MC4.

Uso:
    python3 plot_mc4.py /tmp/puzzlebot_logs/run_sequential_sem_XXXX.csv
    python3 plot_mc4.py <csv> --outdir ./plots --epsilon 0.05

Salidas (en --outdir, default: carpeta del CSV):
    01_trajectory.png         trayectoria XY con waypoints y estado del semaforo
    02_position_error.png     error de posicion vs tiempo
    03_angle_error.png        error angular vs tiempo
    04_velocity_cmds.png      v y w comandadas: lo que pidio el PID vs lo publicado
    05_semaphore_timeline.png linea de tiempo del estado del semaforo y el latch
    06_dashboard.png          panel resumen con las graficas clave
    metrics.txt               metricas numericas del run

Requiere numpy y matplotlib. Es una herramienta de analisis OFFLINE: no
forma parte del codigo que corre en el robot, asi que el uso de matplotlib
no contradice la regla de "solo librerias estandar o NumPy" del reto.
"""

import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# Colores por estado del semaforo
SEM_COLORS = {
    'UNKNOWN': '#9AA5B1',
    'GREEN':   '#06A77D',
    'YELLOW':  '#F4A261',
    'RED':     '#D62828',
}
ACCENT = '#0077B6'
ACCENT_LIGHT = '#00B4D8'


# ============================================================
# carga del CSV
# ============================================================

def load_csv(path):
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        for r in reader:
            rows.append(r)
    return rows, fieldnames


def col_float(rows, key, default=0.0):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(key, default)))
        except (ValueError, TypeError):
            out.append(default)
    return np.array(out, dtype=float)


def col_str(rows, key, default=''):
    return np.array([str(r.get(key, default)) for r in rows])


def find_segments(mask, t):
    """Lista de (t_ini, t_fin) para regiones contiguas donde mask es True."""
    segments = []
    in_seg = False
    seg_start = None
    for i in range(len(mask)):
        if mask[i] and not in_seg:
            in_seg = True
            seg_start = t[i]
        elif not mask[i] and in_seg:
            in_seg = False
            segments.append((seg_start, t[i]))
    if in_seg:
        segments.append((seg_start, t[-1]))
    return segments


def shade_red(ax, red_mask, t, label='RED / latch'):
    """Sombra las regiones donde el robot estuvo bloqueado por rojo."""
    segs = find_segments(red_mask, t)
    first = True
    for (a, b) in segs:
        ax.axvspan(a, b, color=SEM_COLORS['RED'], alpha=0.15,
                   label=label if first else None)
        first = False
    return len(segs)


# ============================================================
# graficas
# ============================================================

def plot_trajectory(data, outdir):
    t = data['t']
    x = data['x']
    y = data['y']
    sem = data['sem_state']
    gx = data['goal_x']
    gy = data['goal_y']

    fig, ax = plt.subplots(figsize=(7, 7))

    # path completo en gris fino
    ax.plot(x, y, color='#C2CBD4', linewidth=1.0, zorder=1)

    # puntos coloreados por estado del semaforo
    for state, color in SEM_COLORS.items():
        m = (sem == state)
        if np.any(m):
            ax.scatter(x[m], y[m], c=color, s=10, zorder=2,
                       label=f'sem: {state}')

    # waypoints unicos (en orden de aparicion)
    seen = []
    for i in range(len(gx)):
        p = (round(gx[i], 3), round(gy[i], 3))
        if p not in seen:
            seen.append(p)
    if seen:
        wx = [p[0] for p in seen]
        wy = [p[1] for p in seen]
        ax.scatter(wx, wy, marker='*', s=320, c='#FFD60A',
                   edgecolors='#1A2332', linewidths=1.2, zorder=4,
                   label='waypoints')
        for i, (px, py) in enumerate(seen):
            ax.annotate(f'wp{i}', (px, py), textcoords='offset points',
                        xytext=(8, 8), fontsize=9, fontweight='bold')

    # inicio y fin reales del recorrido
    ax.scatter([x[0]], [y[0]], marker='o', s=120, c='#06A77D',
               edgecolors='#1A2332', linewidths=1.2, zorder=5, label='inicio')
    ax.scatter([x[-1]], [y[-1]], marker='s', s=120, c='#D62828',
               edgecolors='#1A2332', linewidths=1.2, zorder=5, label='fin')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Trayectoria recorrida y estado del semaforo')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8, framealpha=0.9)
    fig.tight_layout()
    path = os.path.join(outdir, '01_trajectory.png')
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def plot_position_error(data, outdir, epsilon):
    t = data['t']
    err = data['error_pos']
    red_mask = data['red_active']

    fig, ax = plt.subplots(figsize=(9, 4))
    shade_red(ax, red_mask, t)
    ax.plot(t, err, color=ACCENT, linewidth=1.4, label='error de posicion')
    ax.axhline(epsilon, color='#06A77D', linestyle='--', linewidth=1.0,
               label=f'tolerancia ({epsilon} m)')
    ax.set_xlabel('tiempo [s]')
    ax.set_ylabel('error de posicion [m]')
    ax.set_title('Error de posicion vs tiempo')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)
    ax.set_ylim(bottom=0)
    fig.tight_layout()
    path = os.path.join(outdir, '02_position_error.png')
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def plot_angle_error(data, outdir):
    t = data['t']
    err_deg = np.degrees(data['error_ang'])
    red_mask = data['red_active']

    fig, ax = plt.subplots(figsize=(9, 4))
    shade_red(ax, red_mask, t)
    ax.plot(t, err_deg, color='#7B2CBF', linewidth=1.2,
            label='error angular')
    ax.axhline(0, color='#888888', linewidth=0.8)
    ax.set_xlabel('tiempo [s]')
    ax.set_ylabel('error angular [grados]')
    ax.set_title('Error angular vs tiempo')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)
    fig.tight_layout()
    path = os.path.join(outdir, '03_angle_error.png')
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def plot_velocity_cmds(data, outdir):
    t = data['t']
    v_pre = data['v_cmd_pre_sem']
    v_post = data['v_cmd']
    w_pre = data['w_cmd_pre_sem']
    w_post = data['w_cmd']
    red_mask = data['red_active']

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    # velocidad lineal
    shade_red(ax1, red_mask, t)
    ax1.plot(t, v_pre, color='#C2CBD4', linewidth=1.6, linestyle='--',
             label='v pedida por el PID')
    ax1.plot(t, v_post, color=ACCENT, linewidth=1.4,
             label='v publicada (post semaforo + slew)')
    ax1.set_ylabel('v lineal [m/s]')
    ax1.set_title('Comando de velocidad: lo que el PID pidio vs lo que se publico')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=8)

    # velocidad angular
    shade_red(ax2, red_mask, t)
    ax2.plot(t, w_pre, color='#C2CBD4', linewidth=1.6, linestyle='--',
             label='w pedida por el PID')
    ax2.plot(t, w_post, color='#7B2CBF', linewidth=1.4,
             label='w publicada')
    ax2.set_xlabel('tiempo [s]')
    ax2.set_ylabel('w angular [rad/s]')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=8)

    fig.tight_layout()
    path = os.path.join(outdir, '04_velocity_cmds.png')
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def plot_semaphore_timeline(data, outdir):
    t = data['t']
    sem = data['sem_state']
    red_latched = data['red_latched']

    fig, ax = plt.subplots(figsize=(9, 2.6))

    # banda de color por estado
    state_to_y = {'UNKNOWN': 0, 'GREEN': 1, 'YELLOW': 2, 'RED': 3}
    for i in range(len(t) - 1):
        s = sem[i]
        color = SEM_COLORS.get(s, '#9AA5B1')
        ax.axvspan(t[i], t[i + 1], color=color, alpha=0.85)

    # linea del latch
    ax.plot(t, red_latched * 0.5 + 0.25, color='#1A2332', linewidth=1.5,
            drawstyle='steps-post', label='red_latched (0/1)')

    ax.set_yticks([])
    ax.set_xlabel('tiempo [s]')
    ax.set_title('Linea de tiempo del semaforo (color = estado detectado)')

    # leyenda manual
    from matplotlib.patches import Patch
    legend_items = [Patch(facecolor=c, label=s) for s, c in SEM_COLORS.items()]
    legend_items.append(plt.Line2D([0], [0], color='#1A2332',
                                   linewidth=1.5, label='red_latched'))
    ax.legend(handles=legend_items, loc='upper right', fontsize=8, ncol=5,
              framealpha=0.95)
    fig.tight_layout()
    path = os.path.join(outdir, '05_semaphore_timeline.png')
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def plot_dashboard(data, outdir, epsilon):
    t = data['t']
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.22)

    # --- trayectoria ---
    ax0 = fig.add_subplot(gs[:, 0])
    x, y, sem = data['x'], data['y'], data['sem_state']
    ax0.plot(x, y, color='#C2CBD4', linewidth=1.0)
    for state, color in SEM_COLORS.items():
        m = (sem == state)
        if np.any(m):
            ax0.scatter(x[m], y[m], c=color, s=10, label=state)
    ax0.scatter([x[0]], [y[0]], marker='o', s=110, c='#06A77D',
                edgecolors='#1A2332', zorder=5)
    ax0.scatter([x[-1]], [y[-1]], marker='s', s=110, c='#D62828',
                edgecolors='#1A2332', zorder=5)
    ax0.set_title('Trayectoria recorrida')
    ax0.set_xlabel('x [m]'); ax0.set_ylabel('y [m]')
    ax0.set_aspect('equal', adjustable='datalim')
    ax0.grid(True, alpha=0.3)
    ax0.legend(fontsize=7, loc='best')

    # --- error de posicion ---
    ax1 = fig.add_subplot(gs[0, 1])
    shade_red(ax1, data['red_active'], t)
    ax1.plot(t, data['error_pos'], color=ACCENT, linewidth=1.2)
    ax1.axhline(epsilon, color='#06A77D', linestyle='--', linewidth=1.0)
    ax1.set_title('Error de posicion [m]')
    ax1.set_xlabel('t [s]')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(bottom=0)

    # --- velocidad ---
    ax2 = fig.add_subplot(gs[1, 1])
    shade_red(ax2, data['red_active'], t)
    ax2.plot(t, data['v_cmd_pre_sem'], color='#C2CBD4', linestyle='--',
             linewidth=1.4, label='v pedida')
    ax2.plot(t, data['v_cmd'], color=ACCENT, linewidth=1.2,
             label='v publicada')
    ax2.set_title('Comando de velocidad lineal [m/s]')
    ax2.set_xlabel('t [s]')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=7, loc='best')

    fig.suptitle('MC4 - Panel resumen del recorrido', fontsize=14,
                 fontweight='bold')
    path = os.path.join(outdir, '06_dashboard.png')
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# ============================================================
# metricas
# ============================================================

def compute_metrics(data, epsilon):
    t = data['t']
    err = data['error_pos']
    err_ang = np.abs(data['error_ang'])
    red_mask = data['red_active']
    sem = data['sem_state']
    goal_id = data['goal_id']

    total_time = float(t[-1] - t[0]) if len(t) > 1 else 0.0

    # tiempo bloqueado por rojo
    red_time = 0.0
    for i in range(len(t) - 1):
        if red_mask[i]:
            red_time += (t[i + 1] - t[i])

    # tiempo efectivo de movimiento
    moving_time = total_time - red_time

    # error final por goal: ultima fila de cada goal_id
    final_errors = {}
    for i in range(len(goal_id)):
        gid = int(goal_id[i])
        if gid >= 0:
            final_errors[gid] = err[i]

    # smoothness: integral de |dw/dt|
    w = data['w_cmd']
    if len(t) > 1:
        dt = np.diff(t)
        dt_safe = np.where(dt > 0, dt, np.nan)
        dwdt = np.diff(w) / dt_safe
        smoothness = float(np.nansum(np.abs(dwdt) * dt_safe))
    else:
        smoothness = 0.0

    # cuantos estados unicos de semaforo se detectaron
    states_seen = sorted(set(sem.tolist()))

    return {
        'total_time_s': total_time,
        'red_blocked_time_s': red_time,
        'moving_time_s': moving_time,
        'red_blocked_pct': (100.0 * red_time / total_time) if total_time > 0 else 0.0,
        'mean_pos_error_m': float(np.mean(err)),
        'max_pos_error_m': float(np.max(err)),
        'mean_abs_ang_error_deg': float(np.degrees(np.mean(err_ang))),
        'final_errors': final_errors,
        'smoothness': smoothness,
        'states_detected': states_seen,
        'n_samples': len(t),
    }


def write_metrics(metrics, outdir):
    lines = []
    lines.append('=' * 52)
    lines.append('  MC4 - Metricas del recorrido')
    lines.append('=' * 52)
    lines.append(f"  Muestras totales        : {metrics['n_samples']}")
    lines.append(f"  Tiempo total            : {metrics['total_time_s']:.2f} s")
    lines.append(f"  Tiempo en movimiento    : {metrics['moving_time_s']:.2f} s")
    lines.append(f"  Tiempo bloqueado (rojo) : {metrics['red_blocked_time_s']:.2f} s "
                 f"({metrics['red_blocked_pct']:.1f}%)")
    lines.append('')
    lines.append(f"  Error medio de posicion : {metrics['mean_pos_error_m']:.4f} m")
    lines.append(f"  Error maximo de posicion: {metrics['max_pos_error_m']:.4f} m")
    lines.append(f"  Error angular medio     : {metrics['mean_abs_ang_error_deg']:.2f} grados")
    lines.append(f"  Suavidad (int|dw/dt|)   : {metrics['smoothness']:.3f}")
    lines.append('')
    lines.append(f"  Estados de semaforo     : {', '.join(metrics['states_detected'])}")
    lines.append('')
    lines.append('  Error final por waypoint:')
    for gid in sorted(metrics['final_errors'].keys()):
        lines.append(f"    goal {gid:>2d} : {metrics['final_errors'][gid]:.4f} m")
    lines.append('=' * 52)
    text = '\n'.join(lines)

    path = os.path.join(outdir, 'metrics.txt')
    with open(path, 'w') as f:
        f.write(text + '\n')
    print(text)
    return path


# ============================================================
# main
# ============================================================

def main():
    ap = argparse.ArgumentParser(description='Genera plots del reporte MC4.')
    ap.add_argument('csv', help='ruta al CSV del controller')
    ap.add_argument('--outdir', default=None,
                    help='carpeta de salida (default: junto al CSV)')
    ap.add_argument('--epsilon', type=float, default=0.05,
                    help='tolerancia de posicion para la linea de referencia')
    args = ap.parse_args()

    if not os.path.isfile(args.csv):
        print(f'ERROR: no existe el archivo {args.csv}')
        sys.exit(1)

    outdir = args.outdir or os.path.dirname(os.path.abspath(args.csv))
    os.makedirs(outdir, exist_ok=True)

    rows, fieldnames = load_csv(args.csv)
    if not rows:
        print('ERROR: el CSV esta vacio')
        sys.exit(1)

    print(f'Cargado {args.csv} ({len(rows)} filas)')
    print(f'Columnas: {fieldnames}')

    # armar el diccionario de datos. Si faltan columnas (CSV viejo de MC3),
    # se rellenan con ceros / UNKNOWN para que el script no truene.
    has_sem = 'sem_state' in fieldnames
    if not has_sem:
        print('AVISO: el CSV no tiene columnas de semaforo (parece de MC3). '
              'Los plots del semaforo saldran vacios.')

    sem_state = (col_str(rows, 'sem_state', 'UNKNOWN') if has_sem
                 else np.array(['UNKNOWN'] * len(rows)))
    red_latched = (col_float(rows, 'red_latched', 0.0) if has_sem
                   else np.zeros(len(rows)))

    data = {
        't': col_float(rows, 't'),
        'x': col_float(rows, 'x'),
        'y': col_float(rows, 'y'),
        'theta': col_float(rows, 'theta'),
        'goal_x': col_float(rows, 'goal_x'),
        'goal_y': col_float(rows, 'goal_y'),
        'error_pos': col_float(rows, 'error_pos'),
        'error_ang': col_float(rows, 'error_ang'),
        'v_cmd': col_float(rows, 'v_cmd'),
        'w_cmd': col_float(rows, 'w_cmd'),
        'goal_id': col_float(rows, 'goal_id', -1.0),
        'sem_state': sem_state,
        'red_latched': red_latched,
    }
    # pre-sem: si no existe (CSV de MC3), igualar al comando publicado
    data['v_cmd_pre_sem'] = (col_float(rows, 'v_cmd_pre_sem')
                             if 'v_cmd_pre_sem' in fieldnames
                             else data['v_cmd'].copy())
    data['w_cmd_pre_sem'] = (col_float(rows, 'w_cmd_pre_sem')
                             if 'w_cmd_pre_sem' in fieldnames
                             else data['w_cmd'].copy())

    # mascara de "robot bloqueado por rojo": latch activo O estado RED
    data['red_active'] = np.logical_or(data['red_latched'] > 0.5,
                                       data['sem_state'] == 'RED')

    # generar plots
    generated = []
    generated.append(plot_trajectory(data, outdir))
    generated.append(plot_position_error(data, outdir, args.epsilon))
    generated.append(plot_angle_error(data, outdir))
    generated.append(plot_velocity_cmds(data, outdir))
    if has_sem:
        generated.append(plot_semaphore_timeline(data, outdir))
    generated.append(plot_dashboard(data, outdir, args.epsilon))

    # metricas
    metrics = compute_metrics(data, args.epsilon)
    generated.append(write_metrics(metrics, outdir))

    print('\nArchivos generados:')
    for g in generated:
        print(f'  {g}')


if __name__ == '__main__':
    main()
