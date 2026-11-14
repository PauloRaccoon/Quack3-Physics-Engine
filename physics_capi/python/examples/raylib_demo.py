"""Exemplo com janela: a física do motor (via API C) desenhada com raylib (pacote `raylib` do PyPI, módulo `pyray`).

    pip install raylib
    python examples/raylib_demo.py

Mostra o desacoplamento física/render: a física roda a 60 Hz fixos (world.update acumula o tempo de cada quadro)
e o desenho usa a pose INTERPOLADA (world.read_transforms), então o movimento fica suave em qualquer FPS.
Teclas: ESPAÇO lança uma esfera na torre, R recomeça, setas giram a câmera.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import quack_physics as qp  # noqa: E402

try:
    import pyray as rl
except ImportError:
    sys.exit("instale a raylib para Python:  pip install raylib")


def quat_to_axis_angle(q):
    """(w, x, y, z) -> (ângulo em graus, eixo). Eixo qualquer se o giro for ~0."""
    w, x, y, z = q
    w = max(-1.0, min(1.0, w))
    s = math.sqrt(max(0.0, 1.0 - w * w))
    angle = math.degrees(2.0 * math.acos(w))
    return (angle, (x / s, y / s, z / s)) if s > 1e-6 else (0.0, (0.0, 1.0, 0.0))


def build(world):
    caixa = qp.Shape.box((0.5, 0.5, 0.5))
    world.create_body(qp.Shape.box((10, 0.5, 10)), type="static", position=(0, -0.5, 0))
    bodies = {}
    for andar in range(6):
        for col in range(2):
            b = world.create_body(caixa, position=(col * 1.05 - 0.5, 0.5 + 1.0 * andar, 0), friction=0.6)
            bodies[b.id] = ("box", (0.5, 0.5, 0.5))
    return bodies


def main():
    rl.init_window(1000, 640, "Quack3 - API C + Python")
    rl.set_target_fps(0)                                  # FPS livre: a física não depende dele
    world = qp.World()
    sizes = build(world)
    sizes[0] = ("floor", (10, 0.5, 10))
    esfera = qp.Shape.sphere(0.4)
    cam = rl.Camera3D(rl.Vector3(9, 6, 12), rl.Vector3(0, 3, 0), rl.Vector3(0, 1, 0), 45.0, rl.CAMERA_PERSPECTIVE)
    yaw = 0.0
    while not rl.window_should_close():
        dt = min(rl.get_frame_time(), 0.1)
        if rl.is_key_pressed(rl.KEY_R):
            world.close()
            world = qp.World()
            sizes = build(world)
            sizes[0] = ("floor", (10, 0.5, 10))
        if rl.is_key_pressed(rl.KEY_SPACE):
            b = world.create_body(esfera, position=(-8, 3, 0), linear_velocity=(18, 4, 0), density=3.0, restitution=0.2)
            sizes[b.id] = ("sphere", 0.4)
        if rl.is_key_down(rl.KEY_LEFT):
            yaw -= 1.5 * dt
        if rl.is_key_down(rl.KEY_RIGHT):
            yaw += 1.5 * dt
        cam.position = rl.Vector3(math.sin(yaw) * 14, 6, math.cos(yaw) * 14)

        world.update(dt)                                  # física: passos fixos de 1/60 s
        poses = world.read_transforms(interpolated=True)  # desenho: pose interpolada, em UMA chamada

        rl.begin_drawing()
        rl.clear_background(rl.Color(34, 36, 44, 255))
        rl.begin_mode_3d(cam)
        for bid, (px, py, pz), q in poses:
            kind, size = sizes.get(bid, ("box", (0.5, 0.5, 0.5)))
            ang, ax = quat_to_axis_angle(q)
            rl.rl_push_matrix()
            rl.rl_translatef(px, py, pz)
            rl.rl_rotatef(ang, ax[0], ax[1], ax[2])
            if kind == "sphere":
                rl.draw_sphere(rl.Vector3(0, 0, 0), size, rl.Color(240, 150, 60, 255))
            else:
                w, h, d = (2 * size[0], 2 * size[1], 2 * size[2])
                col = rl.Color(90, 98, 120, 255) if kind == "floor" else rl.Color(110, 160, 255, 255)
                rl.draw_cube(rl.Vector3(0, 0, 0), w, h, d, col)
                rl.draw_cube_wires(rl.Vector3(0, 0, 0), w, h, d, rl.Color(20, 22, 28, 255))
            rl.rl_pop_matrix()
        rl.end_mode_3d()
        st = world.stats
        rl.draw_text(f"{rl.get_fps()} FPS | corpos {st.bodies} acordados {st.awake_bodies} | alpha {world.alpha:.2f}", 12, 10, 20, rl.RAYWHITE)
        rl.draw_text("ESPACO: esfera | R: recomecar | setas: girar", 12, 36, 18, rl.LIGHTGRAY)
        rl.end_drawing()
    world.close()
    rl.close_window()


if __name__ == "__main__":
    main()
