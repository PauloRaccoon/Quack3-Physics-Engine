"""Exemplo sem janela: empilha caixas, roda 4 s de simulação e imprime posição, quatérnio e eventos de contato.

    python examples/stack_boxes.py            (de dentro de physics_capi/python, com a biblioteca já compilada)
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import quack_physics as qp  # noqa: E402


def main() -> None:
    with qp.World() as world:
        chao = world.create_body(qp.Shape.box((10, 0.5, 10)), type="static", position=(0, -0.5, 0))
        caixa = qp.Shape.box((0.5, 0.5, 0.5))
        torre = [world.create_body(caixa, position=(0.02 * i, 0.5 + 1.05 * i, 0), friction=0.6) for i in range(5)]
        batidas = []
        for _ in range(240):                     # 4 s a 60 Hz
            world.update(1 / 60)
            batidas += [e for e in world.contact_events() if e.kind == "begin"]

        print(f"motor {qp.version()} ({'double' if qp.real_is_double() else 'float'}), {world.stats}")
        for i, b in enumerate(torre):
            w, x, y, z = b.rotation
            ang = 2 * math.degrees(math.acos(max(-1.0, min(1.0, w))))
            print(f"caixa {i}: y = {b.position[1]:.3f} m   q = ({w:+.4f}, {x:+.4f}, {y:+.4f}, {z:+.4f})   giro = {ang:.2f} graus")
        print(f"{len(batidas)} inicios de contato; maior velocidade de impacto = {max((e.approach_speed for e in batidas), default=0):.2f} m/s")
        hit = world.raycast((0, 20, 0), (0, -1, 0))
        print("raio de cima para baixo:", hit)


if __name__ == "__main__":
    main()
