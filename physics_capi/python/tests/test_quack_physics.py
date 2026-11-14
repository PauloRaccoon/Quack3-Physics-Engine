"""Testes da ligação Python (V26): exercitam a biblioteca C de verdade (sem mocks)."""
import math
import unittest

import quack_physics as qp


def make_scene(w):
    floor = w.create_body(qp.Shape.box((10, 0.5, 10)), type="static", position=(0, -0.5, 0))
    box = w.create_body(qp.Shape.box((0.5, 0.5, 0.5)), position=(0, 3, 0))
    return floor, box


class BasicsTest(unittest.TestCase):
    def test_version_and_precision(self):
        self.assertEqual(qp.version(), qp.__version__)
        self.assertIsInstance(qp.real_is_double(), bool)

    def test_world_defaults_and_setters(self):
        with qp.World(gravity=(0, -3, 0)) as w:
            self.assertAlmostEqual(w.gravity[1], -3, places=5)
            w.gravity = (0, -1, 0)
            self.assertAlmostEqual(w.gravity[1], -1, places=5)
            w.fixed_time_step = 1 / 120
            self.assertAlmostEqual(w.fixed_time_step, 1 / 120, places=6)
            with self.assertRaises(qp.PhysicsError) as cm:
                w.fixed_time_step = 0.0
            self.assertEqual(cm.exception.code, qp.ERR_INVALID_ARGUMENT)
            with self.assertRaises(qp.PhysicsError):
                w.gravity = (0, float("nan"), 0)
            w.set_solver_iterations(20, 4)
            w.set_orientation_exponential(False)
            w.set_sleep(True, 0.5)
            w.wake_all()

    def test_invalid_world_settings(self):
        with self.assertRaises(qp.PhysicsError):
            qp.World(fixed_time_step=float("nan"))
        with self.assertRaises(qp.PhysicsError):
            qp.World(max_bodies=0)

    def test_use_after_close(self):
        w = qp.World()
        shape = qp.Shape.sphere(0.5)
        b = w.create_body(shape)
        w.close()
        w.close()  # idempotente
        with self.assertRaises(qp.PhysicsError) as cm:
            w.update(0.1)
        self.assertEqual(cm.exception.code, qp.ERR_INVALID_HANDLE)
        self.assertFalse(b.valid)
        with self.assertRaises(qp.PhysicsError):
            _ = b.position


class BodiesTest(unittest.TestCase):
    def test_box_falls_and_rests(self):
        with qp.World() as w:
            floor, box = make_scene(w)
            steps = sum(w.update(1 / 60) for _ in range(300))
            self.assertEqual(steps, 300)
            self.assertAlmostEqual(box.position[1], 0.5, delta=0.02)
            self.assertLess(abs(box.linear_velocity[1]), 0.05)
            self.assertEqual(floor.type, "static")
            self.assertEqual(floor.mass, 0.0)
            self.assertEqual(w.stats.bodies, 2)
            self.assertEqual(w.stats.step_count, 300)
            q = box.rotation
            self.assertAlmostEqual(math.sqrt(sum(c * c for c in q)), 1.0, places=5)

    def test_validation_and_ids(self):
        with qp.World(max_bodies=2) as w:
            s = qp.Shape.sphere(0.5)
            a = w.create_body(s)
            b = w.create_body(s)
            with self.assertRaises(qp.PhysicsError) as cm:
                w.create_body(s)
            self.assertEqual(cm.exception.code, qp.ERR_CAPACITY)
            a.destroy()
            self.assertFalse(a.valid)
            with self.assertRaises(qp.PhysicsError) as cm:
                a.destroy()
            self.assertEqual(cm.exception.code, qp.ERR_INVALID_HANDLE)
            self.assertTrue(b.valid)
            with self.assertRaises(qp.PhysicsError):
                w.create_body(s, position=(0, float("inf"), 0))
            with self.assertRaises(qp.PhysicsError):
                w.create_body(s, rotation=(0, 0, 0, 0))
            with self.assertRaises(ValueError):
                w.create_body(s, type="fluido")

    def test_shape_can_be_closed_after_use(self):
        with qp.World() as w:
            s = qp.Shape.capsule(0.25, 0.5)
            b = w.create_body(s, position=(0, 1, 0))
            s.close()
            w.update(0.1)
            self.assertTrue(b.valid)
            with self.assertRaises(qp.PhysicsError):
                w.create_body(s)  # forma já fechada

    def test_bad_shapes(self):
        for make in (lambda: qp.Shape.sphere(-1), lambda: qp.Shape.sphere(float("nan")), lambda: qp.Shape.box((0, 1, 1)),
                     lambda: qp.Shape.capsule(0.5, -1), lambda: qp.Shape.cylinder(0, 1)):
            with self.assertRaises(qp.PhysicsError):
                make()

    def test_polyhedron(self):
        v = [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]
        faces = [(0, 1, 2), (0, 3, 1), (0, 2, 3), (1, 3, 2)]
        with qp.World() as w:
            b = w.create_body(qp.Shape.polyhedron(v, faces), position=(0, 5, 0))
            self.assertGreater(b.mass, 0)
        with self.assertRaises(qp.PhysicsError):
            qp.Shape.polyhedron(v, [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)])  # faces invertidas
        with self.assertRaises(qp.PhysicsError):
            qp.Shape.polyhedron(v, [(0, 1, 2), (0, 3, 1), (0, 2, 3), (1, 3, 9)])  # índice fora da faixa

    def test_forces_impulses_and_transform(self):
        with qp.World(gravity=(0, 0, 0)) as w:
            b = w.create_body(qp.Shape.sphere(0.5))
            m = b.mass
            for _ in range(60):
                b.apply_force((10, 0, 0))
                w.step()
            self.assertAlmostEqual(b.linear_velocity[0], 10 / m, delta=0.02 * 10 / m)
            b.linear_velocity = (0, 0, 0)
            cm = b.center_of_mass
            b.apply_impulse((0, 0, 2), at_point=(cm[0] + 0.5, cm[1], cm[2]))
            self.assertAlmostEqual(b.linear_velocity[2], 2 / m, places=3)
            self.assertGreater(abs(b.angular_velocity[1]), 0.1)
            b.set_transform((1, 2, 3), (0, 0, 1, 0))
            self.assertAlmostEqual(b.position[0], 1, places=6)
            self.assertAlmostEqual(b.rotation[2], 1, places=6)
            self.assertAlmostEqual(b.interpolated[0][1], 2, places=5)
            with self.assertRaises(qp.PhysicsError):
                b.set_transform((0, 0, 0), (0, 0, 0, 0))
            b.set_material(0.8, 0.3)
            b.set_damping(0.1, 0.1)
            b.set_gravity_scale(0.0)
            b.awake = True


class JointsTest(unittest.TestCase):
    def test_hinge_pendulum_and_joint_lifetime(self):
        with qp.World() as w:
            s_anchor, s_arm = qp.Shape.box((0.1, 0.1, 0.1)), qp.Shape.box((1.0, 0.1, 0.1))
            a = w.create_body(s_anchor, type="static", position=(0, 5, 0))
            b = w.create_body(s_arm, position=(1, 5, 0))
            other = w.create_body(s_arm, position=(5, 8, 0))
            hinge = w.create_hinge(a, b, (0, 5, 0), (0, 0, 1))
            self.assertTrue(hinge.valid)
            with self.assertRaises(qp.PhysicsError):
                w.create_hinge(a, a, (0, 5, 0), (0, 0, 1))  # corpos iguais
            with self.assertRaises(qp.PhysicsError):
                w.create_hinge(a, b, (0, 5, 0), (0, 0, 0))  # eixo nulo
            ball = w.create_ball_socket(a, other, (1, 8, 0))
            dist = w.create_distance(a, other, (1, 8, 0), (5, 8, 0))
            self.assertEqual(w.stats.joints, 3)
            for _ in range(120):
                w.step()
            p = b.center_of_mass
            self.assertLess(p[1], 4.5)
            self.assertAlmostEqual(math.hypot(p[0], p[1] - 5), 1.0, delta=0.05)
            self.assertLess(abs(p[2]), 0.02)
            hinge.destroy()
            self.assertFalse(hinge.valid)
            with self.assertRaises(qp.PhysicsError):
                hinge.destroy()
            other.destroy()  # leva as juntas dele junto
            self.assertFalse(ball.valid)
            self.assertFalse(dist.valid)
            w.step()


class EventsQueriesTest(unittest.TestCase):
    def test_events_raycast_overlap_bulk(self):
        with qp.World() as w:
            floor = w.create_body(qp.Shape.box((10, 0.5, 10)), type="static", position=(0, -0.5, 0))
            box = w.create_body(qp.Shape.box((0.5, 0.5, 0.5)), position=(0, 1.0, 0))
            begins = []
            for _ in range(90):
                w.step()
                begins += [e for e in w.contact_events() if e.kind == "begin"]
            self.assertTrue(begins)
            e = begins[0]
            self.assertLess(e.body_a, e.body_b)
            self.assertGreaterEqual(e.approach_speed, 0.0)
            self.assertEqual({e.body_a, e.body_b}, {floor.id, box.id})
            w.clear_contact_events()
            self.assertEqual(w.contact_events(), [])

            h = w.raycast((0, 10, 0), (0, -1, 0))
            self.assertIsNotNone(h)
            self.assertEqual(h.body_id, box.id)
            self.assertAlmostEqual(h.distance, 9.0, delta=0.05)
            self.assertAlmostEqual(h.normal[1], 1.0, places=3)
            self.assertEqual(w.raycast((0, 10, 0), (0, -1, 0), include=["static"]).body_id, floor.id)
            self.assertEqual(w.raycast((0, 10, 0), (0, -1, 0), ignore=box).body_id, floor.id)
            self.assertIsNone(w.raycast((0, 10, 0), (0, -1, 0), max_distance=2.0))
            self.assertIsNone(w.raycast((0, 10, 0), (0, 0, 0)))  # direção nula: nada (não é erro)
            hits = w.raycast_all((0, 10, 0), (0, -1, 0))
            self.assertEqual([x.body_id for x in hits], [box.id, floor.id])
            with self.assertRaises(qp.PhysicsError):
                w.raycast((float("nan"), 0, 0), (0, -1, 0))

            self.assertEqual(w.overlap_sphere((0, 0.5, 0), 0.3, include=["dynamic"]), [box.id])
            self.assertEqual(w.overlap_point((0, 0.5, 0), include=["dynamic"]), [box.id])
            self.assertEqual(w.overlap_sphere((0, 0.5, 0), -1.0), [])

            self.assertEqual(w.body_count, 2)
            tr = w.read_transforms(interpolated=False)
            self.assertEqual([t[0] for t in tr], [floor.id, box.id])
            self.assertAlmostEqual(tr[0][1][1], -0.5, places=6)
            self.assertAlmostEqual(tr[1][1][1], 0.5, delta=0.02)
            self.assertAlmostEqual(math.sqrt(sum(c * c for c in tr[1][2])), 1.0, places=5)
            try:
                import numpy  # noqa: F401
            except ImportError:
                return
            ids, data = w.read_transforms_numpy()
            self.assertEqual(list(ids), [floor.id, box.id])
            self.assertEqual(data.shape, (2, 7))

    def test_interpolation_between_steps(self):
        with qp.World() as w:
            b = w.create_body(qp.Shape.sphere(0.5), position=(0, 100, 0))
            w.update(1 / 60)
            w.update(1 / 60)
            w.update(0.5 / 60)
            self.assertAlmostEqual(w.alpha, 0.5, delta=1e-3)
            drawn, simulated = b.interpolated[0][1], b.position[1]
            self.assertGreaterEqual(drawn, simulated)   # em queda: a pose desenhada fica um pouco acima da simulada
            self.assertLess(drawn - simulated, 0.05)


class LibraryTest(unittest.TestCase):
    def test_missing_library_message(self):
        with self.assertRaises(FileNotFoundError):
            qp.load_library("/nao/existe/physics_c.so")


if __name__ == "__main__":
    unittest.main()
