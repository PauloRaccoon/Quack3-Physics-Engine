"""
quack_physics (V26): ligação Python (ctypes) do motor de física, via a API C `physics_c`.

Sem dependências: só a biblioteca padrão (numpy é opcional, para leitura em lote).

    import quack_physics as qp

    world = qp.World(gravity=(0, -9.81, 0))
    chao = world.create_body(qp.Shape.box((10, 0.5, 10)), type="static", position=(0, -0.5, 0))
    caixa = world.create_body(qp.Shape.box((0.5, 0.5, 0.5)), position=(0, 3, 0))
    for _ in range(120):
        world.update(1 / 60)
    print(caixa.position, caixa.rotation)

Convenções: tudo em unidades SI; vetores são tuplas (x, y, z); quatérnios são tuplas (w, x, y, z).
Erros do motor viram `PhysicsError` (com `.code` e a mensagem do motor).

A biblioteca compartilhada é procurada, nesta ordem: variável de ambiente QUACK_PHYSICS_LIB (caminho do
arquivo), ao lado deste pacote, e as pastas de build usuais (../build*/ e ../../build*/ a partir de physics_capi/).
"""
from __future__ import annotations

import ctypes as C
import math
import os
import sys
from typing import Iterable, List, Optional, Sequence, Tuple

__all__ = ["World", "Shape", "Body", "Joint", "PhysicsError", "Hit", "ContactEvent", "Stats", "load_library"]
__version__ = "1.0.0"

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

_ABI = 1

# ---- códigos e constantes (espelham physics_c.h) ---------------------------------------------------------------------
OK = 0
ERR_INVALID_ARGUMENT, ERR_INVALID_HANDLE, ERR_CAPACITY, ERR_INTERNAL = -1, -2, -3, -4
INVALID_BODY = 0xFFFFFFFF
BODY_TYPES = {"static": 0, "kinematic": 1, "dynamic": 2}
_BODY_NAMES = {v: k for k, v in BODY_TYPES.items()}
QUERY_STATIC, QUERY_KINEMATIC, QUERY_DYNAMIC, QUERY_ALL = 1, 2, 4, 7


class PhysicsError(RuntimeError):
    """Erro devolvido pela API C. `code` é um dos PC_ERR_* (negativo)."""

    def __init__(self, code: int, message: str):
        super().__init__(f"{message} (codigo {code})" if message else f"erro {code}")
        self.code = code
        self.message = message


# ---- estruturas ctypes (mesmo layout do cabeçalho) ------------------------------------------------------------------
class _Vec3(C.Structure):
    _fields_ = [("x", C.c_double), ("y", C.c_double), ("z", C.c_double)]


class _Quat(C.Structure):
    _fields_ = [("w", C.c_double), ("x", C.c_double), ("y", C.c_double), ("z", C.c_double)]


class _WorldSettings(C.Structure):
    _fields_ = [("gravity", _Vec3), ("fixed_time_step", C.c_double), ("max_sub_steps", C.c_int32), ("max_bodies", C.c_uint32),
                ("velocity_iterations", C.c_int32), ("position_iterations", C.c_int32), ("warm_starting", C.c_int32),
                ("orientation_exponential", C.c_int32), ("sleep_enabled", C.c_int32), ("sleep_time", C.c_double)]


class _WorldStats(C.Structure):
    _fields_ = [("bodies", C.c_int32), ("awake_bodies", C.c_int32), ("sleeping_bodies", C.c_int32), ("islands", C.c_int32),
                ("joints", C.c_int32), ("pairs", C.c_int32), ("touching_pairs", C.c_int32), ("contact_points", C.c_int32),
                ("step_count", C.c_uint64)]


class _BodyDesc(C.Structure):
    _fields_ = [("type", C.c_int32), ("shape", C.c_void_p), ("density", C.c_double), ("mass", C.c_double), ("position", _Vec3),
                ("rotation", _Quat), ("linear_velocity", _Vec3), ("angular_velocity", _Vec3), ("friction", C.c_double),
                ("restitution", C.c_double), ("linear_damping", C.c_double), ("angular_damping", C.c_double),
                ("gravity_scale", C.c_double), ("allow_sleep", C.c_int32)]


class _BodyState(C.Structure):
    _fields_ = [("type", C.c_int32), ("awake", C.c_int32), ("mass", C.c_double), ("position", _Vec3), ("center_of_mass", _Vec3),
                ("rotation", _Quat), ("linear_velocity", _Vec3), ("angular_velocity", _Vec3)]


class _ContactEvent(C.Structure):
    _fields_ = [("type", C.c_int32), ("body_a", C.c_uint32), ("body_b", C.c_uint32), ("point", _Vec3), ("normal", _Vec3),
                ("depth", C.c_double), ("point_count", C.c_int32), ("normal_impulse", C.c_double), ("approach_speed", C.c_double)]


class _RayHit(C.Structure):
    _fields_ = [("body", C.c_uint32), ("distance", C.c_double), ("point", _Vec3), ("normal", _Vec3), ("started_inside", C.c_int32)]


# ---- carregar a biblioteca --------------------------------------------------------------------------------------------
def _lib_names() -> List[str]:
    if sys.platform.startswith("win"):
        return ["physics_c.dll", "libphysics_c.dll"]
    if sys.platform == "darwin":
        return ["libphysics_c.dylib"]
    return ["libphysics_c.so"]


def _candidates() -> List[str]:
    out: List[str] = []
    env = os.environ.get("QUACK_PHYSICS_LIB")
    if env:
        out.append(env)
    here = os.path.dirname(os.path.abspath(__file__))
    capi = os.path.abspath(os.path.join(here, "..", ".."))           # .../physics_capi
    roots = [here, os.path.dirname(here)]
    for base in (capi, os.path.dirname(capi)):
        try:
            for d in sorted(os.listdir(base)):
                if d.startswith("build"):
                    roots.append(os.path.join(base, d))
                    roots.append(os.path.join(base, d, "Release"))
        except OSError:
            pass
    for r in roots:
        for n in _lib_names():
            out.append(os.path.join(r, n))
    return out


_lib = None


def load_library(path: Optional[str] = None):
    """Carrega (uma vez) a biblioteca physics_c. `path` força um caminho; senão usa a busca descrita no módulo."""
    global _lib
    if _lib is not None and path is None:
        return _lib
    tried = [path] if path else _candidates()
    last: Optional[Exception] = None
    for p in tried:
        if p and os.path.isfile(p):
            try:
                lib = C.CDLL(os.path.abspath(p))
            except OSError as e:  # arquivo existe mas não carrega (arquitetura/dependência)
                last = e
                continue
            _declare(lib)
            if lib.pc_abi_version() != _ABI:
                raise RuntimeError(f"physics_c incompativel: ABI {lib.pc_abi_version()} (esperada {_ABI}) em {p}")
            _lib = lib
            return lib
    raise FileNotFoundError("biblioteca physics_c nao encontrada. Compile physics_capi (cmake) e aponte QUACK_PHYSICS_LIB para o arquivo "
                            f"{_lib_names()[0]}. Procurei em: " + "; ".join(t for t in tried[:8] if t) + (f" ... ({last})" if last else ""))


def _declare(lib) -> None:
    P = C.POINTER
    vp, i32, u32, dbl, ci = C.c_void_p, C.c_int32, C.c_uint32, C.c_double, C.c_int

    def sig(name, res, *args):
        f = getattr(lib, name)
        f.restype = res
        f.argtypes = list(args)

    sig("pc_abi_version", ci)
    sig("pc_version_string", C.c_char_p)
    sig("pc_real_is_double", ci)
    sig("pc_last_error", C.c_char_p)
    sig("pc_world_settings_default", ci, P(_WorldSettings))
    sig("pc_world_create", vp, P(_WorldSettings))
    sig("pc_world_destroy", None, vp)
    sig("pc_world_update", ci, vp, dbl)
    sig("pc_world_step", ci, vp)
    sig("pc_world_get_alpha", ci, vp, P(dbl))
    sig("pc_world_get_gravity", ci, vp, P(_Vec3))
    sig("pc_world_set_gravity", ci, vp, P(_Vec3))
    sig("pc_world_get_fixed_time_step", ci, vp, P(dbl))
    sig("pc_world_set_fixed_time_step", ci, vp, dbl)
    sig("pc_world_set_solver_iterations", ci, vp, i32, i32)
    sig("pc_world_set_orientation_exponential", ci, vp, i32)
    sig("pc_world_set_sleep", ci, vp, i32, dbl)
    sig("pc_world_wake_all", ci, vp)
    sig("pc_world_get_stats", ci, vp, P(_WorldStats))
    sig("pc_shape_create_sphere", vp, dbl)
    sig("pc_shape_create_box", vp, P(_Vec3))
    sig("pc_shape_create_capsule", vp, dbl, dbl)
    sig("pc_shape_create_cylinder", vp, dbl, dbl)
    sig("pc_shape_create_polyhedron", vp, P(_Vec3), i32, P(i32), P(i32), i32)
    sig("pc_shape_destroy", None, vp)
    sig("pc_body_desc_default", ci, P(_BodyDesc))
    sig("pc_body_create", ci, vp, P(_BodyDesc), P(u32))
    sig("pc_body_destroy", ci, vp, u32)
    sig("pc_body_is_valid", ci, vp, u32)
    sig("pc_body_get_state", ci, vp, u32, P(_BodyState))
    sig("pc_body_get_interpolated", ci, vp, u32, P(_Vec3), P(_Quat))
    sig("pc_body_set_transform", ci, vp, u32, P(_Vec3), P(_Quat))
    sig("pc_body_set_linear_velocity", ci, vp, u32, P(_Vec3))
    sig("pc_body_set_angular_velocity", ci, vp, u32, P(_Vec3))
    sig("pc_body_apply_force", ci, vp, u32, P(_Vec3))
    sig("pc_body_apply_force_at_point", ci, vp, u32, P(_Vec3), P(_Vec3))
    sig("pc_body_apply_torque", ci, vp, u32, P(_Vec3))
    sig("pc_body_apply_impulse_at_point", ci, vp, u32, P(_Vec3), P(_Vec3))
    sig("pc_body_set_material", ci, vp, u32, dbl, dbl)
    sig("pc_body_set_damping", ci, vp, u32, dbl, dbl)
    sig("pc_body_set_gravity_scale", ci, vp, u32, dbl)
    sig("pc_body_set_awake", ci, vp, u32, i32)
    sig("pc_world_read_transforms", ci, vp, i32, u32, P(u32), P(dbl))
    sig("pc_joint_create_ball_socket", ci, vp, u32, u32, P(_Vec3), i32, P(u32))
    sig("pc_joint_create_distance", ci, vp, u32, u32, P(_Vec3), P(_Vec3), dbl, i32, P(u32))
    sig("pc_joint_create_hinge", ci, vp, u32, u32, P(_Vec3), P(_Vec3), i32, P(u32))
    sig("pc_joint_destroy", ci, vp, u32)
    sig("pc_joint_is_valid", ci, vp, u32)
    sig("pc_world_read_contact_events", ci, vp, u32, P(_ContactEvent))
    sig("pc_world_clear_contact_events", ci, vp)
    sig("pc_world_raycast", ci, vp, P(_Vec3), P(_Vec3), dbl, u32, u32, P(_RayHit))
    sig("pc_world_raycast_all", ci, vp, P(_Vec3), P(_Vec3), dbl, u32, u32, u32, P(_RayHit))
    sig("pc_world_overlap_sphere", ci, vp, P(_Vec3), dbl, u32, u32, u32, P(u32))
    sig("pc_world_overlap_point", ci, vp, P(_Vec3), u32, u32, u32, P(u32))
    # as funções de junta usam (vp, u32 id) para destroy/is_valid: ids de junta são uint32 também
    lib.pc_joint_destroy.argtypes = [vp, u32]
    lib.pc_joint_is_valid.argtypes = [vp, u32]


def _check(code: int) -> int:
    if code < 0:
        msg = load_library().pc_last_error()
        raise PhysicsError(code, msg.decode("utf-8", "replace") if msg else "")
    return code


def _v(t: Sequence[float]) -> _Vec3:
    x, y, z = t
    return _Vec3(float(x), float(y), float(z))


def _q(t: Sequence[float]) -> _Quat:
    w, x, y, z = t
    return _Quat(float(w), float(x), float(y), float(z))


def _tv(v: _Vec3) -> Vec3:
    return (v.x, v.y, v.z)


def _tq(q: _Quat) -> Quat:
    return (q.w, q.x, q.y, q.z)


def _flags(include: Optional[Iterable[str]]) -> int:
    if include is None:
        return QUERY_ALL
    m = {"static": QUERY_STATIC, "kinematic": QUERY_KINEMATIC, "dynamic": QUERY_DYNAMIC}
    f = 0
    for name in include:
        f |= m[name]
    return f


# ---- tipos de resultado -------------------------------------------------------------------------------------------------
class Hit:
    """Resultado de um raycast."""

    __slots__ = ("body_id", "distance", "point", "normal", "started_inside")

    def __init__(self, h: _RayHit):
        self.body_id = h.body
        self.distance = h.distance
        self.point = _tv(h.point)
        self.normal = _tv(h.normal)
        self.started_inside = bool(h.started_inside)

    def __repr__(self) -> str:
        return f"Hit(body={self.body_id}, distance={self.distance:.4f}, point={self.point}, normal={self.normal})"


class ContactEvent:
    """Início (kind == 'begin') ou fim ('end') de contato entre dois corpos."""

    __slots__ = ("kind", "body_a", "body_b", "point", "normal", "depth", "point_count", "normal_impulse", "approach_speed")

    def __init__(self, e: _ContactEvent):
        self.kind = "begin" if e.type == 0 else "end"
        self.body_a, self.body_b = e.body_a, e.body_b
        self.point, self.normal = _tv(e.point), _tv(e.normal)
        self.depth, self.point_count = e.depth, e.point_count
        self.normal_impulse, self.approach_speed = e.normal_impulse, e.approach_speed

    def __repr__(self) -> str:
        return f"ContactEvent({self.kind}, {self.body_a}-{self.body_b}, impacto={self.approach_speed:.3f} m/s)"


class Stats:
    __slots__ = ("bodies", "awake_bodies", "sleeping_bodies", "islands", "joints", "pairs", "touching_pairs", "contact_points", "step_count")

    def __init__(self, s: _WorldStats):
        for n in self.__slots__:
            setattr(self, n, getattr(s, n))

    def __repr__(self) -> str:
        return "Stats(" + ", ".join(f"{n}={getattr(self, n)}" for n in self.__slots__) + ")"


# ---- formas ------------------------------------------------------------------------------------------------------------
class Shape:
    """Forma geométrica. Use os construtores: Shape.sphere, box, capsule, cylinder, polyhedron.
    Um corpo guarda a sua própria referência: pode chamar close() (ou deixar o coletor de lixo agir) depois de criar os corpos."""

    def __init__(self, handle: int):
        self._h = handle

    @staticmethod
    def _make(handle) -> "Shape":
        if not handle:
            msg = load_library().pc_last_error()
            raise PhysicsError(ERR_INVALID_ARGUMENT, msg.decode("utf-8", "replace") if msg else "forma invalida")
        return Shape(handle)

    @classmethod
    def sphere(cls, radius: float) -> "Shape":
        return cls._make(load_library().pc_shape_create_sphere(float(radius)))

    @classmethod
    def box(cls, half_extents: Sequence[float]) -> "Shape":
        return cls._make(load_library().pc_shape_create_box(C.byref(_v(half_extents))))

    @classmethod
    def capsule(cls, radius: float, half_height: float) -> "Shape":
        return cls._make(load_library().pc_shape_create_capsule(float(radius), float(half_height)))

    @classmethod
    def cylinder(cls, radius: float, half_height: float) -> "Shape":
        return cls._make(load_library().pc_shape_create_cylinder(float(radius), float(half_height)))

    @classmethod
    def polyhedron(cls, vertices: Sequence[Sequence[float]], faces: Sequence[Sequence[int]]) -> "Shape":
        """Poliedro convexo. faces: lista de listas de índices, anti-horárias vistas de fora."""
        vs = (_Vec3 * len(vertices))(*[_v(v) for v in vertices])
        idx = [int(i) for f in faces for i in f]
        sizes = [len(f) for f in faces]
        a = (C.c_int32 * len(idx))(*idx)
        s = (C.c_int32 * len(sizes))(*sizes)
        return cls._make(load_library().pc_shape_create_polyhedron(vs, len(vertices), a, s, len(faces)))

    def close(self) -> None:
        if self._h:
            load_library().pc_shape_destroy(self._h)
            self._h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


# ---- corpo e junta (handles leves: mundo + id) -------------------------------------------------------------------------------
class Body:
    """Referência a um corpo do mundo (guarda o mundo e o id). Depois de destroy() ou de world.close() os métodos levantam PhysicsError."""

    def __init__(self, world: "World", body_id: int):
        self._w = world
        self.id = body_id

    def _lib(self):
        return self._w._lib()

    def _state(self) -> _BodyState:
        s = _BodyState()
        _check(self._lib().pc_body_get_state(self._w._h, self.id, C.byref(s)))
        return s

    @property
    def valid(self) -> bool:
        return bool(self._w._h) and bool(self._lib().pc_body_is_valid(self._w._h, self.id))

    @property
    def type(self) -> str:
        return _BODY_NAMES[self._state().type]

    @property
    def mass(self) -> float:
        return self._state().mass

    @property
    def position(self) -> Vec3:
        return _tv(self._state().position)

    @property
    def center_of_mass(self) -> Vec3:
        return _tv(self._state().center_of_mass)

    @property
    def rotation(self) -> Quat:
        return _tq(self._state().rotation)

    @property
    def linear_velocity(self) -> Vec3:
        return _tv(self._state().linear_velocity)

    @linear_velocity.setter
    def linear_velocity(self, v: Sequence[float]) -> None:
        _check(self._lib().pc_body_set_linear_velocity(self._w._h, self.id, C.byref(_v(v))))

    @property
    def angular_velocity(self) -> Vec3:
        return _tv(self._state().angular_velocity)

    @angular_velocity.setter
    def angular_velocity(self, w: Sequence[float]) -> None:
        _check(self._lib().pc_body_set_angular_velocity(self._w._h, self.id, C.byref(_v(w))))

    @property
    def awake(self) -> bool:
        return bool(self._state().awake)

    @awake.setter
    def awake(self, on: bool) -> None:
        _check(self._lib().pc_body_set_awake(self._w._h, self.id, 1 if on else 0))

    @property
    def interpolated(self) -> Tuple[Vec3, Quat]:
        """(posição, rotação) para DESENHAR: interpoladas com o α do mundo."""
        p, q = _Vec3(), _Quat()
        _check(self._lib().pc_body_get_interpolated(self._w._h, self.id, C.byref(p), C.byref(q)))
        return _tv(p), _tq(q)

    def set_transform(self, position: Sequence[float], rotation: Sequence[float] = (1, 0, 0, 0)) -> None:
        _check(self._lib().pc_body_set_transform(self._w._h, self.id, C.byref(_v(position)), C.byref(_q(rotation))))

    def apply_force(self, force: Sequence[float], at_point: Optional[Sequence[float]] = None) -> None:
        if at_point is None:
            _check(self._lib().pc_body_apply_force(self._w._h, self.id, C.byref(_v(force))))
        else:
            _check(self._lib().pc_body_apply_force_at_point(self._w._h, self.id, C.byref(_v(force)), C.byref(_v(at_point))))

    def apply_torque(self, torque: Sequence[float]) -> None:
        _check(self._lib().pc_body_apply_torque(self._w._h, self.id, C.byref(_v(torque))))

    def apply_impulse(self, impulse: Sequence[float], at_point: Optional[Sequence[float]] = None) -> None:
        pt = at_point if at_point is not None else self.center_of_mass
        _check(self._lib().pc_body_apply_impulse_at_point(self._w._h, self.id, C.byref(_v(impulse)), C.byref(_v(pt))))

    def set_material(self, friction: float, restitution: float) -> None:
        _check(self._lib().pc_body_set_material(self._w._h, self.id, float(friction), float(restitution)))

    def set_damping(self, linear: float, angular: float) -> None:
        _check(self._lib().pc_body_set_damping(self._w._h, self.id, float(linear), float(angular)))

    def set_gravity_scale(self, scale: float) -> None:
        _check(self._lib().pc_body_set_gravity_scale(self._w._h, self.id, float(scale)))

    def destroy(self) -> None:
        _check(self._lib().pc_body_destroy(self._w._h, self.id))

    def __repr__(self) -> str:
        return f"Body(id={self.id})"


class Joint:
    """Referência a uma junta (some quando um corpo ligado a ela é destruído)."""

    def __init__(self, world: "World", joint_id: int):
        self._w = world
        self.id = joint_id

    @property
    def valid(self) -> bool:
        return bool(self._w._h) and bool(self._w._lib().pc_joint_is_valid(self._w._h, self.id))

    def destroy(self) -> None:
        _check(self._w._lib().pc_joint_destroy(self._w._h, self.id))

    def __repr__(self) -> str:
        return f"Joint(id={self.id})"


# ---- mundo ---------------------------------------------------------------------------------------------------------------------
class World:
    """Mundo de física. Use como `with World() as w:` ou chame close()."""

    def __init__(self, gravity: Sequence[float] = (0.0, -9.81, 0.0), fixed_time_step: float = 1.0 / 60.0, max_bodies: int = 4096,
                 max_sub_steps: int = 8, velocity_iterations: int = 10, position_iterations: int = 3,
                 orientation_exponential: bool = True, sleep: bool = True, sleep_time: float = 0.5, warm_starting: bool = True):
        lib = load_library()
        s = _WorldSettings()
        _check(lib.pc_world_settings_default(C.byref(s)))
        s.gravity = _v(gravity)
        s.fixed_time_step = float(fixed_time_step)
        s.max_bodies = int(max_bodies)
        s.max_sub_steps = int(max_sub_steps)
        s.velocity_iterations = int(velocity_iterations)
        s.position_iterations = int(position_iterations)
        s.orientation_exponential = 1 if orientation_exponential else 0
        s.sleep_enabled = 1 if sleep else 0
        s.sleep_time = float(sleep_time)
        s.warm_starting = 1 if warm_starting else 0
        self._h = lib.pc_world_create(C.byref(s))
        if not self._h:
            msg = lib.pc_last_error()
            raise PhysicsError(ERR_INVALID_ARGUMENT, msg.decode("utf-8", "replace") if msg else "mundo invalido")
        self._bodies = 0

    @staticmethod
    def _lib():
        return load_library()

    def _need(self) -> int:
        if not self._h:
            raise PhysicsError(ERR_INVALID_HANDLE, "mundo ja fechado")
        return self._h

    def close(self) -> None:
        if self._h:
            self._lib().pc_world_destroy(self._h)
            self._h = None

    def __enter__(self) -> "World":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    # --- simulação ---
    def update(self, frame_time: float) -> int:
        """Avança `frame_time` s de tempo de render (passos fixos + acumulador). Devolve quantos passos fixos rodaram."""
        return _check(self._lib().pc_world_update(self._need(), float(frame_time)))

    def step(self) -> None:
        _check(self._lib().pc_world_step(self._need()))

    @property
    def alpha(self) -> float:
        a = C.c_double()
        _check(self._lib().pc_world_get_alpha(self._need(), C.byref(a)))
        return a.value

    @property
    def gravity(self) -> Vec3:
        g = _Vec3()
        _check(self._lib().pc_world_get_gravity(self._need(), C.byref(g)))
        return _tv(g)

    @gravity.setter
    def gravity(self, g: Sequence[float]) -> None:
        _check(self._lib().pc_world_set_gravity(self._need(), C.byref(_v(g))))

    @property
    def fixed_time_step(self) -> float:
        d = C.c_double()
        _check(self._lib().pc_world_get_fixed_time_step(self._need(), C.byref(d)))
        return d.value

    @fixed_time_step.setter
    def fixed_time_step(self, dt: float) -> None:
        _check(self._lib().pc_world_set_fixed_time_step(self._need(), float(dt)))

    def set_solver_iterations(self, velocity: int, position: int) -> None:
        _check(self._lib().pc_world_set_solver_iterations(self._need(), int(velocity), int(position)))

    def set_orientation_exponential(self, on: bool) -> None:
        _check(self._lib().pc_world_set_orientation_exponential(self._need(), 1 if on else 0))

    def set_sleep(self, enabled: bool, sleep_time: float = 0.5) -> None:
        _check(self._lib().pc_world_set_sleep(self._need(), 1 if enabled else 0, float(sleep_time)))

    def wake_all(self) -> None:
        _check(self._lib().pc_world_wake_all(self._need()))

    @property
    def stats(self) -> Stats:
        s = _WorldStats()
        _check(self._lib().pc_world_get_stats(self._need(), C.byref(s)))
        return Stats(s)

    # --- corpos ---
    def create_body(self, shape: Shape, type: str = "dynamic", position: Sequence[float] = (0, 0, 0),
                    rotation: Sequence[float] = (1, 0, 0, 0), density: float = 1.0, mass: float = 0.0,
                    linear_velocity: Sequence[float] = (0, 0, 0), angular_velocity: Sequence[float] = (0, 0, 0),
                    friction: float = 0.5, restitution: float = 0.0, linear_damping: float = 0.0, angular_damping: float = 0.01,
                    gravity_scale: float = 1.0, allow_sleep: bool = True) -> Body:
        if type not in BODY_TYPES:
            raise ValueError(f"type deve ser um de {sorted(BODY_TYPES)}")
        if not shape._h:
            raise PhysicsError(ERR_INVALID_HANDLE, "forma ja fechada")
        d = _BodyDesc()
        _check(self._lib().pc_body_desc_default(C.byref(d)))
        d.type = BODY_TYPES[type]
        d.shape = shape._h
        d.density, d.mass = float(density), float(mass)
        d.position, d.rotation = _v(position), _q(rotation)
        d.linear_velocity, d.angular_velocity = _v(linear_velocity), _v(angular_velocity)
        d.friction, d.restitution = float(friction), float(restitution)
        d.linear_damping, d.angular_damping = float(linear_damping), float(angular_damping)
        d.gravity_scale = float(gravity_scale)
        d.allow_sleep = 1 if allow_sleep else 0
        bid = C.c_uint32()
        _check(self._lib().pc_body_create(self._need(), C.byref(d), C.byref(bid)))
        return Body(self, bid.value)

    def body(self, body_id: int) -> Body:
        """Embrulha um id já conhecido (p.ex. vindo de um raycast ou de um evento)."""
        return Body(self, int(body_id))

    @property
    def body_count(self) -> int:
        return _check(self._lib().pc_world_read_transforms(self._need(), 0, 0, None, None))

    def read_transforms(self, interpolated: bool = True) -> List[Tuple[int, Vec3, Quat]]:
        """Poses de TODOS os corpos em uma só chamada: [(id, (px,py,pz), (qw,qx,qy,qz)), ...]. É o que um render deve usar."""
        lib, h = self._lib(), self._need()
        n = _check(lib.pc_world_read_transforms(h, 0, 0, None, None))
        ids = (C.c_uint32 * max(n, 1))()
        data = (C.c_double * max(7 * n, 1))()
        _check(lib.pc_world_read_transforms(h, 1 if interpolated else 0, n, ids, data))
        return [(ids[i], (data[7 * i], data[7 * i + 1], data[7 * i + 2]),
                 (data[7 * i + 3], data[7 * i + 4], data[7 * i + 5], data[7 * i + 6])) for i in range(n)]

    def read_transforms_numpy(self, interpolated: bool = True):
        """Como read_transforms, mas devolve (ids: ndarray[n], dados: ndarray[n,7]); exige numpy (cópia única, sem laço Python)."""
        import numpy as np  # opcional

        lib, h = self._lib(), self._need()
        n = _check(lib.pc_world_read_transforms(h, 0, 0, None, None))
        ids = np.zeros(n, dtype=np.uint32)
        data = np.zeros((n, 7), dtype=np.float64)
        if n:
            _check(lib.pc_world_read_transforms(h, 1 if interpolated else 0, n,
                                                ids.ctypes.data_as(C.POINTER(C.c_uint32)), data.ctypes.data_as(C.POINTER(C.c_double))))
        return ids, data

    # --- juntas ---
    def create_ball_socket(self, a: Body, b: Body, anchor: Sequence[float], collide_connected: bool = False) -> Joint:
        j = C.c_uint32()
        _check(self._lib().pc_joint_create_ball_socket(self._need(), a.id, b.id, C.byref(_v(anchor)), 1 if collide_connected else 0, C.byref(j)))
        return Joint(self, j.value)

    def create_distance(self, a: Body, b: Body, anchor_a: Sequence[float], anchor_b: Sequence[float], length: float = -1.0,
                        collide_connected: bool = False) -> Joint:
        j = C.c_uint32()
        _check(self._lib().pc_joint_create_distance(self._need(), a.id, b.id, C.byref(_v(anchor_a)), C.byref(_v(anchor_b)), float(length),
                                                    1 if collide_connected else 0, C.byref(j)))
        return Joint(self, j.value)

    def create_hinge(self, a: Body, b: Body, anchor: Sequence[float], axis: Sequence[float], collide_connected: bool = False) -> Joint:
        j = C.c_uint32()
        _check(self._lib().pc_joint_create_hinge(self._need(), a.id, b.id, C.byref(_v(anchor)), C.byref(_v(axis)),
                                                 1 if collide_connected else 0, C.byref(j)))
        return Joint(self, j.value)

    # --- eventos ---
    def contact_events(self) -> List[ContactEvent]:
        """Eventos de contato do último update/step (modelo pull)."""
        lib, h = self._lib(), self._need()
        n = _check(lib.pc_world_read_contact_events(h, 0, None))
        buf = (_ContactEvent * max(n, 1))()
        _check(lib.pc_world_read_contact_events(h, n, buf))
        return [ContactEvent(buf[i]) for i in range(n)]

    def clear_contact_events(self) -> None:
        _check(self._lib().pc_world_clear_contact_events(self._need()))

    # --- consultas ---
    def raycast(self, origin: Sequence[float], direction: Sequence[float], max_distance: float = math.inf,
                include: Optional[Iterable[str]] = None, ignore: Optional[Body] = None) -> Optional[Hit]:
        h = _RayHit()
        r = _check(self._lib().pc_world_raycast(self._need(), C.byref(_v(origin)), C.byref(_v(direction)), float(max_distance),
                                                _flags(include), ignore.id if ignore else INVALID_BODY, C.byref(h)))
        return Hit(h) if r == 1 else None

    def raycast_all(self, origin: Sequence[float], direction: Sequence[float], max_distance: float = math.inf,
                    include: Optional[Iterable[str]] = None, ignore: Optional[Body] = None) -> List[Hit]:
        lib, hd = self._lib(), self._need()
        args = (C.byref(_v(origin)), C.byref(_v(direction)), float(max_distance), _flags(include), ignore.id if ignore else INVALID_BODY)
        n = _check(lib.pc_world_raycast_all(hd, *args, 0, None))
        buf = (_RayHit * max(n, 1))()
        _check(lib.pc_world_raycast_all(hd, *args, n, buf))
        return [Hit(buf[i]) for i in range(n)]

    def overlap_sphere(self, center: Sequence[float], radius: float, include: Optional[Iterable[str]] = None,
                       ignore: Optional[Body] = None) -> List[int]:
        """Ids dos corpos que tocam a esfera."""
        lib, h = self._lib(), self._need()
        args = (C.byref(_v(center)), float(radius), _flags(include), ignore.id if ignore else INVALID_BODY)
        n = _check(lib.pc_world_overlap_sphere(h, *args, 0, None))
        ids = (C.c_uint32 * max(n, 1))()
        _check(lib.pc_world_overlap_sphere(h, *args, n, ids))
        return [ids[i] for i in range(n)]

    def overlap_point(self, point: Sequence[float], include: Optional[Iterable[str]] = None, ignore: Optional[Body] = None) -> List[int]:
        lib, h = self._lib(), self._need()
        args = (C.byref(_v(point)), _flags(include), ignore.id if ignore else INVALID_BODY)
        n = _check(lib.pc_world_overlap_point(h, *args, 0, None))
        ids = (C.c_uint32 * max(n, 1))()
        _check(lib.pc_world_overlap_point(h, *args, n, ids))
        return [ids[i] for i in range(n)]


def version() -> str:
    """Versão da biblioteca C carregada."""
    return load_library().pc_version_string().decode()


def real_is_double() -> bool:
    """True se o motor foi compilado com double."""
    return bool(load_library().pc_real_is_double())
