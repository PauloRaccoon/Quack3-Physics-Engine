# Quack3 — API C e ligação Python (`physics_capi`)

Esta pasta fica **ao lado** de `physics_engine/` (o motor Quack3) e apenas o consome. Ela compila o motor numa **biblioteca
compartilhada** (`physics_c.dll`, `libphysics_c.so` ou `libphysics_c.dylib`) com uma **API C99** estável, e traz
uma ligação **Python** (`quack_physics`) feita com `ctypes`, sem dependências. Qualquer linguagem capaz de
chamar funções C (Python, C#, Rust, Lua, Java, Go, Julia, Zig...) usa o motor por aqui.

```text
physics_capi/
├── include/physics_c/physics_c.h     a API (C99 puro, comentada): ~55 funções
├── src/physics_c.cpp                 implementação sobre o motor C++
├── tests/capi_tests.c                testes escritos em C puro
├── python/quack_physics/             ligação Python (ctypes, sem dependências)
│   ├── tests/test_quack_physics.py   testes da ligação
│   └── examples/                     stack_boxes.py (sem janela), raylib_demo.py (com janela)
└── CMakeLists.txt
```

---

## 1. Compilar

Requisitos: CMake ≥ 3.14 e um compilador C99/C++17. Python 3 é opcional (só para os testes e a ligação).

```bash
cmake -S physics_capi -B build_capi            # float (padrão do motor)
cmake --build build_capi -j
ctest --test-dir build_capi                    # testes em C + testes Python (se houver python3)
```

| Opção do CMake | Padrão | O que faz |
|---|---|---|
| `PC_USE_DOUBLE` | `OFF` | compila o motor em `double` (a API continua toda em `double`) |
| `PC_SANITIZE` | `OFF` | AddressSanitizer + UBSan (GCC/Clang; para os testes) |
| `PC_BUILD_TESTS` | `ON` | compila `capi_tests` e registra os testes Python |
| `PHYSICS_ENGINE_DIR` | `../physics_engine` | caminho do motor, se estiver em outro lugar |

Detalhes da biblioteca:

- O motor entra como biblioteca estática **dentro** da `.so`/`.dll`; só os símbolos `pc_*` são exportados.
- No Windows com MinGW, a `physics_c.dll` sai **autossuficiente** (libstdc++ e libgcc embutidas): o Python a carrega sem precisar do `mingw64\bin` no `PATH`.
- A saída fica em `build_capi/` (ou `build_capi/Release/` em geradores multi-configuração).

---

## 2. Python

### Instalação e busca da biblioteca

Não há pacote para `pip`: o módulo é a pasta `physics_capi/python/quack_physics`. Coloque `physics_capi/python`
no `PYTHONPATH`. A biblioteca compartilhada é procurada, nesta ordem:

1. variável de ambiente `QUACK_PHYSICS_LIB` (caminho do arquivo);
2. ao lado do pacote;
3. pastas `build*/` (e `build*/Release/`) dentro de `physics_capi/` e ao lado dela.

Se nada for achado, `FileNotFoundError` lista onde procurou. Ao carregar, o módulo confere `pc_abi_version()`.

```bash
# Linux / macOS
QUACK_PHYSICS_LIB=build_capi/libphysics_c.so PYTHONPATH=physics_capi/python python3 physics_capi/python/examples/stack_boxes.py
# Windows (PowerShell)
$env:QUACK_PHYSICS_LIB="build_capi\physics_c.dll"; $env:PYTHONPATH="physics_capi\python"; python physics_capi\python\examples\stack_boxes.py
```

### Primeiros passos

```python
import quack_physics as qp

with qp.World(gravity=(0, -9.81, 0)) as world:
    chao = world.create_body(qp.Shape.box((10, 0.5, 10)), type="static", position=(0, -0.5, 0))
    caixa = world.create_body(qp.Shape.box((0.5, 0.5, 0.5)), position=(0, 3, 0))
    for _ in range(120):
        world.update(1 / 60)                      # tempo de render; o motor roda passos fixos
    print(caixa.position, caixa.rotation)         # rotation = quatérnio (w, x, y, z)
    print(world.raycast((0, 10, 0), (0, -1, 0)))
```

Convenções: tudo em unidades SI; vetores são tuplas `(x, y, z)`; quatérnios são tuplas `(w, x, y, z)`. Erros do
motor viram `PhysicsError` (com `.code` e a mensagem do motor). `World` e `Shape` liberam a memória no `close()`,
no `with` ou ao serem coletados.

### Referência da ligação

**`World(gravity, fixed_time_step, max_bodies, max_sub_steps, velocity_iterations, position_iterations, orientation_exponential, sleep, sleep_time, warm_starting)`**

| Membro | Descrição |
|---|---|
| `update(frame_time)` | avança com o tempo de render (acumulador); devolve quantos passos fixos rodaram |
| `step()` | exatamente um passo fixo |
| `alpha` | α = acumulador/Δt, para interpolar |
| `gravity`, `fixed_time_step` | leitura e escrita (Δt entre 1e-4 e 1 s) |
| `set_solver_iterations(velocity, position)` | iterações do solver e da correção de posição |
| `set_orientation_exponential(on)` | orientação por exponencial de quatérnio (padrão) ou Euler |
| `set_sleep(enabled, sleep_time)`, `wake_all()` | controle do sleeping |
| `stats` | `Stats`: corpos, acordados, dormindo, ilhas, juntas, pares, pares tocando, pontos de contato, passos |
| `create_body(shape, type, position, rotation, density, mass, linear_velocity, angular_velocity, friction, restitution, linear_damping, angular_damping, gravity_scale, allow_sleep)` | cria um corpo (`type` = `"static"`, `"kinematic"` ou `"dynamic"`) e devolve `Body` |
| `body(id)` | embrulha um id já conhecido (de um raycast ou de um evento) |
| `body_count` | número de corpos vivos |
| `read_transforms(interpolated=True)` | **todas** as poses em uma chamada: `[(id, (px,py,pz), (qw,qx,qy,qz)), ...]` |
| `read_transforms_numpy(interpolated=True)` | o mesmo em `(ids, dados[n,7])`; exige numpy (opcional) |
| `create_ball_socket(a, b, anchor, collide_connected)` | junta bola-soquete |
| `create_distance(a, b, anchor_a, anchor_b, length=-1, collide_connected)` | junta de distância (`length < 0` = a distância atual) |
| `create_hinge(a, b, anchor, axis, collide_connected)` | dobradiça |
| `contact_events()`, `clear_contact_events()` | eventos de contato do último `update`/`step` (modelo *pull*) |
| `raycast(origin, direction, max_distance, include, ignore)` | acerto mais próximo (`Hit`) ou `None` |
| `raycast_all(...)` | todos os acertos, ordenados por distância |
| `overlap_sphere(center, radius, include, ignore)`, `overlap_point(point, include, ignore)` | ids dos corpos que tocam |

`include` é uma lista com `"static"`, `"kinematic"` e/ou `"dynamic"` (padrão: todos); `ignore` é um `Body` a descartar.

**`Shape`** — `Shape.sphere(radius)`, `Shape.box(half_extents)`, `Shape.capsule(radius, half_height)`,
`Shape.cylinder(radius, half_height)`, `Shape.polyhedron(vertices, faces)`, `close()`. Cápsula e cilindro têm o
eixo no Y local. No poliedro (**convexo**), cada face é uma lista de índices em sentido anti-horário visto de
fora; a API recusa poliedros côncavos, com faces não planares ou com normais para dentro. Uma forma serve a
vários corpos, e pode ser fechada logo depois de criar os corpos.

**`Body`** — propriedades `valid`, `type`, `mass`, `position`, `center_of_mass`, `rotation`, `linear_velocity`
(leitura e escrita), `angular_velocity` (leitura e escrita), `awake` (leitura e escrita), `interpolated`
(`(posição, rotação)` para desenhar); métodos `set_transform(position, rotation)` (teletransporte),
`apply_force(force, at_point=None)`, `apply_torque(torque)`, `apply_impulse(impulse, at_point=None)`,
`set_material(friction, restitution)`, `set_damping(linear, angular)`, `set_gravity_scale(scale)`, `destroy()`.

**`Joint`** — `valid`, `destroy()`. Uma junta deixa de ser válida quando um corpo ligado a ela é destruído.

**Resultados** — `Hit` (`body_id`, `distance`, `point`, `normal`, `started_inside`); `ContactEvent` (`kind` =
`"begin"` ou `"end"`, `body_a`, `body_b`, `point`, `normal`, `depth`, `point_count`, `normal_impulse`,
`approach_speed`); `Stats`.

**Funções do módulo** — `qp.version()`, `qp.real_is_double()`, `qp.load_library(path)`.

### Exemplo: juntas, poliedro, consultas e eventos

```python
import quack_physics as qp

with qp.World() as world:
    chao = world.create_body(qp.Shape.box((10, 0.5, 10)), type="static", position=(0, -0.5, 0))

    # pêndulo: um corpo estático faz o papel do mundo
    pivo = world.create_body(qp.Shape.sphere(0.1), type="static", position=(0, 5, 0))
    peso = world.create_body(qp.Shape.sphere(0.3), position=(2, 5, 0), mass=2)
    junta = world.create_ball_socket(pivo, peso, anchor=(0, 5, 0))

    # tetraedro convexo
    tetra = qp.Shape.polyhedron([(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)],
                                [(0, 1, 2), (0, 3, 1), (0, 2, 3), (1, 3, 2)])
    t = world.create_body(tetra, position=(4, 2, 0))

    for _ in range(60):
        world.update(1 / 60)
        for e in world.contact_events():
            if e.kind == "begin" and e.approach_speed > 1.0:
                print("impacto", e)

    print(world.overlap_sphere((0, 0, 0), 20, include=["dynamic"]))
    print(world.raycast((4, 20, 0), (0, -1, 0), include=["dynamic"]))
```

### Desenhar com interpolação

Física e render têm taxas diferentes: a física roda em passos fixos e o desenho usa a **pose interpolada**,
lida de uma vez para todos os corpos:

```python
world.update(frame_time)                           # no laço de render
for body_id, pos, quat in world.read_transforms():  # interpolated=True
    draw(body_id, pos, quat)
```

### Exemplos

| Arquivo | O que faz |
|---|---|
| `python/examples/stack_boxes.py` | sem janela: empilha caixas, roda 4 s, imprime posições, quatérnios, eventos de contato e um raycast |
| `python/examples/raylib_demo.py` | com janela (`pip install raylib`): pilha de caixas desenhada pela raylib com pose interpolada; `Espaço` lança uma esfera, `R` recomeça, `←` e `→` giram a câmera |

---

## 3. API C

O cabeçalho `include/physics_c/physics_c.h` é a documentação de referência (cada função está comentada).

### Regras

- **Tudo é `double`**, em unidades SI, qualquer que seja a precisão interna do motor (`pc_real_is_double()` informa).
- **Quatérnios** em ordem `(w, x, y, z)`. A API normaliza o que receber; norma zero é recusada.
- **Estruturas** (`pc_vec3`, `pc_quat`, `pc_body_desc`...) têm layout fixo (apenas `double`, `int32_t`, `uint32_t`) e são passadas **por ponteiro**. As funções `pc_*_default()` preenchem os valores padrão.
- **Handles:** `pc_world*` e `pc_shape*` são opacos; corpos e juntas são ids numéricos (`pc_body_id`, `pc_joint_id`). Um id que não existe mais (corpo destruído, ou junta de um corpo destruído) é recusado com erro, e nenhum ponteiro fica pendurado.
- **Erros:** as funções devolvem `int`; `>= 0` é sucesso (ou uma contagem) e `< 0` é erro. As funções que criam handles devolvem `NULL` em erro. `pc_last_error()` devolve o texto do último erro **da thread**. Nenhuma exceção C++ atravessa a API.

| Código | Significado |
|---|---|
| `PC_OK` (0) | sucesso |
| `PC_ERR_INVALID_ARGUMENT` (−1) | ponteiro nulo, valor não finito, faixa inválida, forma inválida |
| `PC_ERR_INVALID_HANDLE` (−2) | mundo ou forma nulos; corpo ou junta que não existe (mais) |
| `PC_ERR_CAPACITY` (−3) | `max_bodies` atingido |
| `PC_ERR_INTERNAL` (−4) | qualquer outra exceção do motor (veja `pc_last_error()`) |

- **Entradas ruins** (NaN, infinito, ponteiro nulo, forma inválida, valores fora da faixa) viram erro; o motor nunca recebe lixo.
- **Threads:** um mundo não é thread-safe; use-o de uma thread por vez. Mundos diferentes são independentes.
- **Versão da ABI:** `pc_abi_version()` sobe quando algo muda de forma incompatível. A ligação Python confere ao carregar.
- **Leituras em lote** (poses, eventos, `raycast_all`, `overlap`) usam o padrão “chame com `capacity = 0` para contar, depois com um buffer”: devolvem sempre o **total**, e copiam no máximo `capacity` itens.

### Funções

**Informações:** `pc_abi_version`, `pc_version_string`, `pc_real_is_double`, `pc_last_error`.

**Mundo**

| Função | Descrição |
|---|---|
| `pc_world_settings_default`, `pc_world_create(settings)`, `pc_world_destroy` | cria (com `NULL` = padrões) e destrói; destruir libera corpos, juntas e formas internas |
| `pc_world_update(world, frame_time)` | avança com o tempo de render; devolve o nº de passos executados |
| `pc_world_step` | exatamente um passo fixo |
| `pc_world_get_alpha` | α para interpolação |
| `pc_world_get_gravity`, `pc_world_set_gravity` | gravidade |
| `pc_world_get_fixed_time_step`, `pc_world_set_fixed_time_step` | Δt fixo, de 1e-4 a 1 s |
| `pc_world_set_solver_iterations` | iterações de velocidade e de posição |
| `pc_world_set_orientation_exponential` | exponencial de quatérnio ou Euler |
| `pc_world_set_sleep`, `pc_world_wake_all` | sleeping |
| `pc_world_get_stats` | `pc_world_stats` |

`pc_world_settings`: `gravity` (0, −9,81, 0), `fixed_time_step` (1/60), `max_sub_steps` (8), `max_bodies` (4096),
`velocity_iterations` (10), `position_iterations` (3), `warm_starting` (1), `orientation_exponential` (1),
`sleep_enabled` (1), `sleep_time` (0,5 s).

**Formas:** `pc_shape_create_sphere`, `pc_shape_create_box`, `pc_shape_create_capsule`,
`pc_shape_create_cylinder`, `pc_shape_create_polyhedron(vertices, n_vertices, face_indices, face_sizes, n_faces)`,
`pc_shape_destroy`. O corpo guarda a sua própria referência: chamar `pc_shape_destroy` logo após criar os
corpos é seguro.

**Corpos**

| Função | Descrição |
|---|---|
| `pc_body_desc_default`, `pc_body_create(world, desc, &id)`, `pc_body_destroy`, `pc_body_is_valid` | ciclo de vida; destruir um corpo destrói também as juntas dele |
| `pc_body_get_state` | `pc_body_state`: tipo, acordado, massa, posição, centro de massa, rotação e velocidades |
| `pc_body_get_interpolated` | pose para desenhar (lerp na posição, slerp na orientação) |
| `pc_body_set_transform` | teletransporte |
| `pc_body_set_linear_velocity`, `pc_body_set_angular_velocity` | velocidades |
| `pc_body_apply_force`, `pc_body_apply_force_at_point`, `pc_body_apply_torque` | valem no próximo passo |
| `pc_body_apply_impulse_at_point` | muda a velocidade imediatamente |
| `pc_body_set_material`, `pc_body_set_damping`, `pc_body_set_gravity_scale`, `pc_body_set_awake` | parâmetros do corpo |
| `pc_world_read_transforms(world, interpolated, capacity, ids, data)` | poses em lote: para cada corpo vivo, em ordem crescente de id, `ids[i]` e 7 `double` em `data[7*i]`: `px py pz qw qx qy qz` |

Tipos de corpo: `PC_BODY_STATIC` (0), `PC_BODY_KINEMATIC` (1), `PC_BODY_DYNAMIC` (2).
`pc_body_desc`: `type`, `shape`, `density` (1), `mass` (> 0 sobrescreve a densidade), `position`, `rotation`,
`linear_velocity`, `angular_velocity`, `friction` (0,5), `restitution` (0), `linear_damping` (0),
`angular_damping` (0,01), `gravity_scale` (1), `allow_sleep` (1).

**Juntas:** `pc_joint_create_ball_socket`, `pc_joint_create_distance`, `pc_joint_create_hinge`, `pc_joint_destroy`,
`pc_joint_is_valid`. Âncoras e eixos em coordenadas do **mundo**, na pose atual dos corpos. Para prender um
corpo ao mundo, crie um corpo estático e use-o como `a`.

**Eventos de contato (modelo *pull*):** `pc_world_read_contact_events(world, capacity, out)` e
`pc_world_clear_contact_events`. Cada `pc_contact_event` traz `type` (`PC_CONTACT_BEGIN` ou `PC_CONTACT_END`),
`body_a < body_b`, `point`, `normal` (de a para b), `depth`, `point_count` (1 a 4), `normal_impulse` e
`approach_speed` (a “velocidade de impacto”). Os eventos valem até o `update`/`step` seguinte, que os esvazia.

**Consultas espaciais:** `pc_world_raycast` (devolve 1 se acertou, 0 se não), `pc_world_raycast_all`,
`pc_world_overlap_sphere`, `pc_world_overlap_point`. Os filtros são `PC_QUERY_STATIC` (1),
`PC_QUERY_KINEMATIC` (2), `PC_QUERY_DYNAMIC` (4) e `PC_QUERY_ALL` (7); `ignore_body` aceita `PC_INVALID_BODY`
para não ignorar nenhum; `max_distance` pode ser `INFINITY`. `pc_raycast_hit` traz `body`, `distance`, `point`,
`normal` e `started_inside`.

### Exemplo completo em C

```c
#include <stdio.h>
#include "physics_c/physics_c.h"

int main(void) {
    pc_world* w = pc_world_create(NULL);                       /* padrões */
    if (!w) { printf("erro: %s\n", pc_last_error()); return 1; }

    pc_vec3 half = {10, 0.5, 10};
    pc_shape* chao_forma = pc_shape_create_box(&half);
    pc_body_desc d; pc_body_desc_default(&d);
    d.type = PC_BODY_STATIC; d.shape = chao_forma; d.position.y = -0.5;
    pc_body_id chao; pc_body_create(w, &d, &chao);

    pc_shape* bola = pc_shape_create_sphere(0.5);
    pc_body_desc_default(&d);
    d.shape = bola; d.mass = 1; d.position.y = 3; d.restitution = 0.4;
    pc_body_id id;
    if (pc_body_create(w, &d, &id) < 0) { printf("%s\n", pc_last_error()); return 1; }
    pc_shape_destroy(chao_forma); pc_shape_destroy(bola);      /* os corpos guardam a sua referência */

    for (int i = 0; i < 120; ++i) pc_world_update(w, 1.0 / 60.0);

    pc_body_state s; pc_body_get_state(w, id, &s);
    printf("y = %.3f  q = (%.3f %.3f %.3f %.3f)\n", s.position.y, s.rotation.w, s.rotation.x, s.rotation.y, s.rotation.z);
    pc_world_destroy(w);
    return 0;
}
```

Compilar e ligar (Linux): `gcc -std=c99 -Iphysics_capi/include exemplo.c -Lbuild_capi -lphysics_c -Wl,-rpath,build_capi -o exemplo`.
No Windows, `physics_c.dll` precisa estar ao lado do executável (ou no `PATH`).

---

## 4. Outras linguagens

A API C é a mesma para todas: carregue a biblioteca, copie as estruturas do cabeçalho (mesma ordem de campos;
`int32_t`, `uint32_t` e `double`) e confira `pc_abi_version()`. Os testes automatizados deste repositório cobrem
C e Python; os trechos abaixo mostram o formato da ligação nas demais linguagens.

C# (P/Invoke):

```csharp
[StructLayout(LayoutKind.Sequential)] struct Vec3 { public double x, y, z; }
[DllImport("physics_c")] static extern IntPtr pc_world_create(IntPtr settingsOrNull);
[DllImport("physics_c")] static extern int pc_world_update(IntPtr world, double frameTime);
[DllImport("physics_c")] static extern void pc_world_destroy(IntPtr world);
```

Rust:

```rust
#[link(name = "physics_c")]
extern "C" {
    fn pc_world_create(settings: *const std::ffi::c_void) -> *mut std::ffi::c_void;
    fn pc_world_update(world: *mut std::ffi::c_void, frame_time: f64) -> i32;
    fn pc_world_destroy(world: *mut std::ffi::c_void);
}
```

Lua (LuaJIT FFI):

```lua
local ffi = require("ffi")
ffi.cdef[[ typedef struct pc_world pc_world; pc_world* pc_world_create(const void*); int pc_world_update(pc_world*, double); void pc_world_destroy(pc_world*); ]]
local lib = ffi.load("physics_c")
```

---

## 5. Limites da API

- **Sem *callbacks*** do C para a linguagem hospedeira: os eventos de contato são lidos por *pull* (`pc_world_read_contact_events` depois do `update`).
- **Sem filtros de colisão por camada** e **sem `userData`**: guarde do seu lado um dicionário `id → objeto`.
- **Sem criação de malhas a partir de `.obj`**: o fecho convexo de modelos é função do viewer. Pela API, passe vértices e faces já convexos a `pc_shape_create_polyhedron`.
- **Sem pacote para `pip`**: a ligação Python usa a biblioteca que você compilou (`QUACK_PHYSICS_LIB`).
- **Não é thread-safe** (ver acima).

---

## 6. Testes

`ctest --test-dir build_capi` roda:

- `capi_tests` — testes em C puro que exercitam toda a API (criação e destruição, estados, juntas, eventos, consultas, leitura em lote e entradas inválidas);
- `python_tests` — os testes da ligação Python (`python/tests/test_quack_physics.py`).

Ambos passam com o motor em `float` e em `double`, e `capi_tests` também sob AddressSanitizer + UBSan
(`-DPC_SANITIZE=ON`).
