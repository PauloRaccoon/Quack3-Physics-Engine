# Quack3 — Motor de Física 3D — v1.0.0

**Quack3** é um motor de física de **corpos rígidos 3D** em **C++17**, escrito do zero e **sem nenhuma dependência externa**
(nem GLM, Eigen, Bullet ou PhysX). A orientação é representada por **quatérnios** do início ao fim da
simulação; a colisão usa **GJK/EPA** sobre uma broadphase de **árvore AABB dinâmica**; a resolução usa
**Sequential Impulses** com *warm starting*, atrito de Coulomb, restituição, correção de posição NGS e juntas.

O nome é um trocadilho com *quaternion*: **Quack** + **terno** (3). Ele batiza o projeto inteiro: o motor, a API C
e o viewer.

O projeto é formado por três pastas. O motor não depende de nenhuma das outras duas:

```text
projeto/
├── README.md              este arquivo (o motor)
├── physics_engine/        o motor Quack3: biblioteca C++, testes, demo sem janela
├── physics_capi/          API C + ligação Python (o Quack3 para outras linguagens)
└── physics_viewer/        Quack3 Viewer: viewer interativo (C++ + raylib)
```

| Documento | Conteúdo |
|---|---|
| `README.md` (este) | O que o motor faz, como compilar, convenções, guia da API C++ e limites |
| [`physics_capi/README.md`](physics_capi/README.md) | Biblioteca compartilhada `physics_c`, referência completa da API C e da ligação Python |
| [`physics_viewer/README.md`](physics_viewer/README.md) | Uso completo do viewer: controles, painéis, cenas, importação de modelos 3D, gravação CSV |
| [`physics_viewer/scenes/LEIA-ME.txt`](physics_viewer/scenes/LEIA-ME.txt) | A pasta de cenas `.scene` |
| [`physics_viewer/models/LEIA-ME.txt`](physics_viewer/models/LEIA-ME.txt) | A pasta de modelos `.obj` |

---

## 1. O que o motor faz

### Matemática (`math/`)
`Vector3`, `Matrix3`, `Quaternion` e `Transform` próprios. O quatérnio é o estado de orientação do corpo:
normalização, multiplicação, rotação de vetores, conversão para matriz e eixo-ângulo, **slerp** e integração
por **exponencial de quatérnio** (`q ← exp(½·Δt·ω)·q`, exata para ω constante dentro do passo).

### Formas (`geometry/`)

| Forma | Observações |
|---|---|
| `SphereShape` | raio |
| `BoxShape` | meias-arestas |
| `CapsuleShape` | raio e meia-altura do segmento; eixo no Y local; com segmento nulo equivale a uma esfera |
| `CylinderShape` | raio e meia-altura; eixo no Y local |
| `PolyhedronShape` | poliedro **convexo** qualquer, descrito por vértices e faces (`HalfEdgeStructure`); a validação exige convexidade, faces planares e normais para fora |

Cada forma fornece função de suporte (usada pelo GJK/EPA), AABB, volume, **tensor de inércia** e raycast. As
fórmulas de inércia são conferidas contra integração numérica nos testes. Formas são imutáveis e podem ser
compartilhadas entre vários corpos (`std::shared_ptr<const Shape>`).

### Corpos rígidos (`dynamics/`)
- Três tipos: **estático** (massa infinita, imóvel), **cinemático** (movido por quem usa o motor; empurra os dinâmicos) e **dinâmico**.
- Massa por densidade, por valor direto ou por `MassData` personalizado (massa, centro de massa e inércia). O centro de massa pode ficar fora do frame do corpo.
- Atrito, restituição, amortecimento linear e angular, escala de gravidade, permissão de dormir e `userData`.
- Forças, torques e impulsos (no centro de massa ou em um ponto do mundo); teletransporte por `setBodyTransform`.
- **Integrador** semi-implícito com:
  - orientação por exponencial de quatérnio (padrão) ou Euler;
  - **termo giroscópico** (ω × Iω) por um passo de Newton implícito, para corpos assimétricos em rotação livre;
  - **divisão de Strang** do giro livre ao redor do solver, que mantém giroscópios e piões estáveis presos a juntas (um disco a 60 rad/s num pivô precessiona a 0,782 rad/s contra 0,785 da teoria, sem tombar a 60 Hz);
  - limites de velocidade (`kMaxLinearSpeed` = 200 m/s e `kMaxAngularSpeed` = 200 rad/s) que preservam a direção.

### Detecção de colisão (`collision/`)
- **Broadphase:** `DynamicAABBTree` balanceada por rotações AVL, com *fat AABBs* (margem de 0,1 m e antecipação do movimento), inserção e remoção incrementais e geração de pares.
- **Narrowphase:** **GJK** para distância e interpenetração rasa, **EPA** para profundidade e normal, e `ContactManifold` com até 4 pontos.
- **Contatos persistentes:** os pontos são casados entre passos (raio de 2 cm), o que permite *warm starting* e repouso estável.
- **Contatos especulativos:** formas separadas por até 5 mm já geram contato.
- Cápsulas e cilindros deitados geram **2 pontos** de contato, então não balançam sobre o chão.
- **Filtro de colisão** definido por quem usa o motor (camadas, grupos), aplicado a cada par novo.
- Corpos ligados por junta não colidem entre si por padrão (configurável por junta).

### Resolução de restrições (`constraints/`)
- **Sequential Impulses** com *warm starting*; 10 iterações de velocidade por padrão.
- **Atrito de Coulomb** em duas direções tangentes, com mistura `√(μa·μb)`, e **restituição** com *velocity slop*.
- **Atrito de torção** (giro em torno da normal do contato) e **resistência à rolagem** para esfera, cápsula e cilindro. Sem eles, uma bola girando sobre um único ponto de contato nunca pararia. Ambos são proporcionais a μ·Σλn e ajustáveis em `SolverSettings::spinFriction` e `rollingFriction` (0 desliga).
- **Correção de posição NGS** (Gauss–Seidel não linear, 3 iterações por padrão): move as poses diretamente, sem criar velocidade, e conserva o momento angular do corpo. `positionIterations = 0` volta ao Baumgarte clássico.
- **Juntas** (entre dois corpos; para prender um corpo ao mundo, use um corpo estático):
  - `BallSocketJoint` — bola-soquete, fixa um ponto;
  - `DistanceJoint` — distância fixa entre duas âncoras;
  - `HingeJoint` — dobradiça com eixo, que deixa só a rotação em torno dele.

### Pipeline e simulação (`pipeline/`)
`PhysicsWorld` é o ponto de entrada. Cada passo fixo executa:

```text
atualizar AABBs → broadphase → narrowphase (manifolds persistentes)
→ ilhas (acordar) → forças e gravidade → solver de velocidade (contatos + juntas)
→ integrar posições → correção de posição (NGS) → registrar pose para o render
→ sleeping → eventos de contato
```

- **Passo fixo com acumulador:** `update(frameTime)` recebe o tempo do quadro de render, executa de 0 a `maxSubSteps` (8) passos fixos e atualiza o α. O Δt pode ser trocado em execução (`setFixedTimeStep`, de 1e-4 a 1 s) sem que a imagem “pule”.
- **Interpolação de render:** cada corpo guarda as duas últimas poses; `getInterpolatedTransform` devolve `lerp` na posição e `slerp` na orientação com o α do acumulador. A simulação não muda com o FPS do render.
- **Sleeping com ilhas:** um corpo dorme quando |v| e |ω| ficam abaixo dos limiares por `timeToSleep`, **junto com toda a sua ilha** (corpos ligados por contato ou junta). Corpos dormindo não são integrados, não entram no solver e seus pares não rodam a narrowphase. Acordam por força, impulso, velocidade, teletransporte, mudança de massa ou gravidade, remoção do apoio, criação ou remoção de junta e contato com corpo acordado ou cinemático em movimento.
- **Eventos de contato:** início e fim de contato com velocidade de aproximação, impulso normal, ponto e normal. Consumo por ouvinte (`IContactListener`) ou por buffer (`getContactEvents`).
- **Estatísticas e perfilamento:** `getStats()` informa corpos acordados e dormindo, ilhas, juntas, pares, pontos de contato e contatos no solver; com o perfilamento ligado informa também o tempo (µs) de cada fase do último passo. Desligado, não custa nada.

### Consultas espaciais
`raycast`, `raycastAll` (ordenado por distância), `overlapSphere`, `overlapPoint` e `overlapShape` (teste exato por GJK). Todas aceitam um `QueryFilter` (incluir ou excluir estáticos, cinemáticos e dinâmicos; ignorar um corpo; predicado próprio), são `const` e não alteram o mundo nem o determinismo. Entradas inválidas (NaN, alcance negativo) resultam em “nada encontrado”, nunca em exceção.

### Ponte com o renderizador (`bridge/`)
`MotionState` guarda as duas poses de cada corpo. `IPhysicsTransformBridge` recebe callbacks de criação, destruição e atualização; `BufferedTransformBridge` já implementa um buffer contíguo de 7 valores por corpo (`px py pz qx qy qz qw`), indexado pelo `BodyId`.

### Debug draw (`debug/`)
O motor gera as linhas e o renderizador só precisa desenhar um segmento 3D (`IDebugRenderer::drawLine`). Há flags para formas, AABBs, *fat* AABBs, pontos e normais de contato, juntas, centro de massa, velocidades e eixos locais. `DebugLineBuffer` acumula as linhas num vetor contíguo.

### Robustez
O motor é submetido a *fuzzing* determinístico (sequências aleatórias de chamadas com NaN, ±Inf, `1e30`, `FLT_MAX`, ids inválidos, formas degeneradas, juntas absurdas e passos de tempo extremos) e a testes de regressão dos defeitos que ele revelou. O contrato resultante está na seção 6.

---

## 2. Compilar e testar

Requisitos: CMake ≥ 3.14 e um compilador C++17 (GCC, Clang ou MSVC).

```bash
cmake -S physics_engine -B build
cmake --build build -j
ctest --test-dir build            # 16 testes: 15 suítes + o demo (que se auto-verifica)
```

| Opção do CMake | Padrão (projeto raiz) | O que faz |
|---|---|---|
| `PHYSICS_USE_DOUBLE` | `OFF` | usa `double` em vez de `float` em todo o motor |
| `PHYSICS_BUILD_TESTS` | `ON` | compila as suítes de teste |
| `PHYSICS_BUILD_EXAMPLES` | `ON` | compila `physics_demo` |
| `PHYSICS_INSTALL` | `ON` | gera as regras de `install` e `find_package` |

Usado como **subprojeto** (`add_subdirectory`), os três últimos já nascem `OFF`: só a biblioteca é compilada.

> **`float` e `double` não se misturam.** A precisão faz parte da ABI. Todos os testes passam nas duas
> configurações, mas o programa e a biblioteca precisam ser compilados com a mesma.

O código compila sem warnings com `-Wall -Wextra -Wpedantic` e passa nas suítes também com
AddressSanitizer + UBSan.

### Suítes de teste
`math`, `geometry`, `dynamics`, `broadphase`, `narrowphase`, `constraints`, `pipeline` (empilhamento,
pirâmide, quique, plataforma cinemática, juntas, determinismo, independência da taxa de quadros), `raycast`
(comparado com marcha de raio por força bruta), `fuzz`, `robustness`, `sleeping` (inclui a invariante de ilhas
e a equivalência com e sem *sleeping*), `events`, `capsule_cylinder`, `friction` e `inertia` (inércia conferida
por integração numérica, pião livre e centro de massa deslocado), mais o `physics_demo`.

---

## 3. Usar o motor em um projeto C++

### `add_subdirectory` (recomendado durante o desenvolvimento)

```cmake
cmake_minimum_required(VERSION 3.14)
project(meu_app LANGUAGES CXX)

add_subdirectory(caminho/para/physics_engine physics_build)

add_executable(meu_app src/main.cpp)
target_link_libraries(meu_app PRIVATE physics::physics)   # inclui os headers e a precisão (float/double)
```

### Instalar e usar `find_package`

```bash
cmake -S physics_engine -B build && cmake --build build
cmake --install build --prefix ~/physics_install
```

```cmake
find_package(physics REQUIRED)
target_link_libraries(meu_app PRIVATE physics::physics)
# cmake -DCMAKE_PREFIX_PATH=~/physics_install ...
```

O pacote instalado registra se foi compilado com `float` ou `double`.

---

## 4. Conceitos e convenções

- **Unidades:** metros, quilogramas, segundos, radianos. **Y é para cima** (gravidade padrão `(0, -9.81, 0)`).
- **Mão direita.** No C++ os quatérnios seguem a ordem `(x, y, z, w)` e o buffer da bridge usa `px py pz qx qy qz qw`. A API C e a ligação Python usam `(w, x, y, z)`.
- **`Real`** é `float` (padrão) ou `double`.
- **Passo fixo:** a física roda a Δt = 1/60 s por padrão. `update()` acumula o tempo de render e executa de 0 a 8 passos fixos, o que impede a “espiral da morte”.
- **Determinismo:** dois mundos construídos com as mesmas chamadas produzem resultado idêntico bit a bit (mesma máquina, mesmo compilador). Entre máquinas, compiladores ou flags (`-ffast-math`, FMA) o resultado **não** é garantido.
- **Memória:** os corpos ficam num `std::vector` de capacidade fixa (`maxBodies`, padrão 4096), então os ponteiros usados pelo solver nunca são invalidados. Criar corpos além do limite lança `std::length_error`. Um `BodyId` destruído é reaproveitado.
- **Sem alocação no laço:** com o mundo em regime, `step()` não aloca. Alocam a criação e a destruição de corpos e juntas e o nascimento de pares de colisão novos.
- **Não é thread-safe.** Um mundo deve ser usado por uma thread por vez; mundos diferentes são independentes.

---

## 5. Guia da API C++

### Exemplo mínimo

```cpp
#include <memory>
#include "physics/geometry/BoxShape.h"
#include "physics/pipeline/PhysicsWorld.h"
using namespace physics;

int main() {
    PhysicsWorld world;                       // gravidade, Δt = 1/60, 10 iterações do solver

    RigidBodyDesc ground;                     // chão estático
    ground.type = BodyType::Static;
    ground.shape = std::make_shared<BoxShape>(Vector3(20, 1, 20));
    ground.transform = Transform(Vector3(0, -1, 0));
    world.createBody(ground);

    RigidBodyDesc box;                        // caixa dinâmica de 1 kg
    box.shape = std::make_shared<BoxShape>(Vector3(0.5, 0.5, 0.5));
    box.mass = 1;
    box.transform = Transform(Vector3(0, 3, 0));
    BodyId id = world.createBody(box);

    for (int frame = 0; frame < 120; ++frame) {
        world.update(1.0 / 60.0);                                // tempo do quadro em segundos
        Transform pose = world.getInterpolatedTransform(id);     // pose de render
        // desenha o modelo em pose.position / pose.rotation
    }
}
```

`examples/main.cpp` (alvo `physics_demo`) é um programa completo sem janela: empilha caixas, confere o
repouso e demonstra a interpolação de render.

### Juntas

```cpp
world.createBallSocketJoint(a, b, /*âncora no mundo*/ Vector3(0, 5, 0));
world.createDistanceJoint(a, b, ancoraA, ancoraB, /*comprimento; <0 = o atual*/ -1);
world.createHingeJoint(a, b, /*âncora*/ Vector3(0, 1, 0), /*eixo*/ Vector3(0, 0, 1));
```

Âncoras e eixos são dados em coordenadas do mundo, com os corpos na pose atual. Para prender um corpo ao
mundo, use um corpo estático como `a`. Destruir um corpo destrói as juntas ligadas a ele.

### Ponte com o renderizador

```cpp
BufferedTransformBridge bridge;     // 7 floats por corpo, contíguos
world.setBridge(&bridge);
// a cada quadro, depois de world.update(dt):
const float* poses = bridge.data(); // slot = BodyId; bridge.isAlive(id) informa se existe
```

Para outro tipo de renderizador, implemente `IPhysicsTransformBridge`.

### Consultas espaciais

```cpp
RaycastHit hit;
if (world.raycast(origem, direcao, /*alcance*/ 100, hit)) {
    // hit.body, hit.point, hit.normal, hit.distance, hit.startedInside
}
std::vector<RaycastHit> todos;  world.raycastAll(origem, direcao, 100, todos);   // ordenados por distância
std::vector<BodyId> ids;        world.overlapSphere(centro, raio, ids);
                                world.overlapPoint(ponto, ids);
```

### Debug draw

```cpp
struct MeuRenderer : IDebugRenderer {
    void drawLine(const Vector3& a, const Vector3& b, const DebugColor& c) override { /* DrawLine3D(...) */ }
};
MeuRenderer r;
debugDraw(world, r);                                   // formas, contatos, normais, juntas
DebugDrawSettings s; s.flags = kDebugAll;              // + AABBs, fat AABBs, velocidades, eixos...
debugDraw(world, r, s);
```

### Eventos de contato

```cpp
struct Som : IContactListener {
    void onContactBegin(const ContactEvent& e) override {
        if (e.approachSpeed > 2.0f) tocarSom(e.approachSpeed, e.point);   // e.a, e.b, e.normal, e.depth...
    }
    void onContactEnd(const ContactEvent& e) override { /* ... */ }
};
Som som;
world.setContactListener(&som);          // (1) ouvinte: chamado no fim de cada passo fixo

world.update(dt);
for (const ContactEvent& e : world.getContactEvents()) { /* ... */ }   // (2) buffer: eventos do último update()
```

Há contato quando o par tem ao menos um ponto de contato (formas a no máximo `kContactMargin` = 5 mm). Um
par que persiste não gera novo evento. Os eventos disparam **depois** do solver, então o ouvinte pode
criar ou destruir corpos e aplicar impulsos, mas não pode chamar `step()` nem `update()` (lança
`std::logic_error`). Destruir um corpo gera um evento de fim para cada contato dele. Os dados do evento são
cópias e continuam válidos depois.

### Sleeping

```cpp
world.sleepSettings().timeToSleep = 1.0f;     // padrão 0,5 s (limiares: 0,03 m/s e 0,05 rad/s)
world.sleepSettings().enabled = false;        // desliga tudo
desc.allowSleep = false;                      // este corpo (e a ilha dele) nunca dorme
world.isAwake(id);  world.setAwake(id, true); world.wakeAll();
```

A pose, as consultas espaciais e a interpolação de render não mudam quando um corpo dorme.

### Ajustes em execução

```cpp
world.setGravity(Vector3(0, -3.7f, 0));
world.setFixedTimeStep(1.0f / 120.0f);        // aceita 1e-4 a 1 s; devolve false se recusado
world.solverSettings();                       // iterações de velocidade e posição, slop, warm starting...
world.integratorSettings().orientation = OrientationIntegration::Exponential;
world.setCollisionFilter([](BodyId a, BodyId b) { return true; /* colidem? */ });
```

O acumulador preserva o α ao trocar o Δt. Os impulsos de *warm start* guardados vêm do Δt anterior e não
são reescalados, então a primeira iteração depois da troca pode ter um pequeno transiente de contato.

### Perfilamento

```cpp
PhysicsWorldSettings s;  s.enableProfiling = true;       // desligado por padrão: custo zero
PhysicsWorld world(s);
world.setProfilingEnabled(true);   // também liga e desliga em execução (desligar zera os tempos)
const WorldStats st = world.getStats();
// st.timings.collisionUs / islandsUs / constraintsUs / integrationUs / sleepEventsUs / totalUs  (último passo)
// st.awakeBodies, st.sleepingBodies, st.islands, st.frozenPairs, st.solverContacts, st.solverJoints
```

---

## 6. Limites suportados e entradas inválidas

| Entrada | Reação |
|---|---|
| NaN/±Inf em setters, forças e impulsos | **ignorada**: o estado do corpo não muda |
| NaN/±Inf em descrições de corpo ou de junta | `std::invalid_argument` |
| Forma fora de `[kMinShapeExtent, kMaxShapeExtent]` = `[1e-4, 1e5]` m | `std::invalid_argument` |
| Posição, âncora ou centro de massa além de `kMaxCoordinate` = `1e6` m | `std::invalid_argument` (construtores) ou ignorada (setters, teleporte) |
| Atrito | limitado a `[0, kMaxFriction]` = `[0, 1e3]`; restituição a `[0, 1]` |
| Velocidade acima do teto (`kMaxLinearSpeed` 200 m/s, `kMaxAngularSpeed` 200 rad/s) | **limitada**, preservando a direção |
| `BodyId` inválido em `getBody` e similares | `std::out_of_range` |
| Corpos além de `maxBodies` | `std::length_error` |

As constantes estão em `physics_engine/include/physics/common/Settings.h`. Construtores lançam sempre
`std::invalid_argument` para entrada inválida.

**O que não é garantido.** Valores finitos dentro dos limites ainda podem ser numericamente ruins. Em
`float`, uma troca de massa de `1e12 kg` para `1 kg` com contato persistido perde precisão no *warm start*;
o motor garante que o estado continua **finito e abaixo do teto de velocidade**, mas não que o resultado seja
fisicamente bom. Evite razões de massa muito grandes.

---

## 7. Estrutura do código

```text
physics_engine/
├── include/physics/
│   ├── common/        Settings.h, Types.h                constantes, Real, tipos básicos
│   ├── math/          Vector3, Matrix3, Quaternion, Transform, MathUtils
│   ├── geometry/      Shape, Sphere/Box/Capsule/Cylinder/PolyhedronShape, HalfEdgeStructure, AABB
│   ├── dynamics/      MassData, RigidBody, Integrator
│   ├── collision/
│   │   ├── broadphase/   DynamicAABBTree, Broadphase
│   │   ├── narrowphase/  SupportFunction, GJK, EPA, ContactManifold
│   │   └── CollisionWorld.h, RayCast.h
│   ├── constraints/   Constraint, ContactConstraint, JointConstraint, SequentialImpulseSolver
│   ├── pipeline/      PhysicsWorld, ContactEvents        ponto de entrada
│   ├── bridge/        MotionState, IPhysicsTransformBridge
│   └── debug/         DebugDraw
├── src/               .cpp espelhando as pastas de include/
├── tests/             15 suítes de teste
├── examples/main.cpp  demo sem janela
└── cmake/             modelo do pacote para find_package
```

---

## 8. Limitações conhecidas

- **Sem colisão contínua (CCD).** Corpos rápidos e pequenos podem atravessar paredes finas.
- **Formas:** esfera, caixa, cápsula, cilindro e poliedro convexo. Não há forma composta (várias formas num corpo), cone, elipsoide nem malha estática para cenário côncavo. Um objeto côncavo precisa ser aproximado por uma forma convexa.
- **Juntas só de igualdade** (bola-soquete, distância, dobradiça): sem limites angulares, motores, molas ou quebra.
- **Atrito e restituição ficam no corpo**, não na forma.
- **Torres altas balançam.** Uma torre de 10 caixas alinhadas oscila devagar com 10 iterações de velocidade e, por ter velocidade acima do limiar de repouso, não adormece. Com 40 iterações ela fica praticamente parada. É um limite do solver iterativo: aumente `solverSettings().velocityIterations` para pilhas altas.
- **Juntas com NGS dissipam energia em balanços rápidos.** A correção de posição zera o erro de comprimento de um pêndulo, mas o passo semi-implícito perde energia (um pêndulo de 2 m solto a 90° perde cerca de 40% da energia do balanço em 12 s a 60 Hz; cerca de 10% se solto a 30°). A perda é de primeira ordem no passo e cai aproximadamente pela metade ao dobrar a taxa. Para pêndulos e correntes rápidas, use um passo menor ou `positionIterations = 0` (Baumgarte, que conserva melhor a energia mas deixa um erro de comprimento de cerca de 0,7% em alta velocidade).
- **Determinismo entre plataformas** não é garantido (ver seção 4).
- **Cenas caóticas divergem com e sem *sleeping*.** Dormir congela cada corpo onde estava; numa chuva de dezenas de corpos isso basta para o resultado final mudar (ambos são fisicamente válidos). Em cenas estáveis a diferença é de décimos de milímetro.
- **Orientação:** a integração por exponencial é exata para ω constante dentro do passo; a dinâmica continua de primeira ordem (termo giroscópico por um passo de Newton, juntas e contatos).

---

## 9. Rodar o viewer (resumo)

O **Quack3 Viewer** é o viewer interativo do motor (C++17 + raylib): 14 cenas prontas, editor de cena, importação de
modelos 3D, painel do quatérnio e mais. Tudo está descrito em [`physics_viewer/README.md`](physics_viewer/README.md).
Para rodar:

```bash
cmake -S physics_viewer -B build_viewer
cmake --build build_viewer -j
./build_viewer/physics_viewer          # Windows: build_viewer\Release\physics_viewer.exe
```

Na primeira configuração a raylib 5.5 e o raygui 4.0 são baixados do GitHub (é preciso acesso à internet).
Em Linux são necessários os pacotes de desenvolvimento de X11/OpenGL; os detalhes por sistema estão no README
do viewer. As cenas começam pausadas: `Espaço` roda e pausa.

## 10. Usar o motor em outras linguagens (resumo)

A pasta `physics_capi/` compila o Quack3 numa biblioteca compartilhada (`physics_c.dll`, `libphysics_c.so` ou
`libphysics_c.dylib`) com uma **API C** estável e traz uma ligação **Python** sem dependências
(`quack_physics`). Qualquer linguagem que chame funções C (C#, Rust, Lua, Java, Go...) pode usar a mesma
biblioteca. A referência completa está em [`physics_capi/README.md`](physics_capi/README.md).

```bash
cmake -S physics_capi -B build_capi && cmake --build build_capi -j
QUACK_PHYSICS_LIB=<arquivo da biblioteca> PYTHONPATH=physics_capi/python python physics_capi/python/examples/stack_boxes.py
```

```python
import quack_physics as qp

with qp.World() as world:
    world.create_body(qp.Shape.box((10, 0.5, 10)), type="static", position=(0, -0.5, 0))
    caixa = world.create_body(qp.Shape.box((0.5, 0.5, 0.5)), position=(0, 3, 0))
    for _ in range(120):
        world.update(1 / 60)
    print(caixa.position, caixa.rotation)     # rotation = quatérnio (w, x, y, z)
```

---

## 11. Referências

O motor foi guiado por *Game Physics Engine Development* (Ian Millington), pelas notas do curso *Physically
Based Modeling* (SIGGRAPH 2001) e pela série de Allen Chou sobre física de jogos (impulsos sequenciais,
GJK/EPA, colisão e *warm starting*).
