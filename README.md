# Motor de Física 3D (`physics_engine`)

Motor de física de corpos rígidos 3D em **C++17**, escrito do zero, sem nenhuma dependência
externa (nem GLM, Eigen, Bullet ou PhysX). Orientação por **quatérnios**, detecção de colisão
**GJK/EPA**, broadphase por **árvore AABB dinâmica** e resolução por **Sequential Impulses**
(arquitetura de impulsos sequenciais de Allen Chou), com *warm starting*, atrito de Coulomb
linearizado e juntas.

Este README fica **fora** da pasta `physics_engine/` de propósito: a ligação com um motor
gráfico (raylib ou outro) deve viver numa pasta **ao lado**, e o motor continua puro.

```text
projeto/
├── README.md              (este arquivo)
├── physics_engine/        o motor: biblioteca, testes, demo headless  (não depende de gráfico)
└── physics_viewer/        a ponte com o renderizador (C++ + raylib); usa o motor, nunca o contrário
```

---

## 1. Estado do projeto

| Fase | Conteúdo | Estado |
|---|---|---|
| 1 | `common/` e `math/` (Vector3, Matrix3, Quaternion, Transform) | feita |
| 2 | `geometry/` (Sphere, Box, Polyhedron, HalfEdgeStructure, AABB) | feita |
| 3 | `dynamics/` (MassData, RigidBody, Integrator) | feita |
| 4 | `collision/broadphase/` (DynamicAABBTree, Broadphase) | feita |
| 5 | `collision/narrowphase/` (GJK, EPA, ContactManifold) | feita |
| 6 | `constraints/` (contato, juntas, solver) | feita |
| 7 | `pipeline/` (PhysicsWorld), `bridge/` (MotionState), `examples/main.cpp` | feita |
| 8 | raycast, overlap, debug draw, fuzzing, endurecimento da API, `install()` | feita |
| 9 | eventos de contato, *sleeping* com ilhas, correção de posição (NGS), perfilamento por fase | feita |
| V1 | `physics_viewer/`: janela, câmera orbital, quatérnio→matriz, caixa em queda (ver seção 9) | feita (compilou e rodou no Windows do usuário) |
| V2 | 7 cenas selecionáveis, esferas e poliedros, juntas desenhadas, reinício (ver seção 9) | feita (janela não testada por mim; núcleo testado) |
| V3 | painel do quatérnio, modos slerp / lerp sem normalizar / Euler, gimbal lock, slider de FPS, raygui (ver seção 9) | feita (janela não testada por mim; núcleo testado) |
| V4 | selecionar por clique (raycast), arrastar com mola, lançar e apagar corpos, câmera com alvo mundo/selecionado e reset (ver seção 9) | feita (janela não testada por mim; núcleo testado) |
| V5 | barra de controle: pausa/play, passo a passo, câmera lenta, reiniciar, cena < >, resetar câmera, alvo da câmera (ver seção 9) | feita (janela não testada por mim; núcleo testado) |
| V6 | painel do objeto selecionado: massa, atrito, restituição, amortecimentos, gravidade, posição, velocidades, orientação por eixo-ângulo, acordar/dormir (ver seção 9) | feita (janela não testada por mim; núcleo testado) |
| V7 | painel do mundo (gravidade, Δt, solver, sleeping), debug draw do motor no viewer, estatísticas e tempos (ver seção 9); o motor ganhou `setFixedTimeStep` e `setProfilingEnabled` | feita (janela não testada por mim; núcleo e motor testados) |
| V8 | painel **Adicionar objeto** (tipo, tamanho, massa, posição, orientação por eixo-ângulo ou quatérnio, velocidades; criar no ponto clicado ou lançar da câmera), abas Objeto/Adicionar | feita (janela não testada por mim; núcleo testado: 503 verificações) |
| V9 | eventos e sleeping: cor por estado (dormindo = azul), flash e anéis no início do contato (∝ velocidade de impacto), aba Evento com contadores | feita (janela não testada por mim; núcleo testado: 523 verificações) |
| V10 | fechamento: janela com tamanho mínimo e F11, `viewer_smoke` no ctest (todas as cenas 20 s + fuzz), READMEs finais | feita (janela não testada por mim; ctest do viewer 2/2 em float, double e ASan) |
| V11 | cenas 10–14: impacto na pirâmide, porta giratória (eixo Y), gangorra (eixo Z), pêndulo-martelo (eixo Z), bilhar; HUD com a velocidade angular w | feita (janela não testada por mim; 962 verificações do viewer em float, double e ASan) |
| V12 | salvar/carregar cenas em `.scene` (texto, quatérnio w x y z): aba Cenas, 14 exemplos, `scene_export`, arquivo inválido não altera a cena | feita (janela não testada por mim; 1336 + 467 verificações em float, double e ASan) |
| V13 | painéis em colunas fixas com rolagem e largura ajustável pela borda; cena 3D numa textura do tamanho da área central; layout salvo (`DockLayout`) | feita (janela não testada por mim; 1415 + 467 verificações em float, double e ASan) |
| V14 | editor de cena: cena nova vazia/só chão, objetos estáticos com arestas x/y/z, juntas (dobradiça, bola-soquete, distância) entre corpos ou corpo e mundo, salvar pelo `.scene` (`JointEditor`) | feita (janela não testada por mim; 1483 + 644 verificações em float, double e ASan) |
| V15a | núcleo da importação 3D: fecho convexo próprio (quickhull + fusão de faces coplanares), leitor `.obj`/`.mtl`, forma de colisão do modelo (`ConvexHull`, `ObjModel`, `ModelCollider`), exemplos em `models/` | feita (sem janela ainda; 1610 + 3834 verificações em float, double e ASan) |
| V15b | importar modelo 3D na janela: aba Adicionar > modelo 3D (lista de `models/`, tamanho, textura, aviso de côncavo, fio do fecho), malha com textura desenhada, `.scene` com linha `model` | feita (janela não testada por mim; 1669 + 3825 verificações em float, double e ASan) |
| V16 | gravar a trajetória e os quatérnios em CSV (painel na coluna esquerda, F7) e modo apresentação (F5: tela limpa com quatérnio grande; F6: câmera giratória) | feita (janela não testada por mim; 1692 + 3833 verificações em float, double e ASan) |
| V17 | corrige as sobreposições dos painéis (alturas calculadas e testadas, texto que se ajusta), importar o próprio `.obj` (arrastar e soltar na janela ou colar o caminho) e escolher o colisor do modelo (fecho, caixa ou esfera) | feita (janela não testada por mim; 1778 + 4176 verificações em float, double e ASan) |
| V18 | **cápsula e cilindro** no motor (`CapsuleShape`, `CylinderShape`: suporte, inércia, AABB, contatos de 1 e 2 pontos para deitados, raycast, debug draw) e no viewer (criar, usar como colisor de modelos com ajuste automático e fino, `.scene` com `shape capsule`/`shape cylinder`) | feita (janela não testada por mim; motor: 14 suítes, 1010 verificações novas; viewer: 1890 + 4294 verificações em float, double e ASan) |
| V19 | **correção do atrito**: atrito de torção (giro em torno da normal) e resistência à rolagem em esfera, cápsula e cilindro (antes uma bola girando em Y ou rolando nunca parava); **WASD** move a câmera pelo cenário (E/Q sobem e descem, Shift acelera) | feita (janela não testada por mim; motor: 15 suítes, 17 verificações novas; viewer: 1903 + 4294 em float, double e ASan) |
| V20 | **ajustes de painel pedidos com prints**: aba Debug sem sobreposição (rótulos cortados dentro da coluna), caminho do CSV inteiro em várias linhas + botão *Copiar caminho*, sliders que usam a largura da coluna, e **clique duplo em qualquer slider** abre um campo de texto (Enter ou clique fora aplica, Esc cancela sem fechar o programa) | feita (janela não testada por mim; viewer: 1935 + 4294 verificações em float, double e ASan) |
| V21 | **corrige a V20**: o rótulo das linhas de slider invadia a barra (texto por baixo dela) | feita (janela não testada por mim) |
| V22 | **painel Cenas próprio** (no topo da coluna direita, fora do painel de objetos) e **lista única de cenas**: as originais e os `.scene` salvos aparecem juntos e as setas seguem essa ordem; dá para **reordenar, apagar e restaurar**, e isso fica salvo (`scenes/biblioteca.txt`) | feita (janela não testada por mim; viewer: 2060 + 4294 verificações em float, double e ASan) |
| V23 | o programa agora se chama **Quack3** (versão `v0.<fase>`, título "Quack3 v0.23"); **tema cinza-grafite** no lugar do branco; botões/abas cujo texto sempre cabe (fonte reduz ao estreitar a coluna); **Interpolação volta ao topo** da coluna direita e os títulos das colunas listam os painéis na ordem certa | feita (janela não testada por mim; viewer: 2064 + 4294 verificações em float, double e ASan) |
| V23.1 | correção de cores: botões, sliders e cabeçalhos trocados com o fundo dos painéis; título "Quack3 v0.23.1" | feita (janela não testada por mim) |
| V24 | **integração da orientação por exponencial de quatérnio** (`q ← exp(½Δt ω) q`, exata para ω constante; Euler continua como opção em `IntegratorSettings::orientation` e numa caixa no painel do mundo), **esfera de arcball** (arrastar gira o corpo selecionado), slider que **mantém o valor antigo editável** e aceita **contas** (`4/2`, `1-4`, `2*(3+1)`) | feita (janela não testada por mim; motor 15 suítes float/double/ASan; viewer 2127 + 4294) |

**Ainda não existe** (ver seção 8): colisão contínua (CCD), forma composta,
limites e motores de junta, malha estática, API C para outras linguagens.

---

## 2. Compilar e testar

Requisitos: CMake ≥ 3.14 e um compilador C++17 (testado com GCC).

```bash
cmake -S physics_engine -B build
cmake --build build -j
ctest --test-dir build            # 13 testes: 12 suítes + o demo (que se auto-verifica)
```

Opções do CMake:

| Opção | Padrão (projeto raiz) | O que faz |
|---|---|---|
| `PHYSICS_USE_DOUBLE` | `OFF` | usa `double` em vez de `float` em todo o motor |
| `PHYSICS_BUILD_TESTS` | `ON` | compila as suítes de teste |
| `PHYSICS_BUILD_EXAMPLES` | `ON` | compila `physics_demo` |
| `PHYSICS_INSTALL` | `ON` | gera as regras de `install` / `find_package` |

Como **subprojeto** (via `add_subdirectory`) os três últimos já nascem `OFF`: quem usa o motor
só compila a biblioteca.

> **`float` e `double` não se misturam.** A precisão faz parte da ABI. Todos os testes passam nas
> duas, mas o programa e a biblioteca precisam ser compilados com a mesma.

Verificação feita na Fase 9: as 13 em `float` e em `double`, sem warnings com
`-Wall -Wextra -Wpedantic`, e também com **AddressSanitizer + UBSan** (que aborta em qualquer
comportamento indefinido). O motor também foi usado a partir de um projeto vizinho, por
`add_subdirectory` e por pacote instalado.

---

## 3. Usar o motor a partir de outra pasta

### Opção A — `add_subdirectory` (recomendada durante o desenvolvimento)

`physics_viewer/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.14)
project(physics_viewer LANGUAGES CXX)

add_subdirectory(../physics_engine physics_build)

add_executable(viewer src/main.cpp)
target_link_libraries(viewer PRIVATE physics::physics)   # inclui os headers e a precisão (float/double)
```

### Opção B — instalar e usar `find_package`

```bash
cmake -S physics_engine -B build && cmake --build build
cmake --install build --prefix ~/physics_install
```

```cmake
find_package(physics 0.9 REQUIRED)
target_link_libraries(viewer PRIVATE physics::physics)
# cmake -DCMAKE_PREFIX_PATH=~/physics_install ...
```

As duas opções foram testadas a partir de um projeto separado, em `float` e em `double`.

---

## 4. Conceitos e convenções

- **Unidades:** metros, quilogramas, segundos, radianos. **Y é para cima** (gravidade `(0,-9.81,0)`).
- **Mão direita.** Quatérnios em ordem `(x, y, z, w)`; o buffer da bridge usa `px py pz qx qy qz qw`.
- **`Real`** é `float` (padrão) ou `double`.
- **Passo fixo:** a física roda a Δt = 1/60 s. O `update(frameTime)` acumula o tempo de render,
  executa 0..N passos fixos (N limitado a `maxSubSteps = 8`, contra a "espiral da morte") e
  atualiza α = acumulador/Δt.
- **Interpolação:** cada corpo guarda a pose anterior e a atual; o render desenha
  `lerp` (posição) + `slerp` (orientação) com α. O FPS pode variar à vontade sem a simulação
  mudar (há teste de independência de taxa de quadros).
- **Determinismo:** dois mundos construídos com as mesmas chamadas dão resultado idêntico
  bit a bit (mesma máquina e mesmo compilador). Entre máquinas/compiladores/flags
  (`-ffast-math`, FMA) **não** é garantido.
- **Memória:** corpos em um `std::vector` de capacidade fixa (`maxBodies`, padrão 4096), então os
  ponteiros usados pelo solver nunca são invalidados. Criar corpos além disso lança
  `std::length_error`. `BodyId` destruído é reaproveitado.
- **Sem alocação no laço:** com o mundo em regime, `step()` não aloca (medido: 0 alocações em 60 passos,
  com e sem *sleeping*, com e sem ouvinte). Alocam a criação/destruição de corpos e juntas e o
  nascimento de pares de colisão novos (medido: 2 alocações quando um corpo novo cai e assenta).
- **Não é thread-safe.**

---

## 5. Exemplo mínimo

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

    while (/* janela aberta */ false) {
        world.update(/* tempo do quadro em segundos */ 0.016);
        Transform pose = world.getInterpolatedTransform(id);   // pose de render
        // desenha o modelo em pose.position / pose.rotation
    }
}
```

### Ponte com o renderizador (`bridge/`)

```cpp
BufferedTransformBridge bridge;     // 7 floats por corpo, contíguos
world.setBridge(&bridge);
// a cada quadro, depois de world.update(dt):
const float* poses = bridge.data(); // slot = BodyId; use bridge.isAlive(id) para saber se existe
```

Para outro tipo de renderizador, implemente `IPhysicsTransformBridge` (callbacks de criação,
destruição e atualização de pose).

### Consultas espaciais

```cpp
RaycastHit hit;
if (world.raycast(origem, direcao, /*alcance*/ 100, hit)) {
    // hit.body, hit.point, hit.normal, hit.distance, hit.startedInside
}
std::vector<RaycastHit> todos;  world.raycastAll(origem, direcao, 100, todos);   // ordenados por distância
std::vector<BodyId> ids;        world.overlapSphere(centro, raio, ids);          // teste exato (GJK)
                                world.overlapPoint(ponto, ids);
```

Todas aceitam um `QueryFilter` (incluir/excluir estáticos, cinemáticos, dinâmicos; ignorar um
corpo; predicado próprio). São `const`: não alteram o mundo nem o determinismo. Entradas
inválidas (NaN, alcance negativo...) resultam em "nada encontrado", nunca em exceção.

### Debug draw (`debug/`)

O motor gera as linhas; o renderizador só precisa desenhar uma linha 3D:

```cpp
struct MeuRenderer : IDebugRenderer {
    void drawLine(const Vector3& a, const Vector3& b, const DebugColor& c) override { /* DrawLine3D(...) */ }
};
MeuRenderer r;
debugDraw(world, r);                                   // formas, contatos, normais, juntas
DebugDrawSettings s; s.flags = kDebugAll;              // + AABBs, fat AABBs, velocidades, eixos...
debugDraw(world, r, s);
```

`DebugLineBuffer` acumula as linhas num vetor contíguo (layout simples, útil se um dia o
renderizador estiver em outra linguagem).

### Eventos de contato

Início e fim do toque entre dois corpos, com a velocidade de impacto e o impulso aplicado (o que
se usa para som, partículas, dano). Dois jeitos de consumir, combináveis:

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

Contato = o par tem ao menos um ponto de contato (formas a no máximo `kContactMargin` = 5 mm, então
contatos especulativos contam). Um par que persiste não gera evento. Os eventos disparam **depois**
do solver, então o ouvinte pode criar/destruir corpos e aplicar impulsos; só não pode chamar
`step()`/`update()` (lança `std::logic_error`). Destruir um corpo gera um evento de fim para cada
contato dele. Os dados do evento são cópias e continuam válidos depois.

Medido: bola de 2 kg solta de 2 m, velocidade de aproximação 6,21 m/s (ideal 6,26) e impulso 12,75
N·s (ideal 12,53); a contabilidade `begins − ends` bate com os contatos vivos em todos os passos.

### Sleeping

Um corpo dinâmico dorme quando `|v|` e `|ω|` ficam abaixo dos limiares por `timeToSleep` seguidos, e
**só junto com toda a sua ilha** (corpos ligados por contato ou junta; estáticos e cinemáticos não
ligam ilhas). Dormindo, o corpo não é integrado nem entra no solver, e o par dele com outro corpo
parado não roda a narrowphase.

```cpp
world.sleepSettings().timeToSleep = 1.0f;     // padrão 0,5 s (limiares: 0,03 m/s e 0,05 rad/s)
world.sleepSettings().enabled = false;        // desliga tudo
desc.allowSleep = false;                      // este corpo (e a ilha dele) nunca dorme
world.isAwake(id);  world.setAwake(id, true); world.wakeAll();
```

Acordam: força, impulso, velocidade não nula, `setBodyTransform`, mudança de massa, de `gravityScale`
ou da gravidade, remoção de um corpo que o apoiava, criação/remoção de junta, e contato com um corpo
acordado ou com um cinemático em movimento. Pose, consultas espaciais e a interpolação de render não
mudam quando um corpo dorme (a pose de render de um corpo dormindo é estável para qualquer α).

Medido (`float`): 196 caixas em repouso custam **608 µs/passo acordadas e 9 µs/passo dormindo**.
Verificado em 12 mundos aleatórios × 600 passos (~77 mil pares): nunca há um par de corpos dinâmicos em
contato com um dormindo e o outro acordado.

### Mudar o passo fixo em execução

```cpp
world.setFixedTimeStep(1.0f / 120.0f);   // aceita 1e-4 a 1 s; devolve false se recusado
```

O acumulador preserva o α (a fração do passo já acumulada), então a imagem não "pula". Os impulsos de
*warm start* guardados vêm do Δt antigo e **não são reescalados**: a primeira iteração após a troca pode
ter um pequeno transiente de contato.

### Perfilamento

```cpp
PhysicsWorldSettings s;  s.enableProfiling = true;       // desligado por padrão: custo zero
PhysicsWorld world(s);
world.setProfilingEnabled(true);   // também dá para ligar/desligar em execução (desligar zera os tempos)
const WorldStats st = world.getStats();
// st.timings.collisionUs / islandsUs / constraintsUs / integrationUs / sleepEventsUs / totalUs  (último passo)
// st.awakeBodies, st.sleepingBodies, st.islands, st.frozenPairs, st.solverContacts, st.solverJoints
```

---

## 6. Limites suportados e como o motor reage a entradas inválidas

O motor foi submetido a *fuzzing* determinístico (sequências aleatórias de chamadas com NaN, ±Inf,
`1e30`, `FLT_MAX`, ids inválidos, formas degeneradas, juntas absurdas, passos de tempo malucos). O
contrato resultante:

| Entrada | Reação |
|---|---|
| NaN/±Inf em setters, forças e impulsos | **ignorada**: o estado do corpo não muda |
| NaN/±Inf em descrições de corpo ou de junta | `std::invalid_argument` |
| Forma fora de `[kMinShapeExtent, kMaxShapeExtent]` = `[1e-4, 1e5]` m | `std::invalid_argument` |
| Posição/âncora/centro de massa além de `kMaxCoordinate` = `1e6` m | `std::invalid_argument` (construtores) ou ignorada (setters, teleporte) |
| Atrito | limitado a `[0, kMaxFriction]` = `[0, 1e3]`; restituição a `[0, 1]` |
| Velocidade acima do teto (`kMaxLinearSpeed` 200 m/s, `kMaxAngularSpeed` 100 rad/s) | **limitada**, preservando a direção (antes, em `float`, `1e30` virava 0) |
| `BodyId` inválido em `getBody` e similares | `std::out_of_range` |
| Corpos além de `maxBodies` | `std::length_error` |

As constantes estão em `include/physics/common/Settings.h`. Construtores lançam **sempre**
`std::invalid_argument` para entrada inválida (não `logic_error`).

**O que NÃO é garantido.** Valores finitos dentro dos limites podem ainda ser numericamente
ruins. Em `float`, uma troca de massa de `1e12 kg` para `1 kg` com contato persistido perde
precisão no *warm start*. O motor garante que o estado continua **finito e abaixo do teto de
velocidade** (há teste), mas não que o resultado seja fisicamente bom. Em `double` isso não ocorre
até onde medi. Evite razões de massa muito grandes.

---

## 7. Estrutura do código

```text
physics_engine/
├── include/physics/
│   ├── common/        Settings.h, Types.h             constantes, Real, tipos básicos
│   ├── math/          Vector3, Matrix3, Quaternion, Transform, MathUtils
│   ├── geometry/      Shape, Sphere/Box/PolyhedronShape, HalfEdgeStructure, AABB
│   ├── dynamics/      MassData, RigidBody, Integrator
│   ├── collision/
│   │   ├── broadphase/   DynamicAABBTree, Broadphase
│   │   ├── narrowphase/  SupportFunction, GJK, EPA, ContactManifold
│   │   ├── CollisionWorld.h, RayCast.h
│   ├── constraints/   Constraint, ContactConstraint, JointConstraint, SequentialImpulseSolver
│   ├── pipeline/      PhysicsWorld, ContactEvents     ponto de entrada
│   ├── bridge/        MotionState, IPhysicsTransformBridge
│   └── debug/         DebugDraw
├── src/               .cpp espelhando as pastas de include/
├── tests/             12 suítes de teste
├── examples/main.cpp  demo headless: empilhamento de caixas e interpolação de render
└── cmake/             modelo do pacote para find_package
```

Pipeline de cada passo fixo: atualizar AABBs → broadphase → narrowphase (manifolds persistentes; pares
de corpos dormindo são pulados) → ilhas (acordar) → forças e gravidade → solver de velocidade (contatos +
juntas) → integrar posições → correção de posição (NGS) → registrar pose para o render → *sleeping* →
eventos de contato.

**Testes** (`ctest`): matemática, geometria, dinâmica, broadphase, narrowphase, restrições,
pipeline (empilhamento, pirâmide, quique, plataforma cinemática, juntas, determinismo, independência
de taxa de quadros), raycast (comparado com marcha de raio por força bruta), fuzzing,
`robustness_tests` (regressões determinísticas de defeitos achados pelo fuzz; ex.: em `float`, uma
velocidade de `1e30` virava ~0 na Fase 7 em vez de ser limitada), `sleeping_tests` (inclui a invariante
de ilhas e a equivalência com/sem *sleeping*) e `events_tests`.

---

## 8. Limitações conhecidas e próximos passos

Limitações atuais:

- **Orientação (V24):** por padrão avança por exponencial de quatérnio (exata para ω constante dentro do passo); a dinâmica continua de primeira ordem (giroscópico por um passo de Newton, juntas, contatos).
- **Sem CCD.** Corpos rápidos e pequenos podem atravessar paredes finas.
- **Torres altas balançam.** Uma torre de 10 caixas alinhadas oscila devagar com 10 iterações do solver
  (`float`, 20 s: topo a até ~12 cm do eixo, velocidade tardia ~0,12 m/s; a correção de posição não
  muda isso: medido, o resultado é idêntico com `positionIterations` 0 ou 3). Como a velocidade fica acima do limiar, a
  torre também não adormece. Com 40 iterações ela fica praticamente parada (~0,003 m/s). É limite do
  solver iterativo, não instabilidade: aumente `solverSettings().velocityIterations` para torres altas.
- **Juntas com NGS dissipam energia em balanços rápidos.** A correção de posição (padrão:
  `positionIterations = 3`) zera o erro de comprimento de um pêndulo (de 1,3 cm para ~0), mas o passo
  semi-implícito perde energia: um pêndulo de 2 m solto a 90° perde cerca de 40% da energia do balanço
  em 12 s a 60 Hz (cerca de 10% se for solto a 30°). A perda é de primeira ordem no passo (cai
  aproximadamente pela metade ao dobrar a taxa). Com `positionIterations = 0` volta o Baumgarte antigo, que conserva melhor a
  energia nesse caso, mas deixa um erro de comprimento de ~0,7% em alta velocidade. Escolha conforme
  a cena: pêndulos e correntes rápidas pedem passo menor ou `positionIterations = 0`.
- **Juntas só de igualdade** (bola-soquete, distância, dobradiça): sem limites angulares,
  motores, molas ou quebra.
- **Esfera, caixa, cápsula, cilindro e poliedro convexo** (cápsula e cilindro desde a V18, eixo Y
  local). Sem forma composta (várias formas num corpo), cone, elipsoide ou malha estática (cenário côncavo).
- **Atrito e restituição ficam no corpo**, não na forma. Desde a V19 há atrito de torção e resistência à rolagem para formas redondas (`SolverSettings::spinFriction` e `rollingFriction`, em metros, vezes μ·Σλn; 0 desliga): antes o giro em Y de uma esfera nunca parava, pois o cone de Coulomb só atua nos pontos de contato e a esfera toca em um só.
- **Sem API C** (`extern "C"`) nem biblioteca compartilhada: o motor se usa por C++.
- **Determinismo entre plataformas** não garantido (ver seção 4).
- **Cenas caóticas divergem com e sem *sleeping*.** Dormir congela cada corpo onde estava; numa chuva de
  dezenas de corpos isso basta para um ou outro terminar em outro lugar (os dois resultados são
  fisicamente válidos). Em cenas estáveis a diferença é de décimos de milímetro.

Sugestão de ordem: CCD → mais formas, juntas completas (limites, motores) e materiais por forma →
API C, se a camada gráfica for em outra linguagem. A camada gráfica (`physics_viewer/`) já começou (seção 9): o motor já tem poses interpoladas, raycast, overlap, debug draw, eventos de contato e *sleeping*.

---

## 9. O viewer (`physics_viewer/`)

C++ + [raylib](https://www.raylib.com/) + raygui, numa pasta ao lado do motor (que não depende dele).
Decisão: C++ direto, em vez de Python, porque não exige uma API C nem *bindings* (menos código e menos
camadas a manter), e porque o quatérnio é o tema do trabalho: o viewer tem controle total do desenho e
do painel para mostrá-lo. A raylib recebe a matriz que o **motor** calcula a partir do quatérnio;
o quatérnio continua sendo o estado da física.

Roteiro, em etapas pequenas (cada uma deixa o viewer funcionando):

| Etapa | Entrega | Tamanho |
|---|---|---|
| **V1. Esqueleto** (feita) | `CMakeLists.txt`, janela, câmera orbital, chão, caixa em queda, quatérnio→matriz. Compilou e rodou no Windows. | média |
| **V2. Cenas** (feita) | Registro de 7 cenas (caixa, pilha, pirâmide, chuva, pêndulo duplo, corrente, poliedros), esferas e poliedros desenhados, ←/→ e 1–7 trocam de cena, `R` reinicia. Núcleo testado (123 verificações, float/double/ASan+UBSan); `main.cpp` só verificado por tipos. | média |
| **V3. Quatérnios na tela** (feita) | Painel `x y z w`, norma, eixo-ângulo e Euler do corpo em foco da cena (a seleção por clique chega na V4). Chave para escolher a interpolação de render: **slerp**, **lerp sem normalizar** ou **Euler**. Cena de gimbal lock. FPS de render ajustável por slider. Duas cenas novas de passo largo (gimbal lock e interpolação) para o efeito ser visível; raygui entra no CMake. Núcleo testado (175 verificações, float/double/ASan+UBSan); `main.cpp` só verificado por tipos. | média |
| **V4. Interação** (feita) | Clique seleciona (raycast do motor), arrastar com mola, lançar caixa ou esfera, apagar corpo. **Câmera:** girar ao redor do alvo com a **roda do mouse pressionada** (botão do meio) + mover o mouse (o botão direito continua girando); o alvo é o **centro do mundo** ou o **objeto selecionado** (alternável por tecla `T`; sem seleção, volta ao centro da cena); `Home` já reseta a câmera. Núcleo testado (229 verificações, float/double/ASan+UBSan); `main.cpp` só verificado por tipos. | média |
| **V5. Barra de controle** (feita) | Botão **Pausa/Play** (substitui a tecla `P`, com o estado visível), **passo a passo** (1 passo fixo), reiniciar, câmera lenta (escala de tempo) e seletor de cena. Botão **Resetar câmera** (volta ao enquadramento inicial da cena, com o alvo no centro do mundo) e botão do alvo da câmera (mundo / objeto selecionado). Núcleo `TimeControl` testado (318 verificações, float/double/ASan+UBSan); `main.cpp` só verificado por tipos. | pequena |
| **V6. Painel do objeto** (feita) | Parâmetros ajustáveis do objeto selecionado: massa, atrito, restituição, amortecimento linear e angular, escala de gravidade, posição, orientação (quatérnio e eixo-ângulo), velocidades linear e angular, acordar/dormir. Núcleo `BodyEditor` testado (370 verificações, float/double/ASan+UBSan); `main.cpp` só verificado por tipos. | média |
| **V7. Painel do mundo e debug** (feita) | Debug draw do motor ligado ao `DrawLine3D`, com caixas de seleção por flag. Parâmetros ajustáveis do mundo: gravidade, Δt fixo, iterações de velocidade, `positionIterations`, sleeping e slop. Painel de estatísticas e tempos. Núcleo `WorldEditor` testado (420 verificações, float/double/ASan+UBSan; ctest do motor 13/13); `main.cpp` só verificado por tipos. | média |
| **V8. Adicionar objetos** (feita) | Painel para adicionar objetos na cena atual: tipo (caixa, esfera, tetraedro, octaedro, prisma, pirâmide), tamanho, massa, posição, orientação inicial (eixo-ângulo ou quatérnio) e velocidade. Entra no ponto clicado ou é lançado da câmera. | média |
| **V9. Eventos e sleeping** (feita) | Cor por estado (dormindo/acordado), flash no início de contato proporcional à velocidade de impacto, contador de contatos. | pequena |
| **V10. Fechamento** (feita) | Janela redimensionável, README do viewer, testes de fumaça sem janela no `ctest`, empacotamento. | pequena |
| **V11. Cenas de colisão e rotação** (feita) | Cena nova "Impacto na pirâmide" (projétil com velocidade inicial contra a 3ª cena; a Pirâmide original continua) e cenas com **eixo de rotação fixo** (`HingeJoint` ligado ao chão: porta/pá/catraca atingida na lateral gira só em torno do eixo): porta, cata-vento, pêndulo-martelo e bilhar de esferas. HUD mostra o quatérnio e a velocidade angular do objeto articulado. | pequena |
| **V12. Salvar e carregar cenas** (feita) | Formato de arquivo de texto (`.scene`, legível, com versão): corpos (forma, tamanho, massa, material, pose como quatérnio, velocidades), juntas, gravidade e Δt. Botões **Salvar cena** / **Carregar cena** (campo de nome; pasta `scenes/`) e cenas de exemplo no formato. Testes de ida e volta sem janela. | média |
| **V13. Painéis acoplados e rolagem** (feita) | Os painéis deixam de flutuar sobre a cena: ficam em **duas colunas fixas nas laterais** (esquerda: quatérnio e mundo; direita: interpolação, objeto, adicionar e cenas), cada uma com **barra de rolagem** (roda do mouse e barra), então campos novos nunca mais se sobrepõem. A largura de cada coluna muda **arrastando a borda** (cursor de redimensionar), e a **cena 3D ocupa o espaço que sobra** (renderizada numa textura do tamanho exato, com a proporção e o mouse corretos), como no Blender; botão para recolher cada coluna. Layout (larguras, recolhido) num núcleo testável sem janela (`DockLayout`). Divisão: **V13a** = núcleo do layout + testes, cena na área restante, bordas arrastáveis, colunas recolhíveis; **V13b** = rolagem com recorte e bloqueio de cliques fora da área visível, painéis migrados, layout salvo em arquivo. | grande (2 partes) |
| **V14. Editor de cena** (feita) | Criar a própria cena do zero: cena vazia (só chão ou nem isso), objetos **estáticos** (paredes, rampas) e dinâmicos, **juntas** (dobradiça com eixo, bola-soquete, distância) entre dois corpos escolhidos ou entre um corpo e o mundo, ajustar tudo com os painéis existentes e salvar pela V12. | média |
| **V15. Importar objeto 3D com textura** (feita: V15a + V15b) | Carregar `.obj` (+ `.mtl`) com textura (`.png`/`.jpg`) numa cena: o modelo é desenhado com a textura da raylib e a colisão usa o **fecho convexo** dos vértices (algoritmo próprio, sem biblioteca; o motor só aceita formas convexas), com opção de caixa/esfera envolvente. Escala, massa e orientação inicial por quatérnio. Entra no arquivo de cena (V12). | grande (dividida em V15a fecho convexo + testes e V15b modelo/textura na janela) |
| **V16. Opcional, para o relatório** (feita) | Exportar a trajetória e os quatérnios em CSV para gráficos, e um modo apresentação. | pequena |
| **V17. Painéis sem sobreposição, seu próprio modelo e colisor** (feita) | Alturas dos painéis calculadas do conteúdo (e testadas) e texto que sempre cabe na coluna; **importar o seu `.obj`** (arrastar para a janela ou colar o caminho; copia `.mtl` e texturas para `models/`); escolher a **forma física** do modelo: fecho, caixa ou esfera. | média |
| **V18. Cápsula e cilindro** (feita) | **No motor**: `CapsuleShape` e `CylinderShape` (função de suporte, inércia, AABB, contatos de face/segmento, raycast, debug draw), e no viewer: caixa de seleção para criá-las e como colisor de modelos importados (ex.: cápsula para o corpo de uma pessoa), com altura e raio ajustáveis e ajuste automático ao modelo; `.scene` com `shape capsule`/`shape cylinder`. | grande (motor + viewer) |

Observações do planejamento:

- `P` já pausa desde a V1; o botão chega na V5.
- A câmera já gira com o botão direito desde a V1; a V4 acrescenta a roda pressionada e o alvo (mundo ou objeto selecionado), que depende da seleção; o botão de resetar fica na V5, com os demais botões (a tecla `Home` pode antecipá-lo na V4).
- O motor já tem o necessário para os painéis (`setFriction`, `setRestitution`, `setLinearDamping`,
  `setMassData`, `setGravityScale`, `setPose`, `setLinearVelocity`, `setGravity`, `createBody`,
  `destroyBody`); ajustes que faltarem (por exemplo, mudar iterações do solver em execução) entram
  na etapa que os usa, sem mudar o comportamento atual.
- Corpos adicionados ou apagados ficam registrados em `Scene::bodies`; reiniciar volta ao estado
  original da cena.
- Os painéis (objeto, mundo, adicionar) ficam recolhíveis à direita, para não cobrir a cena.

Para compilar e rodar, veja `physics_viewer/README.md`.

---

## 10. Referências

Implementação guiada por *Game Physics Engine Development* (Ian Millington) e pelas notas do
curso *Physically Based Modeling* (SIGGRAPH 2001), além da série de Allen Chou sobre física de
jogos (impulsos sequenciais, GJK/EPA, colisão e *warm starting*).
