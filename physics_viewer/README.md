# physics_viewer

Janela de demonstração do `../physics_engine` (C++17 + [raylib](https://www.raylib.com/)). Fica numa pasta
**ao lado** do motor, que continua sem nenhuma dependência gráfica.

> Estado: **V6** (painel do objeto selecionado, além da barra de controle, da interação com o mouse e do quatérnio na tela). O roteiro completo
> está no `README.md` da pasta acima.

## Compilar e rodar

Requisitos: CMake ≥ 3.14, um compilador C++17 e acesso ao GitHub na primeira configuração (a raylib 5.5 é
baixada e compilada automaticamente, a menos que já esteja instalada no sistema; o raygui 4.0, um único
cabeçalho para a interface, é sempre baixado).

| Sistema | Pacotes necessários para a raylib |
|---|---|
| Linux (Debian/Ubuntu) | `sudo apt install build-essential cmake git libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev` |
| Windows | CMake + Visual Studio Build Tools (MSVC) **ou** MinGW-w64. Nada mais. |
| macOS | `xcode-select --install` e CMake |

```bash
cmake -S physics_viewer -B build_viewer
cmake --build build_viewer -j
./build_viewer/physics_viewer          # Windows: build_viewer\Release\physics_viewer.exe
```

Só o núcleo e os testes, sem raylib nem janela:

```bash
cmake -S physics_viewer -B build_core -DVIEWER_BUILD_APP=OFF
cmake --build build_core -j && ctest --test-dir build_core
```

Para usar `double` no motor: `-DPHYSICS_USE_DOUBLE=ON` (o viewer converte para `float` só ao desenhar).

## Controles

| Entrada | Ação |
|---|---|
| botão esquerdo | selecionar o corpo sob o mouse e **arrastá-lo com uma mola** (clicar no vazio ou no chão desmarca) |
| roda, durante o arrasto | afastar / aproximar o ponto-alvo da mola (profundidade) |
| botão direito **ou roda pressionada** + mover | girar a câmera ao redor do alvo |
| roda do mouse | zoom (desligada sobre os painéis) |
| `T` ou botão **Alvo** | alvo da câmera: centro do mundo ⇄ corpo selecionado (transição suave) |
| `Home` ou botão **Reset cam** | resetar a câmera (ângulos padrão, alvo no centro do mundo, distância inicial da cena) |
| `B` / `O` | lançar uma caixa / esfera na direção do mouse (18 m/s) |
| `Delete` ou `X` | apagar o corpo selecionado (juntas ligadas a ele também somem) |
| painel à direita, em cima | escolher o modo de interpolação e o FPS de render (slider de 10 a 240) |
| painel à direita, embaixo | editar o corpo selecionado (abas Material, Posicao, Veloc, Orient) |
| `←` / `→` | cena anterior / próxima |
| `1` … `9` | ir direto para a cena |
| `R` | reiniciar a cena atual (mantém a câmera) |
| `I` | próximo modo de interpolação (slerp → lerp → Euler) |
| `Espaço`, `P` ou botão **Pausar/Continuar** | pausar / continuar (selecionar continua funcionando; arrastar só atua com a física rodando) |
| `.` ou botão **Passo** | avança **um** passo físico fixo (se estava rodando, pausa primeiro) |
| botões `1x` `1/2` `1/4` `1/10` | escala de tempo (câmera lenta) |
| botões `<` `>` e **Reiniciar** | cena anterior / próxima; recriar a cena atual (como `R`) |
| `ESC` | sair |

Verificação automática, sem interação (roda 240 quadros, salva uma captura e sai com código 0):

```bash
./build_viewer/physics_viewer --frames 240 --screenshot saida.png
```

## Cenas

| # | Cena | O que mostra |
|---|---|---|
| 1 | Caixa em queda | uma caixa já girando; o HUD mostra seu quatérnio |
| 2 | Pilha de caixas | 6 caixas empilhadas (HUD: caixa do topo) |
| 3 | Pirâmide de caixas | 5 fileiras, 15 caixas |
| 4 | Chuva de corpos | 36 caixas e esferas em orientações aleatórias, com paredes |
| 5 | Pêndulo duplo | duas esferas ligadas por juntas de distância (linhas brancas) |
| 6 | Corrente (juntas) | 8 elos com juntas bola-soquete |
| 7 | Formas (poliedros) | tetraedro, octaedro, prisma triangular, pirâmide, caixas e esferas |
| 8 | Gimbal lock (passo 0,5 s) | gravidade zero; uma caixa gira em torno de X e atravessa pitch = 90° |
| 9 | Interpolacao (passo 0,35 s) | gravidade zero; um cubo gira ~110° entre dois passos físicos |

Os poliedros são desenhados a partir da `HalfEdgeStructure` do motor (faces em leque de triângulos, com
tom pela normal no mundo, mais as arestas). As cenas ficam em `src/core/Scene.cpp`; para criar outra,
escreva uma função `makeXScene()` e acrescente-a em `sceneList()`.

## Painel do objeto selecionado (V6)

Aparece à direita quando há um corpo selecionado (clique). A edição passa por `BodyEditor.h` (sem raylib,
testado); todas as funções recusam corpos estáticos, ids inválidos e valores não finitos, e limitam o que
excede a faixa.

| Aba | O que edita | Observações |
|---|---|---|
| Material | massa (0,05 a 100 kg, slider em escala √), atrito (0 a 2), restituição (0 a 1), amortecimento linear e angular (0 a 5), escala de gravidade (−2 a 5), pode dormir | mudar a massa recalcula massa **e tensor de inércia** pela forma (`MassData::fromShapeMass`); escala 0 = sem gravidade, negativa = o corpo sobe |
| Posicao | x, y, z do frame do corpo; botões Acordar / Dormir | mover é um "teletransporte" (`setBodyTransform`): sem rastro na interpolação, velocidades e orientação não mudam |
| Veloc | velocidade linear e angular (±20); Zerar velocidades | |
| Orient | eixo (x, y, z) + ângulo → **quatérnio** `(cos a/2, sen a/2 · eixo unitário)`; botões Aplicar q e Identidade | o rascunho parte da orientação atual ao selecionar e só vai ao corpo em **Aplicar q**; o painel do quatérnio (à esquerda) mostra o resultado |

Limites conhecidos: os sliders usam faixas fixas (posição ±12 m, y de −2 a 14 m; velocidades ±20); um valor
fora da faixa continua valendo no corpo, mas só aparece "preso" na borda do slider. Um slider só edita
quando o mouse está sobre a barra com o botão apertado, para que a faixa não altere valores sozinha.

## Barra de controle (V5)

A barra do topo usa `TimeControl.h` (sem raylib, testado). Como o raygui devolve o clique durante o desenho,
os botões levantam um pedido que é tratado no início do quadro seguinte.

- **Pausar / Passo:** pausado, nada avança (nem α). **Passo** executa exatamente um `world.step()` (um passo
  físico fixo, o mesmo de `update()`), e logo depois o desenho usa α = 1, isto é, mostra o estado novo e não
  uma mistura com o anterior. O HUD mostra `passo #N`. Ao continuar, o acumulador retoma de onde estava.
- **Câmera lenta:** o tempo de render é multiplicado por 1, 1/2, 1/4 ou 1/10 antes de entrar no acumulador; a
  física continua em passos fixos (o resultado é o mesmo, só mais devagar), e a interpolação mantém o
  movimento suave.
- **Reset cam / Alvo:** `OrbitCamera::reset` e alternar o alvo (mundo ⇄ selecionado); resetar também volta o
  alvo para o centro do mundo.
- **Largura mínima:** a barra ocupa ~850 px; com a janela mais estreita que ~1220 px o painel da direita
  desce para não ficar embaixo dela.

## Interação (V4)

Tudo isto vive em `Interaction.h` (sem raylib, testado); a janela só converte o mouse em um raio.

- **Selecionar:** `PhysicsWorld::raycast` com filtro que ignora corpos estáticos. O selecionado ganha contorno
  amarelo e passa a ser o corpo do painel do quatérnio.
- **Arrastar com mola:** a mola-amortecedor liga o *ponto clicado* do corpo (guardado no referencial do corpo)
  ao ponto do raio do mouse na mesma profundidade. A força entra por `applyForceAtPoint`, então puxar pela
  quina também gera torque e muda o quatérnio. Rigidez ω0 = 8 rad/s por unidade de massa, amortecimento
  crítico, aceleração limitada a 150 m/s². **Aproximação conhecida:** o motor consome as forças só no passo
  fixo, então a força de cada quadro de render é escalada por `frameTime/Δt` para dar o impulso certo
  qualquer que seja o FPS (e a escala de tempo da câmera lenta: usa-se o tempo físico do quadro); com passo físico largo (cenas 8 e 9) ω0 é reduzido para `0,5/Δt` para a mola
  explícita não divergir.
- **Lançar / apagar:** corpos criados ou apagados em tempo de execução ficam registrados em `Scene::bodies`
  e `Scene::links`; `R` volta ao estado original da cena. Estáticos (chão, âncoras) não podem ser apagados.
- **Câmera:** `OrbitCamera::reset` e `followTarget` (suavização exponencial, independente do FPS).

## Painel do quatérnio e modos de interpolação (V3)

O painel à esquerda mostra, para o **corpo em foco** da cena, a pose que está sendo *desenhada*:
`x y z w`, `|q|`, eixo-ângulo e ângulos de Euler (yaw/pitch/roll, convenção YXZ). Perto de pitch = ±90° o
painel avisa **GIMBAL LOCK**: yaw e roll giram o mesmo eixo e a conversão perde um grau de liberdade
(ela devolve roll = 0). O painel da direita escolhe como a orientação é interpolada entre os dois últimos
passos físicos (`RenderPose.h`, sem raylib):

| Modo | O que faz | O que se vê |
|---|---|---|
| slerp | interpolação esférica (a do motor) | giro de velocidade constante, `\|q\| = 1` |
| lerp sem normalizar | `(1-t)·q0 + t·q1`, sem renormalizar | `\|q\|` cai no meio do passo (≈ cos(Δ/4), Δ = giro do passo): a matriz sai deformada, o cubo encolhe e o ritmo varia |
| Euler | interpola yaw/pitch/roll e reconverte | caminho torto; atravessando o gimbal lock a pose "pula" (desvio ≈ 90° − pitch inicial) |

Com passo físico de 1/60 s e giro de poucos graus por passo, os três modos são quase idênticos. Por isso
as cenas 8 e 9 usam passo largo (0,5 s e 0,35 s; 49° e 110° por passo, medidos): a interpolação deixa de
ser um detalhe e vira a coisa mais visível da tela.

## Onde o quatérnio entra

A orientação do corpo é um quatérnio no motor do começo ao fim (integração, normalização, slerp na
interpolação de render). A matriz 4×4 que vai para a placa de vídeo é calculada por
`viewer::toMat4()` (`include/viewer/core/ViewerMath.h`), que usa o `Quaternion::toMatrix()` **do motor** e
a entrega ao OpenGL por `rlMultMatrixf()`. O HUD mostra `w x y z` e `|q|` da pose interpolada e a janela
desenha os eixos locais do corpo (X vermelho, Y verde, Z azul).

## Estrutura

```text
physics_viewer/
├── CMakeLists.txt
├── include/viewer/core/   ViewerMath (pose→matriz), OrbitCamera, Scene, RenderPose, Interaction, TimeControl, BodyEditor   — SEM raylib
├── src/core/
├── src/app/main.cpp       a janela (única parte que usa raylib)
└── tests/viewer_tests.cpp testes do núcleo, sem janela
```

A lógica fica fora da janela de propósito: cenas, câmera e matemática são testadas sem tela, e a camada
raylib é fina.

## O que foi verificado

- Núcleo e testes (370 verificações: matemática, câmera, as 9 cenas rodando 300 passos, pilha/pirâmide de pé, juntas coesas, `RenderPose`: Euler ida e volta, os três modos, gimbal lock, eixo-ângulo; `Interaction`: pick, mola, lançar, apagar; câmera: reset e follow; `TimeControl`: pausa, passo a passo, escalas de tempo, α de render, determinismo do passo manual; `BodyEditor`: massa com inércia, limites, gravidade 0/negativa, teletransporte sem rastro, eixo-ângulo, velocidades, acordar/dormir, recusa de estáticos e ids inválidos, pilha com topo 50× mais pesado): `float` e `double`, sem warnings (`-Wall -Wextra -Wpedantic`) e sob ASan+UBSan.
- `main.cpp`: passa na verificação de tipos contra os cabeçalhos da raylib 5.5 e do raygui 4.0, sem warnings, em `float` e
  `double`. A convenção de matriz foi conferida no código-fonte da raylib (campo `mN` = elemento `N` do
  array passado a `rlMultMatrixf`).
- **Não verificado:** a janela em si (linkar, abrir e desenhar). O ambiente de desenvolvimento não tinha as
  bibliotecas X11/OpenGL. Na V5, confira se os botões da barra cabem e respondem (o botão **Passo** deve avançar um passo por clique, com o contador `passo #N` do HUD subindo de 1 em 1) e se a câmera lenta mantém o movimento suave. Na V6, confira se os sliders do painel do objeto respondem ao arrastar e se o texto cabe nos 320 px (a aba "Material" é a mais cheia). Na V4, confira se o clique seleciona o corpo certo (o raio sai do pixel do mouse), se o arrastar acompanha o mouse sem "chicotear" e se o botão do meio gira a câmera (alguns mouses/touchpads não têm; o botão direito continua valendo). Na V3, confira também se os painéis aparecem legíveis (texto sem cortes), se o modo lerp encolhe o cubo na cena 9 e se o modo Euler "pula" na cena 8. Na V2, confira também se os poliedros aparecem sólidos (não furados nem invertidos) e se as linhas das juntas ligam os corpos certos. Quem compila na primeira vez deve conferir se a caixa cai girando, se os eixos
  coloridos acompanham a caixa e se o `|q|` fica em 1.000000.
