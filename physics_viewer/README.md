# physics_viewer

Janela de demonstração do `../physics_engine` (C++17 + [raylib](https://www.raylib.com/)). Fica numa pasta
**ao lado** do motor, que continua sem nenhuma dependência gráfica.

> Estado: **V8** (painel "Adicionar objeto"; sobre a V7.1, que corrigiu a legibilidade dos painéis). O roteiro completo
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
| painel à direita, embaixo | abas **Objeto** (editar o corpo selecionado: Material, Posicao, Veloc, Orient) e **Adicionar** (criar objetos, ver abaixo) |
| `←` / `→` | cena anterior / próxima |
| `1` … `9` | ir direto para a cena |
| `R` | reiniciar a cena atual (mantém a câmera) |
| `I` | próximo modo de interpolação (slerp → lerp → Euler) |
| `Espaço`, `P` ou botão **Pausar/Continuar** | pausar / continuar (selecionar continua funcionando; arrastar só atua com a física rodando) |
| `.` ou botão **Passo** | avança **um** passo físico fixo (se estava rodando, pausa primeiro) |
| botões `1x` `1/2` `1/4` `1/10` | escala de tempo (câmera lenta) |
| botões `<` `>` e **Reiniciar** | cena anterior / próxima; recriar a cena atual (como `R`) |
| `M` | mostrar / esconder o painel do mundo (à esquerda, embaixo) |
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

## Painel do mundo e debug (V7)

Painel à esquerda, embaixo (tecla `M` mostra/esconde), com quatro abas. A edição passa por `WorldEditor.h`
(sem raylib, testado); valores não finitos são ignorados e o que excede a faixa é limitado.

| Aba | Conteúdo |
|---|---|
| Mundo | gravidade x, y, z (±30 m/s²), frequência da física (10 a 240 Hz, ou seja, Δt fixo = 1/Hz), sleeping ligado e tempo para dormir; **Restaurar padroes da cena** |
| Solver | iterações de velocidade (1 a 50), iterações de posição / NGS (0 = Baumgarte legado, até 20), slop linear (0 a 50 mm), slop de velocidade (0 a 2 m/s), warm starting; **Restaurar** |
| Debug | as 9 opções do debug draw do motor (formas, AABB, Fat AABB, pontos e normais de contato, juntas, centro de massa, velocidades, eixos locais) desenhadas com `DrawLine3D` a partir do `DebugLineBuffer`; **esconder modelos** (só o debug); atalhos Padrao / Desligar |
| Stats | corpos (acordados / dormindo), ilhas, juntas, pares (tocando / congelados), pontos de contato e contatos no solver, nº do passo; com **medir tempos** ligado, o custo (µs) de cada fase do último passo |

Observações:

- **Experimente:** na cena 2 (pilha), baixe as iterações de velocidade para 2 e veja a pilha amolecer; ponha
  `iter. posicao` em 0 e compare; desligue o sleeping e veja a pilha ficar sempre "acordada" na aba Stats.
- Mudar o Δt em execução preserva o α do acumulador; os impulsos de warm start vêm do Δt antigo e não são
  reescalados (pequeno transiente). Regravar a gravidade ou desligar o sleeping acorda todos os corpos.
- O debug draw usa a pose interpolada pelo **slerp** do motor, mesmo se o modo de render for lerp ou Euler
  (as linhas podem divergir um pouco do modelo nesses modos). Logo após um passo manual usa a pose atual.
- O botão **Restaurar** volta aos valores com que a cena foi criada (as cenas 8 e 9 já têm gravidade zero e passo largo).
- As cenas 8 e 9 têm Δt de 0,5 s e 0,35 s, abaixo da faixa do slider de frequência (10 Hz); o slider mostra o
  extremo, mas o valor só muda se você o arrastar.

## Adicionar objetos (V8)

Aba **Adicionar** do painel à direita. A criação passa por `ObjectSpawner.h` (sem raylib, testado).

| Sub-aba | O que define |
|---|---|
| Forma | tipo (caixa, esfera, tetraedro, octaedro, prisma, pirâmide), tamanho (0,2 a 4; 1 = os mesmos poliedros da cena Formas), massa, atrito, restituição |
| Posicao | posição x y z; velocidade de lançamento |
| Orient | orientação inicial por **eixo-ângulo** ou por **quatérnio w x y z** (normalizado; o quatérnio efetivo aparece na tela; entrada nula usa a identidade) |
| Veloc | velocidades linear e angular iniciais |

- **Adicionar** cria o objeto na posição dos sliders e o seleciona (a aba Objeto mostra os parâmetros dele).
- **Posicionar com clique**: ligado, o clique esquerdo na cena cria o objeto onde o mouse aponta (chão ou topo de outro corpo, com folga proporcional ao tamanho) e uma prévia em fio amarelo mostra o ponto; selecionar fica desligado enquanto estiver ligado e a aba Adicionar aberta.
- **Lancar da camera**: o objeto nasce 2 m à frente da câmera e sai na direção para onde ela olha, com a velocidade do slider.
- Limites: massa 0,05 a 100 kg, |v| ≤ 60 m/s, |w| ≤ 30 rad/s; entradas não finitas ou mundo cheio não criam nada (o título do painel avisa).

## Painel do objeto selecionado (V6)

Aba **Objeto** do painel à direita; mostra os parâmetros quando há um corpo selecionado (clique). A edição passa por `BodyEditor.h` (sem raylib,
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
├── include/viewer/core/   ViewerMath (pose→matriz), OrbitCamera, Scene, PolyShapes, ObjectSpawner, RenderPose, Interaction, TimeControl, BodyEditor, WorldEditor   — SEM raylib
├── src/core/
├── src/app/main.cpp       a janela (única parte que usa raylib)
└── tests/viewer_tests.cpp testes do núcleo, sem janela
```

A lógica fica fora da janela de propósito: cenas, câmera e matemática são testadas sem tela, e a camada
raylib é fina.

## O que foi verificado

- Núcleo e testes (503 verificações: matemática, câmera, as 9 cenas rodando 300 passos, pilha/pirâmide de pé, juntas coesas, `RenderPose`: Euler ida e volta, os três modos, gimbal lock, eixo-ângulo; `Interaction`: pick, mola, lançar, apagar; câmera: reset e follow; `TimeControl`: pausa, passo a passo, escalas de tempo, α de render, determinismo do passo manual; `BodyEditor`: massa com inércia, limites, gravidade 0/negativa, teletransporte sem rastro, eixo-ângulo, velocidades, acordar/dormir, recusa de estáticos e ids inválidos, pilha com topo 50× mais pesado; `ObjectSpawner`: os 6 tipos, limites de tamanho/massa/velocidade, entradas inválidas sem mudar a cena, orientação (quatérnio normalizado, eixo-ângulo, nulo → identidade), ponto de clique pelo raycast, lançamento, simulação estável com os novos corpos; `WorldEditor`: captura e aplicação dos parâmetros do mundo com limites, Δt em execução, tabela das 9 flags de debug, linhas de debug com pose interpolada x atual): `float` e `double`, sem warnings (`-Wall -Wextra -Wpedantic`) e sob ASan+UBSan.
- `main.cpp`: passa na verificação de tipos contra os cabeçalhos da raylib 5.5 e do raygui 4.0, sem warnings, em `float` e
  `double`. A convenção de matriz foi conferida no código-fonte da raylib (campo `mN` = elemento `N` do
  array passado a `rlMultMatrixf`).
- **A janela:** o usuário compilou e executou no Windows até a V7 (capturas de tela). Desde a V7.1, o texto dos
  painéis é escuro (os painéis do raygui são claros) e o texto direto sobre a cena ganha uma faixa escura
  translúcida. **Não verificado por mim:** o ajuste fino dos tamanhos depois da V7.1 (não consigo abrir a janela
  no ambiente de desenvolvimento, sem X11/OpenGL); as larguras foram calculadas a partir das capturas.
  Na V8, confira: se o painel Adicionar cabe (344 px de altura) e nenhum texto é cortado, se as duas fileiras de tipos
  mudam a forma, se a prévia amarela acompanha o mouse e se o clique cria o objeto no ponto certo.
  Confira na V7.1: se todo texto dos painéis é legível, se cada slider mostra seu número à direita, se nenhum
  rótulo é cortado pela barra do slider e se o texto de baixo (FPS e atalhos) aparece sobre a faixa escura.