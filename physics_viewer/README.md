# physics_viewer

Janela de demonstração do `../physics_engine` (C++17 + [raylib](https://www.raylib.com/)). Fica numa pasta
**ao lado** do motor, que continua sem nenhuma dependência gráfica.

> Estado: **V15** (importação de modelos 3D com textura: fecho convexo próprio, leitor .obj/.mtl, aba na janela e `.scene` com modelos); **V14** (editor de cena: cena nova, objetos estáticos, juntas; sobre a V13, painéis acoplados nas laterais, com rolagem e largura ajustável; sobre a V12, salvar e carregar cenas; sobre as cenas de colisão e rotação da V11 e o fechamento da V10). Janela redimensionável, painéis de
> quatérnio, mundo, objeto e adicionar objetos, eventos de contato, e testes sem janela no `ctest`. O roteiro completo
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

`ctest` roda dois testes do núcleo, sem janela: `viewer_tests` (verificações unitárias) e `viewer_smoke` (todas as cenas por
20 s no mesmo caminho da janela, com poses finitas e quatérnios unitários nos 3 modos de interpolação, mais um *fuzz*
determinístico: centenas de operações aleatórias de criar, lançar, apagar, editar parâmetros, mudar o mundo e trocar de
cena). Para também registrar a abertura da janela por 5 quadros (precisa de tela/OpenGL): `-DVIEWER_TEST_APP=ON`.

Para usar `double` no motor: `-DPHYSICS_USE_DOUBLE=ON` (o viewer converte para `float` só ao desenhar).

## Controles

| Entrada | Ação |
|---|---|
| botão esquerdo | selecionar o corpo sob o mouse e **arrastá-lo com uma mola** (clicar no vazio ou no chão desmarca) |
| roda, durante o arrasto | afastar / aproximar o ponto-alvo da mola (profundidade) |
| botão direito **ou roda pressionada** + mover | girar a câmera ao redor do alvo |
| roda do mouse | zoom (desligada sobre os painéis) |
| `F5` / `F6` / `F7` | apresentação (tela limpa + quatérnio grande) / câmera giratória / iniciar-parar gravação CSV |
| `T` ou botão **Alvo** | alvo da câmera: centro do mundo ⇄ corpo selecionado (transição suave) |
| `Home` ou botão **Reset cam** | resetar a câmera (ângulos padrão, alvo no centro do mundo, distância inicial da cena) |
| `B` / `O` | lançar uma caixa / esfera na direção do mouse (18 m/s) |
| `Delete` ou `X` | apagar o corpo selecionado (juntas ligadas a ele também somem) |
| coluna da direita, em cima | escolher o modo de interpolação e o FPS de render (slider de 10 a 240) |
| coluna da direita, embaixo | abas **Objeto** (editar o corpo selecionado: Material, Posicao, Veloc, Orient) e **Adicionar** (criar objetos, ver abaixo) |
| `←` / `→` | cena anterior / próxima |
| `1` … `9` | ir direto para a cena |
| `R` | reiniciar a cena atual (mantém a câmera) |
| `I` | próximo modo de interpolação (slerp → lerp → Euler) |
| `Espaço`, `P` ou botão **Pausar/Continuar** | pausar / continuar (selecionar continua funcionando; arrastar só atua com a física rodando) |
| (ao abrir, trocar ou reiniciar a cena) | **a cena começa pausada**: aperte `Espaço`/**Continuar** para rodar; o `--frames` automático não pausa |
| `.` ou botão **Passo** | avança **um** passo físico fixo (se estava rodando, pausa primeiro) |
| botões `1x` `1/2` `1/4` `1/10` | escala de tempo (câmera lenta) |
| botões `<` `>` e **Reiniciar** | cena anterior / próxima; recriar a cena atual (como `R`) |
| `M` | mostrar / esconder o painel do mundo (coluna da esquerda, embaixo) |
| arrastar a **borda** de uma coluna | muda a largura dela (340 px a 45% da janela); a cena 3D ocupa o que sobra |
| botão `<` / `>` no topo da coluna | recolhe / reabre a coluna (faixa de 28 px) |
| roda sobre uma coluna, ou arrastar a barra fina à direita dela | rola a coluna (a roda só dá zoom quando o mouse está sobre a cena) |
| `F11` | tela cheia sem borda (e volta). A janela é redimensionável (mínimo 1100 × 700) |
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
| 10 | Impacto na piramide | a pirâmide da cena 3 (com paredes) e uma bola de 8 kg rolando a 14 m/s contra ela |
| 11 | Porta giratoria (eixo Y) | porta de 2 m presa a um poste por **dobradiça de eixo Y**; uma bola a atinge na face e ela só gira em torno do eixo |
| 12 | Gangorra (eixo Z) | prancha de 6 m com dobradiça de eixo Z; uma bola de 4 kg cai numa ponta e a bola leve da outra é lançada |
| 13 | Pendulo-martelo (eixo Z) | haste de 5 kg presa por dobradiça de eixo Z, solta de 63° contra uma torre de 3 caixas |
| 14 | Bilhar (esferas) | 10 esferas em triângulo e a bola branca a 14 m/s, com paredes e restituição 0,92 |

As cenas 10 a 14 (V11) são de **colisão e rotação**: nas de eixo fixo (`HingeJoint`), o quatérnio do corpo fica sempre
no plano `(w, eixo)`: só o componente do eixo muda, e o painel mostra também a velocidade angular **w** do corpo. As teclas
`1`…`9` vão às nove primeiras; as outras se alcançam com `<` `>` / `←` `→`. Todas começam pausadas: aperte `Espaço`.

Os poliedros são desenhados a partir da `HalfEdgeStructure` do motor (faces em leque de triângulos, com
tom pela normal no mundo, mais as arestas). As cenas ficam em `src/core/Scene.cpp`; para criar outra,
escreva uma função `makeXScene()` e acrescente-a em `sceneList()`.

## Painel do mundo e debug (V7)

Painel na coluna da esquerda, embaixo (tecla `M` mostra/esconde), com quatro abas. A edição passa por `WorldEditor.h`
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

## Atrito corrigido e WASD (V19)

**Bug do atrito.** O atrito de deslizamento sempre funcionou (uma esfera lançada a 5 m/s passa a rolar a 3,57 m/s), mas **nada freava o giro em torno da normal do contato (Y, no chão) nem a rolagem**: o cone de Coulomb só age nos pontos de contato e uma esfera toca em um só, sem braço de alavanca. Só a restituição, em batidas, tirava um pouco de giro. O motor agora tem, para contatos com **esfera, cápsula ou cilindro**, duas linhas angulares: *torção* (limite de torque = `kSpinFriction`=0,03 m · μ · Σλn) e *resistência à rolagem* (`kRollingFriction`=0,02 m · μ · Σλn). Como multiplicam o μ do par de materiais (média geométrica), atrito 0 continua sendo gelo e atrito máximo para mais depressa. Caixas e poliedros não mudam. Efeito de bola: uma esfera girando a 10 rad/s com atrito 0,5 para em poucos segundos, e uma bola rolando termina parando; a cena Gangorra mudou de forma legítima (a bola freia antes e a prancha volta mais tarde).

**WASD.** **W/S** movem a câmera para a frente/trás (na direção em que ela olha, projetada no chão), **A/D** para os lados, **E** sobe e **Q** desce, **Shift** acelera 3x. A velocidade cresce com a distância da câmera (3 m/s + 0,8 m/s por metro). Não mexe com as teclas já usadas e não age enquanto você digita em um campo de texto. **Home** (ou o botão de câmera) volta ao enquadramento inicial, e trocar de cena também zera o deslocamento.

## Cápsula e cilindro (V18)

**No motor.** `CapsuleShape(raio, meiaAlturaDoSegmento)` e `CylinderShape(raio, meiaAltura)`, ambos com o **eixo no Y local**. A cápsula é um segmento mais um raio (altura total = 2·(meia-altura + raio)); com segmento nulo vira esfera. O motor trata esfera e cápsula como "núcleo + raio" no GJK/EPA, e deitados (cápsula ou cilindro no chão) geram **2 pontos de contato** em vez de 1, então não ficam balançando. Inércia, volume, AABB, raycast e debug draw têm fórmulas próprias, conferidas por integração numérica nos testes.

**No viewer.** Em **Adicionar → Forma** há três linhas de botões: a última tem *capsula*, *cilindro* e *modelo 3D*. Para os dois primeiros aparecem os sliders **raio** e **altura** (a altura é total, em metros; o raio é multiplicado pelo "tamanho"). Num **modelo 3D** o colisor agora pode ser *Capsula* ou *Cilindro*: o ajuste é automático à caixa envolvente do modelo (raio = a maior meia-aresta entre X e Z; altura pelo Y; na cápsula as pontas arredondadas ficam dentro dessa altura), e dois sliders (**raio do colisor** e **altura do colisor**, de 25% a 200%) ajustam em cima. Ideal para uma pessoa em pé: o modelo precisa ter o **Y para cima**; se o eixo mais longo não for Y o painel avisa. "mostrar o colisor (fio)" desenha a forma física por cima da malha. O `.scene` guarda `shape capsule <raio> <meia-altura>` e `shape cylinder <raio> <meia-altura>` (e a linha `model` vale com eles).

## Seu próprio modelo 3D e o tipo de colisor (V17)

**Enviar o seu arquivo.** Três jeitos: (1) **arraste o `.obj`** (ou uma pasta com `.obj`) para a janela do viewer; (2) em **Adicionar → Forma → "modelo 3D (.obj)"** cole o caminho completo no campo e aperte **Importar**; (3) copie a pasta do modelo para `physics_viewer/models/` e aperte **Atualizar lista**. Nos dois primeiros o viewer copia o `.obj`, os `.mtl` que ele cita e só as texturas que ele usa (`.png .jpg .bmp .tga`) para `models/<nome>/`, **nunca sobrescreve** (se o nome existe vira `<nome>_2`), recusa caminhos com `..`/absolutos dentro do arquivo, e valida o modelo antes de copiar (arquivo inválido não deixa lixo). O modelo importado já fica selecionado na lista. Só o formato `.obj` por enquanto (de `.fbx/.glb/.stl`: converta, por exemplo no Blender, File → Export → Wavefront).

**Tipo de colisor.** Abaixo da lista, **colisor (forma física do corpo)**: *Fecho* (fecho convexo, o mais fiel), *Caixa* (caixa envolvente alinhada aos eixos do modelo) ou *Esfera* (esfera envolvente). A malha desenhada é sempre o modelo; o resumo mostra o quanto o colisor sobra em volume, e "mostrar o colisor (fio)" desenha a forma física por cima. A escolha vai para o `.scene` (a linha `model` agora vale com `shape box`, `shape sphere` e `shape poly`). Desde a V18 há também *Capsula* e *Cilindro* (veja a seção seguinte).

**Painéis sem sobreposição.** As alturas dos painéis agora são **calculadas do conteúdo** (`PanelMetrics.h`, testado) e todo texto de painel encolhe/corta para caber na coluna, em vez de passar da borda. (Corrige os prints: caixa de seleção "estático" sobre "posicionar com clique" e o painel do quatérnio.)

## Gravar trajetória em CSV e modo apresentação (V16)

**Gravar.** Coluna da esquerda, painel **Gravar trajetoria (CSV)** (ou `F7`): escolha *Selecionado* (o corpo clicado) ou *Todos* (os dinâmicos, até 200), aperte **Gravar** e rode a simulação. Uma linha por passo físico (parar a simulação não grava linhas repetidas). A gravação para sozinha se a cena for trocada ou reiniciada, se o corpo gravado for apagado ou ao chegar a 500 000 linhas. **Salvar CSV** grava em `scenes/exports/trajetoria_<cena>_<n>.csv`.

Colunas: `t, passo, corpo, x, y, z, qw, qx, qy, qz, norma_q, eixo_x, eixo_y, eixo_z, angulo_graus, yaw_graus, pitch_graus, roll_graus, vx, vy, vz, wx, wy, wz`. O quatérnio é o **estado do motor** (sem interpolação); `norma_q` deve ficar em 1; Euler YXZ é o mesmo do painel do quatérnio (útil para mostrar o gimbal lock num gráfico). Vírgula como separador, ponto decimal, dígitos suficientes para voltar ao mesmo valor; nunca `nan`/`inf`.

**Apresentação (`F5`).** Esconde as colunas e a barra de controle: a cena ocupa a janela inteira, com o nome da cena e o **quatérnio grande** (w, x, y, z, |q|, eixo e ângulo) do corpo em foco/selecionado. `F6` liga a câmera giratória (gira sozinha ao redor do alvo; segurar o botão direito assume o controle). `ESC` sai da apresentação (um segundo `ESC` fecha o programa). Pausa, passo, reiniciar, trocar de cena e gravar continuam pelo teclado.

## Importar modelo 3D com textura (V15)

Na janela: aba **Adicionar → Forma → "modelo 3D (.obj)"**. A lista mostra os `.obj` de `physics_viewer/models/` (e de uma subpasta); escolha um, ajuste **maior dimensao** (o modelo é escalado automaticamente para essa medida), massa, atrito, restituição, posição, orientação (quatérnio) e velocidades como em qualquer objeto, e use **Adicionar**, **Lançar da câmera** ou "posicionar com clique". O resumo mostra triângulos, o tamanho do fecho, a textura (e se não foi achada) e avisa quando o modelo é **côncavo** (colide como o fecho; o buraco é preenchido). **estatico** vale para modelos (um cenário com textura). "mostrar o fecho convexo (fio)" desenha, por cima da malha, a forma com que o motor realmente colide.

- O modelo é desenhado com a malha original e a textura (`map_Kd`, `.png`/`.jpg`), iluminado por uma luz fixa; sem textura usa o `Kd` do `.mtl` ou a cor do corpo. Faces sem o sentido certo aparecem dos dois lados.
- **Salvar/carregar:** o `.scene` guarda o fecho (como qualquer poliedro) **e** uma linha `model <escala> <deslocamento> <arquivo.obj>`. Ao carregar com a pasta `models/` presente, a malha volta com a textura; se o `.obj` sumiu, o corpo carrega como poliedro comum (continua colidindo igual) e a mensagem mostra um AVISO. Exemplo: `scenes/16_Modelos_importados.scene` (cubos com textura empilhados, esfera listrada, toro caindo).
- **Limites:** `.obj` até 32 MB, 400 mil vértices, 800 mil triângulos; fecho com até 4096 vértices e 8192 faces; tamanho final de 0,05 a 40 m; `.scene` até 32 MB. Em `float`, nuvens muito densas (dezenas de milhares de vértices quase coplanares) são recusadas como "numericamente instável": use um modelo menos detalhado (ou compile com `-DPHYSICS_USE_DOUBLE`). A malha é desenhada triângulo a triângulo: modelos acima de ~60 mil triângulos podem ficar lentos.

### Núcleo (testado sem janela)

- **`ConvexHull`** — fecho convexo 3D por *quickhull*, algoritmo próprio: tetraedro inicial com pontos extremos, "olho" = ponto mais distante, região visível e horizonte, faces novas ligadas ao olho. Triângulos vizinhos e coplanares são **fundidos em polígonos** (um cubo volta com 6 faces de 4 vértices, não 12 triângulos, e apoia estável na face inteira). A tolerância é relativa ao tamanho e menor que a do motor; o resultado é conferido como o motor vai conferir e, se a fusão falhar, volta a triângulos. Entradas degeneradas (menos de 4 pontos, NaN, coincidentes, colineares, coplanares, finíssimas) dão erro com mensagem, nunca um poliedro inválido.
- **`ObjModel`** — leitor de `.obj` (`v`, `vt`, `f` em qualquer das formas `v`, `v/vt`, `v//vn`, `v/vt/vn`, índices negativos, polígonos em leque, `usemtl`, `mtllib`) e `.mtl` (`newmtl`, `Kd`, `map_Kd`). Erros com número de linha e **sem modelo parcial**; limites (32 MB, 400 mil vértices, 800 mil triângulos, coordenadas até 1e6); `mtllib`/`map_Kd` com `..`, caminho absoluto ou unidade são recusados; `.mtl` ausente só gera aviso.
- **`ModelCollider`** — do modelo à forma de colisão: centra na caixa envolvente (precisão com coordenadas grandes), calcula o fecho, centra no **centroide** (o corpo gira em torno do centro de massa) e devolve o `PolyhedronShape`, o deslocamento para desenhar a malha original e indicadores (vértices/faces do fecho, volume do modelo × do fecho, `concave`). `autoScale` ajusta a maior dimensão a um tamanho-alvo. Escala de 0,001 a 1000; maior dimensão final de 0,05 a 40 m.
- **Limitação assumida:** um modelo côncavo (toro, xícara) **colide como o seu fecho** — o buraco é preenchido (teste: uma esfera pousa no "toro cheio"). A malha desenhada continua sendo o modelo original.
- Exemplos em `physics_viewer/models/`: `cubo` (textura xadrez), `esfera` (icosfera de 320 faces, listras) e `toro` (côncavo), com `.mtl` e `.png`.

## Editor de cena (V14)

Monte a própria cena do zero e salve pela aba **Cenas** (V12):

1. Aba **Cenas** → **Nova (vazia)** ou **Nova (so o chao)**. A cena nasce pausada. (R recria a cena a partir do estado em que ela nasceu, então salve antes de reiniciar.)
2. Aba **Adicionar** → **Forma**: a caixa ganha três sliders de aresta (x, y, z; aresta = tamanho × escala, de 0,05 a 40 m) e a opção **estatico**. Um objeto estático tem massa infinita: serve de parede, plataforma ou, inclinado pela aba **Orient**, de rampa. Massa e velocidades são ignoradas para ele.
3. Aba **Objeto**: marque **clicar tambem em estaticos** para selecionar o cenário (chão, paredes). Um estático selecionado aceita posição, orientação (quatérnio por eixo-ângulo), atrito e restituição; **Apagar este objeto** (ou Del/X) o remove, inclusive o chão.
4. Aba **Juntas**: A = corpo selecionado (clique); B = o **mundo** (cria um pivô fixo cinza no ponto de ancoragem), um corpo já escolhido ou "B: clicar". A âncora vem dos sliders, do último **ponto clicado** na cena ou do **centro de A**. Dobradiça usa o eixo (sliders); distância liga o centro de A ao ponto/centro de B com o comprimento atual. A cena mostra uma prévia (esfera amarela na âncora, linhas até A e B, eixo da dobradiça em vermelho). **Apagar a ultima** desfaz a última junta e o pivô que ficou sem uso.

O núcleo (sem janela, testado) está em `Scene` (`makeEmptyScene`), `ObjectSpawner` (`isStatic`, `boxScale`), `BodyEditor` (pose e material de estáticos), `Interaction` (`pickBody(..., includeStatic)`, `deleteBody(..., allowStatic)`) e `JointEditor` (`createJoint`, `destroyLastJoint`, `jointCount`, `pointOnRay`). Exemplo pronto: `scenes/15_Meu_cenario_editor.scene` (rampa, muro, bola e uma pá presa ao mundo por dobradiça de eixo Z). Cenas sem nenhum corpo agora são arquivos válidos.

## Painéis acoplados e rolagem (V13)

Os painéis não flutuam mais sobre a cena: ficam em **duas colunas fixas nas laterais**, e a cena 3D ocupa o espaço entre
elas (como no Blender). Isso acaba com a sobreposição de textos: campo novo só aumenta a rolagem.

- **Esquerda:** quatérnio do corpo em foco (com a velocidade angular **w**) e o painel do mundo/debug (`M` esconde).
  **Direita:** interpolação de render e FPS, e o painel de abas Objeto | Adicionar | Cenas.
- **Largura:** arraste a borda (cursor de redimensionar). Limites: mínimo 340 px, máximo 45% da janela, e a cena nunca
  fica com menos de 320 px. O botão `<` / `>` do topo recolhe e reabre a coluna.
- **Rolagem:** roda do mouse sobre a coluna, ou arrastar / clicar na barra fina à direita dela. Os controles recortados
  pela coluna ficam bloqueados (um botão escondido não dispara por baixo).
- **A cena 3D** é desenhada numa **textura do tamanho exato da área central**: a proporção da câmera acompanha a área, e o
  clique/arrasto usa o raio calculado dentro dela. Girar a câmera só começa com o botão apertado sobre a cena. Mudar a
  largura de uma coluna reenquadra a cena (o campo de visão vertical é fixo: a largura aparece ou some nas laterais).
  Observação: a textura não usa o antisserrilhamento (MSAA) da janela, então as bordas dos objetos ficam um pouco mais serrilhadas.
- **Layout salvo:** larguras e colunas recolhidas ficam em `scenes/viewer_layout.ini` e voltam na próxima execução.
  Arquivo inválido é ignorado (vale o padrão).
- Núcleo `DockLayout.h` (sem raylib, testado): retângulos das colunas, bordas, viewport e rolagem (polegar proporcional,
  limites, roda); janela pequena demais faz as colunas cederem espaço à cena.

## Salvar e carregar cenas (V12)

Aba **Cenas** da coluna da direita (`SceneFile.h`, sem raylib, testado). Os arquivos ficam em `physics_viewer/scenes/`
(o caminho é fixado na compilação).

- **Salvar cena atual:** grava o **estado de agora** (poses, quatérnios, velocidades, parâmetros dos corpos, objetos que
  você adicionou ou lançou, juntas, gravidade, Δt, iterações e a câmera inicial) em `<nome>.scene`. O nome digitado vira o
  nome da cena; símbolos, acentos e barras são removidos do nome do arquivo (`Minha cena 2` → `Minha_cena_2.scene`).
- **Carregar:** escolha na lista e clique em *Carregar*. A cena entra **pausada**; `R` / **Reiniciar** a recria a partir
  do arquivo. As teclas `←` `→` voltam às cenas embutidas. O HUD mostra `[arquivo]`.
- Com o campo do nome em edição, os atalhos de teclado ficam desligados (digite à vontade); `ESC` sai da edição e só um
  segundo `ESC` fecha o programa.
- Os arquivos `01…14` da pasta são as cenas embutidas, gravadas por `scene_export` (`scene_export physics_viewer/scenes`),
  como exemplos para copiar e editar.

Formato (texto, comentários com `#`): cabeçalho `physics_scene 1`, parâmetros do mundo, blocos `body … end` (tipo
`static|dynamic|kinematic`, forma `box|sphere|poly|none`, massa, posição, **`orientation w x y z`**, velocidades, material,
cor, `focus`), `joint hinge|ballsocket|distance …` e `link`. Corpos são numerados 1, 2, 3… pela ordem no arquivo. A
descrição completa está no cabeçalho de `include/viewer/core/SceneFile.h`.

- **Não é salvo:** estado de sono (a cena carrega com todos acordados), contatos e impulsos acumulados (*warm start*).
  Uma cena recém-criada, salva e recarregada reproduz a mesma trajetória (testado nas 14 cenas).
- **Arquivo inválido** (número malformado ou NaN/inf, quatérnio nulo, índice de corpo ou de junta fora da faixa, poliedro
  côncavo, mais de 2000 corpos, mais de 8 MB…): mensagem com o número da linha e a cena atual **não é alterada**.

## Eventos e sleeping (V9)

O viewer escuta os eventos de contato do motor (`IContactListener`, via `ContactTracker.h`, sem raylib, testado):

- **Cor por estado:** corpo **adormecido** (sleeping) fica azulado; acordado, com a cor normal. O chão não muda.
- **Flash no início do contato:** os dois corpos piscam em branco-amarelado, mais forte quanto maior a velocidade de impacto
  (`approachSpeed` do motor; 8 m/s ou mais = flash máximo; abaixo de 0,3 m/s é repouso e não pisca). Decai em 0,4 s de tempo
  **físico** (na câmera lenta dura mais; pausado, congela).
- **Anéis** no ponto do impacto (até 32 ao mesmo tempo).
- Aba **Evento** do painel do mundo: liga/desliga cada efeito; mostra contatos agora, corpos dormindo, nº de inícios e fins de
  contato, último e maior impacto (m/s); **Zerar contadores**.
- Dica: na cena Pilha, deixe rodar uns segundos (os blocos adormecem e ficam azuis); dê um empurrão (arraste um bloco) e eles
  acordam. Contatos de repouso contam como início, mas não piscam.

## Adicionar objetos (V8)

Aba **Adicionar** da coluna da direita. A criação passa por `ObjectSpawner.h` (sem raylib, testado).

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

Aba **Objeto** da coluna da direita; mostra os parâmetros quando há um corpo selecionado (clique). A edição passa por `BodyEditor.h` (sem raylib,
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
- **Largura mínima:** a barra ocupa ~850 px e agora atravessa a janela inteira (as colunas ficam abaixo dela).

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
├── include/viewer/core/   ViewerMath (pose→matriz), OrbitCamera, Scene, PolyShapes, ObjectSpawner, JointEditor, ConvexHull, ObjModel, ModelCollider, ContactTracker, TrajectoryRecorder, ModelImporter, PanelMetrics, SceneFile, RenderPose, Interaction, TimeControl, BodyEditor, WorldEditor   — SEM raylib
├── src/core/
├── src/app/main.cpp       a janela (única parte que usa raylib)
├── tools/scene_export.cpp  grava as cenas embutidas como .scene (sem raylib)
├── scenes/                exemplos e cenas salvas (.scene)
├── tests/viewer_tests.cpp testes unitários do núcleo, sem janela
└── tests/viewer_smoke.cpp fumaça: todas as cenas + fuzz, sem janela
```

A lógica fica fora da janela de propósito: cenas, câmera e matemática são testadas sem tela, e a camada
raylib é fina.

## O que foi verificado

- Núcleo e testes (1903 verificações em `viewer_tests`, mais 4294 em `viewer_smoke` (V19: movimento da câmera (frente/direita para qualquer yaw, velocidade por distância, dt limitado, entradas inválidas); V18: criar cápsula/cilindro e simular, limites e entradas inválidas, `.scene` ida e volta e 5 linhas de teste, ajuste automático e fino do colisor, modelo com colisor cápsula/cilindro simulado e salvo/recarregado, alturas do painel com todas as combinações; V17: alturas dos painéis (conteúdo sempre acima do rodapé), nomes seguros, importação de `.obj`+`.mtl`+textura (cópia idêntica, só o que é usado, nunca sobrescreve, 8 recusas sem lixo, caminhos inseguros ignorados, pasta inteira), colisores caixa/esfera/fecho (centro, meias-arestas, raio, folga, modelo plano, entradas inválidas), corpos com cada colisor simulados e salvos/recarregados com texto idêntico, e fuzz de 180 `.obj` de lixo; V16: gravador — recusas, linha inicial, t e passos crescentes, |q| = 1, CSV com 24 colunas em todas as linhas e sem nan/inf, quatérnio lido do texto == do corpo, parada por reinício/corpo apagado, salvar em arquivo, e o fuzz grava junto; o fuzz também salva e recarrega a cena no meio, cria estáticos de qualquer tamanho, cria/apaga juntas e troca para cenas vazias); V15a: fecho (cubo com 1000 pontos internos, cubo girado de 1 mm a 600 m, esfera, nuvens, duplicados, 8 entradas degeneradas, determinismo, aceitação pelo motor), 15 entradas `.obj` inválidas com a mensagem esperada, `.mtl`, carregamento do disco com avisos, forma centrada no centroide, toro côncavo sem cair pelo buraco, cubo importado repousando sobre a face; fuzz de 1200 nuvens (a metade degenerada) e 1200 textos `.obj` aleatórios; V15b: lista de modelos, asset (escala automática, nomes inseguros), malha para desenhar (triângulos, UV invertido, normais para fora, esfera centrada), spawn com `model` (dinâmico e estático), `.scene` com `model` (ida e volta com texto idêntico, sem pasta/arquivo → poliedro + aviso, 8 linhas inválidas, disco), fecho grande demais recusado, e o fuzz do editor também espalha modelos e recarrega; V14: cena vazia ida e volta, caixa estática com arestas e limites, `BodyEditor` em estáticos, `JointEditor` (3 tipos, corpo-mundo, 8 entradas inválidas sem mudar a cena, apagar com pivô órfão, simulação estável, ida e volta pelo `.scene`), `pointOnRay`; `DockLayout`: larguras e limites das colunas, arrastar a borda, recolher, janelas de 2000 a 1 px sem sobreposição, arquivo de layout (ida e volta e 9 entradas inválidas), rolagem (roda, polegar, limites, NaN); `SceneFile`: ida e volta das 14 cenas (texto idêntico e trajetória igual), arquivo em disco, 22 entradas inválidas com a mensagem esperada, limites e CRLF/comentários; as cenas da V11: bola que desmonta a pirâmide, porta que só gira em Y com o centro a 1 m do eixo, gangorra só em Z, martelo que derruba a torre, bilhar sem ganho de energia: matemática, câmera, as 14 cenas rodando 300 passos, pilha/pirâmide de pé, juntas coesas, `RenderPose`: Euler ida e volta, os três modos, gimbal lock, eixo-ângulo; `Interaction`: pick, mola, lançar, apagar; câmera: reset e follow; `TimeControl`: pausa, passo a passo, escalas de tempo, α de render, determinismo do passo manual; `BodyEditor`: massa com inércia, limites, gravidade 0/negativa, teletransporte sem rastro, eixo-ângulo, velocidades, acordar/dormir, recusa de estáticos e ids inválidos, pilha com topo 50× mais pesado; `ObjectSpawner`: os 6 tipos, limites de tamanho/massa/velocidade, entradas inválidas sem mudar a cena, orientação (quatérnio normalizado, eixo-ângulo, nulo → identidade), ponto de clique pelo raycast, lançamento, simulação estável com os novos corpos; `ContactTracker`: intensidade por velocidade, tinta por estado, flash que decai, marcadores limitados, contadores, eventos reais do motor (caixa que cai) e a pilha adormecendo; `WorldEditor`: captura e aplicação dos parâmetros do mundo com limites, Δt em execução, tabela das 9 flags de debug, linhas de debug com pose interpolada x atual): `float` e `double`, sem warnings (`-Wall -Wextra -Wpedantic`) e sob ASan+UBSan.
- `main.cpp`: passa na verificação de tipos contra os cabeçalhos da raylib 5.5 e do raygui 4.0, sem warnings, em `float` e
  `double`. A convenção de matriz foi conferida no código-fonte da raylib (campo `mN` = elemento `N` do
  array passado a `rlMultMatrixf`).
- **A janela:** o usuário compilou e executou no Windows até a V7 (capturas de tela). Desde a V7.1, o texto dos
  painéis é escuro (os painéis do raygui são claros) e o texto direto sobre a cena ganha uma faixa escura
  translúcida. **Não verificado por mim:** o ajuste fino dos tamanhos depois da V7.1 (não consigo abrir a janela
  no ambiente de desenvolvimento, sem X11/OpenGL); as larguras foram calculadas a partir das capturas.
  Na V19, confira (janela, que não consigo abrir aqui): WASD/QE movem a câmera e Shift acelera; esferas com atrito alto param de girar em Y; Home volta.
  Na V18, confira (janela, que não consigo abrir aqui): os 9 botões de forma cabem em 3 linhas; criar cápsula/cilindro e ver a prévia em fio; escolher Capsula/Cilindro como colisor de um modelo e ver os sliders de ajuste e o fio acompanharem; nenhum painel sobreposto com os sliders novos.
  Na V17, confira (janela, que não consigo abrir aqui): arrastar um `.obj` seu para a janela o coloca na lista já selecionado; o campo de caminho + Importar faz o mesmo e mostra a mensagem; trocar Fecho/Caixa/Esfera atualiza o resumo e o "fio" mostra a forma; na aba Adicionar → caixa o checkbox "estático" fica acima do rodapé; textos longos do painel do quatérnio não passam da borda.
  Na V16, confira (janela, que não consigo abrir aqui): F5 esconde as colunas e mostra o quatérnio grande e ESC volta; F6 gira a câmera; gravar um corpo caindo, salvar e abrir o CSV (24 colunas, `norma_q` = 1); "Todos" grava um corpo por linha; trocar de cena para a gravação com aviso.
  Na V15, confira (desenho com textura, que não consigo abrir aqui): a lista mostra cubo, esfera e toro; a textura xadrez/listras aparece e acompanha a rotação; o brilho muda com a rotação; o fio do fecho coincide com o corpo; "CONCAVO" aparece no toro; salvar e carregar traz o modelo com a textura de volta.
  Na V14, confira: a aba Juntas cabe no painel (5 sliders, botões, mensagens) e rola com a coluna; o clique na cena escolhe A/B/ponto sem arrastar o corpo; a prévia amarela acompanha os sliders; criar uma dobradiça ao mundo faz o corpo girar preso ao pivô cinza; Nova (vazia) mostra a grade sem corpos; salvar e carregar mantém paredes e juntas.
  Na V13, confira (a mudança mais arriscada, feita sem poder abrir a janela): as colunas aparecem nas laterais, a cena
  fica no meio e **não está esticada** ao arrastar a borda; clicar e arrastar objetos acerta onde o mouse aponta; a
  rolagem funciona e nada sai da coluna; o texto do HUD aparece dentro da cena; `<` `>` recolhem e reabrem; o layout volta
  ao reabrir o programa.
  Na V12, confira: a aba Cenas cabe no painel (campo, botões, lista de 104 px, mensagem), digitar o nome não aciona
  atalhos, Salvar cria o arquivo em `physics_viewer/scenes/` e Carregar traz a cena de volta pausada.
  Na V10, confira: se a janela redimensiona sem sobrepor painéis (mínimo 960 × 700) e se F11 entra e sai da tela cheia.
  Na V9, confira: se a aba "Evento" cabe no painel (5 abas) e se o flash/anéis aparecem ao soltar uma caixa; os blocos
  da Pilha ficam azuis ao dormir e voltam à cor ao acordar.
  Na V8, confira: se o painel Adicionar cabe (344 px de altura; a aba Objeto tem 306 px) e nenhum texto é cortado, se as duas fileiras de tipos
  mudam a forma, se a prévia amarela acompanha o mouse e se o clique cria o objeto no ponto certo.
  Confira na V7.1: se todo texto dos painéis é legível, se cada slider mostra seu número à direita, se nenhum
  rótulo é cortado pela barra do slider e se o texto de baixo (FPS e atalhos) aparece sobre a faixa escura.