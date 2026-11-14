# Quack3 Viewer — viewer do motor de física

Aplicação interativa que mostra o motor Quack3 (`../physics_engine`) em funcionamento (C++17 + [raylib](https://www.raylib.com/)
e raygui). Ela fica numa pasta **ao lado** do motor, que continua sem nenhuma dependência gráfica.

O viewer reúne:

- **14 cenas prontas** (empilhamento, pirâmide, pêndulos, correntes, poliedros, colisões, gimbal lock, interpolação...) e cenas suas, salvas em arquivo;
- **edição ao vivo** do objeto selecionado, do mundo e do solver, com sliders que aceitam digitar valores e contas;
- **painel do quatérnio** (componentes, norma, eixo-ângulo, Euler, aviso de gimbal lock), esfera de **arcball** e três modos de interpolação de render;
- **criação de objetos** (caixa, esfera, cápsula, cilindro, poliedros e modelos 3D), **editor de cena** com objetos estáticos e juntas;
- **importação de modelos `.obj`** com textura;
- **debug draw** do motor, estatísticas, tempos por fase e indicadores de eventos de contato e de sleeping;
- **gravação de trajetória em CSV** e **modo apresentação**.

A orientação dos corpos é um quatérnio no motor do começo ao fim. A matriz 4×4 enviada à placa de vídeo é
calculada por `viewer::toMat4()` a partir do `Quaternion::toMatrix()` **do motor**, então o que aparece na tela é
exatamente o estado da física.

---

## 1. Compilar e rodar

Requisitos: CMake ≥ 3.14, um compilador C++17 e acesso ao GitHub na primeira configuração (a raylib 5.5 é
baixada e compilada automaticamente, a menos que já esteja instalada no sistema; o raygui 4.0, um único
cabeçalho, é sempre baixado).

| Sistema | Pacotes necessários para a raylib |
|---|---|
| Linux (Debian/Ubuntu) | `sudo apt install build-essential cmake git libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev` |
| Windows | CMake + Visual Studio Build Tools (MSVC) **ou** MinGW-w64 |
| macOS | `xcode-select --install` e CMake |

```bash
cmake -S physics_viewer -B build_viewer
cmake --build build_viewer -j
./build_viewer/physics_viewer          # Windows (MSVC): build_viewer\Release\physics_viewer.exe
```

O programa precisa das pastas `scenes/` e `models/` do código-fonte: os caminhos são fixados na compilação.

### Opções do CMake

| Opção | Padrão | O que faz |
|---|---|---|
| `VIEWER_BUILD_APP` | `ON` | compila a janela (precisa da raylib); `OFF` compila só o núcleo e os testes |
| `VIEWER_BUILD_TESTS` | `ON` | compila os testes do núcleo (não precisam da raylib) |
| `VIEWER_TEST_APP` | `OFF` | registra no `ctest` um teste que abre a janela por 5 quadros (precisa de tela e OpenGL) |
| `PHYSICS_USE_DOUBLE` | `OFF` | motor em `double` (o viewer converte para `float` só ao desenhar) |
| `PHYSICS_ENGINE_DIR` | `../physics_engine` | caminho do motor, se estiver em outro lugar |

### Só o núcleo e os testes (sem raylib e sem janela)

```bash
cmake -S physics_viewer -B build_core -DVIEWER_BUILD_APP=OFF
cmake --build build_core -j && ctest --test-dir build_core
```

### Linha de comando

```bash
./build_viewer/physics_viewer --frames 240 --screenshot saida.png
```

`--frames N` roda N quadros sem pausar a cena e sai com código 0; `--screenshot arquivo.png` salva uma captura
ao final. Serve para verificação automática.

---

## 2. A janela

A janela é redimensionável (mínimo 1100 × 700; `F11` alterna tela cheia sem borda). A tela tem quatro regiões:

```text
┌───────────────────────────────────────────────────────────────────────────┐
│ barra de controle: Pausar/Continuar · Passo · câmera lenta · < > Reiniciar │
├──────────────┬────────────────────────────────────────┬───────────────────┤
│ coluna da    │                                        │ coluna da direita │
│ esquerda     │            cena 3D                     │ Interpolação      │
│ Quatérnio,   │                                        │ Cenas             │
│ arcball,     │                                        │ Objeto | Adicionar│
│ gravação,    │                                        │ | Juntas          │
│ mundo e debug│                                        │                   │
└──────────────┴────────────────────────────────────────┴───────────────────┘
```

- **Colunas fixas** nas laterais, cada uma com **barra de rolagem**: a roda do mouse sobre a coluna ou a barra fina à direita dela rola o conteúdo. Controles cortados pela borda da coluna ficam bloqueados.
- **Largura ajustável:** arraste a borda da coluna (340 px até 45% da janela; a cena nunca fica com menos de 320 px). O botão `<` / `>` no topo da coluna a recolhe e reabre.
- **A cena 3D** ocupa o espaço restante, desenhada numa textura do tamanho exato da área (a proporção da câmera acompanha a área e os cliques usam o raio calculado dentro dela). A roda do mouse só dá zoom quando o cursor está sobre a cena.
- **Layout salvo:** larguras e colunas recolhidas ficam em `scenes/viewer_layout.ini` e voltam na próxima execução. Arquivo inválido é ignorado.
- **Tema** cinza-grafite com destaque azul. Botões, abas e textos reduzem a fonte para sempre caber na coluna.
- **Sliders:** além de arrastar, o **clique duplo** troca a barra por um campo de texto (ver seção 4.1).
- **Cenas começam pausadas** ao abrir, trocar ou reiniciar.

---

## 3. Controles

| Entrada | Ação |
|---|---|
| botão esquerdo | selecionar o corpo sob o mouse e **arrastá-lo com uma mola** (clicar no vazio ou no chão desmarca) |
| roda, durante o arrasto | afasta ou aproxima o ponto-alvo da mola (profundidade) |
| botão direito **ou** roda pressionada + mover | girar a câmera ao redor do alvo |
| roda do mouse | zoom (só sobre a cena) |
| `W` `A` `S` `D` | mover a câmera pelo cenário (frente, esquerda, trás, direita) |
| `E` / `Q` | subir / descer a câmera |
| `Shift` | acelera o movimento da câmera (3×) |
| `T` ou botão **Alvo** | alvo da câmera: centro do mundo ⇄ corpo selecionado (transição suave) |
| `Home` ou botão **Reset cam** | enquadramento inicial da cena, alvo no centro do mundo |
| `B` / `O` | lançar uma caixa / esfera na direção do mouse (18 m/s) |
| `Delete` ou `X` | apagar o corpo selecionado (as juntas ligadas a ele também somem) |
| `Espaço`, `P` ou botão **Pausar/Continuar** | pausa / continua (selecionar funciona pausado; arrastar só atua com a física rodando) |
| `.` ou botão **Passo** | avança **um** passo físico fixo (se estava rodando, pausa primeiro) |
| botões `1x` `1/2` `1/4` `1/10` | escala de tempo (câmera lenta) |
| `←` / `→` ou botões `<` `>` | cena anterior / próxima |
| `1` … `9` | ir direto para a cena de mesmo número na lista |
| `R` ou botão **Reiniciar** | recria a cena atual (mantém a câmera) |
| `I` | próximo modo de interpolação (slerp → lerp → Euler) |
| `M` | mostra / esconde o painel do mundo (coluna da esquerda) |
| `F5` | modo apresentação |
| `F6` | câmera giratória |
| `F7` | iniciar / parar a gravação de trajetória |
| `F11` | tela cheia sem borda |
| `ESC` | sai da edição de texto; fora dela, sai da apresentação e depois fecha o programa |

Com um campo de texto em edição (nome de cena, caminho de modelo, valor de slider), os atalhos de teclado
ficam desligados para você digitar à vontade.

**Câmera.** `W/S` movem para a frente e para trás na direção em que a câmera olha (projetada no chão), `A/D` para
os lados. A velocidade cresce com a distância da câmera ao alvo. Trocar de cena zera o deslocamento.

**Arrastar com mola.** Uma mola amortecida liga o ponto clicado do corpo (guardado no referencial dele) ao ponto
do raio do mouse na mesma profundidade. A força entra por `applyForceAtPoint`, então puxar por uma quina também
gera torque e muda o quatérnio. A aceleração é limitada e a mola é ajustada ao passo físico, para não divergir
em cenas de passo largo.

---

## 4. Painéis

### 4.1 Sliders editáveis

Em qualquer slider (objeto, adicionar, juntas, mundo, solver, FPS de render), um **clique duplo rápido** abre um
campo de texto já preenchido com o **valor atual** e o cursor no fim.

- Digite um número (vírgula ou ponto decimal, notação científica, unidade no fim, como `2,5 m`) ou uma **conta**: `+ - * /`, parênteses e sinal (`4/2` → 2, `1-4` → −3, `2*(3+1)` → 8).
- **Enter** ou clique fora aplica; **Esc** cancela sem fechar o programa.
- O resultado é limitado à faixa do slider. Divisão por zero, sintaxe inválida ou texto que não é número mantêm o valor anterior.
- Se a coluna rolar ou a aba mudar durante a edição, ela é cancelada.

### 4.2 Barra de controle

- **Pausar/Continuar:** pausado, nada avança. **Passo** executa exatamente um passo fixo e depois desenha com α = 1 (o estado novo, sem mistura com o anterior). O HUD mostra `passo #N`.
- **Câmera lenta** (1, 1/2, 1/4, 1/10): multiplica o tempo de render antes do acumulador. A física continua em passos fixos, então o resultado é o mesmo, só mais devagar, e a interpolação mantém o movimento suave.
- **`<` `>` Reiniciar:** troca de cena e recria a cena atual.
- **Reset cam** e **Alvo:** ver a seção 3.

### 4.3 Coluna da esquerda — quatérnio e arcball

O painel do quatérnio mostra, para o **corpo em foco** (o selecionado, ou o corpo principal da cena), a pose
que está sendo *desenhada*: componentes `x y z w`, norma `|q|`, eixo-ângulo, ângulos de Euler (yaw, pitch, roll,
convenção YXZ) e a velocidade angular **w**. Perto de pitch = ±90° o painel avisa **GIMBAL LOCK**: yaw e roll
passam a girar o mesmo eixo e a conversão perde um grau de liberdade (devolve roll = 0).

O **arcball** é uma esfera com os 3 círculos máximos dos eixos locais do corpo (x vermelho, y verde, z azul; a
metade de trás é apagada) e, em laranja, o **eixo de rotação do quatérnio**. Com um corpo selecionado, arrastar
sobre a esfera gira o corpo: o ponto agarrado acompanha o cursor. A velocidade angular é zerada durante o gesto;
pause a cena para girar com calma.

### 4.4 Coluna da direita — interpolação

Escolhe como a orientação é interpolada entre os dois últimos passos físicos, e o FPS de render (10 a 240):

| Modo | O que faz | O que se vê |
|---|---|---|
| **slerp** | interpolação esférica (a do motor) | giro de velocidade constante, `\|q\| = 1` |
| **lerp sem normalizar** | `(1−t)·q0 + t·q1`, sem renormalizar | `\|q\|` cai no meio do passo (≈ cos(Δ/4)): a matriz sai deformada, o cubo encolhe e o ritmo varia |
| **Euler** | interpola yaw/pitch/roll e reconverte | caminho torto; atravessando o gimbal lock a pose “pula” |

Com passo de 1/60 s e giro de poucos graus por passo, os três modos são quase idênticos. As cenas 8 e 9 usam passo
largo exatamente para tornar a diferença visível.

### 4.5 Painel Cenas

Lista única com as 14 cenas originais e os arquivos `.scene` da pasta `scenes/`. As setas, os botões `<` `>` e as
teclas `1`–`9` percorrem a lista na ordem em que ela está, com volta ao início.

| Controle | O que faz |
|---|---|
| **Salvar cena atual** | grava o **estado de agora** em `scenes/<nome>.scene` (poses, quatérnios, velocidades, parâmetros dos corpos, objetos adicionados, juntas, gravidade, Δt, iterações e câmera inicial). Coloca a cena no fim da lista, ou atualiza a que tinha esse nome |
| **Carregar a escolhida** | vai para a cena marcada (entra pausada; `R` a recria a partir do arquivo) |
| **Nova (vazia)** / **Nova (so o chao)** | cena do zero, para o editor (seção 4.8) |
| **Subir / Descer** | reordena a cena escolhida (original ou salva) |
| **Apagar** | pede um segundo clique (**Confirmar apagar**). Apagar uma salva apaga o `.scene` do disco; apagar uma original só a tira da lista. Sempre resta ao menos uma cena |
| **Restaurar originais (n)** | traz de volta as originais apagadas, no fim da lista |
| **Atualizar lista** | relê a pasta: arquivos novos entram no fim, os que sumiram saem |

Detalhes: o nome digitado vira o nome da cena e símbolos, acentos e barras são removidos do nome do arquivo
(`Minha cena 2` → `Minha_cena_2.scene`). Arquivo com erro aparece com `[ERRO]`, é pulado pelas setas e mostra o
motivo. A ordem e as originais escondidas ficam em `scenes/biblioteca.txt` (se for apagado ou corrompido, volta
a ordem padrão). A cena que está rodando fica marcada com `>`; apagá-la não a interrompe.
Os arquivos `01_…` a `14_…` da pasta são a exportação das originais e não aparecem duplicados na lista.

### 4.6 Painel do objeto selecionado (aba Objeto)

Mostra os parâmetros do corpo clicado. A edição recusa corpos estáticos (salvo com a opção abaixo), ids inválidos e
valores não finitos, e limita o que passa da faixa.

| Sub-aba | O que edita |
|---|---|
| **Material** | massa (0,05 a 100 kg), atrito (0 a 2), restituição (0 a 1), amortecimento linear e angular (0 a 5), escala de gravidade (−2 a 5; 0 = sem gravidade, negativa = o corpo sobe), pode dormir |
| **Posicao** | x, y, z do frame do corpo; botões **Acordar** / **Dormir**. Mover é um teletransporte: sem rastro na interpolação, velocidades e orientação não mudam |
| **Veloc** | velocidade linear e angular (±20); **Zerar velocidades** |
| **Orient** | orientação por **eixo-ângulo** (eixo x y z + ângulo → quatérnio `(cos a/2, sen a/2·eixo)`) ou por **omega x/y/z** (ver abaixo); **Aplicar q** e **Identidade** |

Mudar a massa recalcula massa **e tensor de inércia** pela forma. No modo eixo-ângulo, o rascunho parte da
orientação atual ao selecionar e só vai ao corpo em **Aplicar q**; o painel do quatérnio mostra o resultado.
No modo **omega x y z**, os sliders aplicam na hora: o corpo gira ω_x em torno de X, depois ω_y em Y e depois ω_z
em Z (eixos fixos do mundo; `q = qz·qy·qx`); **Ler do corpo** preenche os sliders com a orientação atual; ω_y vai
de −90° a 90° (acima disso há gimbal lock).

Para selecionar o cenário (chão, paredes), marque **clicar tambem em estaticos**. Um estático selecionado aceita
posição, orientação, atrito e restituição, e **Apagar este objeto** (ou `Del`/`X`) o remove, inclusive o chão.

### 4.7 Painel Adicionar

| Sub-aba | O que define |
|---|---|
| **Forma** | tipo, tamanho, massa, atrito, restituição, opção **estatico** |
| **Posicao** | posição x y z e velocidade de lançamento |
| **Orient** | orientação inicial por **eixo-ângulo** ou **quatérnio w x y z** (normalizado; o quatérnio efetivo aparece na tela; entrada nula usa a identidade) |
| **Veloc** | velocidades linear e angular iniciais |

**Tipos:** caixa, esfera, tetraedro, octaedro, prisma, pirâmide, **cápsula**, **cilindro** e **modelo 3D (.obj)**.
Tamanho de 0,2 a 4 (1 = os mesmos poliedros da cena Formas). A caixa tem três sliders de aresta (aresta = tamanho ×
escala, de 0,05 a 40 m). Cápsula e cilindro têm **raio** e **altura** (altura total, em metros; o raio é
multiplicado pelo tamanho).

**Como criar:**

- **Adicionar:** cria o objeto na posição dos sliders e o seleciona.
- **Posicionar com clique:** com a opção ligada, o clique esquerdo na cena cria o objeto onde o mouse aponta (chão ou topo de outro corpo, com folga proporcional ao tamanho). Uma prévia em fio amarelo mostra o ponto. Enquanto estiver ligada e a aba aberta, selecionar fica desligado.
- **Lancar da camera:** o objeto nasce 2 m à frente da câmera e sai na direção para onde ela olha, com a velocidade do slider.

**Estático:** um objeto estático tem massa infinita e serve de parede, plataforma ou, inclinado pela aba Orient, de
rampa. Massa e velocidades são ignoradas para ele.

Limites: massa de 0,05 a 100 kg, |v| ≤ 60 m/s, |w| ≤ 30 rad/s. Entradas não finitas ou mundo cheio não criam nada
(o título do painel avisa).

### 4.8 Editor de juntas e de cena (aba Juntas)

Para montar a própria cena: **Nova (vazia)** ou **Nova (so o chao)** no painel Cenas, objetos estáticos e
dinâmicos pela aba Adicionar, e juntas pela aba **Juntas**. Salve pelo painel Cenas (o `R` recria a cena a partir
do estado em que ela nasceu, então salve antes de reiniciar).

Na aba Juntas:

- **A** é o corpo selecionado (clique). **B** pode ser o **mundo** (cria um pivô fixo cinza no ponto de ancoragem), um corpo já escolhido ou **B: clicar**.
- A **âncora** vem dos sliders, do último **ponto clicado** na cena ou do **centro de A**.
- Tipos: **dobradiça** (usa o eixo dos sliders), **bola-soquete** e **distância** (liga o centro de A ao ponto ou centro de B com o comprimento atual).
- A cena mostra uma prévia: esfera amarela na âncora, linhas até A e B e o eixo da dobradiça em vermelho.
- **Apagar a ultima** desfaz a última junta e o pivô que ficou sem uso.

O exemplo `scenes/15_Meu_cenario_editor.scene` (rampa, muro, bola e uma pá presa ao mundo por dobradiça de eixo Z)
foi montado com este editor.

### 4.9 Coluna da esquerda — gravação de trajetória (CSV)

Painel **Gravar trajetoria (CSV)** (ou `F7`). Escolha **Selecionado** (o corpo clicado) ou **Todos** (os dinâmicos,
até 200), aperte **Gravar** e rode a simulação. Há uma linha por passo físico (parar a simulação não grava linhas
repetidas). A gravação para sozinha se a cena for trocada ou reiniciada, se o corpo gravado for apagado ou ao
chegar a 500 000 linhas. **Salvar CSV** grava em `scenes/exports/trajetoria_<cena>_<n>.csv`; o **caminho
completo** fica na tela até o próximo arquivo, com o botão **Copiar caminho**.

Colunas (24):

```text
t, passo, corpo, x, y, z, qw, qx, qy, qz, norma_q, eixo_x, eixo_y, eixo_z, angulo_graus,
yaw_graus, pitch_graus, roll_graus, vx, vy, vz, wx, wy, wz
```

O quatérnio é o **estado do motor**, sem interpolação; `norma_q` deve ficar em 1; os ângulos de Euler (YXZ) são
os mesmos do painel do quatérnio, o que permite mostrar o gimbal lock num gráfico. O separador é a vírgula, o
decimal é o ponto, com dígitos suficientes para reproduzir o valor; nunca aparece `nan` ou `inf`.

### 4.10 Coluna da esquerda — mundo e debug (`M` esconde)

Cinco abas. Valores não finitos são ignorados e o que excede a faixa é limitado.

| Aba | Conteúdo |
|---|---|
| **Mundo** | gravidade x, y, z (±30 m/s²), frequência da física (10 a 240 Hz, ou seja, Δt fixo = 1/Hz), sleeping ligado e tempo para dormir; **Restaurar padroes da cena** |
| **Solver** | iterações de velocidade (1 a 50), iterações de posição / NGS (0 = Baumgarte, até 20), slop linear (0 a 50 mm), slop de velocidade (0 a 2 m/s), warm starting, caixa **orientação: exponencial de quatérnio** (desligada = Euler); **Restaurar** |
| **Debug** | as 9 opções do debug draw do motor (formas, AABB, Fat AABB, pontos de contato, normais de contato, juntas, centro de massa, velocidades, eixos locais) desenhadas com `DrawLine3D`; **esconder modelos** (deixa só o debug); atalhos Padrao / Desligar |
| **Stats** | corpos (acordados / dormindo), ilhas, juntas, pares (tocando / congelados), pontos de contato e contatos no solver, nº do passo; com **medir tempos** ligado, o custo (µs) de cada fase do último passo |
| **Evento** | liga e desliga cada efeito visual de contato; contatos agora, corpos dormindo, nº de inícios e fins de contato, último e maior impacto (m/s); **Zerar contadores** |

Observações:

- Mudar o Δt em execução preserva o α do acumulador; os impulsos de *warm start* vêm do Δt antigo e não são reescalados (pequeno transiente). Regravar a gravidade ou desligar o sleeping acorda todos os corpos.
- O debug draw usa a pose interpolada pelo slerp do motor, mesmo com o modo de render em lerp ou Euler (as linhas podem divergir um pouco do modelo nesses modos).
- **Restaurar** volta aos valores com que a cena foi criada (as cenas 8 e 9 já têm gravidade zero e passo largo; o slider de frequência mostra o extremo de 10 Hz, mas o valor só muda se for arrastado).
- Experimente: na cena 2, baixe as iterações de velocidade para 2 e a pilha amolece; ponha iterações de posição em 0 e compare; desligue o sleeping e veja os corpos ficarem sempre “acordados” na aba Stats.

### 4.11 Eventos de contato e sleeping

O viewer escuta os eventos de contato do motor:

- **Cor por estado:** corpo adormecido fica azulado; acordado, com a cor normal. O chão não muda.
- **Flash no início do contato:** os dois corpos piscam em branco-amarelado, mais forte quanto maior a velocidade de impacto (`approachSpeed`; 8 m/s ou mais é o flash máximo; abaixo de 0,3 m/s é repouso e não pisca). Decai em 0,4 s de tempo físico (na câmera lenta dura mais; pausado, congela).
- **Anéis** no ponto do impacto (até 32 ao mesmo tempo).

Dica: na cena Pilha, deixe rodar alguns segundos (os blocos adormecem e ficam azuis) e arraste um bloco para acordá-los.

### 4.12 Modo apresentação (`F5`) e câmera giratória (`F6`)

`F5` esconde as colunas e a barra de controle: a cena ocupa a janela inteira, com o nome da cena e o **quatérnio
grande** (w, x, y, z, |q|, eixo e ângulo) do corpo em foco. `F6` liga a câmera giratória, que gira sozinha ao
redor do alvo (segurar o botão direito assume o controle). `ESC` sai da apresentação; um segundo `ESC` fecha o
programa. Pausa, passo, reiniciar, troca de cena e gravação continuam pelo teclado.

---

## 5. Cenas incluídas

| # | Cena | O que mostra |
|---|---|---|
| 1 | Caixa em queda | uma caixa já girando; o HUD mostra o quatérnio |
| 2 | Pilha de caixas | 6 caixas empilhadas (HUD: caixa do topo) |
| 3 | Pirâmide de caixas | 5 fileiras, 15 caixas |
| 4 | Chuva de corpos | 36 caixas e esferas em orientações aleatórias, com paredes |
| 5 | Pêndulo duplo | duas esferas ligadas por juntas de distância (linhas brancas) |
| 6 | Corrente (juntas) | 8 elos com juntas bola-soquete |
| 7 | Formas (poliedros) | tetraedro, octaedro, prisma triangular, pirâmide, caixas e esferas |
| 8 | Gimbal lock (passo 0,5 s) | gravidade zero; uma caixa gira em torno de X e atravessa pitch = 90° |
| 9 | Interpolacao (passo 0,35 s) | gravidade zero; um cubo gira ~110° entre dois passos físicos |
| 10 | Impacto na piramide | a pirâmide da cena 3, com paredes, e uma bola de 8 kg rolando a 14 m/s contra ela |
| 11 | Porta giratoria (eixo Y) | porta de 2 m presa a um poste por dobradiça de eixo Y; uma bola a atinge na face e ela só gira em torno do eixo |
| 12 | Gangorra (eixo Z) | prancha de 6 m com dobradiça de eixo Z; uma bola de 4 kg cai numa ponta e a bola leve da outra é lançada |
| 13 | Pendulo-martelo (eixo Z) | haste de 5 kg presa por dobradiça de eixo Z, solta de 63° contra uma torre de 3 caixas |
| 14 | Bilhar (esferas) | 10 esferas em triângulo e a bola branca a 14 m/s, com paredes e restituição 0,92 |
| 15 | Meu cenario (editor) | rampa, muro, bola e uma pá presa ao mundo por dobradiça de eixo Z (arquivo `.scene`) |
| 16 | Modelos importados | cubos com textura empilhados, esfera listrada e um toro caindo (arquivo `.scene`) |

Nas cenas de eixo fixo (11 a 13), o quatérnio do corpo articulado fica sempre no plano `(w, eixo)`: só o
componente do eixo muda, e o painel mostra também a velocidade angular **w**. As teclas `1`…`9` vão às nove
primeiras da lista; as demais se alcançam com `<` `>` / `←` `→`.

Os poliedros são desenhados a partir da `HalfEdgeStructure` do motor (faces em leque de triângulos, com tom pela
normal no mundo, mais as arestas). As cenas embutidas ficam em `src/core/Scene.cpp`; para criar outra no código,
escreva uma função `makeXScene()` e acrescente-a em `sceneList()`.

---

## 6. Modelos 3D

### 6.1 Usar um modelo

Em **Adicionar → Forma → modelo 3D (.obj)**, a lista mostra os `.obj` de `models/` (e de suas subpastas). Escolha
um, ajuste a **maior dimensao** (o modelo é escalado para essa medida), massa, atrito, restituição, posição,
orientação e velocidades como em qualquer objeto, e use **Adicionar**, **Lancar da camera** ou “posicionar com
clique”. **estatico** vale para modelos (um cenário com textura).

O resumo mostra triângulos, o tamanho do fecho, a textura (e se não foi achada) e avisa quando o modelo é
**côncavo**. **mostrar o colisor (fio)** desenha, por cima da malha, a forma com que o motor realmente colide.

O modelo é desenhado com a malha original e a textura (`map_Kd`, `.png`/`.jpg`), com iluminação por uma luz fixa; sem
textura usa o `Kd` do `.mtl` ou a cor do corpo. Faces com o sentido errado aparecem dos dois lados.

### 6.2 Importar o seu próprio modelo

Três jeitos:

1. **arraste o `.obj`** (ou uma pasta com `.obj`) para a janela;
2. em **Adicionar → Forma → modelo 3D**, cole o caminho completo no campo e aperte **Importar**;
3. copie a pasta do modelo para `physics_viewer/models/` e aperte **Atualizar lista**.

Nos dois primeiros, o viewer valida o modelo e copia o `.obj`, os `.mtl` que ele cita e só as texturas que ele usa
(`.png .jpg .bmp .tga`) para `models/<nome>/`. Nada é sobrescrito (se o nome existe, vira `<nome>_2`), caminhos
com `..` ou absolutos dentro do arquivo são recusados e um arquivo inválido não deixa lixo. O modelo importado já
fica selecionado.

O `.mtl` e as texturas são encontrados mesmo se o `.obj` citar outro nome (usa o `.mtl` com o nome do `.obj` ou o
único da pasta), um caminho de outro computador (usa só o nome do arquivo), maiúsculas diferentes, ou nem citar
`mtllib` (o viewer acrescenta). O `.obj` e o `.mtl` copiados são reescritos para apontar certo. A mensagem da
janela mostra `[.mtl: N, texturas: M]` e **todos** os avisos. Dois `.mtl` sem nome que case: o viewer não escolhe
e avisa.

O formato aceito é `.obj` (de `.fbx`, `.glb` ou `.stl`, converta antes, por exemplo no Blender: *File → Export →
Wavefront*).

### 6.3 Colisor do modelo

O motor só aceita formas convexas, então o corpo físico do modelo é uma forma **convexa** escolhida em **colisor
(forma física do corpo)**. A malha desenhada é sempre o modelo.

| Colisor | O que é |
|---|---|
| **Fecho** | fecho convexo dos vértices (o mais fiel); um cubo volta com 6 faces de 4 vértices e apoia estável na face inteira |
| **Caixa** | caixa envolvente alinhada aos eixos do modelo |
| **Esfera** | esfera envolvente |
| **Capsula** | ajuste automático à caixa envolvente (raio = a maior meia-aresta entre X e Z; altura pelo Y; as pontas arredondadas ficam dentro da altura) |
| **Cilindro** | mesmo ajuste automático da cápsula |

Para cápsula e cilindro, os sliders **raio do colisor** e **altura do colisor** (25% a 200%) ajustam em cima do
ajuste automático. São ideais para uma pessoa em pé: o modelo precisa ter o **Y para cima** (o painel avisa se o
eixo mais longo não for Y). O resumo mostra o quanto o colisor sobra em volume.

Um modelo **côncavo** (toro, xícara) colide como o seu fecho: o buraco é preenchido. A malha desenhada continua
sendo a original.

### 6.4 Limites

`.obj` até 32 MB, 400 mil vértices e 800 mil triângulos; coordenadas até 1e6; fecho com até 4096 vértices e 8192
faces; escala de 0,001 a 1000; tamanho final de 0,05 a 40 m; `.scene` até 32 MB. A malha é desenhada triângulo a
triângulo: modelos acima de ~60 mil triângulos podem ficar lentos. Em `float`, nuvens muito densas (dezenas de
milhares de vértices quase coplanares) são recusadas como “numericamente instável”: use um modelo menos detalhado
ou compile com `-DPHYSICS_USE_DOUBLE=ON`.

Sintaxe lida: `.obj` com `v`, `vt`, `f` em qualquer forma (`v`, `v/vt`, `v//vn`, `v/vt/vn`), índices negativos,
polígonos em leque, `usemtl` e `mtllib`; `.mtl` com `newmtl`, `Kd` e `map_Kd`. Erros trazem o número da linha e
nunca produzem um modelo parcial.

Modelos de exemplo em `models/`: `cubo` (textura xadrez), `esfera` (icosfera de 320 faces, listras) e `toro`
(côncavo), cada um com `.mtl` e `.png` (veja `models/LEIA-ME.txt`).

---

## 7. Formato de cena (`.scene`)

Arquivo de texto legível e editável à mão; `#` começa um comentário. A descrição completa está no cabeçalho de
`include/viewer/core/SceneFile.h`.

```text
physics_scene 1
name Minha cena
gravity 0 -9.81 0
timestep 0.0166666667
iterations 10 3              # velocidade, posição
slop 0.005 0.5               # linear [m], velocidade [m/s]
warm_start 1
sleep 1 0.5                  # ligado, tempo [s]
camera 0 1.5 0 12            # alvo x y z e distância
body                         # corpos são numerados 1, 2, 3... na ordem do arquivo
  type dynamic               # static | dynamic | kinematic
  shape box 0.5 0.5 0.5      # box | sphere r | capsule r meia-altura | cylinder r meia-altura | poly | none
  mass 1
  position 0 5 0
  orientation 1 0 0 0        # QUATÉRNIO  w x y z  (normalizado ao carregar; nulo = erro)
  velocity 0 0 0
  angular_velocity 1.2 2 0.7
  material 0.5 0.3           # atrito, restituição
  damping 0 0                # linear, angular
  gravity_scale 1
  allow_sleep 1
  color 240 150 60
  focus                      # o corpo do painel do quatérnio (no máximo um)
  model 0.5 0.1 0.2 0.0 cubo.obj   # opcional: escala, deslocamento, arquivo (relativo a models/)
end
joint hinge 1 2  ax ay az  dx dy dz  collide      # âncora (mundo), eixo, colide 0|1
joint ballsocket 1 2  ax ay az  collide
joint distance 1 2  ax ay az  bx by bz  length collide
link 1 2                     # linha desenhada entre os centros
```

- Em `shape poly`, as linhas `vertex x y z` e `face i j k ...` (índices a partir de 0, anti-horário visto de fora) descrevem o poliedro convexo.
- `shape none` é um pivô estático sem forma, desenhado como uma pequena esfera.
- A linha `model` vale com `shape box`, `sphere`, `capsule`, `cylinder` e `poly`. Ao carregar com a pasta `models/` presente, a malha volta com a textura; se o `.obj` sumiu, o corpo carrega como poliedro comum (colide igual) e a mensagem mostra um AVISO.
- **Não é salvo:** o estado de sono (a cena carrega com todos acordados) e os contatos e impulsos acumulados (*warm start*). Uma cena salva e recarregada reproduz a mesma trajetória.
- **Arquivo inválido** (número malformado, NaN/inf, quatérnio nulo, índice de corpo ou junta fora da faixa, poliedro côncavo, mais de 2000 corpos, mais de 32 MB...): a mensagem traz o número da linha e a cena atual **não é alterada**. Cenas sem nenhum corpo são válidas.
- Limite de precisão: um `.scene` gerado numa build `float` pode ser recusado por uma build `double` (a checagem de planaridade é mais estrita); o exemplo `16_Modelos_importados` é assim.

**`scene_export`** (sem raylib) regrava as cenas embutidas como `.scene` de exemplo:
`scene_export physics_viewer/scenes`.

---

## 8. Estrutura do código

```text
physics_viewer/
├── CMakeLists.txt
├── include/viewer/core/   núcleo, SEM raylib: ViewerMath (pose→matriz), OrbitCamera, Arcball, Scene,
│                          PolyShapes, ObjectSpawner, JointEditor, ConvexHull, ObjModel, ModelCollider,
│                          ModelAsset, ModelImporter, ContactTracker, TrajectoryRecorder, PanelMetrics,
│                          SceneFile, SceneLibrary, RenderPose, Interaction, TimeControl, BodyEditor,
│                          WorldEditor, DockLayout, SliderText, AppInfo
├── src/core/              implementação do núcleo
├── src/app/main.cpp       a janela (a única parte que usa raylib)
├── tools/scene_export.cpp grava as cenas embutidas como .scene (sem raylib)
├── scenes/                cenas .scene (exemplos e salvas), biblioteca.txt, viewer_layout.ini, exports/
├── models/                modelos .obj de exemplo e importados
└── tests/                 viewer_tests.cpp (unitários) e viewer_smoke.cpp (fumaça), sem janela
```

A lógica fica fora da janela de propósito: cenas, câmera, edição, importação e matemática são testadas sem tela, e
a camada raylib é fina.

Os módulos principais do núcleo:

- **`ConvexHull`** — fecho convexo 3D próprio (*quickhull*): tetraedro inicial, ponto mais distante, região visível e horizonte; triângulos vizinhos e coplanares são fundidos em polígonos. Entradas degeneradas (menos de 4 pontos, NaN, coincidentes, colineares, coplanares) dão erro com mensagem, nunca um poliedro inválido.
- **`ObjModel`** e **`ModelImporter`** — leitura do `.obj`/`.mtl` e importação com cópia segura para `models/`.
- **`ModelCollider`** — do modelo à forma de colisão: centra na caixa envolvente, calcula o fecho, centra no centroide (o corpo gira em torno do centro de massa) e devolve a forma, o deslocamento da malha original e indicadores (volume do modelo e do colisor, concavidade).
- **`Interaction`** — seleção por raycast do motor, mola de arrasto, lançamento e remoção.
- **`BodyEditor`, `WorldEditor`, `ObjectSpawner`, `JointEditor`** — edição de corpos, mundo, criação de objetos e juntas.
- **`RenderPose`** — os três modos de interpolação; **`TimeControl`** — pausa, passo e escala de tempo; **`DockLayout`** — geometria das colunas e rolagem; **`SceneFile`**, **`SceneLibrary`** — arquivo de cena e lista de cenas.

---

## 9. Testes

`ctest` roda dois testes do núcleo, sem janela:

- **`viewer_tests`** — verificações unitárias de todos os módulos do núcleo: matemática e câmera, as cenas, `RenderPose`, `Interaction`, `TimeControl`, `BodyEditor`, `ObjectSpawner`, `JointEditor`, `WorldEditor`, `ContactTracker`, `TrajectoryRecorder`, `DockLayout`, `SceneFile` (ida e volta das 14 cenas, 22 entradas inválidas), `SceneLibrary`, `ConvexHull`, `ObjModel`, `ModelCollider`, `ModelImporter` e `SliderText`.
- **`viewer_smoke`** — todas as cenas por 20 s no mesmo caminho da janela, com poses finitas e quatérnios unitários nos três modos de interpolação, mais um *fuzz* determinístico: centenas de operações aleatórias de criar, lançar, apagar, editar parâmetros, mudar o mundo, importar modelos, salvar e recarregar cenas e trocar de cena.

Os testes passam com o motor em `float` e em `double`, sem warnings (`-Wall -Wextra -Wpedantic`) e sob
AddressSanitizer + UBSan. Para também registrar a abertura da janela por 5 quadros (precisa de tela e OpenGL):
`-DVIEWER_TEST_APP=ON`.

---

## 10. Limitações

- Um modelo côncavo colide como o seu fecho convexo (o motor não tem malha estática nem forma composta).
- Só o formato `.obj` é importado.
- A cena 3D é desenhada numa textura sem o antisserrilhamento (MSAA) da janela, então as bordas ficam um pouco mais serrilhadas.
- Os sliders têm faixas fixas (posição ±12 m, y de −2 a 14 m, velocidades ±20): um valor fora da faixa continua valendo no corpo, mas aparece “preso” na borda do slider.
- Um slider só edita quando o cursor está sobre a barra com o botão apertado.
- As limitações do motor valem aqui (ver o README principal): sem colisão contínua, juntas só de igualdade, torres altas balançam.
