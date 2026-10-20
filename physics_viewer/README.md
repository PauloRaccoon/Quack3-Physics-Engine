# physics_viewer

Janela de demonstração do `../physics_engine` (C++17 + [raylib](https://www.raylib.com/)). Fica numa pasta
**ao lado** do motor, que continua sem nenhuma dependência gráfica.

> Estado: **V2** (7 cenas selecionáveis, esferas, caixas e poliedros, juntas desenhadas, reinício). O roteiro completo
> está no `README.md` da pasta acima.

## Compilar e rodar

Requisitos: CMake ≥ 3.14, um compilador C++17 e acesso ao GitHub na primeira configuração (a raylib 5.5 é
baixada e compilada automaticamente, a menos que já esteja instalada no sistema).

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
| botão direito + arrastar | girar a câmera |
| roda do mouse | zoom |
| `←` / `→` | cena anterior / próxima |
| `1` … `7` | ir direto para a cena |
| `R` | reiniciar a cena atual (mantém a câmera) |
| `P` | pausar |
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

Os poliedros são desenhados a partir da `HalfEdgeStructure` do motor (faces em leque de triângulos, com
tom pela normal no mundo, mais as arestas). As cenas ficam em `src/core/Scene.cpp`; para criar outra,
escreva uma função `makeXScene()` e acrescente-a em `sceneList()`.

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
├── include/viewer/core/   ViewerMath (pose→matriz), OrbitCamera, Scene   — SEM raylib
├── src/core/
├── src/app/main.cpp       a janela (única parte que usa raylib)
└── tests/viewer_tests.cpp testes do núcleo, sem janela
```

A lógica fica fora da janela de propósito: cenas, câmera e matemática são testadas sem tela, e a camada
raylib é fina.

## O que foi verificado

- Núcleo e testes (123 verificações: matemática, câmera, as 7 cenas rodando 300 passos, pilha/pirâmide de pé, juntas coesas): `float` e `double`, sem warnings (`-Wall -Wextra -Wpedantic`) e sob ASan+UBSan.
- `main.cpp`: passa na verificação de tipos contra os cabeçalhos da raylib 5.5, sem warnings, em `float` e
  `double`. A convenção de matriz foi conferida no código-fonte da raylib (campo `mN` = elemento `N` do
  array passado a `rlMultMatrixf`).
- **Não verificado:** a janela em si (linkar, abrir e desenhar). O ambiente de desenvolvimento não tinha as
  bibliotecas X11/OpenGL. Na V2, confira também se os poliedros aparecem sólidos (não furados nem invertidos) e se as linhas das juntas ligam os corpos certos. Quem compila na primeira vez deve conferir se a caixa cai girando, se os eixos
  coloridos acompanham a caixa e se o `|q|` fica em 1.000000.
