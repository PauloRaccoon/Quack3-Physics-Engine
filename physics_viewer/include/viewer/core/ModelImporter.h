// =============================================================================
// ModelImporter.h  (V17)
// Traz um modelo 3D do disco do usuário (qualquer pasta) para a pasta de modelos do viewer (`models/`), onde a lista
// da janela e os arquivos `.scene` o encontram. Sem raylib: a janela só chama importModel() com o caminho recebido do
// arrastar-e-soltar ou digitado.
//
// O que é copiado (para `models/<nome>/`): o `.obj`, os `.mtl` que ele cita (`mtllib`) e as texturas `map_Kd` desses
// materiais (só as que o modelo usa). Nada é sobrescrito: se `models/<nome>/` já existe, vira `<nome>_2`, `<nome>_3`...
// Segurança: caminhos com `..`, absolutos ou com unidade são recusados; só extensões de imagem conhecidas; tamanho
// máximo por arquivo e número máximo de arquivos.
// V25: o .mtl e as texturas são procurados com tolerância (outro nome de arquivo, caminho de outro computador como
// "C:\\...\\x.mtl", maiúsculas, .obj sem "mtllib"); quando o nome citado não bate com o arquivo achado, o .obj e o .mtl copiados
// são reescritos para apontar certo, e `notes` conta o que foi feito. O modelo é lido e validado ANTES de copiar (arquivo inválido não
// deixa lixo na pasta).
// =============================================================================
#pragma once

#include <string>
#include <vector>

namespace viewer {

namespace importlimits {
constexpr std::size_t kMaxFileBytes = 32u * 1024u * 1024u;   ///< por arquivo (.obj, .mtl ou textura)
constexpr int kMaxFiles = 64;                                ///< arquivos copiados por modelo
constexpr int kMaxFolderObjs = 20;                           ///< .obj importados de uma pasta arrastada
} // namespace importlimits

struct ImportResult {
    std::string file;                 ///< chave para a lista: "<nome>/<nome>.obj" (relativo à pasta de modelos)
    int copied = 0;                   ///< quantos arquivos foram copiados
    int mtlCount = 0;                 ///< V25: quantos .mtl foram copiados
    int textureCount = 0;             ///< V25: quantas texturas foram copiadas
    std::vector<std::string> notes;   ///< avisos (textura ou .mtl ausente, caminho inseguro ignorado...)
};

/// Nome de pasta/arquivo seguro: letras e dígitos ASCII, `-` e `_` (espaço vira `_`), até 40 caracteres; vazio -> "modelo".
std::string sanitizeModelName(const std::string& typed);

/// Importa `srcObjPath` (um `.obj`) para `modelDir`. false + `error` se: não existe, não é `.obj`, grande demais,
/// sintaxe inválida, sem triângulos, pasta de destino impossível de criar. Em caso de erro NADA fica copiado.
bool importModel(const std::string& srcObjPath, const std::string& modelDir, ImportResult& out, std::string& error);

/// Importa o que o usuário soltou/digitou: um `.obj`, ou uma PASTA (importa os `.obj` dela, um nível de subpasta).
/// Devolve os modelos importados; `messages` recebe uma linha por problema (arquivo ignorado, erro). Retorna
/// true se importou ao menos um modelo.
bool importPath(const std::string& path, const std::string& modelDir, std::vector<ImportResult>& imported,
                std::vector<std::string>& messages);

} // namespace viewer
