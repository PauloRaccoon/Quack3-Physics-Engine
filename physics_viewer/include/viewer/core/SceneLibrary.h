// =============================================================================
// SceneLibrary.h  (V22)
// A lista ÚNICA de cenas do programa: as cenas originais (embutidas em sceneList()) e os arquivos .scene salvos
// na pasta de cenas. As setas e os números do teclado percorrem essa lista, na ordem dela. O usuário pode
// reordenar e apagar; a ordem e as originais escondidas ficam num arquivo de texto (biblioteca.txt) e voltam
// na próxima abertura. Sem raylib, para ser testada.
//
// REGRAS
//   * Primeira abertura (sem biblioteca.txt): as originais, na ordem de sceneList(), e depois os arquivos .scene
//     em ordem alfabética. Arquivos que são só a EXPORTAÇÃO de uma original (nome "NN_" + nome da original, como
//     os exemplos de scene_export) não entram, para a cena não aparecer duas vezes.
//   * Com biblioteca.txt: a ordem salva vale. Arquivos novos (salvos depois) entram no fim; arquivos que sumiram
//     da pasta saem; originais novas de versões futuras entram no fim; originais apagadas ficam escondidas.
//   * Apagar uma ORIGINAL só a esconde (restoreBuiltins() traz de volta); apagar um ARQUIVO é decisão de quem
//     chama (a janela apaga o arquivo do disco depois de pedir confirmação).
// =============================================================================
#pragma once

#include <cstddef>
#include <set>
#include <string>
#include <vector>

#include "viewer/core/Scene.h"

namespace viewer {

struct LibraryEntry {
    enum class Kind { Builtin, File };
    Kind kind = Kind::Builtin;
    int builtinIndex = -1;   ///< Builtin: índice em sceneList()
    std::string file;        ///< File: nome do arquivo sem a extensão
    std::string name;        ///< o que aparece na lista
    bool broken = false;     ///< a última tentativa de carregar falhou
};

namespace scenelibrary {
constexpr const char* kFileName = "biblioteca.txt";
constexpr const char* kHeader = "physics_scene_library 1";
constexpr std::size_t kMaxEntries = 500;
constexpr std::size_t kMaxBytes = 256u * 1024u;
} // namespace scenelibrary

/// `file` (sem extensão) é a exportação de uma cena original? ("07_Formas_poliedros" para "Formas (poliedros)").
bool isBuiltinExportName(const std::string& file);

class SceneLibrary {
public:
    /// Monta a lista a partir dos arquivos existentes e do texto salvo (vazio = primeira abertura).
    void rebuild(const std::vector<std::string>& files, const std::string& savedText);

    std::size_t size() const { return entries_.size(); }
    bool empty() const { return entries_.empty(); }
    const LibraryEntry& at(std::size_t i) const { return entries_.at(i); }
    LibraryEntry& at(std::size_t i) { return entries_.at(i); }
    const std::vector<LibraryEntry>& entries() const { return entries_; }

    /// Procura: -1 se não houver.
    int findFile(const std::string& file) const;
    int findBuiltin(int builtinIndex) const;

    /// Troca com o vizinho. false se já está na ponta ou o índice é inválido. Devolve o novo índice em `newIndex`.
    bool moveUp(std::size_t i, std::size_t* newIndex = nullptr);
    bool moveDown(std::size_t i, std::size_t* newIndex = nullptr);

    /// Tira da lista. Original: fica escondida. Recusa (false) se for a última entrada. `removed` recebe a entrada.
    bool remove(std::size_t i, LibraryEntry* removed = nullptr);

    /// Acrescenta um arquivo salvo no fim (se já existe, só devolve o índice). Devolve o índice.
    std::size_t addFile(const std::string& file);

    /// Traz de volta as originais escondidas, no fim da lista. Devolve quantas voltaram.
    std::size_t restoreBuiltins();
    std::size_t hiddenBuiltinCount() const { return hidden_.size(); }

    /// Texto para salvar em biblioteca.txt.
    std::string serialize() const;

private:
    std::vector<LibraryEntry> entries_;
    std::set<int> hidden_;   ///< originais apagadas
};

/// Cria a cena da entrada: original -> fábrica; arquivo -> lê `sceneDir/<file>.scene` (modelos em `modelDir`).
/// `warning` (opcional) recebe avisos de carregamento. false e `error` se falhar (o arquivo some ou é inválido).
bool buildLibraryScene(const LibraryEntry& entry, const std::string& sceneDir, const std::string& modelDir, Scene& out,
                       std::string& error, std::string* warning = nullptr);

} // namespace viewer
