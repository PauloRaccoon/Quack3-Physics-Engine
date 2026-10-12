// =============================================================================
// HalfEdgeStructure.h
// Estrutura de dados half-edge para poliedros convexos fechados.
//
//   Vértice  → posição + uma half-edge que SAI dele
//   HalfEdge → origem, gêmea (twin), próxima/anterior na face, face à esquerda
//   Face     → normal unitária (para fora), offset do plano e uma half-edge
//
// Convenção: as faces são listadas em sentido ANTI-HORÁRIO visto de fora
// (regra da mão direita aponta para fora). Todos os "ponteiros" são índices
// int32 em vetores contíguos (boa localidade de cache, sem alocações após build).
// =============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "physics/common/Settings.h"
#include "physics/math/Vector3.h"

namespace physics {

struct HalfEdge {
    std::int32_t origin = -1;  ///< vértice de onde a aresta sai
    std::int32_t twin = -1;    ///< half-edge oposta (outra face)
    std::int32_t next = -1;    ///< próxima half-edge na mesma face (anti-horário)
    std::int32_t prev = -1;    ///< anterior na mesma face
    std::int32_t face = -1;    ///< face à esquerda da half-edge
};

struct HEVertex {
    Vector3 position;
    std::int32_t edge = -1;  ///< uma half-edge que sai deste vértice
};

struct HEFace {
    Vector3 normal;          ///< normal unitária apontando para fora
    Real offset = Real(0);   ///< plano: dot(normal, p) = offset
    std::int32_t edge = -1;  ///< uma half-edge da face
    std::int32_t vertexCount = 0;
};

class HalfEdgeStructure {
public:
    /// Constrói a topologia a partir de vértices e faces (listas de índices de
    /// vértices em sentido anti-horário visto de fora). Retorna false e preenche
    /// `error` (se não nulo) quando a malha é inválida: índice fora do limite,
    /// face com < 3 vértices ou repetidos, aresta usada duas vezes no mesmo
    /// sentido (orientação inconsistente), malha aberta, vértice sem face,
    /// face degenerada ou característica de Euler ≠ 2.
    bool build(const std::vector<Vector3>& positions,
               const std::vector<std::vector<int>>& faces,
               std::string* error = nullptr);

    void clear();
    bool isEmpty() const { return vertices_.empty(); }

    /// Verifica convexidade (todos os vértices atrás de todos os planos) e
    /// planaridade das faces dentro da tolerância absoluta `tolerance`.
    /// Também detecta orientação invertida (vértices na frente dos planos).
    bool isConvexAndPlanar(Real tolerance) const;

    /// Checagem de consistência interna (twin/next/prev/face).
    bool validate(std::string* error = nullptr) const;

    // --- Acesso -------------------------------------------------------------------
    int getVertexCount() const { return static_cast<int>(vertices_.size()); }
    int getEdgeCount() const { return static_cast<int>(edges_.size()); }  ///< half-edges
    int getFaceCount() const { return static_cast<int>(faces_.size()); }

    const HEVertex& getVertex(int i) const { return vertices_[static_cast<std::size_t>(i)]; }
    const HalfEdge& getEdge(int i) const { return edges_[static_cast<std::size_t>(i)]; }
    const HEFace& getFace(int i) const { return faces_[static_cast<std::size_t>(i)]; }

    const std::vector<HEVertex>& vertices() const { return vertices_; }
    const std::vector<HalfEdge>& edges() const { return edges_; }
    const std::vector<HEFace>& faces() const { return faces_; }

    /// Vértice de destino da half-edge.
    int destination(int edge) const { return getEdge(getEdge(edge).next).origin; }

    /// Vetor da aresta (destino - origem).
    Vector3 edgeVector(int edge) const {
        return getVertex(destination(edge)).position - getVertex(getEdge(edge).origin).position;
    }

    // --- Iteração topológica (sem alocação) ---------------------------------------------
    /// Chama f(vertexIndex) para cada vértice da face, em ordem anti-horária.
    template <typename F>
    void forEachFaceVertex(int face, F&& f) const {
        const int start = getFace(face).edge;
        int e = start;
        do {
            f(getEdge(e).origin);
            e = getEdge(e).next;
        } while (e != start);
    }

    /// Chama f(neighborVertexIndex) para cada vizinho do vértice.
    template <typename F>
    void forEachVertexNeighbor(int vertex, F&& f) const {
        const int start = getVertex(vertex).edge;
        int e = start;
        do {
            f(destination(e));
            e = getEdge(getEdge(e).twin).next;  // próxima half-edge saindo do vértice
        } while (e != start);
    }

    // --- Consultas geométricas -----------------------------------------------------------
    /// Vértice mais distante em `direction`, por "hill climbing" na vizinhança
    /// a partir de `startVertex` (válido porque o poliedro é convexo: um
    /// máximo local da função linear é o máximo global). Com coerência
    /// temporal (start = resultado do quadro anterior) custa O(1) amortizado.
    int findSupportVertex(const Vector3& direction, int startVertex = 0) const;

    /// Face cuja normal é mais alinhada a `direction` (maior dot).
    int findBestFace(const Vector3& direction) const;

private:
    std::vector<HEVertex> vertices_;
    std::vector<HalfEdge> edges_;
    std::vector<HEFace> faces_;
};

} // namespace physics
