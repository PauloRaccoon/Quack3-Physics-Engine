// =============================================================================
// HalfEdgeStructure.cpp
// =============================================================================
#include "physics/geometry/HalfEdgeStructure.h"

#include <unordered_map>

namespace physics {

namespace {
/// Chave 64-bit de uma aresta direcionada (origem → destino).
inline std::uint64_t edgeKey(std::int32_t from, std::int32_t to) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(from)) << 32) |
           static_cast<std::uint32_t>(to);
}
} // namespace

void HalfEdgeStructure::clear() {
    vertices_.clear();
    edges_.clear();
    faces_.clear();
}

bool HalfEdgeStructure::build(const std::vector<Vector3>& positions,
                              const std::vector<std::vector<int>>& faceIndices,
                              std::string* error) {
    clear();
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        clear();
        return false;
    };

    const int vertexCount = static_cast<int>(positions.size());
    if (vertexCount < 4 || faceIndices.size() < 4) {
        return fail("poliedro precisa de ao menos 4 vertices e 4 faces");
    }

    // --- Vértices ---------------------------------------------------------------
    vertices_.resize(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        if (!positions[i].isFinite()) return fail("vertice " + std::to_string(i) + " nao e finito");
        vertices_[i].position = positions[i];
    }

    // --- Half-edges e faces ---------------------------------------------------------
    std::size_t totalEdges = 0;
    for (const auto& f : faceIndices) totalEdges += f.size();
    edges_.reserve(totalEdges);
    faces_.resize(faceIndices.size());

    std::unordered_map<std::uint64_t, std::int32_t> lookup;
    lookup.reserve(totalEdges * 2);

    for (std::size_t f = 0; f < faceIndices.size(); ++f) {
        const std::vector<int>& idx = faceIndices[f];
        const int k = static_cast<int>(idx.size());
        if (k < 3) return fail("face " + std::to_string(f) + " tem menos de 3 vertices");

        for (int j = 0; j < k; ++j) {
            if (idx[j] < 0 || idx[j] >= vertexCount) {
                return fail("face " + std::to_string(f) + " usa indice de vertice invalido");
            }
            for (int l = j + 1; l < k; ++l) {
                if (idx[j] == idx[l]) {
                    return fail("face " + std::to_string(f) + " repete um vertice");
                }
            }
        }

        const std::int32_t first = static_cast<std::int32_t>(edges_.size());
        faces_[f].edge = first;
        faces_[f].vertexCount = k;

        for (int j = 0; j < k; ++j) {
            const std::int32_t id = first + j;
            HalfEdge he;
            he.origin = idx[j];
            he.next = first + (j + 1) % k;
            he.prev = first + (j + k - 1) % k;
            he.face = static_cast<std::int32_t>(f);
            edges_.push_back(he);

            const std::int32_t dest = idx[(j + 1) % k];
            if (!lookup.emplace(edgeKey(he.origin, dest), id).second) {
                return fail("aresta " + std::to_string(he.origin) + "->" + std::to_string(dest) +
                            " usada duas vezes no mesmo sentido (orientacao inconsistente "
                            "ou malha nao-manifold)");
            }
            if (vertices_[static_cast<std::size_t>(he.origin)].edge < 0) {
                vertices_[static_cast<std::size_t>(he.origin)].edge = id;
            }
        }
    }

    // --- Gêmeas -------------------------------------------------------------------------
    for (std::size_t e = 0; e < edges_.size(); ++e) {
        const std::int32_t from = edges_[e].origin;
        const std::int32_t to = edges_[static_cast<std::size_t>(edges_[e].next)].origin;
        const auto it = lookup.find(edgeKey(to, from));
        if (it == lookup.end()) {
            return fail("malha aberta: a aresta " + std::to_string(from) + "->" +
                        std::to_string(to) + " nao tem gemea");
        }
        edges_[e].twin = it->second;
    }

    for (std::size_t v = 0; v < vertices_.size(); ++v) {
        if (vertices_[v].edge < 0) return fail("vertice " + std::to_string(v) + " nao pertence a nenhuma face");
    }

    // --- Normais (método de Newell: robusto para polígonos não-triangulares) -----------------
    for (std::size_t f = 0; f < faces_.size(); ++f) {
        Vector3 n;
        Vector3 c;
        const int start = faces_[f].edge;
        int e = start;
        do {
            const Vector3& p = vertices_[static_cast<std::size_t>(edges_[static_cast<std::size_t>(e)].origin)].position;
            const int nextE = edges_[static_cast<std::size_t>(e)].next;
            const Vector3& q = vertices_[static_cast<std::size_t>(edges_[static_cast<std::size_t>(nextE)].origin)].position;
            n.x += (p.y - q.y) * (p.z + q.z);
            n.y += (p.z - q.z) * (p.x + q.x);
            n.z += (p.x - q.x) * (p.y + q.y);
            c += p;
            e = nextE;
        } while (e != start);

        if (n.normalize() <= Real(0)) return fail("face " + std::to_string(f) + " e degenerada (area nula)");
        faces_[f].normal = n;
        faces_[f].offset = n.dot(c / static_cast<Real>(faces_[f].vertexCount));
    }

    // --- Característica de Euler: V - E + F = 2 (esfera topológica) ------------------------------
    const long long V = vertexCount;
    const long long E = static_cast<long long>(edges_.size()) / 2;
    const long long F = static_cast<long long>(faces_.size());
    if (V - E + F != 2) {
        return fail("caracteristica de Euler = " + std::to_string(V - E + F) +
                    " (esperado 2): a malha nao e um poliedro fechado de genero 0");
    }
    return true;
}

bool HalfEdgeStructure::isConvexAndPlanar(Real tolerance) const {
    for (const HEFace& face : faces_) {
        // Nenhum vértice pode estar à frente do plano da face.
        for (const HEVertex& v : vertices_) {
            if (face.normal.dot(v.position) - face.offset > tolerance) return false;
        }
    }
    // Vértices da própria face devem estar (quase) sobre o plano (planaridade).
    for (int f = 0; f < getFaceCount(); ++f) {
        const HEFace& face = getFace(f);
        bool planar = true;
        forEachFaceVertex(f, [&](int v) {
            if (std::abs(face.normal.dot(getVertex(v).position) - face.offset) > tolerance) planar = false;
        });
        if (!planar) return false;
    }
    return true;
}

bool HalfEdgeStructure::validate(std::string* error) const {
    auto fail = [&](const std::string& m) {
        if (error) *error = m;
        return false;
    };
    const int n = getEdgeCount();
    for (int e = 0; e < n; ++e) {
        const HalfEdge& he = getEdge(e);
        if (he.twin < 0 || he.twin >= n || getEdge(he.twin).twin != e) return fail("twin inconsistente em " + std::to_string(e));
        if (getEdge(he.next).prev != e || getEdge(he.prev).next != e) return fail("next/prev inconsistente em " + std::to_string(e));
        if (getEdge(he.next).face != he.face) return fail("face inconsistente em " + std::to_string(e));
        if (getEdge(he.twin).origin != destination(e)) return fail("origem da gemea inconsistente em " + std::to_string(e));
    }
    return true;
}

int HalfEdgeStructure::findSupportVertex(const Vector3& direction, int startVertex) const {
    const int n = getVertexCount();
    if (n == 0) return -1;
    int best = startVertex < 0 ? 0 : (startVertex >= n ? n - 1 : startVertex);
    Real bestDot = direction.dot(getVertex(best).position);

    // Cada passo melhora ESTRITAMENTE o valor, portanto termina; em NaN as
    // comparações são falsas e a busca para imediatamente.
    for (;;) {
        bool improved = false;
        const int first = getVertex(best).edge;
        int e = first;
        do {
            const int w = destination(e);
            const Real d = direction.dot(getVertex(w).position);
            if (d > bestDot) {
                bestDot = d;
                best = w;
                improved = true;
                break;
            }
            e = getEdge(getEdge(e).twin).next;
        } while (e != first);
        if (!improved) return best;
    }
}

int HalfEdgeStructure::findBestFace(const Vector3& direction) const {
    int best = 0;
    Real bestDot = -kRealMax;
    for (int f = 0; f < getFaceCount(); ++f) {
        const Real d = direction.dot(getFace(f).normal);
        if (d > bestDot) {
            bestDot = d;
            best = f;
        }
    }
    return best;
}

} // namespace physics
