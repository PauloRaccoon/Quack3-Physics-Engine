// =============================================================================
// ContactTracker.h
// Eventos de contato do motor transformados em efeitos visuais (sem raylib, testável):
//   - flash por corpo no INÍCIO de um contato, proporcional à velocidade de impacto;
//   - marcadores (anéis) no ponto do impacto;
//   - contadores (inícios, fins, impacto mais forte).
// Também: tinta por estado (dormindo = azulado; flash = branco-amarelado).
//
// É um IContactListener (push): cada evento chega UMA vez, mesmo em quadros sem passo físico
// (world.getContactEvents() repetiria o buffer nesses quadros).
// =============================================================================
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "physics/common/Types.h"
#include "physics/pipeline/ContactEvents.h"

namespace viewer {

struct Rgb {
    unsigned char r = 0, g = 0, b = 0;
};

/// Cor de um corpo: `sleeping` mistura com azul (corpo adormecido); `flash` (0..1) mistura com
/// branco-amarelado (impacto recente). Entradas fora de faixa/NaN são limitadas.
Rgb stateTint(Rgb base, bool sleeping, float flash);

class ContactTracker : public physics::IContactListener {
public:
    static constexpr float kFullSpeed = 8.0f;     ///< m/s: impacto que dá flash máximo
    static constexpr float kMinSpeed = 0.3f;      ///< m/s: abaixo disso é repouso/ruído, sem flash
    static constexpr float kFlashSeconds = 0.4f;  ///< duração do flash máximo
    static constexpr std::size_t kMaxMarkers = 32;

    struct Marker {
        physics::Vector3 point;
        float intensity = 0.0f;   ///< 0..1
        float age = 0.0f;         ///< s; some ao passar de kFlashSeconds
    };

    /// Intensidade 0..1 para uma velocidade de aproximação (0 abaixo de kMinSpeed; NaN -> 0).
    static float intensityFromSpeed(float approachSpeed);

    void onContactBegin(const physics::ContactEvent& e) override;
    void onContactEnd(const physics::ContactEvent& e) override;

    /// Avança `dt` segundos (tempo físico): decai flashes e marcadores. dt <= 0 ou NaN: nada.
    void update(float dt);
    /// Esquece tudo (nova cena).
    void reset();

    float flashOf(physics::BodyId id) const;   ///< 0..1
    const std::vector<Marker>& markers() const { return markers_; }

    std::uint64_t begins() const { return begins_; }
    std::uint64_t ends() const { return ends_; }
    float lastImpact() const { return lastImpact_; }   ///< m/s do último início com flash
    float maxImpact() const { return maxImpact_; }     ///< m/s, maior desde o reset

private:
    void addFlash(physics::BodyId id, float intensity);

    std::vector<std::pair<physics::BodyId, float>> flashes_;
    std::vector<Marker> markers_;
    std::uint64_t begins_ = 0, ends_ = 0;
    float lastImpact_ = 0.0f, maxImpact_ = 0.0f;
};

} // namespace viewer
