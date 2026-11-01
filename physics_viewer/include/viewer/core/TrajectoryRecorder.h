// =============================================================================
// TrajectoryRecorder.h  (V16)
// Grava a trajetória de corpos da cena e exporta em CSV (para gráficos no Excel, Python, MATLAB...). Sem raylib.
//
// Uma linha por corpo a cada quadro de render em que a física avançou (`passo` diz o número do passo físico; se
// vários passos rodaram no mesmo quadro, a linha é o estado depois do último). Colunas:
//   t [s]            tempo físico desde o início da gravação (passos × Δt)
//   passo            número do passo físico do mundo
//   corpo            BodyId
//   x y z            posição do referencial do corpo [m]
//   qw qx qy qz      o QUATÉRNIO de orientação (estado do motor, sem interpolação)
//   norma_q          |q| (deve ficar em 1: o motor normaliza a cada passo)
//   eixo_x/y/z       eixo da rotação equivalente (unitário)
//   angulo_graus     ângulo da rotação equivalente [0, 180]
//   yaw/pitch/roll_graus   Euler YXZ (o mesmo do painel do quatérnio; mostra o gimbal lock)
//   vx vy vz         velocidade linear [m/s]
//   wx wy wz         velocidade angular no mundo [rad/s]
// Separador vírgula, ponto decimal, números com dígitos suficientes para voltar ao mesmo valor.
// =============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "physics/common/Types.h"
#include "viewer/core/Scene.h"

namespace viewer {

enum class RecordTarget { Selected = 0, AllDynamic = 1 };

struct TrajectoryRow {
    double t = 0;
    std::uint64_t step = 0;
    physics::BodyId body = physics::kInvalidBodyId;
    double p[3] = {0, 0, 0};
    double q[4] = {1, 0, 0, 0};   ///< w x y z
    double qNorm = 1;
    double axis[3] = {1, 0, 0};
    double angleDeg = 0;
    double eulerDeg[3] = {0, 0, 0};   ///< yaw pitch roll
    double v[3] = {0, 0, 0};
    double w[3] = {0, 0, 0};
};

class TrajectoryRecorder {
public:
    static constexpr std::size_t kMaxRows = 500000;
    static constexpr std::size_t kMaxBodiesPerFrame = 200;   ///< no modo "todos", os primeiros 200 corpos dinâmicos

    /// Começa uma gravação nova (apaga as linhas anteriores) e já grava a linha do instante inicial (t = 0).
    /// Selected: o corpo `selected` (fixado agora; precisa existir e ser dinâmico). AllDynamic: todos os dinâmicos
    /// (os que existirem em cada quadro). Falha (false, `error`) se não houver o que gravar.
    bool start(const Scene& scene, RecordTarget target, physics::BodyId selected, std::string* error = nullptr);

    /// Chame uma vez por quadro de render, depois de avançar a física. Grava se o passo do mundo mudou desde a última
    /// chamada. Para sozinha (e `stopReason()` diz por quê) quando: o contador de passos voltou atrás (cena reiniciada),
    /// o corpo gravado deixou de existir ou o limite de linhas foi atingido. Devolve quantas linhas entraram.
    std::size_t capture(const Scene& scene);

    void stop(const char* reason = "parada pelo usuario") {
        if (recording_) { recording_ = false; stopReason_ = reason; }
    }
    void clear() { rows_.clear(); recording_ = false; stopReason_.clear(); time_ = 0; }

    bool recording() const { return recording_; }
    RecordTarget target() const { return target_; }
    physics::BodyId body() const { return body_; }
    const std::string& stopReason() const { return stopReason_; }
    const std::vector<TrajectoryRow>& rows() const { return rows_; }
    std::size_t rowCount() const { return rows_.size(); }
    double duration() const { return time_; }

    /// CSV completo (cabeçalho + linhas). Vazio de linhas: só o cabeçalho.
    std::string toCsv() const;
    /// Grava o CSV em `path` (cria as pastas que faltam). false + `error` se não conseguir.
    bool save(const std::string& path, std::string& error) const;

    /// Cabeçalho do CSV (uma linha, sem quebra no fim).
    static std::string header();

private:
    void appendRow(const Scene& scene, physics::BodyId id);

    bool recording_ = false;
    RecordTarget target_ = RecordTarget::Selected;
    physics::BodyId body_ = physics::kInvalidBodyId;
    std::uint64_t lastStep_ = 0;
    double time_ = 0;
    std::string stopReason_;
    std::vector<TrajectoryRow> rows_;
};

} // namespace viewer
