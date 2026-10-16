// =============================================================================
// Settings.h
// Configurações globais do motor: tipo escalar, constantes numéricas e
// parâmetros padrão da simulação (usados nas fases seguintes).
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace physics {

// -----------------------------------------------------------------------------
// Precisão numérica. Por padrão usa float (melhor para cache e SIMD futuro).
// Compile com -DPHYSICS_USE_DOUBLE para usar double em todo o motor.
// -----------------------------------------------------------------------------
#ifdef PHYSICS_USE_DOUBLE
using Real = double;
#else
using Real = float;
#endif

namespace detail {
constexpr bool kIsSinglePrecision = (sizeof(Real) == 4);
}

// -----------------------------------------------------------------------------
// Constantes matemáticas
// -----------------------------------------------------------------------------
constexpr Real kPi     = Real(3.14159265358979323846);
constexpr Real kTwoPi  = Real(2) * kPi;
constexpr Real kHalfPi = Real(0.5) * kPi;

// -----------------------------------------------------------------------------
// Tolerâncias numéricas
// -----------------------------------------------------------------------------

/// Épsilon da máquina para Real.
constexpr Real kEpsilon = std::numeric_limits<Real>::epsilon();

/// Maior valor finito representável.
constexpr Real kRealMax = std::numeric_limits<Real>::max();

/// Abaixo deste valor |d| um divisor é tratado como zero (ver safeInverse).
constexpr Real kDivideEpsilon = detail::kIsSinglePrecision ? Real(1e-12) : Real(1e-24);

/// Comprimento^2 mínimo para um vetor/quatérnio poder ser normalizado.
constexpr Real kNormalizeEpsilonSq = detail::kIsSinglePrecision ? Real(1e-18) : Real(1e-36);

/// Tolerância relativa usada para decidir se uma matriz 3x3 é singular.
constexpr Real kSingularEpsilon = Real(16) * kEpsilon;

/// Tolerância genérica para comparações "quase igual".
constexpr Real kCompareEpsilon = detail::kIsSinglePrecision ? Real(1e-5) : Real(1e-10);

// -----------------------------------------------------------------------------
// Parâmetros padrão da simulação (consumidos nas fases 3-7)
// -----------------------------------------------------------------------------
constexpr Real kFixedTimeStep = Real(1) / Real(60);   ///< Δt fixo da física
constexpr int  kMaxSubSteps   = 8;                    ///< Limite anti "espiral da morte"

constexpr Real kLinearSlop      = Real(0.005);        ///< Penetration slop [m]
constexpr Real kVelocitySlop    = Real(0.5);          ///< Velocity slop (restituição) [m/s]
constexpr Real kBaumgarteFactor = Real(0.2);          ///< Fator de correção de posição (bias)
constexpr int  kVelocityIterations = 10;              ///< Iterações do Sequential Impulse
constexpr Real kAABBMargin      = Real(0.1);          ///< Margem dos Fat AABBs [m]
constexpr Real kAABBDisplacementMultiplier = Real(2); ///< Antecipação do movimento no Fat AABB

constexpr Real kGravity         = Real(9.81);         ///< Aceleração da gravidade [m/s²]
constexpr Real kMaxLinearSpeed  = Real(200);          ///< Limite de velocidade linear [m/s]
constexpr Real kMaxAngularSpeed = Real(100);          ///< Limite de velocidade angular [rad/s]

// -----------------------------------------------------------------------------
// Narrowphase (GJK / EPA / contatos)
// -----------------------------------------------------------------------------
constexpr int  kGJKMaxIterations = 64;
/// Critério de convergência do GJK (relativo): |v|² - v·w <= tol * escala.
constexpr Real kGJKTolerance = detail::kIsSinglePrecision ? Real(1e-6) : Real(1e-12);
/// |v|² abaixo disto => a origem está sobre o simplex (formas se tocam/interpenetram).
constexpr Real kGJKOriginEpsilonSq = detail::kIsSinglePrecision ? Real(1e-10) : Real(1e-20);

constexpr int  kEPAMaxIterations = 48;
/// EPA termina quando o suporte avança menos que isto além da face mais próxima [m].
constexpr Real kEPATolerance = detail::kIsSinglePrecision ? Real(1e-4) : Real(1e-8);
/// Uma face é "visível" do novo ponto se este estiver à frente do plano por mais que isto.
constexpr Real kEPAVisibleEpsilon = detail::kIsSinglePrecision ? Real(1e-5) : Real(1e-10);

/// Raio [m] usado para (a) casar novos contatos com contatos persistidos e
/// (b) descartar contatos persistidos que se afastaram ou deslizaram.
constexpr Real kContactBreakingThreshold = Real(0.02);

constexpr Real kContactMargin   = kLinearSlop;        ///< Contatos "especulativos": separação ≤ margem ainda gera contato
constexpr Real kPersistentContactThreshold = Real(0.05); ///< Deriva máx. [m] para reaproveitar impulsos (warm start)

} // namespace physics
