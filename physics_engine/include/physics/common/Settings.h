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
/// Atrito de TORÇÃO: o giro em torno da normal do contato pára com torque máx. = kSpinFriction · μ · Σλn
/// (um ponto de contato não tem braço de alavanca: sem isso uma esfera girando em Y nunca pára). [m]
constexpr Real kSpinFriction    = Real(0.03);
/// Resistência à ROLAGEM (formas redondas): torque máx. = kRollingFriction · μ · Σλn. Sem ela uma bola rola
/// para sempre (deformação do contato real). [m]
constexpr Real kRollingFriction = Real(0.02);
constexpr Real kBaumgarteFactor = Real(0.2);          ///< Fator de correção de posição (bias)
constexpr int  kVelocityIterations = 10;              ///< Iterações do Sequential Impulse
constexpr int  kPositionIterations = 3;               ///< Iterações da correção de posição (NGS); 0 = Baumgarte legado
constexpr Real kMaxLinearCorrection  = Real(0.2);     ///< Correção linear máxima por iteração de posição [m]
constexpr Real kMaxAngularCorrection = Real(0.1);     ///< Correção angular máxima por iteração de posição [rad]
constexpr Real kAABBMargin      = Real(0.1);          ///< Margem dos Fat AABBs [m]
constexpr Real kAABBDisplacementMultiplier = Real(2); ///< Antecipação do movimento no Fat AABB

constexpr Real kGravity         = Real(9.81);         ///< Aceleração da gravidade [m/s²]
constexpr Real kMaxLinearSpeed  = Real(200);          ///< Limite de velocidade linear [m/s]
constexpr Real kMaxAngularSpeed = Real(200);          ///< Limite de velocidade angular [rad/s] (V25: 100 -> 200, para piões e giroscópios rápidos)

/// Dimensões de forma suportadas [m] (raio da esfera, semi-extents da caixa). Abaixo do mínimo
/// a área das faces underflowa/degenera; acima do máximo, volume e inércia estouram em float.
/// Fora do intervalo as formas lançam std::invalid_argument.
constexpr Real kMinShapeExtent  = Real(1e-4);
constexpr Real kMaxShapeExtent  = Real(1e5);
/// Teto do coeficiente de atrito (a mistura sqrt(μa·μb) com valores astronômicos daria inf).
constexpr Real kMaxFriction     = Real(1e3);
/// |coordenada| máxima suportada [m] para posições, centros de massa e offsets locais. Além disso
/// (a precisão de float já é de ~0,06 m a 1e6) somas e rotações podem estourar para Inf. Poses e
/// massas fora do limite são recusadas (invalid_argument) ou ignoradas (setters).
constexpr Real kMaxCoordinate   = Real(1e6);

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
