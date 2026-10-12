// =============================================================================
// math_tests.cpp — testes unitários da Fase 1 (sem frameworks externos).
// =============================================================================
#include <cstdio>
#include <limits>

#include "physics/math/Matrix3.h"
#include "physics/math/Quaternion.h"
#include "physics/math/Transform.h"
#include "physics/math/Vector3.h"

using namespace physics;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++g_checks;                                                              \
        if (!(cond)) {                                                           \
            ++g_failures;                                                        \
            std::printf("FALHOU %s:%d  %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                        \
    } while (0)

static const Real kTol = Real(1e-4);  // tolerância confortável para float

static void testVector3() {
    Vector3 a(1, 2, 3), b(4, -5, 6);
    CHECK(nearlyEqual(dot(a, b), Real(12)));
    CHECK(cross(a, b).nearlyEqual(Vector3(27, 6, -13)));
    CHECK(cross(a, b).dot(a) == 0 || nearlyZero(cross(a, b).dot(a), kTol));
    CHECK((a + b).nearlyEqual(Vector3(5, -3, 9)));
    CHECK((a - b).nearlyEqual(Vector3(-3, 7, -3)));
    CHECK((2 * a).nearlyEqual(Vector3(2, 4, 6)));
    CHECK((a / 2).nearlyEqual(Vector3(0.5f, 1, 1.5f)));
    CHECK(nearlyEqual(tripleProduct(Vector3::unitX(), Vector3::unitY(), Vector3::unitZ()), Real(1)));

    // Proteção contra divisão por zero e normalização degenerada.
    CHECK((a / 0).isZero(0));
    Vector3 z(0, 0, 0);
    CHECK(z.normalize() == 0 && z.isZero(0));
    CHECK(Vector3(1e-30f, 0, 0).normalized().isZero(0));
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    CHECK(Vector3(nan, 0, 0).normalized().isFinite());
    CHECK(!Vector3(nan, 0, 0).isFinite());
    CHECK(nearlyEqual(Vector3(3, 4, 0).normalized().length(), Real(1)));

    // Base ortonormal para vários eixos (inclui casos próximos dos eixos canônicos).
    const Vector3 ns[] = {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1),
                          Vector3(1, 1, 1).normalized(), Vector3(-0.2f, 0.9f, 0.1f).normalized()};
    for (const Vector3& n : ns) {
        Vector3 t1, t2;
        computeOrthonormalBasis(n, t1, t2);
        CHECK(nearlyEqual(t1.length(), Real(1), kTol));
        CHECK(nearlyEqual(t2.length(), Real(1), kTol));
        CHECK(nearlyZero(n.dot(t1), kTol));
        CHECK(nearlyZero(n.dot(t2), kTol));
        CHECK(nearlyZero(t1.dot(t2), kTol));
    }
}

static void testMatrix3() {
    Matrix3 A(2, 1, 0,
              0, 3, 1,
              1, 0, 4);
    CHECK(nearlyEqual(A.determinant(), Real(25)));
    Matrix3 Ai;
    CHECK(A.tryInverse(Ai));
    CHECK((A * Ai).nearlyEqual(Matrix3::identity(), kTol));
    CHECK((Ai * A).nearlyEqual(Matrix3::identity(), kTol));
    CHECK(A.transposed().transposed().nearlyEqual(A));
    CHECK(A.transformTranspose(Vector3(1, 2, 3)).nearlyEqual(A.transposed() * Vector3(1, 2, 3)));

    // Matriz singular → inversa nula, sem NaN.
    Matrix3 S(1, 2, 3, 2, 4, 6, 0, 0, 1);
    Matrix3 Si;
    CHECK(!S.tryInverse(Si));
    CHECK(S.inverse().isFinite());
    CHECK(S.inverse().nearlyEqual(Matrix3::zero()));

    // Tensor de inércia minúsculo continua invertível (teste relativo).
    Matrix3 I = Matrix3::diagonal(1e-4f, 2e-4f, 3e-4f);
    CHECK(I.inverse().nearlyEqual(Matrix3::diagonal(1e4f, 5e3f, 1e4f / 3), Real(1)));

    // skew(v) * w == v × w
    Vector3 v(1, -2, 3), w(0.5f, 4, -1);
    CHECK((Matrix3::skew(v) * w).nearlyEqual(cross(v, w), kTol));
    // outer(a,b) * c == a (b·c)
    CHECK((Matrix3::outer(v, w) * Vector3(1, 1, 1)).nearlyEqual(v * dot(w, Vector3(1, 1, 1)), kTol));
}

static void testQuaternion() {
    const Vector3 axis = Vector3(1, 2, -1).normalized();
    const Real angle = Real(0.9);
    const Quaternion q = Quaternion::fromAxisAngle(axis, angle);
    CHECK(nearlyEqual(q.norm(), Real(1), kTol));

    // Rotação por quatérnio == rotação por matriz == Rodrigues.
    const Vector3 v(0.3f, -1.2f, 2.5f);
    const Matrix3 R = q.toMatrix();
    const Vector3 rodrigues = v * std::cos(angle) + cross(axis, v) * std::sin(angle) +
                              axis * (dot(axis, v) * (1 - std::cos(angle)));
    CHECK(q.rotate(v).nearlyEqual(R * v, kTol));
    CHECK(q.rotate(v).nearlyEqual(rodrigues, kTol));
    CHECK(q.rotateInverse(q.rotate(v)).nearlyEqual(v, kTol));
    CHECK(nearlyEqual(R.determinant(), Real(1), kTol));
    CHECK((R * R.transposed()).nearlyEqual(Matrix3::identity(), kTol));

    // Ida e volta quatérnio <-> matriz em várias orientações (cobre todos os ramos de Shepperd).
    const Vector3 axes[] = {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(1, 1, 0),
                            Vector3(-1, 2, 3)};
    const Real angles[] = {Real(0.1), Real(1.5), Real(3.0), Real(3.14159), Real(-2.2)};
    for (const Vector3& ax : axes) {
        for (Real ang : angles) {
            Quaternion a = Quaternion::fromAxisAngle(ax, ang);
            Quaternion b = Quaternion::fromMatrix(a.toMatrix());
            CHECK(a.nearlyEqual(b, kTol));
        }
    }

    // Composição: (a*b) aplica b primeiro.
    Quaternion qa = Quaternion::fromAxisAngle(Vector3::unitZ(), kHalfPi);
    Quaternion qb = Quaternion::fromAxisAngle(Vector3::unitX(), kHalfPi);
    CHECK((qa * qb).rotate(Vector3::unitY()).nearlyEqual(qa.rotate(qb.rotate(Vector3::unitY())), kTol));
    CHECK((q * q.inverse()).nearlyEqual(Quaternion::identity(), kTol));

    // Degeneração.
    Quaternion bad(0, 0, 0, 0);
    bad.normalize();
    CHECK(bad == Quaternion::identity());
    CHECK(Quaternion::fromAxisAngle(Vector3::zero(), 1) == Quaternion::identity());

    // Eixo-ângulo.
    Vector3 ax2; Real an2;
    q.toAxisAngle(ax2, an2);
    CHECK(ax2.nearlyEqual(axis, kTol) && nearlyEqual(an2, angle, kTol));

    // fromTwoVectors (inclui vetores opostos).
    const Vector3 f(1, 0, 0), t(0, 1, 0);
    CHECK(Quaternion::fromTwoVectors(f, t).rotate(f).nearlyEqual(t, kTol));
    CHECK(Quaternion::fromTwoVectors(f, -f).rotate(f).nearlyEqual(-f, kTol));

    // Slerp: pontos extremos, ponto médio e caminho mais curto.
    Quaternion s0 = Quaternion::identity();
    Quaternion s1 = Quaternion::fromAxisAngle(Vector3::unitY(), Real(1.2));
    CHECK(Quaternion::slerp(s0, s1, 0).nearlyEqual(s0, kTol));
    CHECK(Quaternion::slerp(s0, s1, 1).nearlyEqual(s1, kTol));
    CHECK(Quaternion::slerp(s0, s1, Real(0.5)).nearlyEqual(Quaternion::fromAxisAngle(Vector3::unitY(), Real(0.6)), kTol));
    CHECK(Quaternion::slerp(s0, -s1, Real(0.5)).nearlyEqual(Quaternion::fromAxisAngle(Vector3::unitY(), Real(0.6)), kTol));
    CHECK(Quaternion::slerp(s0, s0, Real(0.3)).isFinite());

    // Integração: ω constante por 1 s (em 600 passos) ≈ rotação exata.
    const Vector3 omega(0, 0, 1);
    Quaternion qi = Quaternion::identity();
    for (int i = 0; i < 600; ++i) qi.integrate(omega, Real(1) / 600);
    CHECK(nearlyEqual(qi.norm(), Real(1), kTol));
    CHECK(qi.nearlyEqual(Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1)), Real(1e-3)));
    CHECK(Quaternion::fromRotationVector(Vector3(0, 0, 1)).nearlyEqual(Quaternion::fromAxisAngle(Vector3::unitZ(), 1), kTol));
    CHECK(Quaternion::fromRotationVector(Vector3::zero()) == Quaternion::identity());
}

static void testTransform() {
    Transform T(Vector3(1, 2, 3), Quaternion::fromAxisAngle(Vector3(0, 1, 1), Real(0.7)));
    const Vector3 p(-1, 0.5f, 2);
    CHECK(T.inverseTransformPoint(T.transformPoint(p)).nearlyEqual(p, kTol));
    CHECK((T.inverse() * T).nearlyEqual(Transform::identity(), kTol));
    CHECK(T.inverse().transformPoint(T.transformPoint(p)).nearlyEqual(p, kTol));

    Transform U(Vector3(-2, 0, 1), Quaternion::fromAxisAngle(Vector3::unitX(), Real(1.1)));
    CHECK((T * U).transformPoint(p).nearlyEqual(T.transformPoint(U.transformPoint(p)), kTol));

    Transform mid = Transform::interpolate(Transform::identity(), Transform(Vector3(2, 0, 0), Quaternion::fromAxisAngle(Vector3::unitZ(), Real(1))), Real(0.5));
    CHECK(mid.position.nearlyEqual(Vector3(1, 0, 0), kTol));
    CHECK(mid.rotation.nearlyEqual(Quaternion::fromAxisAngle(Vector3::unitZ(), Real(0.5)), kTol));

    // Tensor de inércia em mundo permanece simétrico e com mesmo traço.
    Matrix3 Ib = Matrix3::diagonal(1, 2, 3);
    Matrix3 Iw = Matrix3::rotateTensor(Ib, T.rotationMatrix());
    CHECK(Iw.nearlyEqual(Iw.transposed(), kTol));
    CHECK(nearlyEqual(Iw.trace(), Real(6), kTol));
}

int main() {
    testVector3();
    testMatrix3();
    testQuaternion();
    testTransform();
    std::printf("%d verificacoes, %d falhas\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
