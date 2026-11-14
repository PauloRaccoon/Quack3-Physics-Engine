/* =============================================================================
 * capi_tests.c  (V26)
 * Testes da API C, escritos em C99 PURO (compilados como C): garantem que o cabeçalho é C de verdade e que
 * a API se comporta de fora do C++: ciclo de vida, validação de entradas, simulação, juntas, eventos, consultas.
 * ============================================================================= */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "physics_c/physics_c.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fails; printf("  FALHOU (linha %d): %s\n", __LINE__, #cond); } } while (0)
#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

static pc_vec3 V(double x, double y, double z) { pc_vec3 v; v.x = x; v.y = y; v.z = z; return v; }

/* corpo de caixa pronto */
static pc_body_id make_box(pc_world* w, pc_shape* box, int type, pc_vec3 pos) {
    pc_body_desc d;
    pc_body_id id = PC_INVALID_BODY;
    pc_body_desc_default(&d);
    d.shape = box;
    d.type = type;
    d.position = pos;
    CHECK(pc_body_create(w, &d, &id) == PC_OK);
    return id;
}

static void test_basics(void) {
    pc_world_settings s;
    CHECK(pc_abi_version() == PC_ABI_VERSION);
    CHECK(strcmp(pc_version_string(), "1.0.0") == 0);
    CHECK(pc_real_is_double() == 0 || pc_real_is_double() == 1);
    CHECK(pc_world_settings_default(&s) == PC_OK);
    CHECK(NEAR(s.gravity.y, -9.81, 1e-5) && NEAR(s.fixed_time_step, 1.0 / 60.0, 1e-6) && s.max_bodies == 4096u);   /* float no motor: tolerancia de float */
    CHECK(pc_world_settings_default(NULL) == PC_ERR_INVALID_ARGUMENT);
    CHECK(strlen(pc_last_error()) > 0);

    /* mundo com settings inválidos: NULL + mensagem; destroy(NULL) é seguro */
    s.fixed_time_step = NAN;
    CHECK(pc_world_create(&s) == NULL);
    s.fixed_time_step = 1.0 / 60.0;
    s.max_bodies = 0;
    CHECK(pc_world_create(&s) == NULL);
    pc_world_destroy(NULL);
    pc_shape_destroy(NULL);

    pc_world_settings_default(&s);
    s.gravity = V(0, -3, 0);
    s.max_bodies = 8;
    {
        pc_world* w = pc_world_create(&s);
        pc_vec3 g;
        double dt = 0;
        CHECK(w != NULL);
        CHECK(pc_world_get_gravity(w, &g) == PC_OK && NEAR(g.y, -3, 1e-9));
        g = V(0, -1, 0);
        CHECK(pc_world_set_gravity(w, &g) == PC_OK);
        g = V(0, NAN, 0);
        CHECK(pc_world_set_gravity(w, &g) == PC_ERR_INVALID_ARGUMENT);
        CHECK(pc_world_get_gravity(w, &g) == PC_OK && NEAR(g.y, -1, 1e-9));   /* a recusa não alterou nada */
        CHECK(pc_world_set_fixed_time_step(w, 1.0 / 120.0) == PC_OK && pc_world_get_fixed_time_step(w, &dt) == PC_OK && NEAR(dt, 1.0 / 120.0, 1e-6));
        CHECK(pc_world_set_fixed_time_step(w, 0.0) == PC_ERR_INVALID_ARGUMENT);
        CHECK(pc_world_set_fixed_time_step(w, NAN) == PC_ERR_INVALID_ARGUMENT);
        CHECK(pc_world_set_solver_iterations(w, 20, 4) == PC_OK && pc_world_set_solver_iterations(w, 0, 4) == PC_ERR_INVALID_ARGUMENT);
        CHECK(pc_world_set_orientation_exponential(w, 0) == PC_OK && pc_world_set_orientation_exponential(w, 1) == PC_OK);
        CHECK(pc_world_set_sleep(w, 1, 0.5) == PC_OK && pc_world_set_sleep(w, 1, -1.0) == PC_ERR_INVALID_ARGUMENT);
        CHECK(pc_world_update(w, -1.0) == PC_ERR_INVALID_ARGUMENT && pc_world_update(w, NAN) == PC_ERR_INVALID_ARGUMENT);
        CHECK(pc_world_update(NULL, 0.1) == PC_ERR_INVALID_HANDLE && pc_world_step(NULL) == PC_ERR_INVALID_HANDLE);
        pc_world_destroy(w);
    }
}

static void test_shapes_and_bodies(void) {
    pc_world_settings s;
    pc_world* w;
    pc_shape *sph, *box, *cap, *cyl, *bad;
    pc_vec3 h = V(0.5, 0.5, 0.5);
    pc_body_id id, ids[3];
    pc_body_desc d;
    pc_body_state st;

    pc_world_settings_default(&s);
    s.max_bodies = 3;
    w = pc_world_create(&s);
    sph = pc_shape_create_sphere(0.5);
    box = pc_shape_create_box(&h);
    cap = pc_shape_create_capsule(0.25, 0.5);
    cyl = pc_shape_create_cylinder(0.25, 0.5);
    CHECK(sph && box && cap && cyl);
    CHECK(pc_shape_create_sphere(-1.0) == NULL && strlen(pc_last_error()) > 0);
    CHECK(pc_shape_create_sphere(NAN) == NULL);
    CHECK(pc_shape_create_box(NULL) == NULL);
    h = V(0, 1, 1);
    bad = pc_shape_create_box(&h);
    CHECK(bad == NULL);
    h = V(0.5, 0.5, 0.5);

    /* corpo sem forma / valores ruins / tipo inválido */
    pc_body_desc_default(&d);
    CHECK(pc_body_create(w, &d, &id) == PC_ERR_INVALID_ARGUMENT);              /* shape obrigatório */
    d.shape = sph;
    d.position = V(0, 1, 0);
    d.mass = NAN;
    CHECK(pc_body_create(w, &d, &id) == PC_ERR_INVALID_ARGUMENT);
    d.mass = 0;
    d.type = 7;
    CHECK(pc_body_create(w, &d, &id) == PC_ERR_INVALID_ARGUMENT);
    d.type = PC_BODY_DYNAMIC;
    memset(&d.rotation, 0, sizeof d.rotation);
    CHECK(pc_body_create(w, &d, &id) == PC_ERR_INVALID_ARGUMENT);              /* quaternio nulo */
    d.rotation.w = 2.0;                                                       /* não unitário: a API normaliza */
    CHECK(pc_body_create(w, &d, &id) == PC_OK);
    CHECK(pc_body_get_state(w, id, &st) == PC_OK && NEAR(st.rotation.w, 1.0, 1e-6) && NEAR(st.position.y, 1.0, 1e-6));
    CHECK(pc_body_is_valid(w, id) == 1 && st.type == PC_BODY_DYNAMIC && st.mass > 0.0 && st.awake == 1);
    CHECK(NEAR(st.mass, 1.0 * (4.0 / 3.0) * 3.14159265358979 * 0.125, 1e-3));   /* densidade 1, esfera r = 0.5 */

    /* capacidade (max_bodies = 3) */
    ids[0] = id;
    d.rotation.w = 1.0;
    d.shape = box; CHECK(pc_body_create(w, &d, &ids[1]) == PC_OK);
    d.shape = cap; CHECK(pc_body_create(w, &d, &ids[2]) == PC_OK);
    d.shape = cyl; CHECK(pc_body_create(w, &d, &id) == PC_ERR_CAPACITY);
    /* destruir libera o slot; id destruído é recusado */
    CHECK(pc_body_destroy(w, ids[1]) == PC_OK);
    CHECK(pc_body_is_valid(w, ids[1]) == 0 && pc_body_destroy(w, ids[1]) == PC_ERR_INVALID_HANDLE);
    CHECK(pc_body_get_state(w, ids[1], &st) == PC_ERR_INVALID_HANDLE);
    CHECK(pc_body_get_state(w, 9999, &st) == PC_ERR_INVALID_HANDLE && pc_body_get_state(NULL, 0, &st) == PC_ERR_INVALID_HANDLE);
    CHECK(pc_body_get_state(w, ids[0], NULL) == PC_ERR_INVALID_ARGUMENT);

    /* a forma pode ser destruída depois: o corpo guarda a sua referência */
    pc_shape_destroy(sph);
    pc_shape_destroy(cap);
    CHECK(pc_world_step(w) == PC_OK && pc_body_get_state(w, ids[0], &st) == PC_OK);
    pc_shape_destroy(box);
    pc_shape_destroy(cyl);
    pc_world_destroy(w);
}

static void test_polyhedron(void) {
    /* tetraedro regular-ish, faces anti-horárias vistas de fora */
    const pc_vec3 v[4] = {{1, 1, 1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1}};
    const int32_t faces[12] = {0, 1, 2,  0, 3, 1,  0, 2, 3,  1, 3, 2};
    const int32_t sizes[4] = {3, 3, 3, 3};
    pc_shape* t = pc_shape_create_polyhedron(v, 4, faces, sizes, 4);
    pc_world* w = pc_world_create(NULL);
    pc_body_desc d;
    pc_body_id id;
    pc_body_state st;
    CHECK(t != NULL);
    if (t) {
        pc_body_desc_default(&d);
        d.shape = t;
        d.position = V(0, 5, 0);
        CHECK(pc_body_create(w, &d, &id) == PC_OK && pc_body_get_state(w, id, &st) == PC_OK && st.mass > 0.0);
        pc_shape_destroy(t);
    }
    {   /* índice fora da faixa / contagens ruins / faces invertidas (volume negativo) */
        const int32_t bad_idx[12] = {0, 1, 2,  0, 3, 1,  0, 2, 3,  1, 3, 9};
        const int32_t inv[12] = {0, 2, 1,  0, 1, 3,  0, 3, 2,  1, 2, 3};
        CHECK(pc_shape_create_polyhedron(v, 4, bad_idx, sizes, 4) == NULL);
        CHECK(pc_shape_create_polyhedron(v, 4, inv, sizes, 4) == NULL);
        CHECK(pc_shape_create_polyhedron(NULL, 4, faces, sizes, 4) == NULL);
        CHECK(pc_shape_create_polyhedron(v, 2, faces, sizes, 4) == NULL);
    }
    pc_world_destroy(w);
}

/* Caixa cai sobre o chão estático e para; um raio vertical a acha; o mundo dorme. */
static void test_simulation(void) {
    pc_world* w = pc_world_create(NULL);
    pc_vec3 floorH = V(10, 0.5, 10), boxH = V(0.5, 0.5, 0.5);
    pc_shape* fs = pc_shape_create_box(&floorH);
    pc_shape* bs = pc_shape_create_box(&boxH);
    pc_body_id floor_id = make_box(w, fs, PC_BODY_STATIC, V(0, -0.5, 0));
    pc_body_id box_id = make_box(w, bs, PC_BODY_DYNAMIC, V(0, 3, 0));
    pc_body_state st;
    pc_world_stats stats;
    int i, steps = 0;
    CHECK(floor_id != box_id);
    for (i = 0; i < 300; ++i) steps += pc_world_update(w, 1.0 / 60.0);
    CHECK(steps == 300);
    CHECK(pc_body_get_state(w, box_id, &st) == PC_OK);
    CHECK(NEAR(st.position.y, 0.5, 0.02));              /* apoiada no chão (topo do chão em y = 0) */
    CHECK(fabs(st.linear_velocity.y) < 0.05);
    CHECK(pc_world_get_stats(w, &stats) == PC_OK && stats.bodies == 2 && stats.step_count == 300u && stats.touching_pairs >= 1);
    CHECK(pc_body_get_state(w, floor_id, &st) == PC_OK && st.type == PC_BODY_STATIC && st.mass == 0.0);

    /* tempo menor que um passo: nenhum passo, α em [0,1) */
    {
        double alpha = -1;
        CHECK(pc_world_update(w, 0.001) == 0);
        CHECK(pc_world_get_alpha(w, &alpha) == PC_OK && alpha >= 0.0 && alpha < 1.0);
        CHECK(pc_world_get_alpha(w, NULL) == PC_ERR_INVALID_ARGUMENT);
    }
    pc_world_destroy(w);
    pc_shape_destroy(fs);
    pc_shape_destroy(bs);
}

static void test_forces_and_velocity(void) {
    pc_world_settings s;
    pc_world* w;
    pc_shape* sp = pc_shape_create_sphere(0.5);
    pc_body_desc d;
    pc_body_id id;
    pc_body_state st;
    pc_vec3 f = V(10, 0, 0), j = V(0, 0, 2), pt;
    int i;

    pc_world_settings_default(&s);
    s.gravity = V(0, 0, 0);
    w = pc_world_create(&s);
    pc_body_desc_default(&d);
    d.shape = sp;
    CHECK(pc_body_create(w, &d, &id) == PC_OK);
    pc_body_get_state(w, id, &st);
    {
        const double m = st.mass;
        /* força constante F por 1 s (60 passos): v = F t / m */
        for (i = 0; i < 60; ++i) { CHECK(pc_body_apply_force(w, id, &f) == PC_OK); CHECK(pc_world_step(w) == PC_OK); }
        pc_body_get_state(w, id, &st);
        CHECK(NEAR(st.linear_velocity.x, 10.0 / m, 0.02 * 10.0 / m));
        /* impulso num ponto fora do centro: muda v e gera ω = r × J / I */
        pc_body_set_linear_velocity(w, id, &(pc_vec3){0, 0, 0});
        pt = st.center_of_mass;
        pt.x += 0.5;
        CHECK(pc_body_apply_impulse_at_point(w, id, &j, &pt) == PC_OK);
        pc_body_get_state(w, id, &st);
        CHECK(NEAR(st.linear_velocity.z, 2.0 / m, 1e-3));
        CHECK(fabs(st.angular_velocity.y) > 0.1);          /* r = +x, J = +z: torque em -y (r × J) */
    }
    /* validações */
    CHECK(pc_body_apply_force(w, id, NULL) == PC_ERR_INVALID_ARGUMENT);
    f = V(INFINITY, 0, 0);
    CHECK(pc_body_apply_force(w, id, &f) == PC_ERR_INVALID_ARGUMENT);
    CHECK(pc_body_set_material(w, id, 0.8, 0.3) == PC_OK && pc_body_set_material(w, id, NAN, 0.0) == PC_ERR_INVALID_ARGUMENT);
    CHECK(pc_body_set_damping(w, id, 0.1, 0.1) == PC_OK && pc_body_set_gravity_scale(w, id, 0.0) == PC_OK);
    CHECK(pc_body_set_awake(w, id, 1) == PC_OK);
    {   /* teletransporte */
        pc_vec3 p = V(1, 2, 3);
        pc_quat q = {0, 0, 1, 0};   /* 180° em torno de Y */
        pc_vec3 ip; pc_quat iq;
        CHECK(pc_body_set_transform(w, id, &p, &q) == PC_OK);
        pc_body_get_state(w, id, &st);
        CHECK(NEAR(st.position.x, 1, 1e-6) && NEAR(st.rotation.y, 1, 1e-6));
        CHECK(pc_body_get_interpolated(w, id, &ip, &iq) == PC_OK && NEAR(ip.y, 2, 1e-5));   /* sem rastro de interpolação */
        q.y = 0;
        CHECK(pc_body_set_transform(w, id, &p, &q) == PC_ERR_INVALID_ARGUMENT);              /* quaternio nulo */
    }
    pc_world_destroy(w);
    pc_shape_destroy(sp);
}

/* Pêndulo de dobradiça preso a um corpo estático; destruir o corpo some com a junta. */
static void test_joints(void) {
    pc_world_settings s;
    pc_world* w;
    pc_vec3 anchorH = V(0.1, 0.1, 0.1), armH = V(1.0, 0.1, 0.1);
    pc_shape *as, *rs;
    pc_body_id a, b, other;
    pc_joint_id j = 0, j2 = 0, j3 = 0;
    pc_vec3 anchor = V(0, 5, 0), axis = V(0, 0, 1), p0, p1;
    pc_body_state st;
    int i;

    pc_world_settings_default(&s);
    w = pc_world_create(&s);
    as = pc_shape_create_box(&anchorH);
    rs = pc_shape_create_box(&armH);
    a = make_box(w, as, PC_BODY_STATIC, V(0, 5, 0));
    b = make_box(w, rs, PC_BODY_DYNAMIC, V(1.0, 5, 0));
    other = make_box(w, rs, PC_BODY_DYNAMIC, V(5, 8, 0));
    CHECK(pc_joint_create_hinge(w, a, b, &anchor, &axis, 0, &j) == PC_OK && j != 0);
    CHECK(pc_joint_is_valid(w, j) == 1);
    CHECK(pc_joint_create_hinge(w, a, 9999, &anchor, &axis, 0, &j2) == PC_ERR_INVALID_HANDLE);
    CHECK(pc_joint_create_hinge(w, a, a, &anchor, &axis, 0, &j2) == PC_ERR_INVALID_ARGUMENT);      /* corpos iguais */
    axis = V(0, 0, 0);
    CHECK(pc_joint_create_hinge(w, a, b, &anchor, &axis, 0, &j2) == PC_ERR_INVALID_ARGUMENT);      /* eixo nulo */
    {
        pc_vec3 pa = V(1, 8, 0), pb = V(5, 8, 0);
        CHECK(pc_joint_create_ball_socket(w, a, other, &pa, 0, &j2) == PC_OK);
        CHECK(pc_joint_create_distance(w, a, other, &pa, &pb, -1.0, 0, &j3) == PC_OK);
    }
    {
        pc_world_stats stt;
        pc_world_get_stats(w, &stt);
        CHECK(stt.joints == 3);
    }
    pc_body_get_state(w, b, &st);
    p0 = st.center_of_mass;
    for (i = 0; i < 120; ++i) pc_world_step(w);
    pc_body_get_state(w, b, &st);
    p1 = st.center_of_mass;
    CHECK(p1.y < p0.y - 0.5);                              /* o braço caiu girando */
    CHECK(NEAR(sqrt((p1.x - 0.0) * (p1.x - 0.0) + (p1.y - 5.0) * (p1.y - 5.0)), 1.0, 0.05));   /* a distância ao pivô se conserva */
    CHECK(fabs(p1.z) < 0.02);                              /* a dobradiça mantém o plano XY */

    CHECK(pc_joint_destroy(w, j) == PC_OK && pc_joint_is_valid(w, j) == 0);
    CHECK(pc_joint_destroy(w, j) == PC_ERR_INVALID_HANDLE);
    /* destruir um corpo ligado leva as juntas dele junto: o id deixa de valer, sem ponteiro pendurado */
    CHECK(pc_joint_is_valid(w, j2) == 1 && pc_joint_is_valid(w, j3) == 1);
    CHECK(pc_body_destroy(w, other) == PC_OK);
    CHECK(pc_joint_is_valid(w, j2) == 0 && pc_joint_is_valid(w, j3) == 0);
    CHECK(pc_joint_destroy(w, j2) == PC_ERR_INVALID_HANDLE);
    CHECK(pc_world_step(w) == PC_OK);
    pc_world_destroy(w);
    pc_shape_destroy(as);
    pc_shape_destroy(rs);
}

static void test_events_queries_bulk(void) {
    pc_world* w = pc_world_create(NULL);
    pc_vec3 floorH = V(10, 0.5, 10), r = V(0.5, 0.5, 0.5);
    pc_shape* fs = pc_shape_create_box(&floorH);
    pc_shape* bs = pc_shape_create_box(&r);
    pc_body_id fl = make_box(w, fs, PC_BODY_STATIC, V(0, -0.5, 0));
    pc_body_id bx = make_box(w, bs, PC_BODY_DYNAMIC, V(0, 1.0, 0));
    pc_contact_event ev[8];
    int n, i, got_begin = 0;
    pc_raycast_hit hit, hits[4];
    pc_vec3 o = V(0, 10, 0), dir = V(0, -1, 0), c = V(0, 0.5, 0);
    pc_body_id ids[4];
    double data[7 * 4];

    /* eventos: a caixa chega ao chão e gera um Begin com velocidade de impacto > 0 */
    for (i = 0; i < 90; ++i) {
        pc_world_step(w);
        n = pc_world_read_contact_events(w, 8, ev);
        CHECK(n >= 0);
        {
            int k;
            for (k = 0; k < n && k < 8; ++k)
                if (ev[k].type == PC_CONTACT_BEGIN) {
                    got_begin = 1;
                    CHECK(ev[k].body_a < ev[k].body_b && ev[k].point_count >= 1 && ev[k].approach_speed >= 0.0);
                    CHECK((ev[k].body_a == fl && ev[k].body_b == bx) || (ev[k].body_a == bx && ev[k].body_b == fl));
                }
        }
    }
    CHECK(got_begin);
    CHECK(pc_world_clear_contact_events(w) == PC_OK && pc_world_read_contact_events(w, 0, NULL) == 0);

    /* raio de cima para baixo: a caixa primeiro (topo em y ≈ 1), depois o chão */
    CHECK(pc_world_raycast(w, &o, &dir, INFINITY, PC_QUERY_ALL, PC_INVALID_BODY, &hit) == 1);
    CHECK(hit.body == bx && NEAR(hit.distance, 10.0 - 1.0, 0.05) && NEAR(hit.normal.y, 1.0, 1e-3) && hit.started_inside == 0);
    CHECK(pc_world_raycast(w, &o, &dir, INFINITY, PC_QUERY_STATIC, PC_INVALID_BODY, &hit) == 1 && hit.body == fl);
    CHECK(pc_world_raycast(w, &o, &dir, INFINITY, PC_QUERY_ALL, bx, &hit) == 1 && hit.body == fl);   /* ignora a caixa */
    CHECK(pc_world_raycast(w, &o, &dir, 2.0, PC_QUERY_ALL, PC_INVALID_BODY, &hit) == 0);              /* curto demais */
    CHECK(pc_world_raycast_all(w, &o, &dir, INFINITY, PC_QUERY_ALL, PC_INVALID_BODY, 4, hits) == 2 && hits[0].body == bx && hits[1].body == fl);
    CHECK(pc_world_raycast_all(w, &o, &dir, INFINITY, PC_QUERY_ALL, PC_INVALID_BODY, 1, hits) == 2);   /* total mesmo com buffer pequeno */
    {
        pc_vec3 zero = V(0, 0, 0), nan3 = V(NAN, 0, 0);
        CHECK(pc_world_raycast(w, &o, &zero, 10.0, PC_QUERY_ALL, PC_INVALID_BODY, &hit) == 0);        /* direção nula: nada */
        CHECK(pc_world_raycast(w, &nan3, &dir, 10.0, PC_QUERY_ALL, PC_INVALID_BODY, &hit) == PC_ERR_INVALID_ARGUMENT);
        CHECK(pc_world_raycast(w, &o, NULL, 10.0, PC_QUERY_ALL, PC_INVALID_BODY, &hit) == PC_ERR_INVALID_ARGUMENT);
    }
    CHECK(pc_world_overlap_sphere(w, &c, 0.3, PC_QUERY_ALL, PC_INVALID_BODY, 4, ids) >= 1);
    CHECK(pc_world_overlap_sphere(w, &c, 0.3, PC_QUERY_DYNAMIC, PC_INVALID_BODY, 4, ids) == 1 && ids[0] == bx);
    CHECK(pc_world_overlap_point(w, &c, PC_QUERY_DYNAMIC, PC_INVALID_BODY, 4, ids) == 1 && ids[0] == bx);
    CHECK(pc_world_overlap_sphere(w, &c, -1.0, PC_QUERY_ALL, PC_INVALID_BODY, 4, ids) == 0);

    /* leitura em lote: ids + 7 doubles (pos, quat), de todos os corpos vivos */
    CHECK(pc_world_read_transforms(w, 0, 0, NULL, NULL) == 2);                       /* só contar */
    CHECK(pc_world_read_transforms(w, 0, 4, ids, data) == 2);
    CHECK(ids[0] == fl && ids[1] == bx && NEAR(data[1], -0.5, 1e-6) && NEAR(data[3], 1.0, 1e-6));
    CHECK(NEAR(data[7 + 1], 0.5, 0.02) && NEAR(sqrt(data[10] * data[10] + data[11] * data[11] + data[12] * data[12] + data[13] * data[13]), 1.0, 1e-5));
    CHECK(pc_world_read_transforms(w, 1, 1, ids, data) == 2);                         /* capacidade menor: só escreve 1, devolve o total */
    pc_world_destroy(w);
    pc_shape_destroy(fs);
    pc_shape_destroy(bs);
}

/* Interpolação: com tempo de render < Δt o corpo em queda livre aparece ENTRE o passo anterior e o atual. */
static void test_interpolation(void) {
    pc_world_settings s;
    pc_world* w;
    pc_shape* sp = pc_shape_create_sphere(0.5);
    pc_body_desc d;
    pc_body_id id;
    pc_vec3 p, ip;
    pc_quat iq;
    double alpha;
    pc_world_settings_default(&s);
    w = pc_world_create(&s);
    pc_body_desc_default(&d);
    d.shape = sp;
    d.position = V(0, 100, 0);
    pc_body_create(w, &d, &id);
    pc_world_update(w, 1.0 / 60.0);
    pc_world_update(w, 1.0 / 60.0);                 /* dois passos: há pose anterior e atual */
    pc_world_update(w, 0.5 / 60.0);                 /* meio passo no acumulador: α ≈ 0.5 */
    pc_world_get_alpha(w, &alpha);
    CHECK(NEAR(alpha, 0.5, 1e-3));
    {
        pc_body_state st;
        pc_body_get_state(w, id, &st);
        p = st.position;
    }
    CHECK(pc_body_get_interpolated(w, id, &ip, &iq) == PC_OK);
    CHECK(ip.y < 100.0 && NEAR(iq.w, 1.0, 1e-6));
    CHECK(ip.y >= p.y);                             /* o corpo cai: a pose desenhada (entre o passo anterior e o atual) fica ACIMA da simulada... */
    CHECK(ip.y - p.y < 0.05);                       /* ...por menos que um passo de queda (~0,03 m) */
    CHECK(pc_body_get_interpolated(w, id, &ip, NULL) == PC_ERR_INVALID_ARGUMENT);
    pc_world_destroy(w);
    pc_shape_destroy(sp);
}

/* Estresse de uso: criar/destruir muitos corpos, formas e mundos não vaza nem quebra (ASan confere). */
static void test_lifecycle_stress(void) {
    int round, i;
    for (round = 0; round < 20; ++round) {
        pc_world* w = pc_world_create(NULL);
        pc_shape* sp = pc_shape_create_sphere(0.25);
        pc_body_id ids[64];
        pc_body_desc d;
        pc_body_desc_default(&d);
        d.shape = sp;
        for (i = 0; i < 64; ++i) {
            d.position = V((i % 8) * 0.6, 1.0 + (i / 8) * 0.6, 0.0);
            CHECK(pc_body_create(w, &d, &ids[i]) == PC_OK);
        }
        for (i = 0; i < 30; ++i) pc_world_update(w, 1.0 / 60.0);
        for (i = 0; i < 64; i += 2) CHECK(pc_body_destroy(w, ids[i]) == PC_OK);
        for (i = 0; i < 10; ++i) pc_world_update(w, 1.0 / 60.0);
        pc_shape_destroy(sp);        /* forma destruída antes do mundo: corpos continuam válidos */
        pc_world_update(w, 1.0 / 60.0);
        pc_world_destroy(w);
    }
}

int main(void) {
    printf("physics_c %s (real = %s)\n", pc_version_string(), pc_real_is_double() ? "double" : "float");
    test_basics();
    test_shapes_and_bodies();
    test_polyhedron();
    test_simulation();
    test_forces_and_velocity();
    test_joints();
    test_events_queries_bulk();
    test_interpolation();
    test_lifecycle_stress();
    printf("capi_tests: %d verificacoes, %d falhas\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
