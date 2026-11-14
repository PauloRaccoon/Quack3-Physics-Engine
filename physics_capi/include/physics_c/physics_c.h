/* =============================================================================
 * physics_c.h  (V26)
 * API C do motor de física: o jeito de usar o motor a partir de Python (ctypes/cffi), C#, Rust,
 * Lua, Java (JNA/Panama), Go (cgo), etc. É C puro (C99): nenhum tipo do C++ aparece aqui.
 *
 * REGRAS DA API
 *  - Tudo é double, em unidades SI (metros, kg, segundos, radianos), qualquer que seja a precisão
 *    interna do motor (float ou double: veja pc_real_is_double()).
 *  - Objetos do C++ viram HANDLES opacos (pc_world*, pc_shape*) ou números (pc_body_id, pc_joint_id).
 *    Corpos e juntas ficam no mundo; ids inválidos (corpo destruído, etc.) são recusados com erro,
 *    nunca derrubam o programa.
 *  - Funções devolvem int: >= 0 sucesso (algumas devolvem uma contagem), < 0 erro (PC_ERR_*).
 *    pc_last_error() devolve o texto do último erro DESTA thread. Nenhuma exceção C++ atravessa a API.
 *  - Estruturas (pc_vec3, pc_quat, ...) são passadas por PONTEIRO e têm layout fixo (só double/int32/uint32):
 *    fácil de espelhar em qualquer linguagem. Chame pc_*_default() para preencher valores padrão.
 *  - Um pc_world NÃO é thread-safe: use-o de uma thread por vez (ou proteja com um mutex seu).
 *  - Quaternios: ordem (w, x, y, z), norma 1 (a API normaliza o que você mandar; norma 0 é recusada).
 * ============================================================================= */
#ifndef PHYSICS_C_H
#define PHYSICS_C_H

#include <stdint.h>

#if defined(_WIN32) || defined(__CYGWIN__)
#  if defined(PC_BUILDING_DLL)
#    define PC_API __declspec(dllexport)
#  else
#    define PC_API
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define PC_API __attribute__((visibility("default")))
#else
#  define PC_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---- versão --------------------------------------------------------------------------------- */
#define PC_VERSION_MAJOR 1
#define PC_VERSION_MINOR 0
#define PC_VERSION_PATCH 0
/* Sobe quando algo muda de forma incompatível (tamanho/ordem de campos, assinaturas). */
#define PC_ABI_VERSION 1

PC_API int pc_abi_version(void);                    /* == PC_ABI_VERSION da biblioteca carregada */
PC_API const char* pc_version_string(void);         /* "1.0.0" */
PC_API int pc_real_is_double(void);                 /* 1 se o motor foi compilado com double */

/* ---- códigos de erro ----------------------------------------------------------------------------- */
#define PC_OK 0
#define PC_ERR_INVALID_ARGUMENT (-1)   /* ponteiro nulo, valor não finito, faixa inválida, forma inválida... */
#define PC_ERR_INVALID_HANDLE (-2)     /* mundo/forma nulo, corpo ou junta que não existe (mais) */
#define PC_ERR_CAPACITY (-3)           /* max_bodies atingido */
#define PC_ERR_INTERNAL (-4)           /* qualquer outra exceção do motor (veja pc_last_error) */

PC_API const char* pc_last_error(void);             /* texto do último erro desta thread ("" se nenhum) */

/* ---- tipos básicos --------------------------------------------------------------------------------- */
typedef struct pc_vec3 { double x, y, z; } pc_vec3;
typedef struct pc_quat { double w, x, y, z; } pc_quat;

typedef uint32_t pc_body_id;
typedef uint32_t pc_joint_id;
#define PC_INVALID_BODY ((pc_body_id)0xFFFFFFFFu)

typedef struct pc_world pc_world;   /* opaco */
typedef struct pc_shape pc_shape;   /* opaco: forma geométrica compartilhável entre corpos */

/* ---- mundo ---------------------------------------------------------------------------------------- */
typedef struct pc_world_settings {
    pc_vec3 gravity;               /* padrão (0, -9.81, 0) */
    double fixed_time_step;        /* Δt fixo da simulação [s]; padrão 1/60 */
    int32_t max_sub_steps;         /* máx. de passos por pc_world_update; padrão 8 */
    uint32_t max_bodies;           /* capacidade fixa; padrão 4096 */
    int32_t velocity_iterations;   /* iterações do solver de velocidade; padrão 10 */
    int32_t position_iterations;   /* iterações da correção de posição (NGS); padrão 3 */
    int32_t warm_starting;         /* 1 = reinjeta os impulsos do quadro anterior */
    int32_t orientation_exponential; /* 1 = orientação por exp. de quaternio (exata p/ ω const.); 0 = Euler */
    int32_t sleep_enabled;         /* 1 = corpos em repouso adormecem */
    double sleep_time;             /* repouso contínuo para adormecer [s] */
} pc_world_settings;

PC_API int pc_world_settings_default(pc_world_settings* out);

/* Cria o mundo (settings pode ser NULL = padrões). Devolve NULL em erro (veja pc_last_error). */
PC_API pc_world* pc_world_create(const pc_world_settings* settings);
PC_API void pc_world_destroy(pc_world* world);       /* aceita NULL; libera corpos, juntas e formas internas */

/* Avança `frame_time` segundos de tempo de RENDER (acumulador): executa 0..max_sub_steps passos fixos.
 * Devolve o nº de passos executados (>= 0) ou erro. */
PC_API int pc_world_update(pc_world* world, double frame_time);
PC_API int pc_world_step(pc_world* world);           /* exatamente UM passo fixo */
PC_API int pc_world_get_alpha(const pc_world* world, double* out_alpha); /* α = acumulador/Δt, para interpolar */

PC_API int pc_world_get_gravity(const pc_world* world, pc_vec3* out);
PC_API int pc_world_set_gravity(pc_world* world, const pc_vec3* gravity);
PC_API int pc_world_get_fixed_time_step(const pc_world* world, double* out);
PC_API int pc_world_set_fixed_time_step(pc_world* world, double dt);   /* [1e-4, 1] */
PC_API int pc_world_set_solver_iterations(pc_world* world, int32_t velocity_iterations, int32_t position_iterations);
PC_API int pc_world_set_orientation_exponential(pc_world* world, int32_t on);
PC_API int pc_world_set_sleep(pc_world* world, int32_t enabled, double sleep_time);
PC_API int pc_world_wake_all(pc_world* world);

typedef struct pc_world_stats {
    int32_t bodies, awake_bodies, sleeping_bodies, islands, joints;
    int32_t pairs, touching_pairs, contact_points;
    uint64_t step_count;
} pc_world_stats;
PC_API int pc_world_get_stats(const pc_world* world, pc_world_stats* out);

/* ---- formas ------------------------------------------------------------------------------------------ */
/* Cada pc_shape_create_* devolve um handle (ou NULL em erro). O corpo guarda a SUA referência à forma:
 * depois de criar os corpos você pode chamar pc_shape_destroy sem afetá-los. Uma forma serve a vários corpos. */
PC_API pc_shape* pc_shape_create_sphere(double radius);
PC_API pc_shape* pc_shape_create_box(const pc_vec3* half_extents);
PC_API pc_shape* pc_shape_create_capsule(double radius, double half_height);    /* eixo local Y */
PC_API pc_shape* pc_shape_create_cylinder(double radius, double half_height);   /* eixo local Y */
/* Poliedro CONVEXO. vertices: n_vertices pontos locais. faces: índices de todas as faces concatenados
 * (cada face em sentido anti-horário visto de fora); face_sizes[i] = nº de vértices da face i. */
PC_API pc_shape* pc_shape_create_polyhedron(const pc_vec3* vertices, int32_t n_vertices,
                                            const int32_t* face_indices, const int32_t* face_sizes, int32_t n_faces);
PC_API void pc_shape_destroy(pc_shape* shape);       /* aceita NULL */

/* ---- corpos ----------------------------------------------------------------------------------------------- */
#define PC_BODY_STATIC 0
#define PC_BODY_KINEMATIC 1
#define PC_BODY_DYNAMIC 2

typedef struct pc_body_desc {
    int32_t type;                  /* PC_BODY_* */
    const pc_shape* shape;         /* obrigatório */
    double density;                /* usada se mass == 0; padrão 1 */
    double mass;                   /* > 0 sobrescreve a densidade */
    pc_vec3 position;              /* posição do frame do corpo */
    pc_quat rotation;              /* padrão identidade */
    pc_vec3 linear_velocity;
    pc_vec3 angular_velocity;
    double friction;               /* padrão 0.5 */
    double restitution;            /* padrão 0 */
    double linear_damping;         /* padrão 0 */
    double angular_damping;        /* padrão 0.01 */
    double gravity_scale;          /* padrão 1 */
    int32_t allow_sleep;           /* padrão 1 */
} pc_body_desc;

PC_API int pc_body_desc_default(pc_body_desc* out);  /* dinâmico, sem forma, nos padrões acima */

/* Cria o corpo; escreve o id em *out_id. Devolve PC_OK ou erro (PC_ERR_CAPACITY se max_bodies foi atingido). */
PC_API int pc_body_create(pc_world* world, const pc_body_desc* desc, pc_body_id* out_id);
PC_API int pc_body_destroy(pc_world* world, pc_body_id id);   /* também destrói as juntas ligadas a ele */
PC_API int pc_body_is_valid(const pc_world* world, pc_body_id id);   /* 1 ou 0 */

typedef struct pc_body_state {
    int32_t type;                  /* PC_BODY_* */
    int32_t awake;                 /* 1 = acordado */
    double mass;                   /* 0 para estático/cinemático */
    pc_vec3 position;              /* posição do frame do corpo (a origem do seu modelo) */
    pc_vec3 center_of_mass;        /* centro de massa no mundo */
    pc_quat rotation;
    pc_vec3 linear_velocity;       /* do centro de massa */
    pc_vec3 angular_velocity;
} pc_body_state;
PC_API int pc_body_get_state(const pc_world* world, pc_body_id id, pc_body_state* out);

/* Pose para DESENHAR: interpolada entre o penúltimo e o último passo fixo com o α do mundo
 * (lerp na posição, slerp na orientação). Ideal para um render com FPS diferente do da física. */
PC_API int pc_body_get_interpolated(const pc_world* world, pc_body_id id, pc_vec3* out_position, pc_quat* out_rotation);

PC_API int pc_body_set_transform(pc_world* world, pc_body_id id, const pc_vec3* position, const pc_quat* rotation); /* teletransporte */
PC_API int pc_body_set_linear_velocity(pc_world* world, pc_body_id id, const pc_vec3* v);
PC_API int pc_body_set_angular_velocity(pc_world* world, pc_body_id id, const pc_vec3* w);
PC_API int pc_body_apply_force(pc_world* world, pc_body_id id, const pc_vec3* force);                       /* vale no próximo passo */
PC_API int pc_body_apply_force_at_point(pc_world* world, pc_body_id id, const pc_vec3* force, const pc_vec3* world_point);
PC_API int pc_body_apply_torque(pc_world* world, pc_body_id id, const pc_vec3* torque);
PC_API int pc_body_apply_impulse_at_point(pc_world* world, pc_body_id id, const pc_vec3* impulse, const pc_vec3* world_point); /* muda a velocidade já */
PC_API int pc_body_set_material(pc_world* world, pc_body_id id, double friction, double restitution);
PC_API int pc_body_set_damping(pc_world* world, pc_body_id id, double linear, double angular);
PC_API int pc_body_set_gravity_scale(pc_world* world, pc_body_id id, double scale);
PC_API int pc_body_set_awake(pc_world* world, pc_body_id id, int32_t awake);

/* Leitura em LOTE (uma chamada em vez de uma por corpo; é o que um render deve usar).
 * Escreve, para cada corpo vivo (ordem crescente de id), o id em ids[i] e 7 doubles em data[7*i ...]:
 * px, py, pz, qw, qx, qy, qz. `interpolated` = 1: poses interpoladas (render); 0: poses do último passo.
 * Escreve no máximo `capacity` corpos e devolve o TOTAL de corpos vivos (se > capacity, chame de novo com mais espaço).
 * ids e data podem ser NULL (só para contar). */
PC_API int pc_world_read_transforms(const pc_world* world, int32_t interpolated, uint32_t capacity, pc_body_id* ids, double* data);

/* ---- juntas --------------------------------------------------------------------------------------------------- */
/* Ancoragens e eixos em coordenadas do MUNDO, na pose atual dos corpos. Para prender um corpo ao mundo,
 * crie um corpo ESTÁTICO e use-o como `a`. As juntas somem junto com um corpo ligado que for destruído. */
PC_API int pc_joint_create_ball_socket(pc_world* world, pc_body_id a, pc_body_id b, const pc_vec3* anchor,
                                       int32_t collide_connected, pc_joint_id* out_id);
PC_API int pc_joint_create_distance(pc_world* world, pc_body_id a, pc_body_id b, const pc_vec3* anchor_a,
                                    const pc_vec3* anchor_b, double length /* < 0: a distância atual */,
                                    int32_t collide_connected, pc_joint_id* out_id);
PC_API int pc_joint_create_hinge(pc_world* world, pc_body_id a, pc_body_id b, const pc_vec3* anchor,
                                 const pc_vec3* axis, int32_t collide_connected, pc_joint_id* out_id);
PC_API int pc_joint_destroy(pc_world* world, pc_joint_id id);
PC_API int pc_joint_is_valid(const pc_world* world, pc_joint_id id);   /* 1 ou 0 (some quando um corpo ligado é destruído) */

/* ---- eventos de contato (modelo "pull") ------------------------------------------------------------------------ */
#define PC_CONTACT_BEGIN 0
#define PC_CONTACT_END 1
typedef struct pc_contact_event {
    int32_t type;                  /* PC_CONTACT_BEGIN / PC_CONTACT_END */
    pc_body_id body_a, body_b;     /* a < b */
    pc_vec3 point;                 /* média dos pontos de contato (mundo) */
    pc_vec3 normal;                /* de a para b */
    double depth;                  /* maior penetração [m] */
    int32_t point_count;           /* 1..4 */
    double normal_impulse;         /* soma dos impulsos normais no passo [N·s] */
    double approach_speed;         /* velocidade de aproximação antes do solver [m/s]: "velocidade de impacto" */
} pc_contact_event;

/* Eventos acumulados desde o último pc_world_clear_contact_events (ou do início do último update/step: o motor
 * os esvazia no começo do update/step seguinte). Devolve o nº total; copia até `capacity` para out (pode ser NULL). */
PC_API int pc_world_read_contact_events(const pc_world* world, uint32_t capacity, pc_contact_event* out);
PC_API int pc_world_clear_contact_events(pc_world* world);

/* ---- consultas espaciais ------------------------------------------------------------------------------------------ */
#define PC_QUERY_STATIC 1u
#define PC_QUERY_KINEMATIC 2u
#define PC_QUERY_DYNAMIC 4u
#define PC_QUERY_ALL 7u

typedef struct pc_raycast_hit {
    pc_body_id body;
    double distance;
    pc_vec3 point;
    pc_vec3 normal;                /* para fora do corpo */
    int32_t started_inside;        /* a origem já estava dentro do corpo */
} pc_raycast_hit;

/* Acerto mais próximo: devolve 1 (acertou, *out preenchido) ou 0 (nada), < 0 erro. max_distance pode ser INFINITY.
 * query_flags: PC_QUERY_*; ignore_body: corpo a ignorar (ou PC_INVALID_BODY). */
PC_API int pc_world_raycast(const pc_world* world, const pc_vec3* origin, const pc_vec3* direction, double max_distance,
                            uint32_t query_flags, pc_body_id ignore_body, pc_raycast_hit* out);
/* Todos os acertos ordenados por distância: devolve o total; copia até `capacity` (out pode ser NULL). */
PC_API int pc_world_raycast_all(const pc_world* world, const pc_vec3* origin, const pc_vec3* direction, double max_distance,
                                uint32_t query_flags, pc_body_id ignore_body, uint32_t capacity, pc_raycast_hit* out);
/* Corpos que tocam a esfera: devolve o total; copia até `capacity` ids (ordem crescente). */
PC_API int pc_world_overlap_sphere(const pc_world* world, const pc_vec3* center, double radius, uint32_t query_flags,
                                   pc_body_id ignore_body, uint32_t capacity, pc_body_id* out_ids);
PC_API int pc_world_overlap_point(const pc_world* world, const pc_vec3* point, uint32_t query_flags,
                                  pc_body_id ignore_body, uint32_t capacity, pc_body_id* out_ids);

#ifdef __cplusplus
}
#endif
#endif /* PHYSICS_C_H */
