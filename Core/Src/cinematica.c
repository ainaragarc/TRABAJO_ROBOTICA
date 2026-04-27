#include <stdlib.h>
#include "cinematica.h"

static float vx_prev = 0.0f;
static float vy_prev = 0.0f;
static float vz_prev = 0.0f;

c4d posicion_actual(void)      { return cinematica_directa(get_motoresg()); }
motoresg motoresg_actual(void) { return get_motoresg(); }

float radianes(float g) { return (g * (M_PI / 180.0f)); }
float grados(float r)   { return (r * (180.0f / M_PI)); }

motoresg conv_grados_rad(motoresg mot) {
    mot.base = grados(mot.base);
    mot.r1   = grados(mot.r1);
    mot.r2   = grados(mot.r2);
    mot.r3   = grados(mot.r3);
    return mot;
}

float restriccion_angulos(float a) {
    if (a < 0)   a = 0;
    else if (a > 180) a = 180;
    return a;
}

bool dentro_rango(c3d cor) {
    if (cor.z < 0 || cor.z > LBASE)         return false;
    if (cor.y < 20 || cor.y > SEPy + plano) return false;
    if (cor.x > x0 + cor.y - 20)            return false;
    /* M4: cota inferior de x — el brazo no puede extenderse más de L1+L2 hacia atrás */
    if (cor.x < -(L1 + L2))                 return false;
    return true;
}

// ── 1. Convergencia y estado de velocidades ──────────────────────────────────

bool objetivo_alcanzado(c3d act, c3d obj, float tol_mm) {
    float dx = (float)(act.x - obj.x);
    float dy = (float)(act.y - obj.y);
    float dz = (float)(act.z - obj.z);
    return (dx*dx + dy*dy + dz*dz) <= (tol_mm * tol_mm);
}

void velocidad_reset(void) {
    vx_prev = 0.0f;
    vy_prev = 0.0f;
    vz_prev = 0.0f;
}

// ── 2. Paso de coordenadas a plano ───────────────────────────────────────────

c3d plano_dibujo(c2d cor) {
    c3d vuelta;
    if (cor.z < 0)    cor.z = SEPz;
    if (cor.y < 0)    cor.y = SEPy;
    if (cor.z > 200)  cor.z = SEPz + 200;
    if (cor.y > 200)  cor.y = SEPy + 200;
    vuelta.z = cor.z + SEPz;
    vuelta.y = (int16_t)(cor.y * ANG) + SEPy;
    vuelta.x = (int16_t)(cor.y * ANG) + SEPx;
    return vuelta;
}

c3d plano_retroceso(c2d cor) {
    c3d vuelta;
    if (cor.z < 0)    cor.z = SEPz;
    if (cor.y < 0)    cor.y = SEPy;
    if (cor.z > 200)  cor.z = SEPz + 200;
    if (cor.y > 200)  cor.y = SEPy + 200;
    vuelta.z = cor.z + SEPz;
    vuelta.y = (int16_t)(cor.y * ANG) + SEPy;
    vuelta.x = (int16_t)(cor.y * ANG) + SEPx - 20;
    return vuelta;
}

// ── 3. Cinemática directa ─────────────────────────────────────────────────────

c4d cinematica_directa(motoresg m) {
    c4d vuelta = {0};

    vuelta.coor.z = (int16_t)roundf(m.base);

    float t1 = radianes(-m.r1);
    float t2 = radianes(-m.r2);
    float t3 = radianes(-m.r3);

    vuelta.coor.x = (int16_t)roundf(-(L1*cosf(t1) + L2*cosf(t1+t2) + L3*(cosf(t3+t1+t2))));
    vuelta.coor.y = (int16_t)roundf(-(L1*sinf(t1) + L2*sinf(t1+t2) + L3*(sinf(t3+t1+t2))));
    vuelta.ang    = grados(t1 + t2 + t3);

    return vuelta;
}

// ── 4. Cinemática inversa ─────────────────────────────────────────────────────

motoresg cinematica_inversa(c3d cor) {
    motoresg mot = {0};
    mot.base = (float)cor.z;

    float x    = cor.x - L3;   /* posición de la muñeca (L3 horizontal) */
    float y    = cor.y;
    float arg2 = (x*x + y*y - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    if (arg2 >  1.0f) arg2 =  1.0f;
    if (arg2 < -1.0f) arg2 = -1.0f;

    /* El IK de 2 eslabones tiene dos soluciones (codo arriba / codo abajo).
       Se elige la que deje R1 más cerca de R1_PREFERIDO dentro de [R1_MIN, R1_MAX],
       reduciendo así el torque gravitacional sobre el motor de inclinación. */
    float t2_A = -acosf(arg2);   /* codo arriba */
    float t2_B =  acosf(arg2);   /* codo abajo  */

    float base_ang = atan2f(y, x);
    float t1_A = base_ang - atan2f(L2*sinf(t2_A), L1 + L2*cosf(t2_A));
    float t1_B = base_ang - atan2f(L2*sinf(t2_B), L1 + L2*cosf(t2_B));

    float r1_A = 180.0f - grados(t1_A);
    float r1_B = 180.0f - grados(t1_B);

    bool  A_ok  = (r1_A >= R1_MIN_GRADOS && r1_A <= R1_MAX_GRADOS);
    bool  B_ok  = (r1_B >= R1_MIN_GRADOS && r1_B <= R1_MAX_GRADOS);
    bool  A_mejor;

    if (A_ok && B_ok) {
        /* Ambas en rango: la más próxima a R1_PREFERIDO */
        A_mejor = fabsf(r1_A - R1_PREFERIDO_GRADOS) <= fabsf(r1_B - R1_PREFERIDO_GRADOS);
    } else if (A_ok) {
        A_mejor = true;
    } else if (B_ok) {
        A_mejor = false;
    } else {
        /* Ninguna en rango: la menos mala */
        A_mejor = fabsf(r1_A - R1_PREFERIDO_GRADOS) <= fabsf(r1_B - R1_PREFERIDO_GRADOS);
    }

    float t1 = A_mejor ? t1_A : t1_B;
    float t2 = A_mejor ? t2_A : t2_B;
    float t3 = -(t1 + t2);

    mot.r1 = restriccion_angulos(roundf(180.0f - grados(t1)));
    mot.r2 = fminf(R2R3_MAX_GRADOS, restriccion_angulos(roundf(-grados(t2))));
    mot.r3 = fminf(R2R3_MAX_GRADOS, restriccion_angulos(roundf(-grados(t3))));

    return mot;
}

// ── 5. Jacobiano ─────────────────────────────────────────────────────────────

jcb2 jacobiana(float t1, float t2) {
    jcb2 J;
    J.j11 =  L1 * sinf(t1) + L2 * sinf(t1+t2);
    J.j12 =  L2 * sinf(t1+t2);
    J.j21 = -L1 * cosf(t1) - L2 * cosf(t1+t2);
    J.j22 = -L2 * cosf(t1+t2);
    return J;
}

void jacobiana_siguiente(float t1, float t2, float xd, float yd,
                         float *t1d, float *t2d, float *t3d) {
    jcb2 J = jacobiana(t1, t2);
    float landa = 0.01f;

    float a11 = J.j11*J.j11 + J.j12*J.j12 + landa*landa;
    float a12 = J.j11*J.j21 + J.j12*J.j22;
    float a21 = a12;
    float a22 = J.j21*J.j21 + J.j22*J.j22 + landa*landa;

    float detA = a11*a22 - a12*a21;
    if (fabsf(detA) < 0.0001f) { *t1d = 0; *t2d = 0; *t3d = 0; return; }

    float inv = 1.0f / detA;
    float Ainv11 =  a22 * inv,  Ainv12 = -a12 * inv;
    float Ainv21 = -a21 * inv,  Ainv22 =  a11 * inv;

    float K11 = J.j11*Ainv11 + J.j21*Ainv21;
    float K12 = J.j11*Ainv12 + J.j21*Ainv22;
    float K21 = J.j12*Ainv11 + J.j22*Ainv21;
    float K22 = J.j12*Ainv12 + J.j22*Ainv22;

    *t1d = K11 * xd + K12 * yd;
    *t2d = K21 * xd + K22 * yd;
    *t3d = -(*t1d + *t2d);
}

// ── 6. Velocidades ────────────────────────────────────────────────────────────

void velocidad_dibujo_recta(c3d act, c3d objetivo, c3d *velocidades) {
    float dx   = (float)(objetivo.x - act.x);
    float dy   = (float)(objetivo.y - act.y);
    int16_t dz = objetivo.z - act.z;
    float d_xy = sqrtf(dx*dx + dy*dy);

    if (d_xy < 0.001f && dz == 0) {
        velocidades->x = 0; velocidades->y = 0; velocidades->z = 0;
        return;
    }
    if (d_xy < 0.001f) {
        velocidades->x = 0; velocidades->y = 0;
    } else {
        velocidades->x = vconst * dx / d_xy;
        velocidades->y = vconst * dy / d_xy;
    }
    velocidades->z = (dz == 0) ? 0 : vconst * (dz > 0 ? 1 : -1);
}

void velocidad_recta(c3d act, c3d obj, c3d *velocidades) {
    if (!dentro_rango(obj)) {
        if (obj.z < 0)            obj.z = 0;
        else if (obj.z > LBASE)   obj.z = LBASE;
        if (obj.y < 20)           obj.y = 20;
        else if (obj.y > SEPy+plano) obj.y = SEPy + plano;
        if (obj.x > x0+obj.y-20) obj.x = x0 + obj.y - 20;
    }

    float dx = (float)(obj.x - act.x);
    float dy = (float)(obj.y - act.y);
    float dz = (float)(obj.z - act.z);
    float K  = 0.1f;

    float vx = K * dx, vy = K * dy, vz = K * dz;

    float vxy = sqrtf(vx*vx + vy*vy);
    if (vxy > vmax) { float esc = vmax/vxy; vx *= esc; vy *= esc; }
    if (fabsf(vz) > vmax) vz = (vz > 0 ? vmax : -vmax);

    float dvx = vx - vx_prev, dvy = vy - vy_prev, dvz = vz - vz_prev;
    float max_step = amax * DT_CONTROL;

    if (dvx >  max_step) dvx =  max_step;
    if (dvx < -max_step) dvx = -max_step;
    if (dvy >  max_step) dvy =  max_step;
    if (dvy < -max_step) dvy = -max_step;
    if (dvz >  max_step) dvz =  max_step;
    if (dvz < -max_step) dvz = -max_step;

    float vx_def = vx_prev + dvx;
    float vy_def = vy_prev + dvy;
    float vz_def = vz_prev + dvz;

    if (fabsf(vx_def) < 1.0f) vx_def = 0.0f;
    if (fabsf(vy_def) < 1.0f) vy_def = 0.0f;
    if (fabsf(vz_def) < 1.0f) vz_def = 0.0f;

    velocidades->x = (int16_t)vx_def;
    velocidades->y = (int16_t)vy_def;
    velocidades->z = (int16_t)vz_def;

    vx_prev = vx_def; vy_prev = vy_def; vz_prev = vz_def;
}

void velocidad_transicion(c3d act, c3d obj, c3d *velocidades, uint8_t flag) {
    int16_t dir;
    if (flag == 1)      dir =  1;
    else if (flag == 3) dir = -1;
    else return;

    velocidades->x = dir * (vconst / 2);
    velocidades->z = 0;
    velocidades->y = 0;
    if (act.x == obj.x) velocidades->x = 0;
}

// ── 7. Trayectoria jacobiana ──────────────────────────────────────────────────

bool trayectoria(motoresg *salida, c3d obj, uint8_t *flagdibujo) {
    motoresg motores_actualesg = get_motoresg();
    c4d act = cinematica_directa(motores_actualesg);

    if (objetivo_alcanzado(act.coor, obj, TOLERANCIA_MM)) {
        velocidad_reset();
        *salida = motores_actualesg;
        return true;
    }

    c3d velocidades;
    switch (*flagdibujo) {
        case 0: velocidad_recta(act.coor, obj, &velocidades);                    break;
        case 1:
        case 3: velocidad_transicion(act.coor, obj, &velocidades, *flagdibujo);  break;
        case 2: velocidad_dibujo_recta(act.coor, obj, &velocidades);             break;
        default: velocidades.x = 0; velocidades.y = 0; velocidades.z = 0;       break;
    }

    /* t_fk = -R_grados * π/180 — convención FK interna */
    float t1 = radianes(-motores_actualesg.r1);
    float t2 = radianes(-motores_actualesg.r2);
    float t3 = radianes(-motores_actualesg.r3);

    float t1dot, t2dot, t3dot;
    jacobiana_siguiente(t1, t2, velocidades.x, velocidades.y, &t1dot, &t2dot, &t3dot);

    t1 += t1dot * DT_CONTROL;
    t2 += t2dot * DT_CONTROL;
    t3 += t3dot * DT_CONTROL;

    /* Inversión del mapeo FK: R = -grados(t_fk) */
    motoresg vuelta = motores_actualesg;
    vuelta.r1 = restriccion_angulos(-grados(t1));
    vuelta.r2 = restriccion_angulos(-grados(t2));
    vuelta.r3 = restriccion_angulos(-grados(t3));

    /* Base en mm: usar el float almacenado, no el int16_t de la FK */
    vuelta.base = motores_actualesg.base + (float)velocidades.z * DT_CONTROL;

    *salida = vuelta;
    return false;
}

// ── 8. Trayectoria Bresenham ──────────────────────────────────────────────────

bool trayectoria_cutre(motoresg *salida, c3d obj, uint8_t *flagdibujo) {
    motoresg motores_actualesg = get_motoresg();
    c4d act4 = cinematica_directa(motores_actualesg);
    c3d act  = act4.coor;

    if (objetivo_alcanzado(act, obj, TOLERANCIA_MM)) {
        *salida = motores_actualesg;
        return true;
    }

    int dx = abs(obj.x - act.x);
    int dy = abs(obj.y - act.y);
    int dz = abs(obj.z - act.z);
    int sx = (act.x < obj.x) ? 1 : -1;
    int sy = (act.y < obj.y) ? 1 : -1;
    int sz = (act.z < obj.z) ? 1 : -1;
    int paso = (*flagdibujo == 1 || *flagdibujo == 3) ? 1 : 2;

    if (dx >= dy && dx >= dz) {
        act.x += sx * paso;
        if ((sx > 0 && act.x > obj.x) || (sx < 0 && act.x < obj.x)) act.x = obj.x;
        if (dx != 0) {
            int dy_step = (dy * paso) / dx;
            int dz_step = (dz * paso) / dx;
            if (dy_step == 0 && dy != 0) dy_step = sy;
            if (dz_step == 0 && dz != 0) dz_step = sz;
            act.y += dy_step; act.z += dz_step;
        }
    } else if (dy >= dx && dy >= dz) {
        act.y += sy * paso;
        if ((sy > 0 && act.y > obj.y) || (sy < 0 && act.y < obj.y)) act.y = obj.y;
        if (dy != 0) {
            int dx_step = (dx * paso) / dy;
            int dz_step = (dz * paso) / dy;
            if (dx_step == 0 && dx != 0) dx_step = sx;
            if (dz_step == 0 && dz != 0) dz_step = sz;
            act.x += dx_step; act.z += dz_step;
        }
    } else {
        act.z += sz * paso;
        if ((sz > 0 && act.z > obj.z) || (sz < 0 && act.z < obj.z)) act.z = obj.z;
        if (dz != 0) {
            int dx_step = (dx * paso) / dz;
            int dy_step = (dy * paso) / dz;
            if (dx_step == 0 && dx != 0) dx_step = sx;
            if (dy_step == 0 && dy != 0) dy_step = sy;
            act.x += dx_step; act.y += dy_step;
        }
    }

    // saturación
    if (sx > 0 && act.x > obj.x) act.x = obj.x;
    if (sx < 0 && act.x < obj.x) act.x = obj.x;
    if (sy > 0 && act.y > obj.y) act.y = obj.y;
    if (sy < 0 && act.y < obj.y) act.y = obj.y;
    if (sz > 0 && act.z > obj.z) act.z = obj.z;
    if (sz < 0 && act.z < obj.z) act.z = obj.z;

    c3d siguiente = { act.x, act.y, act.z };
    *salida = cinematica_inversa(siguiente);
    return false;
}
