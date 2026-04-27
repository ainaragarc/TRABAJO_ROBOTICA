#ifndef INC_CINEMATICA_H_
#define INC_CINEMATICA_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "motores.h"

/*
 * SISTEMA DE COORDENADAS DEL ROBOT
 * ────────────────────────────────────────────────────────────────────────────
 *  Z  (corredera)  mm desde home (PA9 = limiteTraslacion_A). Rango 0..LBASE.
 *  X  (alcance)    mm desde pivote del brazo, positivo hacia el lienzo.
 *  Y  (altura)     mm desde pivote del brazo, positivo hacia arriba.
 *
 * ÁNGULOS DE MOTORES  (todos en grados, rango 0..180)
 *  R1 hombro   0 = horizontal hacia atrás (home, PA8 activo), 90 = brazo
 *              vertical arriba, 180 = horizontal hacia el lienzo.
 *              Encoder incrementa al girar hacia arriba. TIM2 (PA0/PB3).
 *  R2 codo     0 = extendido (L2 continúa dirección de L1).
 *              Servo posicional TIM5_CH3.
 *  R3 muñeca   0 = extendido.
 *              Servo posicional TIM5_CH4.
 *
 * CONVENCIÓN FK INTERNA
 *  t_fk = -R_grados * π/180   (flip de signo por hardware)
 *  x =  -(L1·cos t1 + L2·cos(t1+t2) + L3·cos(t1+t2+t3))
 *  y =  -(L1·sin t1 + L2·sin(t1+t2) + L3·sin(t1+t2+t3))
 *  La IK es coherente: devuelve R = -grados(t_ik) con t_ik de atan2.
 *  La IK asume L3 horizontal (t3 = -(t1+t2)), que mantiene la herramienta
 *  perpendicular al lienzo durante el dibujo.
 *
 * LIENZO
 *  El lienzo está inclinado ~45° respecto al brazo (ANG = sin 45°).
 *  Coordenadas de dibujo c2d: z = posición en corredera, y = altura en lienzo.
 *  plano_dibujo    → punto en contacto con el lienzo.
 *  plano_retroceso → mismo punto pero 20 mm retirado del lienzo.
 *
 * MODOS DE TRAYECTORIA (flagdibujo)
 *  0 → desplazamiento libre hacia objetivo (sin contacto)
 *  1 → aproximación al lienzo (avanza en X hasta SEPx)
 *  2 → dibujo (mantiene X, mueve Y y Z)
 *  3 → retirada del lienzo (retrocede X hasta SEPx-20)
 * ────────────────────────────────────────────────────────────────────────────
 */

/* ── Dimensiones del robot ───────────────────────────────────────────────── */
#define LBASE   430         /* longitud útil de la corredera (mm)            */
#define L1      139         /* longitud eslabón 1, hombro-codo (mm)          */
#define L2      123         /* longitud eslabón 2, codo-muñeca (mm)          */
#define L3      140         /* longitud eslabón 3, muñeca-herramienta (mm)   */

/* ── Referencias del lienzo ──────────────────────────────────────────────── */
#define SEPz    20          /* mm entre home corredera y borde inferior lienzo*/
#define SEPx    100         /* alcance nominal al lienzo (mm)                */
#define SEPy    20          /* altura nominal del borde inferior del lienzo   */
#define x0      100         /* alcance al punto más bajo del lienzo           */
#define plano   200         /* altura del lienzo (mm)                         */

/* ── Parámetros cinemáticos ─────────────────────────────────────────────── */
#define ANG             0.70710678f /* sin/cos 45° — inclinación del lienzo  */
#define ANGinicial      0.0f        /* ángulo R1 en el fin de carrera PA8     */

/* Rango de R1 para minimizar torque gravitacional en el motor.
   La IK elige la solución (codo arriba/abajo) cuyo R1 esté más cerca de
   R1_PREFERIDO_GRADOS dentro de [R1_MIN, R1_MAX]. Si ninguna cabe en el
   rango, usa la más cercana (sin forzar el motor a una posición imposible).
   Ajusta R1_MAX según dónde coloques el lienzo. */
#define R1_PREFERIDO_GRADOS   90.0f   /* objetivo ideal: mínimo torque          */
#define R1_MIN_GRADOS         80.0f   /* no bajar de aquí                       */
#define R1_MAX_GRADOS        150.0f   /* límite superior — mueve el lienzo      */
#define R2R3_MAX_GRADOS       30.0f   /* máximo ángulo de codo y muñeca         */

#define vmax            20.0f       /* velocidad cartesiana máxima (mm/s)     */
#define amax            2.0f        /* aceleración máxima (mm/s²)             */
#define vconst          10.0f       /* velocidad constante de dibujo (mm/s)   */
#define TOLERANCIA_MM   2.0f        /* tolerancia de llegada al objetivo (mm) */
/* DT_CONTROL viene de motores.h (incluido arriba) */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Tipos de coordenadas ────────────────────────────────────────────────── */
typedef struct { int16_t z; int16_t y; }          c2d; /* coord. sobre lienzo */
typedef struct { int16_t x; int16_t y; int16_t z; } c3d; /* coord. robot 3D  */
typedef struct { c3d coor; float ang; }            c4d; /* pose (pos+ángulo)  */

typedef struct {
    float j11; float j12;
    float j21; float j22;
} jcb2;

/* ── Conversiones ────────────────────────────────────────────────────────── */
float    radianes(float g);
float    grados(float r);
motoresg conv_grados_rad(motoresg mot);
float    restriccion_angulos(float a);

/* ── Estado actual ───────────────────────────────────────────────────────── */
c4d      posicion_actual(void);
motoresg motoresg_actual(void);

/* ── Transformadas de lienzo ─────────────────────────────────────────────── */
c3d  plano_dibujo(c2d cor);      /* punto de contacto con el lienzo          */
c3d  plano_retroceso(c2d cor);   /* mismo punto, 20 mm retirado del lienzo   */
bool dentro_rango(c3d cor);

/* ── Cinemática directa e inversa ────────────────────────────────────────── */
c4d      cinematica_directa(motoresg m);
motoresg cinematica_inversa(c3d cor);

/* ── Planificación de trayectorias ───────────────────────────────────────── */
bool objetivo_alcanzado(c3d act, c3d obj, float tol_mm);
void velocidad_reset(void);

jcb2 jacobiana(float t1, float t2);
void jacobiana_siguiente(float t1, float t2, float xd, float yd,
                         float *t1d, float *t2d, float *t3d);

void velocidad_recta(c3d act, c3d obj, c3d *vel);
void velocidad_dibujo_recta(c3d act, c3d objetivo, c3d *vel);
void velocidad_transicion(c3d act, c3d obj, c3d *vel, uint8_t flag);

bool trayectoria(motoresg *salida, c3d obj, uint8_t *flagdibujo);
bool trayectoria_cutre(motoresg *salida, c3d obj, uint8_t *flagdibujo);

#endif /* INC_CINEMATICA_H_ */
