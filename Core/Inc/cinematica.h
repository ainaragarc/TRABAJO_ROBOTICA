#ifndef INC_CINEMATICA_H_
#define INC_CINEMATICA_H_

#include <stdint.h>
#include <stdbool.h> //para poder usar bool
#include <stddef.h> //para poder usar NULL
#include <math.h>
#include "motores.h"

/*
	PLAN DE ESTE ARCHIVO

	- 1 - PASO DE COORDENADAS NORMALES AL PLANO_DIBUJO
	- 2 - FUNCIÓN CINEMATICA DIRECTA
	- 3 - FUNCIÓN CINEMATICA INVERSA ¡CON RESTRICCIONES!
	- 4 - FUNCIÓN SEGUIR TRAYECTORIAS

	EL EJE Y ES EL DE LA ALTURA DEL ROBOT, EL EJE X ES EL DE EL DESPLAZAMIENTO DEL ROBOT
	EJE Z ES EL DE LA CORREDERA

 */

//MEDIDA DE LOS ESLABONES
#define LBASE 430 //longitud base (corredera) en mm
#define L1 139 //longitud elabon 1
#define L2 123 //longitud elabon 2
#define L3 140 //longitud elabon 3 (revolver + boli)

//OTRAS MEDIDAS IMPORTANTES
#define SEPz 20 //separacion entre el inicio de la corredera y el inicio del lienzo
#define SEPx 20 //separacion entre centro de gravedad del robot (la movil) HORIZONTAL
#define SEPy 20 //separacion entre centro de gravedad del robot (la movil) VERTICAL

#define x0 20 //separacion a la parte mas baja del lienzo

#define plano 200 //separacion entre centro de gravedad del robot (la movil) VERTICAL

#define ANG (0.70710678f) //inclinacion a 45º
#define ANGinicial (0.0f) //angulo incial debido al final de carrera en grados

#define vmax 20
#define amax 2.0f

#define vconst 10
#define DT_CONTROL  0.01f


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//COORDENADAS

//CUIDADITO CON EL SIGNO
typedef struct
{
	int16_t z;
	int16_t y;

} c2d; //SON LAS COORDENADAS 2D DE DIBUJO (se reserva x para el movimiento rotativo del robot)

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} c3d;

typedef struct
{
	c3d coor;
	float ang;
} c4d; //coordenadas 3d mas la orientacion de la punta


typedef struct {
    float j11; //fila 1 columna 1
    float j12; //fila 1 columna 2
    float j21; //fila 2 columna 1
    float j22; //fila 2 columna 2
} jcb2;


float radianes (float g);
float grados (float r);
motores conv_entero( motoresg mot);
motoresg conv_grados( motores mot);
motoresg conv_grados_rad( motoresg mot);
float restriccion_angulos(float a);

c4d posicion_actual(void);
motoresg motoresg_actual(void);
motores motores_actual(void);


//1. PASO DE COORDENADAS A DIBUJO
//asumiendo que el 0,0 esta abajo a la izquierda
c3d plano_dibujo( c2d cor); //da coordenada del plano de dibujo
c3d plano_no_dibujo(c2d cor); //da coordenada a 2cm del plano de  dibujo
bool dentro_rango( c3d cor);



//2. CINEMATICA DIRECTA
//DEVUELVE LAS COORDENADAS DDE LA PUNTA
c4d cinematica_directa( motoresg m);


//3. CINEMATICA INVERSA
motoresg cinematica_inversa( c3d cor);

//4. SEGUIR TRAYECTORIAS
jcb2 jacobiana(float t1, float t2);
void jacobiana_siguiente(float t1, float t2, float xd, float yd, float *t1d, float *t2d, float *t3d);

void velocidad_dibujo_recta(c3d act, c3d objetivo, c3d *velocidades);
void velocidad_recta(c3d act, c3d obj, c3d *velocidades);
void velocidad_transicion(c3d act, c3d obj, c3d *velocidades, uint8_t flag);

bool trayectoria(motoresg *salida, c3d obj, uint8_t *flagdibujo);

bool trayectoria_cutre(motoresg *salida, c3d obj, uint8_t *flagdibujo);

#endif
