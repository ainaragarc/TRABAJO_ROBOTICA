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
#define L1 20 //longitud elabon 1
#define L2 20 //longitud elabon 2
#define L3 20 //longitud elabon 3 (revolver + boli)

//OTRAS MEDIDAS IMPORTANTES
#define SEPz 20 //separacion entre el inicio de la corredera y el inicio del lienzo
#define SEPx 20 //separacion entre centro de gravedad del robot (la movil) HORIZONTAL
#define SEPy 20 //separacion entre centro de gravedad del robot (la movil) VERTICAL
#define plano 200 //separacion entre centro de gravedad del robot (la movil) VERTICAL

#define ANG (0.70710678f) //inclinacion a 45º
#define ANGinicial (0.0f) //angulo incial debido al final de carrera en grados


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

typedef struct
{
	int16_t base;
	int16_t r1;
	int16_t r2;
	int16_t r3;
} motores;

typedef struct
{
	float base; //LA BASE TMB VA EN GRADOS NO EN DESPLAZAMIENTO, MUCHO CUIDADITO
	float r1;
	float r2;
	float r3;
} motoresg;

float radianes (float g){return (g* (M_PI / 180.0));}
float grados (float r){return (r* (180.0 / M_PI));}
motores conv_entero( motoresg mot);
motoresg conv_grados( motores mot);
motoresg conv_grados_rad( motoresg mot);
float restriccion_angulos(float a);

//1. PASO DE COORDENADAS A DIBUJO
//asumiendo que el 0,0 esta abajo a la izquierda
c3d plano_dibujo( c2d cor);



//2. CINEMATICA DIRECTA
//DEVUELVE LAS COORDENADAS DDE LA PUNTA
c4d cinematica_directa( motores mot);


//3. CINEMATICA DIRECTA
motores cinematica_inversa( c4d cor);

//4. SEGUIR TRAYECTORIAS



#endif
