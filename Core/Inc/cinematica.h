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
#define LBASE 430u //longitud base (corredera) en mm
#define L1 20u //longitud elabon 1
#define L2 20u //longitud elabon 2
#define L3 20u //longitud elabon 3 (revolver + boli)

//OTRAS MEDIDAS IMPORTANTES
#define SEPz 20u //separacion entre centro de gravedad del robot (la movil) HORIZONTAL
#define SEPy 20u //separacion entre centro de gravedad del robot (la movil) VERTICAL
#define plano 200u //separacion entre centro de gravedad del robot (la movil) VERTICAL

#define ANG (0.70710678f) //inclinacion a 45º


//COORDENADAS

typedef struct
{
	uint16_t z;
	uint16_t y;

} c2d; //SON LAS COORDENADAS 2D DE DIBUJO (se reserva x para el movimiento rotativo del robot)

typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
} c3d;

typedef struct
{
	uint16_t base;
	uint16_t r1;
	uint16_t r2;
	uint16_t r3;
} motores;

typedef struct
{
	float base;
	float r1;
	float r2;
	float r3;
} motoresg;


//1. PASO DE COORDENADAS A DIBUJO
//asumiendo que el 0,0 esta abajo a la izquierda
c3d plano_dibujo( c2d cor);

motores conv_entero( motoresg mot);
motoresg conv_grados( motores mot);


//2. CINEMATICA DIRECTA
c3d cinematica_directa( motores mot);

//3. CINEMATICA DIRECTA
motores cinematica_inversa( c3d cor);

//4. SEGUIR TRAYECTORIAS



#endif
