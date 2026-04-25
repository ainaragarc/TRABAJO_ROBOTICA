#include "movimiento.h"

static Color color_act=COLOR1;
static Color color_inicial=COLOR1;


//PODRIA EMPEZAR EN 0 TMB
static uint8_t flag_pasos=7; //nuemro de pasos del dibujo
static uint8_t flag_trayctorias=0; //iniciada en 0 porque no dibujo
static uint8_t npuntos=0;


static puntos dibujo={0};

static Color siguiente_color(Color c) {
    switch (c) {
    case COLOR1: return COLOR2;
    case COLOR2: return COLOR3;
    case COLOR3: return COLOR1;
    default:     return COLOR1;
    }
}



void matriz_aleatoria(void){
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < NUMERO_PUNTOS; j++){
            dibujo.p[i][j].y = rand() % 200;
            dibujo.p[i][j].z = rand() % 200;
        }
    }
}

//////////////////////////////////////////////////////
//CAMBIO DE COLOR

bool cambio_color_revolver(Color c, TIM_HandleTypeDef *htim){
	if (c==color_act) return true;
    c4d act = posicion_actual();

	if (!dentro_rango(act.coor)) return false; //SUGNIFICA QUE ESTA PEGADO AL LIENZO Y QUE HAY QUE SEPARARLO

	switch (c){
	case COLOR1:
		set_servo_revolver(htim, entero_revol(0)); //rotar a 0
		break;
	case COLOR2:
		set_servo_revolver(htim, entero_revol(90)); //rotar a 90
		break;
	case COLOR3:
		set_servo_revolver(htim, entero_revol(180)); //rotar a 180
		break;
	}

	color_act=c;
	return true;
}

//////////////////////////////////////////////////////
//DIBUJO RANDOM

/*
	6--> NO HAY DIBUJO, no matriz
	5--> rumbo pintura
	4--> transicion hacia pintar
	3--> pintando
	2--> transicion despintar2
	1 --> cambio de color
*/


//AQUI FALTA UUNA LLAMNADICA A LOS MOTORES
void dibujar( uint8_t flag_parada, uint8_t flag_ON, TIM_HandleTypeDef *htim_revolver){

	c3d objetivo;
	bool i=false;
    motoresg movimiento_motores;

	if (flag_parada>0){ flag_pasos=0; }

	switch (flag_pasos){
	case 7: //SACAR MATRIZ
		//POSICION DE REPOSO
		if (flag_ON==1){flag_pasos=6;}
		//RESETEO
		color_inicial=color_act;
		npuntos=0;
		break;

	case 6: //SACAR MATRIZ
		matriz_aleatoria();
		flag_pasos=5;
		break;

	case 5: //RUMBO AL PRIMER PUNTO
		flag_trayctorias=0;
		objetivo = plano_no_dibujo(dibujo.p[color_act][npuntos]); //punto objetivo
		i = trayectoria( &movimiento_motores ,objetivo, &flag_trayctorias);
		control_loop_motores(movimiento_motores);

		if (i==true){
			flag_pasos=4;
		}


		break;

	case 4://TRANSICION A DIBUJAR
		flag_trayctorias=1;
		objetivo = plano_dibujo(dibujo.p[color_act][npuntos]); //punto objetivo
		i = trayectoria( &movimiento_motores ,objetivo, &flag_trayctorias);
		control_loop_motores(movimiento_motores);

		if (i==true){
			flag_pasos=3;
			i=false;
		}

		break;

	case 3://DIBUJA
		flag_trayctorias=2;

		objetivo = plano_dibujo(dibujo.p[color_act][npuntos]); //punto objetivo
		i = trayectoria( &movimiento_motores ,objetivo, &flag_trayctorias);
		control_loop_motores(movimiento_motores);

		if (i==true){
			if(npuntos==(NUMERO_PUNTOS-1)){ flag_pasos=2;} //si acaba eñ ultimo punto modifica estado
			else{npuntos++;}// si no va a por punto siguiente
			i=false;
		}

		break;

	case 2: //despegarse del lienzo

		flag_trayctorias=3;
		objetivo = plano_no_dibujo(dibujo.p[color_act][npuntos]); //punto objetivo
		i = trayectoria( &movimiento_motores ,objetivo, &flag_trayctorias);
		control_loop_motores(movimiento_motores);

		if (i==true){
			npuntos=0; // SE RESETEA AL PUNTO 0
			flag_pasos=1;
			i=false;
		}


		break;

	case 1://CAMBIO DE COLOR
	    Color nuevo = siguiente_color(color_act);
		i= cambio_color_revolver(nuevo, htim_revolver);

		if (i==true){
			if(color_act==color_inicial){ flag_pasos=0;} //acaba y va a reposo
			else{flag_pasos=5;}// si no va a por punto siguiente
			i=false;
		}

		break;

	case 0://IR A REPOSO
		flag_trayctorias=0;
		//objetivo = posicion_reposo(); //POSICION RESPOSO
		i = trayectoria( &movimiento_motores ,objetivo, &flag_trayctorias);
		control_loop_motores(movimiento_motores);

		if (flag_parada==0 && i==true){
			flag_pasos=7;
		}


		break;
	}

}


void prueba_dibujar(c3d objetivo, uint8_t flag_trayctorias){
	bool i=false;
    motoresg movimiento_motores;
	i = trayectoria( &movimiento_motores ,objetivo, &flag_trayctorias);
	control_loop_motores(movimiento_motores);

}
