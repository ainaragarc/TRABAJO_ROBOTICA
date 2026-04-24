#include "cinematica.h"

static motores motores_actuales; //esto deberia ser la posicion de los motores leido por los sensores
//static motores motores_actuales=LECTURA_MOTORES();

static float vx_prev=0.0f;
static float vy_prev=0.0f;
static float vz_prev=0.0f;

//static float margen_igualdad = 0.5f;

c4d posicion_actual(void){ return cinematica_directa(motores_actuales); }
motoresg motoresg_actual(void){ return conv_grados(motores_actuales); }
motores motores_actual(void){ return (motores_actuales); }

float radianes (float g){return (g* (M_PI / 180.0));}
float grados (float r){return (r* (180.0 / M_PI));}


motores conv_entero( motoresg mot){
	motores i;
	i.base = entero_paso(mot.base);
	i.r1 = entero_rot(mot.r1);
	i.r2 = entero_pos(mot.r2);
	i.r3 = entero_pos(mot.r3);

	return i;
}

motoresg conv_grados( motores mot){
	motoresg i;
	i.base = grados_paso(mot.base);
	i.r1 = grados_rot(mot.r1);
	i.r2 = grados_pos(mot.r2);
	i.r3 = grados_pos(mot.r3);

	return i;
}


motoresg conv_grados_rad( motoresg mot){
	mot.base = grados(mot.base);
	mot.r1 = grados(mot.r1);
	mot.r2 = grados(mot.r2);
	mot.r3 = grados(mot.r3);
	return mot;
}

float restriccion_angulos(float a){
	if (a<0) a=0;
	else if (a>180) a=180;
	return a;
}

bool dentro_rango( c3d cor){

	//Nos aseguramos de que este dentro del lienzo
	if (cor.z<0 || cor.z>LBASE) return false;
	if (cor.y<20 || cor.y>SEPy+plano) return false;
	if (cor.x>x0+cor.y-20) return false;

	return true;
}


//////////////////////////////////////////////////////
//1. PASO DE COORDENADAS A DIBUJO

c3d plano_dibujo( c2d cor){
	c3d vuelta;

	//Nos aseguramos de que este dentro del lienzo
	if (cor.z<0) cor.z=SEPz;
	if (cor.y<0) cor.y=SEPy;
	if (cor.z>(200)) cor.z=SEPz+200;
	if (cor.y>(200)) cor.y=SEPy+200;

	//transformación a sistema 3d
	vuelta.z = cor.z + SEPz;
	vuelta.y = (int16_t)(cor.y * ANG) + SEPy;
	vuelta.x = (int16_t)(cor.y * ANG) + SEPx;
	return vuelta;
}

c3d plano_no_dibujo( c2d cor){
	c3d vuelta;

	//Nos aseguramos de que este dentro del lienzo
	if (cor.z<0) cor.z=SEPz;
	if (cor.y<0) cor.y=SEPy;
	if (cor.z>(200)) cor.z=SEPz+200;
	if (cor.y>(200)) cor.y=SEPy+200;

	//transformación a sistema 3d
	vuelta.z = cor.z + SEPz;
	vuelta.y = (int16_t)(cor.y * ANG) + SEPy;
	vuelta.x = (int16_t)(cor.y * ANG) + SEPx-20;
	return vuelta;
}


//////////////////////////////////////////////////////
//2. CINEMATICA DIRECTA

c4d cinematica_directa( motores mot){

	c4d vuelta = {0};
	motoresg m = conv_grados(mot);

	//1 vuelta son 8mm
	vuelta.coor.z= (int16_t)roundf((m.base * 8.0f) / 360.0f);


	/*
	PROBLEMA, TIENE CEGUERA DE 44º
	si es 179->3mm
	si es 180->4mm
	si es 224->4mm
	si es 225->5mm
	*/

	//ec cinematica directa

	//angulos en radianes
	/* SE CAMBIAN PORQUE SOY IMBECIL, desde el 0,0 tenian que estar obligaados a hacer mas de 180º
	float t1= radianes(-m.r1);
	float t2= radianes(-90-ANGinicial-m.r2);
	float t3=radianes(90 - m.r3);
	*/
	float t1= radianes(-m.r1);
	float t2= radianes(-m.r2);
	float t3= radianes(-m.r3);

	//algunas operaciones para que sea menos insufrible
	//cos(theta1)sen(theta2)+sen(theta1)cos(theta2)
	//float a=cosf(t1)*sinf(t2)+sinf(t1)*cosf(t2)=sinf(t1+t2)

	vuelta.coor.x= (int16_t)roundf(-(L1*cosf(t1)+L2*cosf(t1+t2) +L3*(cosf(t3+t1+t2))));
	vuelta.coor.y= (int16_t)roundf(-(L1*sinf(t1)+L2*sinf(t1+t2) +L3*(sinf(t3+t1+t2))));
	vuelta.ang= (t1+t2+t3); //asinf(sinf(t1+t2+t3))
	vuelta.ang=grados(vuelta.ang);

	return vuelta;
}



//////////////////////////////////////////////////////
//3. CINEMATICA INVERSA

motoresg cinematica_inversa( c3d cor){

	motoresg mot={0};

	//ec cinematica inversa

	//1 vuelta son 8mm, probablemente tmb sufra ceguera
	mot.base =roundf(cor.z * 360.0f / 8.0f);

	//ec cinematica inversa
	float x = cor.x-L3;
	float y = cor.y;


	//el cos(y) por el teorema del coseno
	float t2 = -acos((x*x + y*y - L1*L1 - L2*L2) / (2.0f * L1 * L2));
	float t1 = atan2f(y, x) - atan2f(L2*sinf(t2), L1 + L2*cosf(t2));
	float t3 = - (t1 + t2);


	mot.r1 = restriccion_angulos(roundf(180-grados(t1)));
	mot.r2 = restriccion_angulos(roundf(-grados(t2)));
	mot.r3 = restriccion_angulos(roundf(- grados(t3)));
	
	return mot;
}



//////////////////////////////////////////////////////
//4. JACOBIANO

jcb2 jacobiana(float t1, float t2) { //MUY IMPORTANTE ESTO VA EN RADIANES
	///REVISAR SIGBNO

	jcb2 J;

    J.j11 =  L1 * sinf(t1) + L2 * sinf(t1+t2);
    J.j12 =  L2 * sinf(t1+t2);
    J.j21 = -L1 * cosf(t1) - L2 * cosf(t1+t2);
    J.j22 = -L2 * cosf(t1+t2);

    return J;
}

void jacobiana_siguiente(float t1, float t2, float xd, float yd, float *t1d, float *t2d, float *t3d)
{
	/*
	MUY IMPORTANTE ESTO VA EN RADIANES

	t1 y t2 son la posicion de los motores actual, para sacar jacobiana
	xd e yd son las velocidades deseadas (sacadas a partir de trayectorias)
	t1d, td2, td3 son las velocidades deseadas (calculadas) de los motores
	*/

    // 1. JACOBIANA
	jcb2 J = jacobiana(t1, t2);
    float landa = 0.01f;   // parametro de ajuste, habra que mirar que poner

    // 3. Matriz A = J*J^T + landa^2*I
    float a11 = J.j11*J.j11 + J.j12*J.j12 + landa*landa;
    float a12 = J.j11*J.j21 + J.j12*J.j22;
    float a21 = a12;
    float a22 = J.j21*J.j21 + J.j22*J.j22 + landa*landa;

    // 4. Inversa de A (2×2)
    float detA = a11*a22 - a12*a21;

    //PARA QUE NO COLAPSE
    if (fabsf(detA) < 0.0001f) {
        *t1d = 0.0f;
        *t2d = 0.0f;
        *t3d = 0.0f;
        return;
    }

    float invdetA = 1.0f / detA;

    float Ainv11 =  a22 * invdetA;
    float Ainv12 = -a12 * invdetA;
    float Ainv21 = -a21 * invdetA;
    float Ainv22 =  a11 * invdetA;

    // 5. K = J^T * Ainv
    float K11 = J.j11*Ainv11 + J.j21*Ainv21;
    float K12 = J.j11*Ainv12 + J.j21*Ainv22;
    float K21 = J.j12*Ainv11 + J.j22*Ainv21;
    float K22 = J.j12*Ainv12 + J.j22*Ainv22;

    // 6. qdot = K * xdot
    *t1d = K11 * xd + K12 * yd;
    *t2d = K21 * xd + K22 * yd;
    *t3d = -(*t1d + *t2d);
}


//////////////////////////////////////////////////////
//4. VELOCIDADES

void velocidad_dibujo_recta(c3d act, c3d objetivo, c3d *velocidades)
{

	//c3d objetivo = plano_dibujo(obj); //CAMBIO DE COORDENADAS FUERA MEJOR

    float dx = (float)(objetivo.x - act.x);
    float dy = (float)(objetivo.y - act.y);
    int16_t dz = objetivo.z - act.z;

    float d_xy = sqrtf(dx*dx + dy*dy);

    if (d_xy < 0.001f && dz== 0) {
            velocidades->x = 0;
            velocidades->y = 0;
            velocidades->z = 0;
            return;
    }

    if (d_xy < 0.001f) {
    	velocidades->x = 0.0f;
        velocidades->y = 0.0f;
    }
    else {
        velocidades->x = vconst * dx / d_xy;
        velocidades->y = vconst * dy / d_xy;
    }

    //se mueve hacia el punto a velocidad cte
    if (dz == 0) velocidades->z = 0;
    else velocidades->z = vconst * (dz > 0 ? 1 : -1);


}


void velocidad_recta(c3d act, c3d obj, c3d *velocidades)
{
	//comprobacion de que no toca el lienzo y tal
	if (!dentro_rango(obj)){
		if (obj.z<0) obj.z=0;
		else if (obj.z>LBASE) obj.z=LBASE;

		if (obj.y<20) obj.y=20;
		else if(obj.y>SEPy+plano)obj.y=SEPy+plano;
		if (obj.x > x0+obj.y-20) obj.x = x0+obj.y-20;
	}

    float dx = (float)(obj.x - act.x);
    float dy = (float)(obj.y - act.y);
    float dz = (float)(obj.z - act.z);

    float K = 0.1f;

    float vx = K * dx;
    float vy = K * dy;
    float vz = K * dz;



    // limitar velocidad máxima en XY
    float vxy = sqrtf(vx*vx + vy*vy);
    if (vxy > vmax) {
        float esc = vmax / vxy;
        vx *= esc;
        vy *= esc;
    }

    // limitar velocidad en Z
    if (fabsf(vz) > vmax) vz = (vz > 0 ? vmax : -vmax);



    //FILTRO PARA ACELERACIONES
    //diferencia entre velocidad anterior y la deseada
    float dvx = vx - vx_prev;
    float dvy = vy - vy_prev;
    float dvz = vz - vz_prev;

    float max_step = amax * DT_CONTROL;

    //limitar cambio de velocidad
    if (dvx >  max_step) dvx =  max_step;
    if (dvx < -max_step) dvx = -max_step;

    if (dvy >  max_step) dvy =  max_step;
    if (dvy < -max_step) dvy = -max_step;

    if (dvz >  max_step) dvz =  max_step;
    if (dvz < -max_step) dvz = -max_step;

    // aplicar suavizado
    float vx_def = vx_prev + dvx;
    float vy_def = vy_prev + dvy;
    float vz_def = vz_prev + dvz;


    /*
    ME HA DICHO CHAT QUE PUEDE METER VIBRACIONES MUY CHUNGAS

    velocidades->x = (int16_t)(fabsf(vx) < 1.0f ? (vx > 0 ? 1 : -1) : vx);
    velocidades->y = (int16_t)(fabsf(vy) < 1.0f ? (vy > 0 ? 1 : -1) : vy);
    velocidades->z = (int16_t)(fabsf(vz) < 1.0f ? (vz > 0 ? 1 : -1) : vz);
    */

    if (fabsf(vx_def) < 1.0f) vx_def = 0.0f;
    if (fabsf(vy_def) < 1.0f) vy_def = 0.0f;
    if (fabsf(vz_def) < 1.0f) vz_def = 0.0f;

    velocidades->x = (int16_t)vx_def;
    velocidades->y = (int16_t)vy_def;
    velocidades->z = (int16_t)vz_def;

    vx_prev = vx_def;
    vy_prev = vy_def;
    vz_prev = vz_def;

}

void velocidad_transicion(c3d act, c3d obj, c3d *velocidades, uint8_t flag)
{
    int16_t dir; //= (flag == 1 ? 1 : -1);
    if (flag==1)dir=1;
    else if (flag==3)dir=-1;
    else return;

    velocidades->x = dir * (vconst / 2);
    velocidades->z = 0;
    velocidades->y = 0;

    if (act.x==obj.x) velocidades->x = 0;

}



//////////////////////////////////////////////////////
//6. TRAYECTORIA

/*
 flag=0 -> no dibuja
 flag=1_> transicion a dibujo
 flag=2-> dibujo
 flag=3->transicion a no dibuja
 */
bool trayectoria(c3d obj, uint8_t *flagdibujo){
	//DEBERIA PILLAR LA LECTURA DE LOS SENSORES
	//motores_actuales=LECTURA_MOTORES(); O ALGO ASI
    c4d act = cinematica_directa(motores_actuales);

    if ((act.coor.x==obj.x)&&(act.coor.y==obj.y)&&(act.coor.z==obj.z)) {
    	vx_prev = 0;
    	vy_prev = 0;
    	vz_prev = 0;
    	/*
        if (*flagdibujo==1 || *flagdibujo==5)*flagdibujo=2;
        else if (*flagdibujo==3 || *flagdibujo==4)*flagdibujo=0;
        */
        return true;
    }

    c3d velocidades;
    switch (*flagdibujo){
    	case 0:
    		velocidad_recta(act.coor, obj, &velocidades);
    		break;

    	case 1:
    	case 3:
    		velocidad_transicion(act.coor, obj, &velocidades, *flagdibujo);
    		break;
    	case 2:
    		/* SI SE HICIERA EL CAMBIO DENTRO
    		c2d objetivo_2d;
    		objetivo_2d.z=obj.z;
    		objetivo_2d.y=obj.y;
			*/
    		velocidad_dibujo_recta(act.coor, obj, &velocidades);
    		break;
    }

	motoresg mot=conv_grados(motores_actuales);

	//AQUI???
	//SIGNOS REVISAR!!!!!!!!!!!
	float t1= radianes(-mot.r1);
	float t2= radianes(-mot.r2);
	float t3= radianes(-mot.r3);

	float t1dot, t2dot, t3dot;
	jacobiana_siguiente(t1, t2, velocidades.x, velocidades.y, &t1dot, &t2dot, &t3dot);

	t1 += t1dot * DT_CONTROL;
	t2 += t2dot * DT_CONTROL;
	t3 += t3dot * DT_CONTROL;

	motoresg vuelta= mot;
	vuelta.r1 = grados(t1);
	vuelta.r2 = grados(t2);
	vuelta.r3 = grados(t3);

	vuelta.r1 = restriccion_angulos(vuelta.r1);
	vuelta.r2 = restriccion_angulos(vuelta.r2);
	vuelta.r3 = restriccion_angulos(vuelta.r3);

	//vuelta r3 ????
	float z_actual = act.coor.z;
	float z_vuelta  = z_actual + velocidades.z * DT_CONTROL;

	vuelta.base = z_vuelta * 360.0f / 8.0f;

	motores_actuales = conv_entero(vuelta);

	return false;
}



//bool trayectoria_cutre(c3d obj, uint8_t *flagdibujo){
//		//DEBERIA PILLAR LA LECTURA DE LOS SENSORES
//		//motores_actuales=LECTURA_MOTORES(); O ALGO ASI
//    c4d act4 = cinematica_directa(motores_actuales);
//    c3d act = act4.coor;
//    if (act.x == obj.x && act.y == obj.y && act.z == obj.z) return true;
//
//    //bresenham
//
//    //distancias
//    int dx = abs(obj.x - act.x);
//    int dy = abs(obj.y -act.y);
//    int dz = abs(obj.z - act.z);
//
//    //signos
//    int sx = (act.x < obj.x) ? 1 : -1;
//    int sy = (act.y < obj.y) ? 1 : -1;
//    int sz = (act.z < obj.z) ? 1 : -1;
//
//    //si estoy en 1 o 3 estoy en transicion
//    //si estoy en 0 o 2 avanzo el doble de rapido
//    int paso = (*flagdibujo == 1 || *flagdibujo == 3) ? 1 : 2;
//
//    //dominante en x
//    if (dx >= dy && dx >= dz) {
//    	act.x += sx * paso;
//    	if ((sx > 0 && act.x > obj.x) || (sx < 0 && act.x < obj.x)) act.x = obj.x;
//    	if (dx != 0) {
//    		int dy_step = (dy * paso) / dx;
//    		int dz_step = (dz * paso) / dx;
//    		if (dy_step == 0 && dy != 0) dy_step = 1 * sy;
//    		if (dz_step == 0 && dz != 0) dz_step = 1 * sz;
//    		act.y += dy_step;
//    		act.z += dz_step;
//            }
//        }
//
//    //dominante en y
//    else if (dy >= dx && dy >= dz) {
//    	act.y += sy * paso;
//    	if ((sy > 0 && act.y > obj.y) || (sy < 0 && act.y < obj.y)) act.y = obj.y;
//    	if (dy != 0) {
//    		int dx_step = (dx * paso) / dy;
//    		int dz_step = (dz * paso) / dy;
//    		if (dx_step == 0 && dx != 0) dx_step = 1 * sx;
//    		if (dz_step == 0 && dz != 0) dz_step = 1 * sz;
//    		act.x += dx_step;
//    		act.z += dz_step;
//    	}
//    }
//    //dominante en z
//    else {
//    	act.z += sz * paso;
//    	if ((sz > 0 && act.z > obj.z) || (sz < 0 && act.z < obj.z)) act.z = obj.z;
//
//    	if (dz != 0) {
//    		int dx_step = (dx * paso) / dz;
//    		int dy_step = (dy * paso) / dz;
//    		if (dx_step == 0 && dx != 0) dx_step = 1 * sx;
//    		if (dy_step == 0 && dy != 0) dy_step = 1 * sy;
//
//    		act.x += dx_step;
//    		act.y += dy_step;
//    	}
//    }
//
//
//    //saturacion, me lo ha dicho chat
//    if (sx > 0 && act.x > obj.x) act.x = obj.x;
//    if (sx < 0 && act.x < obj.x) act.x = obj.x;
//    if (sy > 0 && act.y > obj.y) act.y = obj.y;
//    if (sy < 0 && act.y < obj.y) act.y = obj.y;
//    if (sz > 0 && act.z > obj.z) act.z = obj.z;
//    if (sz < 0 && act.z < obj.z) act.z = obj.z;
//
//
//    //cinemática inversa
//    c3d siguiente = { act.x, act.y, act.z };
//    motoresg motg = cinematica_inversa(siguiente);
//    motores_actuales = conv_entero(motg);
//
//    return false;
//
//}




