#include "cinematica.h"


//////////////////////////////////////////////////////
//1. PASO DE COORDENADAS A DIBUJO

c3d plano_dibujo( c2d cor){
	c3d vuelta;

	//Nos aseguramos de que este dentro del lienzo
	if (cor.z<0) cor.z=0;
	if (cor.y<0) cor.y=0;
	if (cor.z>200) cor.z=200;
	if (cor.y>200) cor.y=200;
	
	//transformación a sistema 3d 
	vuelta.z = cor.z + SEPz;
	vuelta.y = (int16_t)(cor.y * ANG) + SEPy;
	vuelta.x = (int16_t)(cor.y * ANG) + SEPx;
	return vuelta;
}

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

//////////////////////////////////////////////////////
//2. CINEMATICA DIRECTA

c4d cinematica_directa( motores mot){

	c4d vuelta = {0};
	motores m = mot;

	//1 vuelta son 8mm
	vuelta.coor.z= (int16_t)roundf((m.base *8/360)); 

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
	float t3=radianes(-m.r3);

	//algunas operaciones para que sea menos insufrible
	//cos(theta1)sen(theta2)+sen(theta1)cos(theta2)
	//float a=cosf(t1)*sinf(t2)+sinf(t1)*cosf(t2)=sinf(t1+t2)

	vuelta.coor.x= (int16_t)roundf(-(L1*cosf(t1)+L2*cosf(t1+t2)+L3*(cosf(t3)*cosf(t1+t2)-sinf(t3)*sinf(t1+t2))));
	vuelta.coor.y= (int16_t)roundf(-(L1*sinf(t1)+L2*sinf(t1+t2)+L3*(cosf(t3)*sinf(t1+t2)+sinf(t3)*cosf(t1+t2))));
	vuelta.ang=acosf(cosf(t3)*cosf(t1+t2)+sinf(t3)*sinf(t1+t2));
	vuelta.ang=180-grados(vuelta.ang);

	return vuelta;
}



//////////////////////////////////////////////////////
//3. CINEMATICA INVERSA

motores cinematica_inversa( c3d cor){

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
	
	motores vuelta = conv_entero(mot);

	return vuelta;
}

