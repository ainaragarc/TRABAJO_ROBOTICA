#include "cinematica.h"


//////////////////////////////////////////////////////
//1. PASO DE COORDENADAS A DIBUJO

c3d plano_dibujo( c2d cor){
	c3d vuelta;

	//Nos aseguramos de que este dentro del lienzo
	if (c2d.z<0) c2d.z=0;
	if (c2d.z>200) c2d.z=200;
	if (c2d.y<0) c2d.y=0;
	if (c2d.y>200) c2d.y=200;
	
	//transformación a sistema 3d 
	vuelta.z = cor.z + SEPz;
	vuelta.y = (uint16_t)(cor.y * ANG) + SEPy;
	vuelta.x = (uint16_t)(cor.y * ANG) + SEPx;
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



//////////////////////////////////////////////////////
//2. CINEMATICA DIRECTA

c3d cinematica_directa( motores mot){

	c3d vuelta;
	motoresg m = conv_grados(mot);

	vuelta.z=m.base;


	return vuelta;
}
