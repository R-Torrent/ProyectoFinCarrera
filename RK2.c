/*
Esquema numérico de Runge-Kutta, de segundo orden
Roger Torrent Ahn
PFC: Canadair CL-415 
Curso 2017-2018
*/

#include "pfc.h"
#include <stdlib.h>

double *rk2(double *u1, double *u2, void (*F)(double *, double *), unsigned len)
{
	double *k1 = (double *)calloc(len, sizeof(double));
	double *k2 = (double *)calloc(len, sizeof(double));
	double *u21 = (double *)calloc(len, sizeof(double));

	unsigned i;

	if (k1==NULL || k2==NULL || u21==NULL){ // Error asignando memoria
		free(k1);
		free(k2);
		free(u21);
		return NULL;
	}

	(*F)(k1, u1);
	for (i=0; i<len; i++)
		u21[i] = *(u1+i) + k1[i]*DT;

	(*F)(k2, u21);
	for (i=0; i<len; i++)
		*(u2+i) = *(u1+i) + .5*(k1[i]+k2[i])*DT;

	free(k1);
	free(k2);
	free(u21);

	return u2;
}

 