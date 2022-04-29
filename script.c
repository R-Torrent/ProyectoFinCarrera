/*
Lectura de 'scripts'
Roger Torrent Ahn
PFC: Canadair CL-415 
Curso 2017-2018
*/

#include "pfc.h"
#include "CL415.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define MAXLINEA 100

double lbh(double *, double *, unsigned);

enum modificacion {
	VELOC = 01, // Recalcular velocidades
	ORIEN = 02, // Recalcular matriz de transformación y cuaternión unitario
	PESOS = 04  // Recalcular pesos embarcados
};

void script(estado_t estado, char nombre[])
{
	FILE *fp;
	char linea[MAXLINEA], *token;
	enum parametros indice;
	int avisos = 0;
	double psi, theta, phi, cpsi, spsi, ctheta, stheta, cphi, sphi;
	double psi2, theta2, phi2, cpsi2, spsi2, ctheta2, stheta2, cphi2, sphi2;
	char direcc[FILENAME_MAX] = ".\\";

	// Apertura del archivo
	fp = fopen(strcat(direcc, nombre), "r");
	if (fp == NULL) return;
	setvbuf(fp, NULL, _IOLBF, BUFSIZ);

	// Lectura de líneas e interpretación
	while (fgets(linea, MAXLINEA, fp) != NULL){
		token = strtok(linea, " \t");
		while (!*token)
			token = strtok(NULL, " \t\n");
		if (*token == '%')
			continue; // Encontrado un comentario en el script; pasar a la siguiente línea
		
		indice = FIN;
		if (!strcmp(token,"DELTA_T0")) indice = DELTA_T0;
		else if (!strcmp(token,"DELTA_E")) indice = DELTA_E;
		else if (!strcmp(token,"DELTA_A")) indice = DELTA_A;
		else if (!strcmp(token,"DELTA_R")) indice = DELTA_R;
		else if (!strcmp(token,"DELTA_WM1")) indice = DELTA_WM1;
		else if (!strcmp(token,"DELTA_WM2")) indice = DELTA_WM2;
		else if (!strcmp(token,"DELTA_NP1")) indice = DELTA_NP1;
		else if (!strcmp(token,"DELTA_NP2")) indice = DELTA_NP2;
		else if (!strcmp(token,"U")) indice = U;
		else if (!strcmp(token,"V")) indice = V;
		else if (!strcmp(token,"W")) indice = W;
		else if (!strcmp(token,"UWIND")) indice = UWIND;
		else if (!strcmp(token,"VWIND")) indice = VWIND;
		else if (!strcmp(token,"WWIND")) indice = WWIND;
		else if (!strcmp(token,"P")) indice = P;
		else if (!strcmp(token,"Q")) indice = Q;
		else if (!strcmp(token,"R")) indice = R;
		else if (!strcmp(token,"PSI")) indice = PSI;
		else if (!strcmp(token,"THETA")) indice = THETA;
		else if (!strcmp(token,"PHI")) indice = PHI;
		else if (!strcmp(token,"LAMBDA")) indice = LAMBDA;
		else if (!strcmp(token,"MU")) indice = MU;
		else if (!strcmp(token,"H")) indice = H;
		else if (!strcmp(token,"WFUEL1")) indice = WFUEL1;
		else if (!strcmp(token,"WFUEL2")) indice = WFUEL2;
		else if (!strcmp(token,"WH2O1")) indice = WH2O1;
		else if (!strcmp(token,"WH2O2")) indice = WH2O2;
		else if (!strcmp(token,"WH2O3")) indice = WH2O3;
		else if (!strcmp(token,"WH2O4")) indice = WH2O4;
				
		if (indice == FIN)
			continue; // Encontrado un identificador desconocido; pasar a la siguiente línea
		
		do
			token = strtok(NULL, " \t");
		while (!*token);

		estado[indice] = atof(token); // Lectura del valor numérico

		// Algunas variables acarrean la modificación de otros registros
		if (indice>=U && indice <=WWIND) avisos |= VELOC;
		else if(indice>=PSI && indice<=PHI) avisos |= ORIEN;
		else if (indice>=WFUEL1 && indice<=WH2O4) avisos |= PESOS;
	}

	if (avisos & (VELOC | ORIEN)){
		if (avisos & ORIEN){
			// Expresiones trigonométricas de los ángulos de Euler
			psi = estado[PSI] / CONV_RD;	 psi2 = psi / 2;
			theta = estado[THETA] / CONV_RD; theta2 = theta / 2;
			phi = estado[PHI] / CONV_RD;	 phi2 = phi / 2;
			
			cpsi = cos(psi);	   spsi = sin(psi);
			ctheta = cos(theta);   stheta = sin(theta);
			cphi = cos(phi);	   sphi = sin(phi);
			cpsi2 = cos(psi2);	   spsi2 = sin(psi2);
			ctheta2 = cos(theta2); stheta2 = sin(theta2);
			cphi2 = cos(phi2);	   sphi2 = sin(phi2);

			// Asignaciones indirectas
			estado[E1] = cpsi2*ctheta2*sphi2 - spsi2*stheta2*cphi2;
			estado[E2] = cpsi2*stheta2*cphi2 + spsi2*ctheta2*sphi2;
			estado[E3] = spsi2*ctheta2*cphi2 - cpsi2*stheta2*sphi2;
			estado[E0] = sqrt(1 - estado[E1]*estado[E1] - estado[E2]*estado[E2] - estado[E3]*estado[E3]);
			estado[LBH11] = cpsi*ctheta;
			estado[LBH12] = spsi*ctheta;
			estado[LBH13] = -stheta;
			estado[LBH21] = cpsi*stheta*sphi - spsi*cphi;
			estado[LBH22] = spsi*stheta*sphi + cpsi*cphi;
			estado[LBH23] = ctheta*sphi;
			estado[LBH31] = cpsi*stheta*cphi + spsi*sphi;
			estado[LBH32] = spsi*stheta*cphi - cpsi*sphi;
			estado[LBH33] = ctheta*cphi;
		}
		estado[UTAS] = estado[U]-lbh(estado+UWIND, estado+LBH11, 1U);
		estado[VTAS] = estado[V]-lbh(estado+UWIND, estado+LBH11, 2U);
		estado[WTAS] = estado[W]-lbh(estado+UWIND, estado+LBH11, 3U);
		estado[ALPHA] = atan2(estado[WTAS], estado[UTAS]) * CONV_RD;
		estado[BETA] = atan2(estado[VTAS], sqrt(estado[UTAS]*estado[UTAS] + estado[WTAS]*estado[WTAS])) * CONV_RD;
		estado[ROC] = -(estado[U]*estado[LBH13] + estado[V]*estado[LBH23] + estado[W]*estado[LBH33]);
	}
	if (avisos & PESOS)
		estado[MTOTAL] = (OEW + estado[WFUEL1] + estado[WFUEL2] +
			estado[WH2O1] + estado[WH2O2] + estado[WH2O3] + estado[WH2O4])/G0;

	// Cierre y retorno
	fclose(fp);
	return;
}