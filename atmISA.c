/*
Atmósfera ISA hasta 36,000 ft de altitud
Roger Torrent Ahn
PFC: Canadair CL-415 
Curso 2017-2018
*/

#include <math.h>

void atmISA(const double h, const double Dtemp0, double *temp, double *dens, double *pres)
{
	const double alpha = -1.98e-3; // (K/ft)
	const double temp0 = 288.15; // Temperatura en h=0 (K)
	const double dens0 = 0.07647; // Densidad en h=0 (lb/ft³)
	const double pres0 = 14.6959; // Presión en h=0 (lb/in² = psi)
	const double grav0 = 32.174; // Aceleración gravitatoria constante (ft/s²)
	const double Rg = 953.61; // Constante de gas ideal para la atmósfera (ft·lb/(slug·K))

	// Ecuaciones válidas sólo hasta la tropopausa (36,000 ft)
	*temp = (temp0+Dtemp0) + alpha*h;
	*dens = dens0 * pow((1 + alpha*h/(temp0+Dtemp0)), -grav0/(Rg*alpha) - 1);
	*pres = pres0 * pow((1 + alpha*h/(temp0+Dtemp0)), -grav0/(Rg*alpha));
	
	return;
}