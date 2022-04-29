/*
Programa principal
Roger Torrent Ahn
PFC: Canadair CL-415 
Curso 2017-2018
*/

#include "pfc.h"
#include "CL415.h"
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>

double lbh(double *, double *, unsigned);
double helice(double *, double, double);
void acellin(double *, double *);
void acelang(double *, double *);
void longlat(double *, double *);
void dcuaternion(double *, double *);

static double XX, YY, ZZ, LL, MM, NN; // Suma de fuerzas y momentos
static estado_t est1, est2; // Variables de estado en t y t+DT

int main()
{
	clock_t t0, t1, t2; // Ticks del reloj en los instantes t0, t y t+DT
	
	double
		VelTAS, // Velocidad aerodinámica verdadera (ft/s)
		VelIAS, // Velocidad aerodinámica indicada al piloto (kn)
		Hi, // Altitud indicada al piloto (1000 ft)
		ROCi, // Velocidad ascensional indicada al piloto (1000 ft/min)
		temp, // Temperatura exterior (K)
		dens, // Densidad exterior (lb/ft³)
		pres, // Presión exterior (lb/in² = psi)
		q0, // Presión dinámica (lb/ft²)
		L, D, XA, YA, ZA, // Fuerzas aerodinámicas (lb)
		LA, MA, NA, // Momentos aerodinámicos (lb·ft)
		Wm1, Wm2, // Potencia transmitida por los motores (shp)
		Wfcon1, Wfcon2, // Consumo de combustible (lb/hr = PPH)
		Cp1, Cp2, // Coeficiente de potencia (-)
		J1, J2, // Parámetro de avance de las hélices (-)
		Qi1, Qi2, // Par motor indicado al usuario (% de Qref = 10420 lb·ft)
		T1, T2, XT, // Fuerzas propulsivas (lb)
		QT, LT, MT, NT, // Pares propulsivos (lb·ft)
		lat, lon, // Latitud y longitud, valor entero (°) 
		min[2], seg[2], // Minutos y segundos de latitud y longitud
		xx; // Variable temporal
	
	// Usar el macro 'rel_cdg' (in,in,in); ver INFORME para «datum» y significado cota -78"
	// Los vectores del tipo 'coord_t' se almacenan en (ft,ft,ft), orientados en ejes cuerpo
	const coord_t
		cdg0 = {406.75, 0.0, 198.0}, // Posición del c.d.g. de avión OEW (in,in,in)
		lH2O1 = rel_cdg(403.0, 35.7, 123.0), lH2O2 = rel_cdg(403.0, 10.2, 123.0), // c.d.g. de cada tanque de agua
		lH2O3 = rel_cdg(403.0, -10.2, 123.0), lH2O4 = rel_cdg(403.0, -35.7, 123.0),
		lH1 = rel_cdg(284.0, 137.3, 272.0), lH2 = rel_cdg(284.0, -137.3, 272.0); // Hélices del avión {1:izqdo 2:drcho}

	const double // Retraso del c.d.g del avión OEW respecto al centro aerodinámico del ala
		cdg0CMA = (cdg0.x-(baCMA+.25*12*CMA))/12;  // (ft)

	enum parametros iv; // Índice de los elementos en las variables 'estado_t'

	int alertas = INIT; // Señales de control, según 'enum flags'
	int tecla; // Carácter leído

	setbuf(stdin, NULL); // Stream de entrada (teclado) sin buffer

	do{ // Bucle principal
/*
		// Lectura del stream del teclado
		while (tecla = getchar()){
			switch (tecla){
				case 'A': case 'a': // Stick izquierda
					est1[DELTA_A] -= 0.25;
					if (est1[DELTA_A] < DAMIN) est1[DELTA_A] = DAMIN;
					break;
				case 'D': case 'd': // Stick derecha
					est1[DELTA_A] += 0.25;
					if (est1[DELTA_A] > DAMAX) est1[DELTA_A] = DAMAX;
					break;
				case 'W': case 'w': // Stick adelante
					est1[DELTA_E] -= 0.25;
					if (est1[DELTA_E] < DEMIN) est1[DELTA_E] = DEMIN;
					break;
				case 'S': case 's': // Strick atrás
					est1[DELTA_E] += 0.25;
					if (est1[DELTA_E] > DEMAX) est1[DELTA_E] = DEMAX;
					break;
				case '\033': // 'ESC'
					getchar(); // Esta tecla no importa
					switch(getchar()){
						case 'D': // Pedal izquierdo
							est1[DELTA_R] -= 0.25;
							if (est1[DELTA_R] < DRMIN) est1[DELTA_R] = DRMIN;
							break;
						case 'C': // Pedal derecho
							est1[DELTA_R] += 0.25;
							if (est1[DELTA_R] > DRMAX) est1[DELTA_R] = DRMAX;
							break;
					}
					break;
				case 'R': case 'r': // +Potencia motor izquierdo
					est1[DELTA_WM1] += 0.05;
					if (est1[DELTA_WM1] > 1.0) est1[DELTA_WM1] = 1.0;
					break;
				case 'F': case 'f': // -Potencia motor izquierdo
					est1[DELTA_WM1] -= 0.05;
					if (est1[DELTA_WM1] < 0.0) est1[DELTA_WM1] = 0.0;
					break;
				case 'T': case 't': // +Potencia motor derecho
					est1[DELTA_WM2] += 0.05;
					if (est1[DELTA_WM2] > 1.0) est1[DELTA_WM2] = 1.0;
					break;
				case 'G': case 'g': // -Potencia motor derecho
					est1[DELTA_WM2] -= 0.05;
					if (est1[DELTA_WM2] < 0.0) est1[DELTA_WM2] = 0.0;
					break;
				case 'Y': case 'y': // +RPM motor izquierdo
					est1[DELTA_NP1] += 10.0;
					if (est1[DELTA_NP1] > NPMAX) est1[DELTA_NP1] = NPMAX;
					break;
				case 'H': case 'h': // -RPM motor izquierdo
					est1[DELTA_NP1] -= 10.0;
					if (est1[DELTA_NP1] < NPMIN) est1[DELTA_NP1] = NPMIN;
					break;
				case 'U': case 'u': // +RPM motor derecho
					est1[DELTA_NP2] += 10.0;
					if (est1[DELTA_NP2] > NPMAX) est1[DELTA_NP2] = NPMAX;
					break;
				case 'J': case 'j': // -RPM motor derecho
					est1[DELTA_NP2] -= 10.0;
					if (est1[DELTA_NP2] < NPMIN) est1[DELTA_NP2] = NPMIN;
					break;
				case 'Q': case 'q': // Terminar la simulación
					alertas |= QUIT;
					break;
				case 'C': case 'c': // Centrar el stick y los pedales
					est1[DELTA_E] = est1[DELTA_A] = est1[DELTA_R] = 0.0;
					break;
				case 'Z': case 'z': // Cargar el script durante la simulación
					alertas |= SETV;
					break;
				case ' ': // Pausa
					alertas ^= PAUS;
					break;
			}
		}
*/
		if (alertas & INIT){ // Cargar fichero 'init.txt'
			alertas &= ~INIT;
			script(est1, "init.txt");
			t0 = clock();
		}
		if (alertas & SETV){ // Cargar ficher 'set.txt'
			alertas &= ~SETV;
			script(est1, "set.txt");
		}
		if (alertas & PAUS) // Pausa en la simulación
			continue; // Volver atrás

		t1 = clock();
		t2 = t1 + (clock_t)(CLOCKS_PER_SEC*DT);

		// Comprobación de la altitud (condición de choque, aerodinámica con efecto suelo...)
		if (est1[H] <= 0.0){
			fprintf(stderr, "\n\n*** CL-415 estrellado a los %.2f segundos de arrancar la simulacion.\n", (t1-t0)/(double)CLOCKS_PER_SEC);
			break;
		}
		
		// Cálculo de las condiciones atmosféricas
		atmISA(est1[H], est1[DELTA_T0], &temp, &dens, &pres);

		// Velocidad aerodiámica verdadera
		VelTAS = sqrt( est1[UTAS]*est1[UTAS] + est1[VTAS]*est1[VTAS] + est1[WTAS]*est1[WTAS] );

		// Valores indicados al piloto
		VelIAS = 1479.14866 * sqrt(pow(dens/G0*VelTAS*VelTAS/4232.4192 + 1, 2.0/7) - 1); // Velocidad aerodinámica (kn)
		xx = 288.15 / (288.15 + est1[DELTA_T0]);
		Hi = xx * est1[H] / 1000; // Altitud (1000 ft)
		ROCi = xx * est1[ROC] / 16.6666667; // Velocidad ascensional (1000 ft/min)

		// Presión dinámica (lb/ft²)
		q0 = 0.5 * (dens/G0) * VelTAS * VelTAS;
		
		// Sustentación y resistencia aerodinámica (lb)
		L = q0*SW*(CL_0 + CL_alpha*est1[ALPHA] + 0.5*CMA/VelTAS*CL_q*est1[Q] + CL_de*est1[DELTA_E]);
		D = q0*SW*(CD_0 + CD_alpha*est1[ALPHA]);

		// Fuerzas aerodinámicas en ejes cuerpo (lb)
		XA = L*sin(est1[ALPHA]/CONV_RD) - D*cos(est1[ALPHA]/CONV_RD);
		YA = q0*SW*(CY_beta*est1[BETA] + 0.5*BW/VelTAS*(CY_p*est1[P] + CY_r*est1[R]) + CY_dr*est1[DELTA_R]);
		ZA = -L*cos(est1[ALPHA]/CONV_RD) - D*sin(est1[ALPHA]/CONV_RD);

		// Momentos aerodinámicos en ejes cuerpo (lb·ft)
		LA = q0*SW*BW*(Cl_beta(est1[ALPHA])*est1[BETA] + 0.5*BW/VelTAS*(Cl_p(est1[ALPHA])*est1[P] +
			Cl_r(est1[ALPHA])*est1[R]) + Cl_da*est1[DELTA_A] + Cl_dr*est1[DELTA_R]);
		MA = q0*SW*CMA*(Cm(est1[ALPHA]) + 0.5*CMA/VelTAS*Cm_q*est1[Q] + Cm_de*est1[DELTA_E]) +
			L*cdg0CMA*cos(est1[ALPHA]/CONV_RD) + D*cdg0CMA*sin(est1[ALPHA]/CONV_RD);
		NA = q0*SW*BW*(Cn_beta(est1[ALPHA])*est1[BETA] +  0.5*BW/VelTAS*(Cn_p(est1[ALPHA])*est1[P] +
			Cn_r(est1[ALPHA])*est1[R]) + Cn_da(est1[ALPHA])*est1[DELTA_A] + Cn_dr*est1[DELTA_R]) + YA*cdg0CMA;

		// Potencia de los motores (shp)
		xx = pres*sqrt(temp)/249.4626;
		Wm1 = Pmax * minimo(1,xx*(0.3901+0.6099*est1[DELTA_WM1]));
		Wm2 = Pmax * minimo(1,xx*(0.3901+0.6099*est1[DELTA_WM2]));
		xx = 1.188e8 / ((dens/G0)*pow(DH,5));
		Cp1 = Wm1 * xx / pow(est1[DELTA_NP1],3);
		Cp2 = Wm2 * xx / pow(est1[DELTA_NP2],3);

		// Pares indicados al usuario (%)
		Qi1 = 50.404 * Wm1 / est1[DELTA_NP1];
		Qi2 = 50.404 * Wm2 / est1[DELTA_NP2];

		// Consumo de los motores (PPH)
		Wfcon1 = CE * Wm1;
		Wfcon2 = CE * Wm2;

		// Parámetros de avance (-)
		J1 = VelTAS / (est1[DELTA_NP1]/60*DH);
		J2 = VelTAS / (est1[DELTA_NP2]/60*DH);

		// Tracción de las hélices (lb)
		xx = 550 / VelTAS;
		T1 = xx * helice(NULL, Cp1, J1) * Wm1;
		T2 = xx * helice(NULL, Cp2, J2) * Wm2;

		// Comprobación del nivel de combustible
		if (est1[WFUEL1] <= 0.0) T1 = 0.0, Qi1 = 0.0, Wfcon1 = 0.0;
		if (est1[WFUEL2] <= 0.0) T2 = 0.0, Qi2 = 0.0, Wfcon2 = 0.0;

		// Par de reacción (lb·ft)
		QT = Qref * (Qi1+Qi2)/100;

		// Fuerzas propulsivas (lb)
		XT = T1 + T2;

		// Momentos propulsivos (lb·ft)
		LT = -QT;
		MT = lH1.z*T1 + lH2.z*T2;
		NT = -lH1.y*T1 - lH2.y*T2;

		// Suma de fuerzas aerodinámicas + propulsivas (lb)
		XX = XT + XA;
		YY = YA;
		ZZ = ZA;

		// Suma de momentos aerodinámicos + propulsivos (lb·ft)
		LL = LT + LA;
		MM = MT + MA;
		NN = NT + NA;
	
		if (rk2(est1+U, est2+U, acellin, 3) == NULL) // Integración de la aceleración lineal
			exit(1);

		if (rk2(est1+P, est2+P, acelang, 3) == NULL) // Integración de la aceleración angular
			exit(1);

		// Integración (evidente) del consumo (lb)
		est2[WFUEL1] = est1[WFUEL1] - Wfcon1*DT/3600;
		if (est2[WFUEL1] < 0.0) est2[WFUEL1] = 0.0;
		est2[WFUEL2] = est1[WFUEL2] - Wfcon2*DT/3600;
		if (est2[WFUEL2] < 0.0) est2[WFUEL2] = 0.0;

		if (rk2(est1+LAMBDA, est2+LAMBDA, longlat, 2) == NULL) // Integración de coordendas horizontales (°,°)
			exit(1);
		
		est2[H] = est1[H] + est1[ROC]*DT; // Integración (evidente) de la altitud (ft)

		if (rk2(est1+E0, est2+E0, dcuaternion, 4) == NULL) // Integración del cuaternión unitario
			exit(1);

		// Matriz de transformación
		est2[LBH11] = 2*(est2[E0]*est2[E0] + est2[E1]*est2[E1]) - 1;
		est2[LBH12] = 2*(est2[E1]*est2[E2] + est2[E0]*est2[E3]);
		est2[LBH13] = 2*(est2[E1]*est2[E3] - est2[E0]*est2[E2]); 
		est2[LBH21] = 2*(est2[E1]*est2[E2] - est2[E0]*est2[E3]); 
		est2[LBH22] = 2*(est2[E0]*est2[E0] + est2[E2]*est2[E2]) - 1;
		est2[LBH23] = 2*(est2[E2]*est2[E3] + est2[E0]*est2[E1]); 
		est2[LBH31] = 2*(est2[E1]*est2[E3] + est2[E0]*est2[E2]); 
		est2[LBH32] = 2*(est2[E2]*est2[E3] - est2[E0]*est2[E1]); 
		est2[LBH33] = 2*(est2[E0]*est2[E0] + est2[E3]*est2[E3]) - 1;

/*
NOTA: Podría implamtarse aquí un modelo del viento, dependiente de la posición del avión o
	  incorporando ráfagas.
*/
		// Velocidad del viento (ft/s)
		est2[UWIND] = est1[UWIND];
		est2[VWIND] = est1[VWIND];
		est2[WWIND] = est1[WWIND];

		// Velocidad aerodinámica (ft/s)
		est2[UTAS] = est2[U]-lbh(est2+UWIND, est1+LBH11, 1U);
		est2[VTAS] = est2[V]-lbh(est2+UWIND, est1+LBH11, 2U);
		est2[WTAS] = est2[W]-lbh(est2+UWIND, est1+LBH11, 3U);
		
		// Ángulos aerodinámicos (°)
		est2[ALPHA] = atan2(est2[WTAS], est2[UTAS]) * CONV_RD;
		est2[BETA] = asin(est2[VTAS]/VelTAS) * CONV_RD;

		// Ángulos de Euler (°)
		est2[PSI] = atan2(est2[LBH12], est2[LBH11]) * CONV_RD; 
		est2[THETA] = -asin(est2[LBH13]) * CONV_RD;
		est2[PHI] = atan2(est2[LBH23], est2[LBH33]) * CONV_RD;

		// Rate of Climb (ft/s)
		est2[ROC] = -(est2[U]*est2[LBH13] + est2[V]*est2[LBH23] + est2[W]*est2[LBH33]);

		// Agua embarcada (lb)
/*
NOTA: El avión de este simulador todavía no puede bombardear incendios.
*/
		est2[WH2O1] = est1[WH2O1];
		est2[WH2O2] = est1[WH2O2];
		est2[WH2O3] = est1[WH2O3];
		est2[WH2O4] = est1[WH2O4];
		
		// Cómputo del peso total (slug)
		est2[MTOTAL] = (OEW + est2[WFUEL1] + est2[WFUEL2] +
			est2[WH2O1] + est2[WH2O2] + est2[WH2O3] + est2[WH2O4])/G0;

		// Conversión de la latitud y longitud a unidades habituales
		min[0] = modf(fabs(est1[LAMBDA]), &lat);
		seg[0] = modf(min[0]*60, min); 
		seg[0] *= 60.0;
		min[1] = modf(fabs(est1[MU]), &lon);
		seg[1] = modf(min[1]*60, min+1);
		seg[1] *= 60.0;

/*
FALTA Dibujar la cabina del avión + línea de horizonte
*/
		system("cls"); // Limpiar la consola
		printf("PSI   %+04d\n", (int)est1[PSI]);
		printf("THETA %+04d\n", (int)est1[THETA]);
		printf("PHI   %+04d\n\n", (int)est1[PHI]);

		printf("LATITUD %d %d\' %.2f\" %c   LONGITUD %d %d\' %.2f\" %c   H %.2f 1000ft\n",
			(int)lat, (int)min[0], seg[0], (est1[LAMBDA] >= 0.0 ? 'N' : 'S'),
			(int)lon, (int)min[1], seg[1], (est1[MU] >= 0.0 ? 'E' : 'O'), Hi);
		printf("IAS %d kn\n", (int)VelIAS);
		printf("ROC %.2f 1000ft/min\n\n", ROCi);

		printf("ELEVADOR %.2f\n", est1[DELTA_E]);
		printf("ALERON   %.2f\n", est1[DELTA_A]);
		printf("TIMON    %.2f\n", est1[DELTA_R]);
		printf("NP1  %d RPM\n", (int)est1[DELTA_NP1]);
		printf("NP2  %d RPM\n\n", (int)est1[DELTA_NP2]);

		printf("TRQ1 %d %%\n", (int)Qi1);
		printf("TRQ2 %d %%\n", (int)Qi2);
		printf("WFUEL1 %d LB   %d PPH\n",  (int)est1[WFUEL1], (int)Wfcon1);
		printf("WFUEL2 %d LB   %d PPH\n\n",  (int)est1[WFUEL2], (int)Wfcon2);

		printf("WH2O_1 %d LB   WH2O_2 %d LB   WH2O_3 %d LB   WH2O_4 %d LB", (int)est1[WH2O1], (int)est1[WH2O2], (int)est1[WH2O3], (int)est1[WH2O4]); 
/*
NOTA: Aquí podría almacenarse los valores de 'est1'. Serviría para trazar la trayectoria,
	  estudiar la estabilidad de los modos del movimiento, etc.
*/

		// 'est2' -> 'est1'
 		for(iv=ALPHA; iv<FIN; iv++) // Los valores 'DELTA_##' se heredan automáticamente
			est1[iv] = est2[iv];

		while (clock() < t2) // Ordenador bloqueado hasta cumplirse el intervalo 'DT'
			;

	} while (!(alertas & QUIT)); // Fin bucle principal (Condición de escape vía teclas 'Q/q')
	
	scanf("%d", &tecla);
	return 0;
}

double lbh(double *h, double *l, unsigned fila)
{
	switch (fila){
		case 1: return prod(*l,*h)	   + prod(*(l+1),*(h+1)) + prod(*(l+2),*(h+2));
		case 2: return prod(*(l+3),*h) + prod(*(l+4),*(h+1)) + prod(*(l+5),*(h+2));
		case 3: return prod(*(l+6),*h) + prod(*(l+7),*(h+1)) + prod(*(l+8),*(h+2));
		default: return DBL_MAX;
	}
}

double helice(double *beta075, double CP, double J)
{
	static double
		tabla_etaP[8][15] = {
			{ 35, 61, 76, 84, 86, 86, 83, 77, 69,  0,  0,  0,  0,  0,  0 },
			{ 22, 42, 61, 74, 82, 86, 88, 90, 90, 89, 86, 82, 76,  0,  0 },
			{ 15, 28, 44, 60, 73, 80, 84, 87, 89, 90, 90, 89, 88, 85, 81 },
			{ 10, 20, 31, 43, 57, 71, 78, 83, 86, 88, 89, 90, 90, 89, 88 },
			{  0,  0,  0,  0,  0,  0,  0, 77, 82, 85, 87, 88, 89, 89, 89 },
			{  0,  0,  0,  0,  0,  0,  0,  0,  0, 81, 84, 86, 88, 88, 88 },
			{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 84, 86, 87, 88 },
			{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 85, 86 } },
		tabla_beta075[8][15] = {
			{ 18, 18, 19, 21, 23, 26, 28, 31, 33,  0,  0,  0,  0,  0,  0 },
			{ 27, 27, 27, 28, 30, 32, 33, 35, 38, 39, 41, 43, 44,  0,  0 },
			{ 35, 35, 35, 35, 36, 36, 37, 39, 40, 42, 43, 45, 46, 48, 49 },
			{ 43, 44, 43, 42, 42, 42, 42, 43, 44, 45, 46, 47, 48, 49, 50 },
			{  0,  0,  0,  0,  0,  0,  0, 46, 47, 48, 48, 49, 50, 51, 52 },
			{  0,  0,  0,  0,  0,  0,  0,  0,  0, 50, 51, 51, 52, 53, 53 },
			{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 53, 54, 54, 55 },
			{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 56, 56 } };

	unsigned short CP1, CP2, J1, J2; // Índices en las tablas
	double x1, x2, x3; // Variables temporales
	
	if (CP < 0.1 || CP > 0.8 || J < 0.2 || J > 3.0){
		beta075 = NULL;
		return -1.0;
	}

	x1 = CP*10.0;
	x2 = J*5.0;

	// Asignación de índices
	CP1 = (unsigned short)floor(x1) - 1;
	CP2 = (unsigned short)ceil(x1) - 1;
	J1 = (unsigned short)floor(x2) - 1;
	J2 = (unsigned short)ceil(x2) - 1;
	
	// Se eliminan las partes enteras
	x1 = modf(x1, &x3);
	x2 = modf(x2, &x3);

	// Interpolación lineal
	if (beta075	!= NULL) *beta075 =
			  x1*(1.0-x2)*tabla_beta075[CP2][J1] +		 x1*x2*tabla_beta075[CP2][J2] +
		(1.0-x1)*(1.0-x2)*tabla_beta075[CP1][J1] + (1.0-x1)*x2*tabla_beta075[CP1][J2] ;

	return (
		      x1*(1.0-x2)*tabla_etaP[CP2][J1] +		  x1*x2*tabla_etaP[CP2][J2] +
		(1.0-x1)*(1.0-x2)*tabla_etaP[CP1][J1] + (1.0-x1)*x2*tabla_etaP[CP1][J2] ) / 100.0;
}

void acellin(double *xp, double *x)
{
	*xp		= XX/est1[MTOTAL] + G0*est1[LBH13] - prod(est1[Q],*(x+2)) + prod(est1[R],*(x+1));
	*(xp+1) = YY/est1[MTOTAL] + G0*est1[LBH23] + prod(est1[P],*(x+2)) - prod(est1[R],*x);
	*(xp+2) = ZZ/est1[MTOTAL] + G0*est1[LBH33] - prod(est1[P],*(x+1)) + prod(est1[Q],*x);

	return;
}

void acelang(double *xp, double *x)
{
	double A, B, C;

	A = LL + est1[MTOTAL]*(diff(RY2,RZ2)*prod(*(x+1),*(x+2)) + PXZ*prod(*x,*(x+1)));
	B = MM + est1[MTOTAL]*(diff(RZ2,RX2)*prod(*x,*(x+2)) + PXZ*diff(prod(*(x+2),*(x+2)),prod(*x,*x)));
	C = NN + est1[MTOTAL]*(diff(RX2,RY2)*prod(*x,*(x+1)) - PXZ*prod(*(x+1),*(x+2)));
	
	*xp		= (RZ2*A + PXZ*C) / (est1[MTOTAL]*diff(RX2*RZ2,PXZ*PXZ));
	*(xp+1) = B / (est1[MTOTAL]*RY2);
	*(xp+2) = (PXZ*A + RZ2*C) / (est1[MTOTAL]*diff(RX2*RZ2,PXZ*PXZ));

	return;
}

void longlat(double *xp, double *x)
{
	double uN = est1[U]*est1[LBH11] + est1[V]*est1[LBH21] + est1[W]*est1[LBH31];
	double vE = est1[U]*est1[LBH12] + est1[V]*est1[LBH22] + est1[W]*est1[LBH32];
	
	*xp		= uN / RT * CONV_RD;
	*(xp+1) = vE / RT * CONV_RD / cos(*x/CONV_RD);

	return;
}

void dcuaternion(double *xp, double *x)
{ 
	double eps = 1 - (*x * *x + *(x+1) * *(x+1) + *(x+2) * *(x+2) + *(x+3) * *(x+3)); // Desviación radial

	*xp		= (prod(KE*eps,*x)	- prod(est1[P],*(x+1)) - prod(est1[Q],*(x+2)) - prod(est1[R],*(x+3)))/(2*CONV_RD);
	*(xp+1) = (prod(est1[P],*x) + prod(KE*eps,*(x+1))  + prod(est1[R],*(x+2)) - prod(est1[Q],*(x+3)))/(2*CONV_RD);
	*(xp+2) = (prod(est1[Q],*x) - prod(est1[R],*(x+1)) + prod(KE*eps,*(x+2))  + prod(est1[P],*(x+3)))/(2*CONV_RD);
	*(xp+3) = (prod(est1[R],*x) + prod(est1[Q],*(x+1)) - prod(est1[P],*(x+2)) +	prod(KE*eps,*(x+3)))/(2*CONV_RD);

	return;
}