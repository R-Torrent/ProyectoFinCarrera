/*
Declaraciones y definiciones
Roger Torrent Ahn
PFC: Canadair CL-415 
Curso 2017-2018
*/

#ifndef PFC

#define PFC

// Constantes
#define DT		0.01666667 // Paso temporal (s) [= 60 Hz]
#define G0		32.174 // Aceleraci�n gravitatoria constante (ft/s�)
#define CONV_RD 57.29577951 // Factor de conversi�n Radianes -> Grados (�/-)
#define KE		((2*CONV_RD)/(4*DT)) // Par�metro estabilizador del cuaterni�n (1/s)
#define RT		20.902233792e6 // Radio volum�trico de la Tierra (elipsoide WGS 84) (ft)

enum parametros {
	DELTA_T0, DELTA_E, DELTA_A, DELTA_R, DELTA_WM1, DELTA_WM2, DELTA_NP1, DELTA_NP2,
	ALPHA, BETA, U, V, W, UTAS, VTAS, WTAS, UWIND, VWIND, WWIND,
	E0, E1, E2, E3, P, Q, R, PSI, THETA, PHI, LAMBDA, MU, H, ROC,
	LBH11, LBH12, LBH13, LBH21, LBH22, LBH23, LBH31, LBH32, LBH33,
	MTOTAL, WFUEL1, WFUEL2, WH2O1, WH2O2, WH2O3, WH2O4, FIN };
/*
	DELTA_T0: Desviaci�n de la temperatura a nivel del mar sobre el est�ndar, 15 �C (K)
	DELTA_E: Deflexi�n del tim�n horizontal {-20..+20} (�)
	DELTA_A: Deflexi�n de los alerones {-13.75..+13.75} (�)
	DELTA_R: Deflexi�n del tim�n de direcci�n {-23..+23} (�)
	DELTA_WM1: Posici�n del la palanca de gases del motor izqdo. {0..1}
	DELTA_WM2: Posici�n del la palanca de gases del motor drcho. {0..1}
	DELTA_NP1: Revoluciones h�lice izqda. (Palanca de condici�n 1) {900..1200} (rev/min = RPM)
	DELTA_NP2: Revoluciones h�lice drcha. (Palanca de condici�n 2) {900..1200} (rev/min = RPM)
	ALPHA:	 �ngulo de ataque del avi�n (�)
	BETA:	 �ngulo de resbalamiento del avi�n (�)
	U, V, W: Velocidad lineal inercial (ejes cuerpo) (ft/s)
	UTAS, VTAS, WTAS: Velocidad lineal aerodin�mica (ejes cuerpo) (ft/s)
	UWIND, VWIND, WWIND: Velocidad del viento (ejes horizonte local) /ft/s)
	E#:		 Elementos del cuaterni�n unitario
	P, Q, R: Velocidad angular inercial (ejes cuerpo) (�/s)
	PSI:	 �ngulo de gui�ada (�)
	THETA:	 �ngulo de asiento (�)
	PHI:	 �ngulo de balance (�)
	LAMBDA:	 Latitud geogr�fica (�)
	MU:		 Longitud geogr�fica (�)
	H:		 Altitud (ft)
	ROC:	 Rate of Climb (ft/s)
	LBH##:	 Elementos de la matriz de transformaci�n Ejes horizonte local -> Ejes cuerpo (-)
	MTOTAL:	 Masa total del avi�n (slug)
	WFUEL1, WFUEL2: Peso tanque de combustible {1:izqdo 2:drcho} (lb)
			 M�ximos (1),(2): 5.204 lb
	WH2O#:	 Agua embarcada en cada dep�sito {1: izdqo.ext., 2:izqdo.int., 3: drcho.int., 4:drcho.ext.} (lb)
			 M�ximos (1),(4): 3.260 lb
					 (2),(3): 3.490 lb
	FIN:	 Marcador final
*/

enum flags {
	QUIT = 01,  // Salida de la simulaci�n
	PAUS = 02,	// Pausa del programa
	INIT = 04,  // Cargar fichero 'init.txt' con las condiciones iniciales
	SETV = 010  // Cargar fichero 'set.txt' con las variables de estado
};

// Macros
#define diff(A,B) ((A) - (B)) // Diferencia
#define prod(A,B) ((A) * (B)) // Producto
#define minimo(A,B) ((A) < (B) ? (A) : (B)) // M�nimo

#define rel_cdg(A,B,C) {-(cdg0.x-(A))/12.0, -(cdg0.y-(B))/12.0, -(cdg0.z-(C))/12.0}
/* Posiciones relativas en el avi�n respecto al c.d.g.
	Entrada: Par�metros en (in,in,in), los tres ejes contrarios a los 'ejes cuerpo' (ver INFORME)
	Salida: Almacenamiento en (ft,ft,ft), 'ejes cuerpo'
*/

// Declaraciones de tipos
typedef double estado_t[FIN]; // Variables de estado, etc., del veh�culo
typedef struct {
	double x;
	double y;
	double z;
} coord_t; // Radio-vectores desde el c.d.g. del veh�culo

// Declaraciones de funciones
double *rk2(double *, double *, void (*)(double *, double *), unsigned); // Runge-Kutta de 2� orden
void atmISA(const double, const double,  double *, double *, double *); // Atm�sfera ISA hasta tropopausa
void script(estado_t, char *); // Lectura de "scripts" con condiciones de vuelo pregrabadas

#endif