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
#define G0		32.174 // Aceleración gravitatoria constante (ft/s²)
#define CONV_RD 57.29577951 // Factor de conversión Radianes -> Grados (°/-)
#define KE		((2*CONV_RD)/(4*DT)) // Parámetro estabilizador del cuaternión (1/s)
#define RT		20.902233792e6 // Radio volumétrico de la Tierra (elipsoide WGS 84) (ft)

enum parametros {
	DELTA_T0, DELTA_E, DELTA_A, DELTA_R, DELTA_WM1, DELTA_WM2, DELTA_NP1, DELTA_NP2,
	ALPHA, BETA, U, V, W, UTAS, VTAS, WTAS, UWIND, VWIND, WWIND,
	E0, E1, E2, E3, P, Q, R, PSI, THETA, PHI, LAMBDA, MU, H, ROC,
	LBH11, LBH12, LBH13, LBH21, LBH22, LBH23, LBH31, LBH32, LBH33,
	MTOTAL, WFUEL1, WFUEL2, WH2O1, WH2O2, WH2O3, WH2O4, FIN };
/*
	DELTA_T0: Desviación de la temperatura a nivel del mar sobre el estándar, 15 °C (K)
	DELTA_E: Deflexión del timón horizontal {-20..+20} (°)
	DELTA_A: Deflexión de los alerones {-13.75..+13.75} (°)
	DELTA_R: Deflexión del timón de dirección {-23..+23} (°)
	DELTA_WM1: Posición del la palanca de gases del motor izqdo. {0..1}
	DELTA_WM2: Posición del la palanca de gases del motor drcho. {0..1}
	DELTA_NP1: Revoluciones hélice izqda. (Palanca de condición 1) {900..1200} (rev/min = RPM)
	DELTA_NP2: Revoluciones hélice drcha. (Palanca de condición 2) {900..1200} (rev/min = RPM)
	ALPHA:	 Ángulo de ataque del avión (°)
	BETA:	 Ángulo de resbalamiento del avión (°)
	U, V, W: Velocidad lineal inercial (ejes cuerpo) (ft/s)
	UTAS, VTAS, WTAS: Velocidad lineal aerodinámica (ejes cuerpo) (ft/s)
	UWIND, VWIND, WWIND: Velocidad del viento (ejes horizonte local) /ft/s)
	E#:		 Elementos del cuaternión unitario
	P, Q, R: Velocidad angular inercial (ejes cuerpo) (°/s)
	PSI:	 Ángulo de guiñada (°)
	THETA:	 Ángulo de asiento (°)
	PHI:	 Ángulo de balance (°)
	LAMBDA:	 Latitud geográfica (°)
	MU:		 Longitud geográfica (°)
	H:		 Altitud (ft)
	ROC:	 Rate of Climb (ft/s)
	LBH##:	 Elementos de la matriz de transformación Ejes horizonte local -> Ejes cuerpo (-)
	MTOTAL:	 Masa total del avión (slug)
	WFUEL1, WFUEL2: Peso tanque de combustible {1:izqdo 2:drcho} (lb)
			 Máximos (1),(2): 5.204 lb
	WH2O#:	 Agua embarcada en cada depósito {1: izdqo.ext., 2:izqdo.int., 3: drcho.int., 4:drcho.ext.} (lb)
			 Máximos (1),(4): 3.260 lb
					 (2),(3): 3.490 lb
	FIN:	 Marcador final
*/

enum flags {
	QUIT = 01,  // Salida de la simulación
	PAUS = 02,	// Pausa del programa
	INIT = 04,  // Cargar fichero 'init.txt' con las condiciones iniciales
	SETV = 010  // Cargar fichero 'set.txt' con las variables de estado
};

// Macros
#define diff(A,B) ((A) - (B)) // Diferencia
#define prod(A,B) ((A) * (B)) // Producto
#define minimo(A,B) ((A) < (B) ? (A) : (B)) // Mínimo

#define rel_cdg(A,B,C) {-(cdg0.x-(A))/12.0, -(cdg0.y-(B))/12.0, -(cdg0.z-(C))/12.0}
/* Posiciones relativas en el avión respecto al c.d.g.
	Entrada: Parámetros en (in,in,in), los tres ejes contrarios a los 'ejes cuerpo' (ver INFORME)
	Salida: Almacenamiento en (ft,ft,ft), 'ejes cuerpo'
*/

// Declaraciones de tipos
typedef double estado_t[FIN]; // Variables de estado, etc., del vehículo
typedef struct {
	double x;
	double y;
	double z;
} coord_t; // Radio-vectores desde el c.d.g. del vehículo

// Declaraciones de funciones
double *rk2(double *, double *, void (*)(double *, double *), unsigned); // Runge-Kutta de 2º orden
void atmISA(const double, const double,  double *, double *, double *); // Atmósfera ISA hasta tropopausa
void script(estado_t, char *); // Lectura de "scripts" con condiciones de vuelo pregrabadas

#endif