/*
Datos del CL-415
Roger Torrent Ahn
PFC: Canadair CL-415 
Curso 2017-2018
*/

// Parámetros másicos
#define RX2 121.55 // Radios de giro al cuadrado (ft²)
#define RY2 133.38
#define RZ2 268.17
#define PXZ 5.36 // Producto de inercia de los planos {Oxy, Oyz} / masa total (ft²)

#define OEW 28548 // Peso vacío operativo (lb)

// Parámetros geométricos del avión CL-415
#define SW 1080 // Superficie alar (ft²)
#define BW 93.8333333 // Envergadura (ft)
#define CMA 11.62 // Cuerda media aerodinámica (ft)
#define baCMA 366.57 // Posición del borde de ataque de la CMA, respecto del «datum» (in)

// Parámetros geométricos de las hélices Hamilton Standard 14SF-19
#define DH 13 // Diámetro hélice (ft)

// Características del motor PW123AF
#define Pmax 2380 // Potencia máxima (shp)
#define Qref 10420 // Par mecánico de referencia (lb·ft)
#define CE 0.470 // Consumo específico (PPH/shp)
#define NPMIN 900.0 // Mínimo régimen de giro en crucero (rpm)
#define NPMAX 1200.0 // Máximo régimen de giro en crucero (rpm)

// Límites de los controles (°)
#define DAMIN -13.75 // Alerones
#define DAMAX 13.75
#define DEMIN -20.0 // Timón horizontal
#define DEMAX 20.0
#define DRMIN -23.0 // Timón vertical
#define DRMAX 23.0

// TODOS LOS COEFICIENTES REFERIDOS A MEDIDAS ANGULARES, LO HACEN
// EN GRADOS (1 REV = 360°), ¡NO EN RADIANES!

// Coeficientes de fuerzas aerodinámicas
#define CL_0 0.494
#define CL_alpha 0.098
#define CL_q 0.0707
#define CL_de 0.06951

#define CD_0 0.0791 
#define CD_alpha 0.00987

#define CY_beta -0.00893
#define CY_p 0
#define CY_r 0
#define CY_dr -0.0028

// Coeficientes de momentos aerodinámicos
	#define Cl_beta_0 -0.00145
	#define Cl_beta_alpha 7.3e-6
#define Cl_beta(A) (Cl_beta_0+Cl_beta_alpha*(A))
	#define Cl_p_0 -0.00876
	#define Cl_p_alpha -9.7e-6
#define Cl_p(A) (Cl_p_0+Cl_p_alpha*(A))
	#define Cl_r_0 0.0207
	#define Cl_r_alpha -0.0013
#define Cl_r(A) (Cl_r_0+Cl_r_alpha*(A))
#define Cl_da 0.00138
#define Cl_dr -0.000662

	#define Cm_0 -0.34
	#define Cm_alpha -0.044
#define Cm(A) (Cm_0 +Cm_alpha*(A))
#define Cm_q -0.24
#define Cm_de -0.0367

	#define Cn_beta_0 0.00191
	#define Cn_beta_alpha 6.3e-6
#define Cn_beta(A) (Cn_beta_0+Cn_beta_alpha*(A))
	#define Cn_p_0 0.0149
	#define Cn_p_alpha 0.0042
#define Cn_p(A) (Cn_p_0+Cn_p_alpha*(A))
	#define Cn_r_0 -0.0908
	#define Cn_r_alpha -0.00232
#define Cn_r(A) (Cn_r_0+Cn_r_alpha*(A))
	#define Cn_da_0 -5.19e-4
	#define Cn_da_alpha 4.4e-5
#define Cn_da(A) (Cn_da_0+Cn_da_alpha*(A))
#define Cn_dr 0.00109