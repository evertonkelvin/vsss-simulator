/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#include "TiposClasses.h"
#include "Auxiliares.h"
#include "Controle.h"

#define CteEstCmd 0.5
#define CteEstVel 0.5
#define CTE_PARADA 4

//#define SO_UM
#define ATEH_DOIS
//#define ATEH_TRES
//#define ATEH_QUATRO
//#define ATEH_CINCO

#define VELOCIDADE_ANGULAR_MAXIMA 17.8	//(vMaxRd-vMaxRe)*RAIO_RODA/DIST_RODAS//Vel1 = 8.9 //Vel2 = 17.85 //Vel3 = 26.8 //Vel4 = 35.7
#define VELOCIDADE_MAX 107.1			//(vMaxRe+vMaxRd)*RAIO_RODA/2 //Vel3 = 107.1 //Vel4 = 142.8 //Vel5 = 178.5
#define RAIO_DA_RODA 1.7 //16,5 + 0,5 = 17mm
#define RAIO_DISTANCIA 5
#define VEL_MAX 7
extern FutebolCamera *futCam[NUM_CAMERAS];

extern Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1],
		estadoPrev[NUM_ROBOS_TIME * 2 + 1];

extern CmdEnviado cmdEnviado[10][NUM_ROBOS_TIME]; //comando enviado aos robos

float xObjAnt[3] = { 0, 0, 0 }, yObjAnt[3] = { 0, 0, 0 };

extern int indAtacante;
//-----------------------------CAMPO POTENCIAL---------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
//#include <tgmath.h>
//#include <cv.h>
//#include <highgui.h>

#define MAX_X 43
#define MAX_Y 33			// Tem que ser 35 e deslocar 1 em todas as coordenadas em Y
#define DIV_CAMPO 4
#define W_SOR 1.8
#define	WND_X 1350			// TAMANHO DA JANELA GRÁFICA EM X ----- VOLTAR PARA 1350
#define	WND_Y 350			// TAMANHO DA JANELA GRÁFICA EM Y -----	VOLTAR PARA 350
#define TAM_RET 10 			// SEMPRE QUE ALTERAR TAM_RET DEVE-SE ALTERAR LINE_LENGTH ------ VOLTAR PARA 10
#define LINE_LENGTH 8.0		// TAM_RET - 0.2*TAM_RET -> ESSA CONSTATE SEMPRE SERÁ DOUBLE/FLOAT
#define E 0.00001			// PRECISÃO DE CONVERGÊNCIA
#define X 0
#define Y 1
#define E_CPO 3
//using namespace cv;

typedef struct position {
	int posX;
	int posY;
	position *nextPosition;
} position_list;

position_list *caminhoRobo[3] = { NULL, NULL, NULL };

typedef struct campo {
	float matPot[MAX_X][MAX_Y]; // Valores pertencentes ao intervalo [0,1]
	bool matBoolPot[MAX_X][MAX_Y]; // true = obstáculo ou meta, false = espaço livre
} campoPot;

typedef struct campoK {
	double matAng [MAX_X][MAX_Y]; // Ângulo do vetor de força daquela célula
	float matPot[MAX_X][MAX_Y]; // Valores pertencentes ao intervalo [0,1]
	bool matBoolPot[MAX_X][MAX_Y]; // (Meta e Obstáculo) = TRUE (Célula livre) = FALSE
}campoPotKhatib;

void inicializa_obst_meta(campoPot *campoPotencial, int xObjetivo,
		int yObjetivo, int indJogador, double angObjetivo) {
	int i, j, k;

	for (i = 0; i < MAX_Y; i++) {
		campoPotencial->matPot[0][i] = 1;
		campoPotencial->matBoolPot[0][i] = true;
	}
	for (i = 0; i < MAX_Y; i++) {
		campoPotencial->matPot[MAX_X - 1][i] = 1;
		campoPotencial->matBoolPot[MAX_X - 1][i] = true;
	}
	for (i = 0; i < MAX_X; i++) {
		campoPotencial->matPot[i][0] = 1;
		campoPotencial->matBoolPot[i][0] = true;
	}
	for (i = 0; i < MAX_X; i++) {
		campoPotencial->matPot[i][MAX_Y - 1] = 1;	// inicializam as paredes
		campoPotencial->matBoolPot[i][MAX_Y - 1] = true;
	}

	for (i = 1; i < MAX_X - 1; i++) {
		for (j = 1; j < MAX_Y - 1; j++) {
			campoPotencial->matBoolPot[i][j] = false; // inicializa células livres
			campoPotencial->matPot[i][j] = 0;
		}
	}

//	for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
//		for (j=yObjetivo-1; j<=yObjetivo+1 ; j++){
//			campoPotencial->matBoolPot[i][j] = true; // meta
//		}
//	}
	campoPotencial->matBoolPot[xObjetivo][yObjetivo] = true;

//	j = 7;
//	for (i=0; i<=14 ; i++){
//		campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//		campoPotencial->matPot[i][j] = 1;
//	}
//
//	for (i=14; i<=42 ; i++){
//		campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//		campoPotencial->matPot[i][j] = 1;
//	}

	if (indJogador == indAtacante) {
//		if (angObjetivo < 90 && angObjetivo > 20) {
		if (true) {
////					j = yObjetivo-1;
////					for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
////						campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
////						campoPotencial->matPot[i][j] = 1;
////					}
//			j = yObjetivo + 2;
//			for (i = xObjetivo - 1; i <= xObjetivo + 1; i++) {
//				campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//				campoPotencial->matPot[i][j] = 1;
//			}
//			i = xObjetivo + 1;
//			for (j = yObjetivo - 1; j <= yObjetivo + 2; j++) {
//				campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//				campoPotencial->matPot[i][j] = 1;
//			}
//			i = xObjetivo + 1;
			for (j = yObjetivo - 1; j <= yObjetivo + 2; j++) {
				for (i = xObjetivo + 1; i < xObjetivo + 4; i++){
					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
					campoPotencial->matPot[i][j] = 1;
					}
				}
		} else {

			if (angObjetivo > -90 && angObjetivo < -20) {
				j = yObjetivo - 2;
				for (i = xObjetivo - 2; i <= xObjetivo + 1; i++) {
					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
					campoPotencial->matPot[i][j] = 1;
				}
//						j = yObjetivo+1;
//						for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
//							campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//							campoPotencial->matPot[i][j] = 1;
//						}
				i = xObjetivo + 1;
				for (j = yObjetivo - 2; j <= yObjetivo + 1; j++) {
					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
					campoPotencial->matPot[i][j] = 1;
				}
			} else {
				j = yObjetivo - 2;
				for (i = xObjetivo; i <= xObjetivo + 1; i++) {
					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
					campoPotencial->matPot[i][j] = 1;
				}
				j = yObjetivo + 2;
				for (i = xObjetivo; i <= xObjetivo + 1; i++) {
					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
					campoPotencial->matPot[i][j] = 1;
				}
				i = xObjetivo + 1;
				for (j = yObjetivo - 2; j <= yObjetivo + 2; j++) {
					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
					campoPotencial->matPot[i][j] = 1;
				}
			}
		}

	}
	for (i = 0; i <= 2; i++) {
		if (i != indJogador) {
			if ((xObjetivo < estadoPrev[i].x / DIV_CAMPO - 1
					|| xObjetivo > estadoPrev[i].x / DIV_CAMPO + 1)
					&& (yObjetivo < estadoPrev[i].y / DIV_CAMPO - 1
							|| yObjetivo > estadoPrev[i].y / DIV_CAMPO + 1))
				for (j = (int) estadoPrev[i].x / DIV_CAMPO - 1;
						j <= (int) estadoPrev[i].x / DIV_CAMPO + 1; j++) {
					for (k = (int) estadoPrev[i].y / DIV_CAMPO - 1;
							k <= (int) estadoPrev[i].y / DIV_CAMPO + 1; k++) {
						campoPotencial->matPot[j][k] = 1;
						campoPotencial->matBoolPot[j][k] = true;
					}
				}
		}
	}
	for (i = 4; i <= 6; i++) {
		campoPotencial->matPot[(int) estadoPrev[i].x / DIV_CAMPO][(int) estadoPrev[i].y
				/ DIV_CAMPO] = 1;
		campoPotencial->matBoolPot[(int) estadoPrev[i].x / DIV_CAMPO][(int) estadoPrev[i].y
				/ DIV_CAMPO] = true;

	}

}

void inicializa_obst_meta_khatib(campoPotKhatib *campoPotencial, int xObjetivo,
		int yObjetivo, int indJogador, double angObjetivo) {
	int i, j, k;
	float K = 0.5;
	double auxAng;

	for (i = 0; i < MAX_Y; i++) {
		campoPotencial->matPot[0][i] = 1;
		campoPotencial->matBoolPot[0][i] = true;
	}
	for (i = 0; i < MAX_Y; i++) {
		campoPotencial->matPot[MAX_X - 1][i] = 1;
		campoPotencial->matBoolPot[MAX_X - 1][i] = true;
	}
	for (i = 0; i < MAX_X; i++) {
		campoPotencial->matPot[i][0] = 1;
		campoPotencial->matBoolPot[i][0] = true;
	}
	for (i = 0; i < MAX_X; i++) {
		campoPotencial->matPot[i][MAX_Y - 1] = 1;	// inicializam as paredes
		campoPotencial->matBoolPot[i][MAX_Y - 1] = true;
	}

	for (i = 1; i < MAX_X - 1; i++) {
		for (j = 1; j < MAX_Y - 1; j++) {
			campoPotencial->matBoolPot[i][j] = false; // inicializa células livres
			campoPotencial->matPot[i][j] = 0;
		}
	}

//	for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
//		for (j=yObjetivo-1; j<=yObjetivo+1 ; j++){
//			campoPotencial->matBoolPot[i][j] = true; // meta
//		}
//	}
	campoPotencial->matBoolPot[xObjetivo][yObjetivo] = true;

//	j = 7;
//	for (i=0; i<=14 ; i++){
//		campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//		campoPotencial->matPot[i][j] = 1;
//	}
//
//	for (i=14; i<=42 ; i++){
//		campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//		campoPotencial->matPot[i][j] = 1;
//	}

	if (indJogador == indAtacante) {

//			if (angObjetivo > -90 && angObjetivo < -20) {
//				j = yObjetivo - 2;
//				for (i = xObjetivo - 2; i <= xObjetivo + 1; i++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
////						j = yObjetivo+1;
////						for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
////							campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
////							campoPotencial->matPot[i][j] = 1;
////						}
//				i = xObjetivo + 1;
//				for (j = yObjetivo - 2; j <= yObjetivo + 1; j++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
//			} else {
//				j = yObjetivo - 2;
//				for (i = xObjetivo; i <= xObjetivo + 1; i++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
//				j = yObjetivo + 2;
//				for (i = xObjetivo; i <= xObjetivo + 1; i++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
//				i = xObjetivo + 1;
//				for (j = yObjetivo - 2; j <= yObjetivo + 2; j++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
//			}

	}
//	for (i = 0; i <= 6; i++) {
//		if (i != indJogador && i != 3) {
//			if ((xObjetivo < estadoPrev[i].x / DIV_CAMPO - 1
//					|| xObjetivo > estadoPrev[i].x / DIV_CAMPO + 1)
//					&& (yObjetivo < estadoPrev[i].y / DIV_CAMPO - 1
//							|| yObjetivo > estadoPrev[i].y / DIV_CAMPO + 1))
//				for (j = (int) estadoPrev[i].x / DIV_CAMPO;
//						j <= (int) estadoPrev[i].x / DIV_CAMPO + 1; j++) {
//					for (k = (int) estadoPrev[i].y / DIV_CAMPO;
//							k <= (int) estadoPrev[i].y / DIV_CAMPO +1; k++) {
//						campoPotencial->matPot[j][k] = 1;
//						campoPotencial->matBoolPot[j][k] = true;
//					}
//				}
//		}
//	}

//		for (i = 0; i <= 6; i++) {
//			if (i != 3 && i != indJogador){
//			campoPotencial->matPot[(int) estadoPrev[i].x / DIV_CAMPO][(int) estadoPrev[i].y
//					/ DIV_CAMPO] = 1;
//			campoPotencial->matBoolPot[(int) estadoPrev[i].x / DIV_CAMPO][(int) estadoPrev[i].y
//					/ DIV_CAMPO] = true;
//			}
//		}

		for (i=1 ; i<MAX_X-1 ; i++){
			for (j=1 ; j<MAX_Y-1 ; j++){
				if (!campoPotencial->matBoolPot[i][j]){ // inicializa células livres
					campoPotencial->matPot[i][j] = K;
					auxAng = atan2((double)(yObjetivo - j), (double)(xObjetivo - i));
					if (auxAng < 0)
						auxAng += 2*M_PI;
	//				if (auxAng >= 2*M_PI)
	//					auxAng -= 2*M_PI;
					campoPotencial->matAng[i][j] = auxAng;
					// printf("%f\n", campoPotencial->matAng[i][j]);

				}
			}
		}
}

void inicializa_obst_meta_khatib_desenha(campoPotKhatib *campoPotencial, int xObjetivo,
		int yObjetivo, int indJogador, double angObjetivo) {
	int i, j, k;
	float K = 0.5;
	double auxAng;

	for (i = 0; i < MAX_Y; i++) {
		campoPotencial->matPot[0][i] = 1;
		campoPotencial->matBoolPot[0][i] = true;
	}
	for (i = 0; i < MAX_Y; i++) {
		campoPotencial->matPot[MAX_X - 1][i] = 1;
		campoPotencial->matBoolPot[MAX_X - 1][i] = true;
	}
	for (i = 0; i < MAX_X; i++) {
		campoPotencial->matPot[i][0] = 1;
		campoPotencial->matBoolPot[i][0] = true;
	}
	for (i = 0; i < MAX_X; i++) {
		campoPotencial->matPot[i][MAX_Y - 1] = 1;	// inicializam as paredes
		campoPotencial->matBoolPot[i][MAX_Y - 1] = true;
	}

	for (i = 1; i < MAX_X - 1; i++) {
		for (j = 1; j < MAX_Y - 1; j++) {
			campoPotencial->matBoolPot[i][j] = false; // inicializa células livres
			campoPotencial->matPot[i][j] = 0;
		}
	}

//	for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
//		for (j=yObjetivo-1; j<=yObjetivo+1 ; j++){
//			campoPotencial->matBoolPot[i][j] = true; // meta
//		}
//	}
	campoPotencial->matBoolPot[xObjetivo][yObjetivo] = true;

//	j = 7;
//	for (i=0; i<=14 ; i++){
//		campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//		campoPotencial->matPot[i][j] = 1;
//	}
//
//	for (i=14; i<=42 ; i++){
//		campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//		campoPotencial->matPot[i][j] = 1;
//	}

	if (indJogador == indAtacante) {
//
//			if (angObjetivo > -90 && angObjetivo < -20) {
//				j = yObjetivo - 2;
//				for (i = xObjetivo - 2; i <= xObjetivo + 1; i++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
////						j = yObjetivo+1;
////						for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
////							campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
////							campoPotencial->matPot[i][j] = 1;
////						}
//				i = xObjetivo + 1;
//				for (j = yObjetivo - 2; j <= yObjetivo + 1; j++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
//			} else {
//				j = yObjetivo - 2;
//				for (i = xObjetivo; i <= xObjetivo + 1; i++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
//				j = yObjetivo + 2;
//				for (i = xObjetivo; i <= xObjetivo + 1; i++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
//				i = xObjetivo + 1;
//				for (j = yObjetivo - 2; j <= yObjetivo + 2; j++) {
//					campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
//					campoPotencial->matPot[i][j] = 1;
//				}
//			}

	}
//	for (i = 0; i <= 2; i++) {
//		if (i != indJogador) {
//			if ((xObjetivo < estadoPrev[i].x / DIV_CAMPO - 1
//					|| xObjetivo > estadoPrev[i].x / DIV_CAMPO + 1)
//					&& (yObjetivo < estadoPrev[i].y / DIV_CAMPO - 1
//							|| yObjetivo > estadoPrev[i].y / DIV_CAMPO + 1))
//				for (j = (int) estadoPrev[i].x / DIV_CAMPO;
//						j <= (int) estadoPrev[i].x / DIV_CAMPO; j++) {
//					for (k = (int) estadoPrev[i].y / DIV_CAMPO;
//							k <= (int) estadoPrev[i].y / DIV_CAMPO; k++) {
//						campoPotencial->matPot[j][k] = 1;
//						campoPotencial->matBoolPot[j][k] = true;
//					}
//				}
//		}
//	}
//	for (i = 0; i <= 6; i++) {
//		if (i != 3){
//		campoPotencial->matPot[(int) estadoPrev[i].x / DIV_CAMPO][(int) estadoPrev[i].y
//				/ DIV_CAMPO] = 1;
//		campoPotencial->matBoolPot[(int) estadoPrev[i].x / DIV_CAMPO][(int) estadoPrev[i].y
//				/ DIV_CAMPO] = true;
//		}
//	}

	for (i=1 ; i<MAX_X-1 ; i++){
		for (j=1 ; j<MAX_Y-1 ; j++){
			if (!campoPotencial->matBoolPot[i][j]){ // inicializa células livres
				campoPotencial->matPot[i][j] = K;
				auxAng = atan2((double)(yObjetivo - j), (double)(xObjetivo - i));
//				if (auxAng < 0)
//					auxAng += 2*M_PI;
				campoPotencial->matAng[i][j] = auxAng;
//				printf("%f\n", campoPotencial->matAng[i][j]);

			}
		}
	}

}


bool calcula_campo_SOR(campoPot *campoPotencial) {
	int i, j;
	float resultTemp;
	bool convergiu = true;
	do {
		convergiu = true;
		for (i = 1; i < MAX_X - 1; i++) {
			for (j = 1; j < MAX_Y - 1; j++) {
				if (campoPotencial->matBoolPot[i][j] == false) {
					resultTemp = W_SOR
							* (campoPotencial->matPot[i + 1][j]
									+ campoPotencial->matPot[i - 1][j]
									+ campoPotencial->matPot[i][j + 1]
									+ campoPotencial->matPot[i][j - 1]
									- 4 * campoPotencial->matPot[i][j]) / 4
							+ campoPotencial->matPot[i][j];
					if ((campoPotencial->matPot[i][j] - resultTemp > E)
							|| (resultTemp - campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}

			}
		}
	} while (!convergiu);
	return convergiu;
}


bool calcula_campo_SOR_CPLO(campoPot *campoPotencial, float v_CPO[2],
		int xObjetivo, int yObjetivo) {
	int i, j, cont = 0;
	float resultTemp;

	bool convergiu = true;
	do {
		for (i = 1; i < MAX_X - 1; i++) {
			for (j = 1; j < MAX_Y - 1; j++) {
				if (campoPotencial->matBoolPot[i][j] == false) {
					if ((i < xObjetivo + 5 && j < yObjetivo + 5)
							&& (i > xObjetivo - 5 && j > yObjetivo - 5)) {
						resultTemp = (campoPotencial->matPot[i + 1][j]
								+ campoPotencial->matPot[i - 1][j]
								+ campoPotencial->matPot[i][j + 1]
								+ campoPotencial->matPot[i][j - 1]) / 4
								+ ((campoPotencial->matPot[i + 1][j]
										- campoPotencial->matPot[i - 1][j])
										* v_CPO[X]
										+ (campoPotencial->matPot[i][j + 1]
												- campoPotencial->matPot[i][j
														- 1]) * v_CPO[Y])
										* E_CPO / 8;
//						resultTemp = (W_SOR
//								* (campoPotencial->matPot[i + 1][j]
//										+ campoPotencial->matPot[i - 1][j]
//										+ campoPotencial->matPot[i][j + 1]
//										+ campoPotencial->matPot[i][j - 1]
//										- 4 * campoPotencial->matPot[i][j]) / 4
//								+ campoPotencial->matPot[i][j])
//								+ ((campoPotencial->matPot[i + 1][j]
//										- campoPotencial->matPot[i - 1][j])
//										* v_CPO[X]
//										+ (campoPotencial->matPot[i][j + 1]
//												- campoPotencial->matPot[i][j
//														- 1]) * v_CPO[Y])
//										* E_CPO / 8;
					} else {
						resultTemp = W_SOR
								* (campoPotencial->matPot[i + 1][j]
										+ campoPotencial->matPot[i - 1][j]
										+ campoPotencial->matPot[i][j + 1]
										+ campoPotencial->matPot[i][j - 1]
										- 4 * campoPotencial->matPot[i][j]) / 4
								+ campoPotencial->matPot[i][j];

					}
					if ((campoPotencial->matPot[i][j] - resultTemp > E)
							|| (resultTemp - campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}

			}
		}
		cont++;
	} while (!convergiu && cont < 500);
	return convergiu;
}


bool calcula_campo_SOR_CPO(campoPot *campoPotencial, float v_CPO[2]){
	int i,j,cont=0;
	float resultTemp;

	bool convergiu = true;
	do{
		for (i=1 ; i<MAX_X-1 ; i++){
			for(j=1 ; j<MAX_Y-1 ; j++){
				if (campoPotencial->matBoolPot[i][j] == false){
					resultTemp = (campoPotencial->matPot[i+1][j] + campoPotencial->matPot[i-1][j] + campoPotencial->matPot[i][j+1] + campoPotencial->matPot[i][j-1])/4  + ((campoPotencial->matPot[i+1][j] - campoPotencial->matPot[i-1][j])*v_CPO[X] + (campoPotencial->matPot[i][j+1] - campoPotencial->matPot[i][j-1])*v_CPO[Y])*E_CPO/8;
//						resultTemp = (W_SOR*(campoPotencial->matPot[i+1][j]+campoPotencial->matPot[i-1][j]+campoPotencial->matPot[i][j+1]+campoPotencial->matPot[i][j-1] - 4*campoPotencial->matPot[i][j])/4 + campoPotencial->matPot[i][j])  + ((campoPotencial->matPot[i+1][j] - campoPotencial->matPot[i-1][j])*v_CPO[X] + (campoPotencial->matPot[i][j+1] - campoPotencial->matPot[i][j-1])*v_CPO[Y])*E_CPO/8;
					if ((campoPotencial->matPot[i][j]-resultTemp > E) || (resultTemp-campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}

			}
		}
	cont++;
	}while(!convergiu && cont<500);
	return convergiu;
}

bool calcula_campo_CP(campoPotKhatib *campoPotencial){
	int i,j,k,l, sum, sum2;
//	float resultTemp, resultTemp2;
	float F1, F2, FR;
	double auxAng, difAng;
	#define K  0.5
	int d = 4;
	bool convergiu = true;
	do{
		convergiu = true;
		for (i=1 ; i<MAX_X-1 ; i++){
			for(j=1 ; j<MAX_Y-1 ; j++){
			if (campoPotencial->matBoolPot[i][j] && campoPotencial->matPot[i][j] == 1){
				sum = i-d;
				if (sum <1)
					sum = 1;
				sum2 = j-d;
				if (sum2 <1)
					sum2 = 1;
				for (k=sum ; k<i+d && k <= MAX_X-1; k++){
					for(l=sum2 ; l<j+d  && k <= MAX_Y-1; l++){
						if (!campoPotencial->matBoolPot[k][l]){
							auxAng = atan2((double) (j-l)*(-1), (double) (i-k)*(-1));
							if (auxAng < 0)
								auxAng += 2*M_PI;
							if (campoPotencial->matAng[k][l] < 0)
								campoPotencial->matAng[k][l]  += 2*M_PI;
//							while (auxAng >= 2*M_PI)
//								auxAng -= 2*M_PI;
							if (auxAng >= campoPotencial->matBoolPot[k][l]){
								difAng = auxAng - campoPotencial->matAng[k][l];
								F2 = campoPotencial->matPot[k][l];
								F1 = K/(pow(k - i,2) + pow(l - j,2));
								auxAng = campoPotencial->matAng[k][l];
							}
							else{
								difAng = campoPotencial->matAng[k][l] - auxAng;
								F1 = campoPotencial->matPot[k][l];
								F2 = K/(pow(k - i,2) + pow(l - j,2));
							}
							FR = sqrt(pow(F1,2) + pow(F2,2) + 2 * F1 * F2 * cos(difAng));
							campoPotencial->matAng[k][l] = asin((F2 * sin (M_PI - difAng))/FR) + auxAng;
							if (FR > 1)
								FR = 1;
							campoPotencial->matPot[k][l] = FR;
						}
					}
				}
			}


			while (campoPotencial->matAng[i][j] < -M_PI)
						campoPotencial->matAng[i][j] += 2*M_PI;
			while (campoPotencial->matAng[i][j] > M_PI)
						campoPotencial->matAng[i][j] -= 2*M_PI;
			}
		}
	}while(!convergiu);
	return convergiu;
}

//----------------------------------------------------------------------------------------------

void calculaVelMotores(int NumRobo, int *me, int *md) {
	float dx, dy, ang, v_cm, v_unid, ve, vd, aux, difVelRodas;
	//Calcula abaixo a velocidade dos motores do robo
	ang = estadoPrev[NumRobo].angulo;
	dx = estadoPrev[NumRobo].dx;
	dy = estadoPrev[NumRobo].dy;

	v_cm = sqrt(dx * dx + dy * dy); //Calcula a velocidade do Robo em cm
	//Calcula as velocidade dos motores em unidades
	v_unid = v_cm * FATOR_VEL_ROBO_UNID_POR_CM;
	if (v_unid <= VEL_MAXIMA_ROBO_UNID) {
		if (abs(difAngMenos180a180(atang2(dy, dx), ang)) > 90) {
			//Robo andando de re';
			v_unid = -v_unid; //frente (etiquetas) para um lado, deslocamento para o outro
		}

		aux = difAngMenos180a180(ang, estadoAnt[NumRobo].angulo);
		// a cada FATOR_ANGULO_DIF_RODAS_UNID_VEL graus significa uma diferenca entre rodas de uma unidade de velocidade
		difVelRodas = abs(aux / FATOR_ANGULO_DIF_RODAS_UNID_VEL);

		if (difVelRodas > VEL_MAXIMA_ROBO_UNID * 2)
			difVelRodas = VEL_MAXIMA_ROBO_UNID * 2;
		if (aux > 0) { //Robo virando para dir.
			ve = v_unid - difVelRodas / 2;
			vd = v_unid + difVelRodas / 2;
			if (ve < -VEL_MAXIMA_ROBO_UNID) {
				ve = -VEL_MAXIMA_ROBO_UNID;
				vd = ve + difVelRodas;
			} else if (vd > VEL_MAXIMA_ROBO_UNID) {
				vd = VEL_MAXIMA_ROBO_UNID;
				ve = vd - difVelRodas;
			}
		} else { //Robo virando para esq
			ve = v_unid + difVelRodas / 2;
			vd = v_unid - difVelRodas / 2;
			if (ve > VEL_MAXIMA_ROBO_UNID) {
				ve = VEL_MAXIMA_ROBO_UNID;
				vd = ve - difVelRodas;
			} else if (vd < -VEL_MAXIMA_ROBO_UNID) {
				vd = -VEL_MAXIMA_ROBO_UNID;
				ve = vd + difVelRodas;
			}
		}
	} else { //robo muito rapido, possivelmente erro
		ve = (cmdEnviado[0][NumRobo].esq + cmdEnviado[1][NumRobo].esq
				+ cmdEnviado[2][NumRobo].esq + cmdEnviado[3][NumRobo].esq) / 4;
		vd = (cmdEnviado[0][NumRobo].dir + cmdEnviado[1][NumRobo].dir
				+ cmdEnviado[2][NumRobo].dir + cmdEnviado[3][NumRobo].dir) / 4;
	}
	*me = ve;
	*md = vd;
}
campoPot campoPotencial[3];
campoPotKhatib campoPotencialKhatib[3];
void calculaCmd(int indJogador, int angObjetivo, int xObjetivo, int yObjetivo,
		int velObjetivo) {

	float angRobo, xRobo, yRobo;
	float ang, auxAng;
	int ve, vd, pe, pd, i, j;
	float wObj, vObj, vMaxima;
	int DA;
	float dx, dy, v_CPO[2];
	float d;
	double directionAngle;
	int flagDirection;
	double lim = 180 / 8;
//	indAtacante = 1;
	if (indJogador == 1 || indJogador == 2) {
//		if(true){

			v_CPO[0] = cos((double)(angObjetivo)/180*M_PI);
			v_CPO[1] = sin((double)(angObjetivo)/180*M_PI);

		if (xObjetivo > 0 && yObjetivo > 0) {
			inicializa_obst_meta(&campoPotencial[indJogador], xObjetivo / DIV_CAMPO, yObjetivo / DIV_CAMPO, indJogador, angObjetivo);
			calcula_campo_SOR_CPLO(&campoPotencial[indJogador], v_CPO, xObjetivo, yObjetivo);
//			calcula_campo_SOR(&campoPotencial[indJogador]);
//			calcula_campo_SOR_CPO(&campoPotencial[indJogador], v_CPO);
			//calcula_campo_CP(&campoPotencialKhatib[indJogador]);

		}

		xObjAnt[indJogador] = xObjetivo;
		yObjAnt[indJogador] = yObjetivo;
		angRobo = estadoPrev[indJogador].angulo;
		xRobo = estadoPrev[indJogador].x;
		yRobo = estadoPrev[indJogador].y;
		dx = xObjetivo - xRobo;
		dy = yObjetivo - yRobo;
		d = sqrt(dx * dx + dy * dy);

		i = xRobo / DIV_CAMPO;
		j = yRobo / DIV_CAMPO;

		directionAngle = atan2(
				(double) (campoPotencial[indJogador].matPot[i][j - 1]
						- campoPotencial[indJogador].matPot[i][j + 1]),
				(double) (campoPotencial[indJogador].matPot[i - 1][j]
						- campoPotencial[indJogador].matPot[i + 1][j]))
				* 180/M_PI;
		//directionAngle = campoPotencialKhatib[indJogador].matAng[i][j] * 180/M_PI;
		//	printf("[%d](%f)\n", indJogador, (float)directionAngle);
//		printf(
//				"[%d](%f)\n[        ][%f][        ]\n[%f][        ][%f]\n[        ][%f][        ]\n",
//				indJogador, (float) directionAngle,
//				campoPotencial[indJogador].matPot[i][j + 1],
//				campoPotencial[indJogador].matPot[i - 1][j],
//				campoPotencial[indJogador].matPot[i + 1][j],
//				campoPotencial[indJogador].matPot[i][j - 1]);

		ang = (float) directionAngle;

		if (ang < 0) {
			ang += 360;
		}
		//		ang = atan2(yObjetivo - yRobo, xObjetivo - xRobo) * 180 / M_PI;
		if ((estadoPrev[indJogador].x > xObjetivo - RAIO_DISTANCIA
		&& estadoPrev[indJogador].x < xObjetivo + RAIO_DISTANCIA)
		&& (estadoPrev[indJogador].y > yObjetivo - RAIO_DISTANCIA
		&& estadoPrev[indJogador].y < yObjetivo + RAIO_DISTANCIA)) {
			ang = atan2(yObjetivo - yRobo, xObjetivo - xRobo) * 180 / M_PI;
			if (ang < 0) {
				ang += 2 * 180;
			}
		}
		// ver cálculo dos ângulos para que o robô não precise dar uma volta inteira. utilizando flag de dire
		// ção, primeiro e quarto quadrante flag = 1, e segundo e terceiro quadrante flag = -1.

		double dAng = ang - angRobo;
		if (dAng < 0) {
			dAng += 360;
		}
//		if (d < 6) {
//			vObj = 1;
//			wObj = 1;
//		} else if (d < 4) {
//			vObj = 0;
//			wObj = 0;

			vObj = 1;
			wObj = 1;

		if (dAng < lim) {
			pe = vObj;
			pd = vObj;
		} else if (dAng < 180 / 2 - lim) {
			pe = 0;
			pd = wObj;
		} else if (dAng < 180 / 2) {
			pe = -wObj;
			pd = wObj;
		} else if (dAng < 180 / 2 + lim) {
			pe = wObj;
			pd = -wObj;
		} else if (dAng < 180 - lim) {
			pe = 0;
			pd = -wObj;
		} else if (dAng < 180 + lim) {
			pe = -vObj;
			pd = -vObj;
		} else if (dAng < 3 * 180 / 2 - lim) {
			pe = -wObj;
			pd = 0;
		} else if (dAng < 3 * 180 / 2) {
			pe = -wObj;
			pd = wObj;
		} else if (dAng < 3 * 180 / 2 + lim) {
			pe = wObj;
			pd = -wObj;
		} else if (dAng < 2 * 180 - lim) {
			pe = wObj;
			pd = 0;
		} else {
			pe = vObj;
			pd = vObj;
		}

		cmdEnviado[0][indJogador].esq = pe * 1;
		cmdEnviado[0][indJogador].dir = pd * 1;

		//COMANDO PARA O VOLANTE OU ATACANTE
		// printf("\nCmdEnviado Robo[%x]: ", indJogador);
		// printf("esq: %x, dir: %x \n", cmdEnviado[0][indJogador].esq, cmdEnviado[0][indJogador].dir);

////		auxAng = dAng;
//		if ((dAng <= 90 && dAng > 0) || (dAng <= 360 && dAng >= 270)) {
//			flagDirection = 1;
//		} else{
//			flagDirection = -1;
//			dAng = 180 + dAng;
//		}
//		if (dAng < 0) {
//			dAng += 360;
//		}
//
//		//		vMaxima = VELOCIDADE_MAX;
//		if ((estadoPrev[indJogador].x > xObjetivo - RAIO_DISTANCIA
//				&& estadoPrev[indJogador].x < xObjetivo + RAIO_DISTANCIA)
//				&& (estadoPrev[indJogador].y > yObjetivo - RAIO_DISTANCIA
//						&& estadoPrev[indJogador].y < yObjetivo + RAIO_DISTANCIA)) {
//			if (indJogador == indAtacante
//					&& estadoPrev[indJogador].x < xObjetivo) {
//				vMaxima = (1.6 - d / (RAIO_DISTANCIA)) * VELOCIDADE_MAX;
//				if (vMaxima > VELOCIDADE_MAX) {
//					vMaxima = VELOCIDADE_MAX;
//				} else {
//					vMaxima = (d / (RAIO_DISTANCIA)) * VELOCIDADE_MAX
//							+ (1 - d / (RAIO_DISTANCIA)) * (velObjetivo * 21);
//
//				}
//
//			}
//		} else {
//			vMaxima = VELOCIDADE_MAX;
//		}
//		if (dAng > 270){
//			dAng-=360;
//		}
//
//		wObj = (dAng / 90) * VELOCIDADE_ANGULAR_MAXIMA;
//		if (dAng > 0)
//			vObj = (1 - dAng/ 90) * vMaxima;
//		else
//			vObj = (1 + dAng / 90))) * vMaxima;
//
//		pe = (int) ((2 * vObj * flagDirection - wObj * DIST_ENTRE_RODAS * flagDirection)/(2 * RAIO_DA_RODA))/21;
////					pe*= flagDirection;
//
//		if (pe > VEL_MAX) {
//			pe = VEL_MAX;
//		} else if (pe < -VEL_MAX) {
//			pe = -VEL_MAX;
//		}
////					if (pe<0)
//		//		pe = -pe + 8;
//
//		pd = (int) ((2 * vObj * flagDirection + wObj * DIST_ENTRE_RODAS * flagDirection)
//				/ (2 * RAIO_DA_RODA)) / 21;
////					pd*= flagDirection;
//
//		if (pd > VEL_MAX) {
//			pd = VEL_MAX;
//		} else if (pd < -VEL_MAX) {
//			pd = -VEL_MAX;
//		}
////					if (pd<0)
//		//		pd = -pd + 8;
//
//		cmdEnviado[0][indJogador].esq = pe * 1;
//		cmdEnviado[0][indJogador].dir = pd * 1;
	} else {
		double lim = 180 / 8;

		angRobo = estadoPrev[indJogador].angulo;
		xRobo = estadoPrev[indJogador].x;
		yRobo = estadoPrev[indJogador].y;
		dx = xObjetivo - xRobo;
		dy = yObjetivo - yRobo;
		d = sqrt(dx * dx + dy * dy);

		ang = atan2(yObjetivo - yRobo, xObjetivo - xRobo) * 180 / M_PI;
		if (ang < 0) {
			ang += 2 * 180;
		}
		double dAng = ang - angRobo;
		if (dAng < 0) {
			dAng += 2 * 180;
		}

		if (d < 10) {
			vObj = 2;
			wObj = 1;
		} else if (d < 6) {
			vObj = 0;
			wObj = 0;
		} else {
			vObj = 3;
			wObj = 1;
		}

		if (dAng < lim) {
			pe = vObj;
			pd = vObj;
		} else if (dAng < 180 / 2 - lim) {
			pe = 0;
			pd = wObj;
		} else if (dAng < 180 / 2) {
			pe = -wObj;
			pd = wObj;
		} else if (dAng < 180 / 2 + lim) {
			pe = wObj;
			pd = -wObj;
		} else if (dAng < 180 - lim) {
			pe = 0;
			pd = -wObj;
		} else if (dAng < 180 + lim) {
			pe = -vObj;
			pd = -vObj;
		} else if (dAng < 3 * 180 / 2 - lim) {
			pe = -wObj;
			pd = 0;
		} else if (dAng < 3 * 180 / 2) {
			pe = -wObj;
			pd = wObj;
		} else if (dAng < 3 * 180 / 2 + lim) {
			pe = wObj;
			pd = -wObj;
		} else if (dAng < 2 * 180 - lim) {
			pe = wObj;
			pd = 0;
		} else {
			pe = vObj;
			pd = vObj;
		}

		cmdEnviado[0][indJogador].esq = pe * 1;
		cmdEnviado[0][indJogador].dir = pd * 1;

		//COMANDO PARA O GOLEIRO
		// printf("\nCmdEnviado Robo[%x]: ", indJogador);
		// printf("esq: %x, dir: %x \n", cmdEnviado[0][indJogador].esq, cmdEnviado[0][indJogador].dir);
	}
}

