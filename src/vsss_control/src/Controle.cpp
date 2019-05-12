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

#define VELOCIDADE_ANGULAR_MAXIMA 127.5 //(vRd-vRe)*RAIO_RODA/DIST_RODAS
#define VELOCIDADE_MAX 510				//(vRe+vRd)*RAIO_RODA/2
#define RAIO_DA_RODA 3.4
#define RAIO_DISTANCIA 5
extern FutebolCamera *futCam[NUM_CAMERAS];

extern Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1], estadoPrev[NUM_ROBOS_TIME * 2 + 1];

extern CmdEnviado cmdEnviado[10][NUM_ROBOS_TIME]; //comando enviado aos robos

float xObjAnt[3] = {0, 0, 0}, yObjAnt[3] = {0, 0, 0};

//-----------------------------CAMPO POTENCIAL---------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
//#include <tgmath.h>

// #include <cv.h>
// #include <highgui.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#define MAX_X 43
#define MAX_Y 33 // Tem que ser 35 e deslocar 1 em todas as coordenadas em Y
#define DIV_CAMPO 4
#define W_SOR 1.8
#define WND_X 1350		// TAMANHO DA JANELA GRÁFICA EM X ----- VOLTAR PARA 1350
#define WND_Y 350		// TAMANHO DA JANELA GRÁFICA EM Y -----	VOLTAR PARA 350
#define TAM_RET 10		// SEMPRE QUE ALTERAR TAM_RET DEVE-SE ALTERAR LINE_LENGTH ------ VOLTAR PARA 10
#define LINE_LENGTH 8.0 // TAM_RET - 0.2*TAM_RET -> ESSA CONSTATE SEMPRE SERÁ DOUBLE/FLOAT
#define E 0.00001		// PRECISÃO DE CONVERGÊNCIA
#define X 0
#define Y 1
#define E_CPO 1

using namespace cv;

typedef struct position
{
	int posX;
	int posY;
	position *nextPosition;
} position_list;

position_list *caminhoRobo[3] = {NULL, NULL, NULL};

Mat ret_image = Mat::zeros(WND_Y, WND_X, CV_8UC3);

typedef struct campo
{
	float matPot[MAX_X][MAX_Y];	// Valores pertencentes ao intervalo [0,1]
	bool matBoolPot[MAX_X][MAX_Y]; // true = obstáculo ou meta, false = espaço livre
} campoPot;

void inicializa_obst_meta(campoPot *campoPotencial, int xObjetivo, int yObjetivo, int indJogador)
{
	int i, j, k;

	for (i = 0; i < MAX_Y; i++)
	{
		campoPotencial->matPot[0][i] = 1;
		campoPotencial->matBoolPot[0][i] = true;
	}
	for (i = 0; i < MAX_Y; i++)
	{
		campoPotencial->matPot[MAX_X - 1][i] = 1;
		campoPotencial->matBoolPot[MAX_X - 1][i] = true;
	}
	for (i = 0; i < MAX_X; i++)
	{
		campoPotencial->matPot[i][0] = 1;
		campoPotencial->matBoolPot[i][0] = true;
	}
	for (i = 0; i < MAX_X; i++)
	{
		campoPotencial->matPot[i][MAX_Y - 1] = 1; // inicializam as paredes
		campoPotencial->matBoolPot[i][MAX_Y - 1] = true;
	}

	for (i = 1; i < MAX_X - 1; i++)
	{
		for (j = 1; j < MAX_Y - 1; j++)
		{
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

	//	if (indJogador > 0){
	//		j = yObjetivo-1;
	//		for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
	//			campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
	//			campoPotencial->matPot[i][j] = 1;
	//		}
	//		j = yObjetivo+1;
	//		for (i=xObjetivo-1; i<=xObjetivo+1 ; i++){
	//			campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
	//			campoPotencial->matPot[i][j] = 1;
	//		}
	//		i = xObjetivo+1;
	//		for (j=yObjetivo-1; j<=yObjetivo+1 ; j++){
	//			campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
	//			campoPotencial->matPot[i][j] = 1;
	//		}
	//	}
	for (i = 0; i <= 6; i++)
	{
		if (i != indJogador && i != 3)
		{
			for (j = (int)estadoPrev[i].x / DIV_CAMPO - 1; j <= (int)estadoPrev[i].x / DIV_CAMPO + 1; j++)
			{
				for (k = (int)estadoPrev[i].y / DIV_CAMPO - 1; k <= (int)estadoPrev[i].y / DIV_CAMPO + 1; k++)
				{
					campoPotencial->matPot[j][k] = 1;
					campoPotencial->matBoolPot[j][k] = true;
				}
			}
		}
	}
}

bool calcula_campo_SOR(campoPot *campoPotencial)
{
	int i, j;
	float resultTemp;
	bool convergiu = true;
	do
	{
		convergiu = true;
		for (i = 1; i < MAX_X - 1; i++)
		{
			for (j = 1; j < MAX_Y - 1; j++)
			{
				if (campoPotencial->matBoolPot[i][j] == false)
				{
					resultTemp = W_SOR * (campoPotencial->matPot[i + 1][j] + campoPotencial->matPot[i - 1][j] + campoPotencial->matPot[i][j + 1] + campoPotencial->matPot[i][j - 1] - 4 * campoPotencial->matPot[i][j]) / 4 + campoPotencial->matPot[i][j];
					if ((campoPotencial->matPot[i][j] - resultTemp > E) || (resultTemp - campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}
			}
		}
	} while (!convergiu);
	return convergiu;
}

bool calcula_campo_CP(campoPot *campoPotencial, float xMeta, float yMeta, float xRobo, float yRobo)
{
	int i, j;
	float resultTemp, resultTemp2;
	float distMeta, distRobo;
#define K 0.5
	bool convergiu = true;
	do
	{
		convergiu = true;
		for (i = 1; i < MAX_X - 1; i++)
		{
			for (j = 1; j < MAX_Y - 1; j++)
			{
				distMeta = sqrt(pow(xMeta / DIV_CAMPO - i, 2) + pow(yMeta / DIV_CAMPO - j, 2));
				distRobo = sqrt(pow(xRobo / DIV_CAMPO - i, 2) + pow(yRobo / DIV_CAMPO - j, 2));
				if (campoPotencial->matBoolPot[i][j] == false)
				{
					resultTemp = K - (distMeta / DIV_CAMPO * 5);
					if (resultTemp > 0.5)
						resultTemp = 0.5;
					if (resultTemp < 0)
						resultTemp = 0;
					resultTemp2 = K / distRobo;
					if (resultTemp2 > 0.5)
						resultTemp2 = 0.5;
					if (resultTemp2 < 0)
						resultTemp2 = 0;
					resultTemp += resultTemp2;
					if ((campoPotencial->matPot[i][j] - resultTemp > E) || (resultTemp - campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}
			}
		}
	} while (!convergiu);
	return convergiu;
}

bool calcula_campo_SOR_CPLO(campoPot *campoPotencial, float v_CPO[2], int xObjetivo, int yObjetivo)
{
	int i, j, cont = 0;
	float resultTemp;

	bool convergiu = true;
	do
	{
		for (i = 1; i < MAX_X - 1; i++)
		{
			for (j = 1; j < MAX_Y - 1; j++)
			{
				if (campoPotencial->matBoolPot[i][j] == false)
				{
					if ((i < xObjetivo + 5 && j < yObjetivo + 5) && (i > xObjetivo - 5 && j > yObjetivo - 5))
					{
						resultTemp = (campoPotencial->matPot[i + 1][j] + campoPotencial->matPot[i - 1][j] + campoPotencial->matPot[i][j + 1] + campoPotencial->matPot[i][j - 1]) / 4 + ((campoPotencial->matPot[i + 1][j] - campoPotencial->matPot[i - 1][j]) * v_CPO[X] + (campoPotencial->matPot[i][j + 1] - campoPotencial->matPot[i][j - 1]) * v_CPO[Y]) * E_CPO / 8;
						//						resultTemp = (W_SOR*(campoPotencial->matPot[i+1][j]+campoPotencial->matPot[i-1][j]+campoPotencial->matPot[i][j+1]+campoPotencial->matPot[i][j-1] - 4*campoPotencial->matPot[i][j])/4 + campoPotencial->matPot[i][j])  + ((campoPotencial->matPot[i+1][j] - campoPotencial->matPot[i-1][j])*v_CPO[X] + (campoPotencial->matPot[i][j+1] - campoPotencial->matPot[i][j-1])*v_CPO[Y])*E_CPO/8;
					}
					else
					{
						resultTemp = W_SOR * (campoPotencial->matPot[i + 1][j] + campoPotencial->matPot[i - 1][j] + campoPotencial->matPot[i][j + 1] + campoPotencial->matPot[i][j - 1] - 4 * campoPotencial->matPot[i][j]) / 4 + campoPotencial->matPot[i][j];
					}
					if ((campoPotencial->matPot[i][j] - resultTemp > E) || (resultTemp - campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}
			}
		}
		cont++;
	} while (!convergiu && cont < 500);
	return convergiu;
}

void desenha_campo(campoPot campoPotencial, Mat ret_image, int DESLOC_IMG, int posRoboX, int posRoboY, int indJogador)
{
	float multColor;
	double directionAngle, dXdouble, dYdouble, auxAngle;
	int sX, sY, dX, dY, descX, descY, auxSoma;
	position_list *q;

	for (int i = 0; i < MAX_X; i++)
	{
		for (int j = 0; j < MAX_Y; j++)
		{
			// multiplica o valor do campo por 255 para atingir um valor de cor dependendo do potencial
			multColor = 255 * (campoPotencial.matPot[i][j]);
			if (campoPotencial.matBoolPot[i][j] || (i == posRoboX && j == posRoboY))
			{
				if (campoPotencial.matPot[i][j] == 0)
				{
					rectangle(ret_image,
							  Point(i * TAM_RET + TAM_RET + DESLOC_IMG, j * TAM_RET + TAM_RET),
							  Point(i * TAM_RET + 2 * TAM_RET + DESLOC_IMG, j * TAM_RET + 2 * TAM_RET),
							  Scalar(0, 0, 0),
							  -1,
							  8);
				}
				else
				{
					if (i == posRoboX && j == posRoboY)
						rectangle(ret_image,
								  Point(i * TAM_RET + TAM_RET + DESLOC_IMG, j * TAM_RET + TAM_RET),
								  Point(i * TAM_RET + 2 * TAM_RET + DESLOC_IMG, j * TAM_RET + 2 * TAM_RET),
								  Scalar(0, 255, 0),
								  -1,
								  8);
					else
						rectangle(ret_image,
								  Point(i * TAM_RET + TAM_RET + DESLOC_IMG, j * TAM_RET + TAM_RET),
								  Point(i * TAM_RET + 2 * TAM_RET + DESLOC_IMG, j * TAM_RET + 2 * TAM_RET),
								  Scalar(255, 0, 0),
								  -1,
								  8);
				}
			}
			else
			{

				directionAngle = atan2((double)(campoPotencial.matPot[i - 1][j] - campoPotencial.matPot[i + 1][j]), (double)(campoPotencial.matPot[i][j - 1] - campoPotencial.matPot[i][j + 1]));

				if ((directionAngle <= -90.0 * (M_PI / 180.0)) && (directionAngle >= -M_PI))
				{
					auxAngle = 270.0 * (M_PI / 180.0);
					directionAngle += auxAngle;
				}
				else
				{
					auxAngle = -90.0 * (M_PI / 180.0);
					directionAngle += auxAngle;
				}
				// PRIMEIRO QUADRANTE
				if ((directionAngle >= 0) && (directionAngle <= 90.0 * (M_PI / 180.0)))
				{
					sX = 0;
					sY = TAM_RET;
					dXdouble = LINE_LENGTH * cos(directionAngle);
					dYdouble = TAM_RET - LINE_LENGTH * sin(directionAngle);
					dX = (int)dXdouble;
					dY = (int)dYdouble;
					descX = (int)(TAM_RET - dX) / 2;
					descY = (dY) / 2;
					sX += descX + i * TAM_RET + TAM_RET + DESLOC_IMG;
					sY += -descY + j * TAM_RET + TAM_RET;
					dX += descX + i * TAM_RET + TAM_RET + DESLOC_IMG;
					dY += -descY + j * TAM_RET + TAM_RET;
				}
				else
				{ // SEGUNDO QUADRANTE
					if ((directionAngle > 90.0 * (M_PI / 180.0)) && (directionAngle <= M_PI))
					{
						sX = TAM_RET;
						sY = TAM_RET;
						dXdouble = TAM_RET - (-1.0) * LINE_LENGTH * cos(directionAngle);
						dYdouble = TAM_RET - LINE_LENGTH * sin(directionAngle);
						dX = (int)dXdouble;
						dY = (int)dYdouble;
						descX = (dX) / 2;
						descY = (dY) / 2;
						sX += -descX + i * TAM_RET + TAM_RET + DESLOC_IMG;
						sY += -descY + j * TAM_RET + TAM_RET;
						dX += -descX + i * TAM_RET + TAM_RET + DESLOC_IMG;
						dY += -descY + j * TAM_RET + TAM_RET;
					}
					else
					{ //QUARTO QUADRANTE
						if ((directionAngle < 0) && (directionAngle >= -90.0 * (M_PI / 180.0)))
						{
							sX = 0;
							sY = 0;
							dXdouble = LINE_LENGTH * cos(directionAngle);
							dYdouble = (-1.0) * LINE_LENGTH * sin(directionAngle);
							dX = (int)dXdouble;
							dY = (int)dYdouble;
							descX = (int)(TAM_RET - dX) / 2;
							descY = (int)(TAM_RET - dY) / 2;
							sX += descX + i * TAM_RET + TAM_RET + DESLOC_IMG;
							sY += descY + j * TAM_RET + TAM_RET;
							dX += descX + i * TAM_RET + TAM_RET + DESLOC_IMG;
							dY += descY + j * TAM_RET + TAM_RET;
						}
						else
						{ // TERCEIRO QUADRANTE
							if ((directionAngle < -90.0 * (M_PI / 180.0)) && (directionAngle >= -M_PI))
							{
								sX = TAM_RET;
								sY = 0;
								dXdouble = TAM_RET - (-1.0) * LINE_LENGTH * cos(directionAngle);
								dYdouble = (-1.0) * LINE_LENGTH * sin(directionAngle);
								dX = (int)dXdouble;
								dY = (int)dYdouble;
								descX = (dX) / 2;
								descY = (int)(TAM_RET - dY) / 2;
								sX += -descX + i * TAM_RET + TAM_RET + DESLOC_IMG;
								sY += descY + j * TAM_RET + TAM_RET;
								dX += -descX + i * TAM_RET + TAM_RET + DESLOC_IMG;
								dY += descY + j * TAM_RET + TAM_RET;
							}
						}
					}
				}
				rectangle(ret_image,
						  Point(i * TAM_RET + TAM_RET + DESLOC_IMG, j * TAM_RET + TAM_RET),
						  Point(i * TAM_RET + 2 * TAM_RET + DESLOC_IMG, j * TAM_RET + 2 * TAM_RET),
						  //									Scalar( (int)multColor, (int)multColor, (int)multColor ),
						  Scalar(255, 255, 255),
						  -1,
						  8);
				line(ret_image,
					 Point(sX, sY),
					 Point(dX, dY),
					 Scalar(0, 0, 0),
					 1,
					 8,
					 0);
				circle(ret_image,
					   Point(dX, dY),
					   2,
					   Scalar(0, 0, 0),
					   -1,
					   8,
					   0);
			}
		}
	}
	//			q = caminhoRobo[indJogador];
	////			auxSoma = -(indJogador+2)*TAM_RET;
	////			if (indJogador == 1)
	////				auxSoma += TAM_RET;
	//			auxSoma = 0;
	//			while (q != NULL){
	//			if ((q->nextPosition) && (q->posX !=0 && q->posY != 0 ))
	//				line(ret_image,
	//					Point(q->posX*(WND_X-2*TAM_RET)/170+DESLOC_IMG+TAM_RET+auxSoma,q->posY*(WND_Y-2*TAM_RET)/130+TAM_RET),
	//					Point(q->nextPosition->posX*(WND_X-2*TAM_RET)/170+DESLOC_IMG+TAM_RET+auxSoma,q->nextPosition->posY*(WND_Y-2*TAM_RET)/130+TAM_RET),
	//					Scalar( 0, 0, 255 ),
	//					2,
	//					8,
	//					0);
	//				q = q->nextPosition;
	//			}
}

//----------------------------------------------------------------------------------------------

void calculaVelMotores(int NumRobo, int *me, int *md)
{
	float dx, dy, ang, v_cm, v_unid, ve, vd, aux, difVelRodas;
	//Calcula abaixo a velocidade dos motores do robo
	ang = estadoPrev[NumRobo].angulo;
	dx = estadoPrev[NumRobo].dx;
	dy = estadoPrev[NumRobo].dy;

	v_cm = sqrt(dx * dx + dy * dy); //Calcula a velocidade do Robo em cm
	//Calcula as velocidade dos motores em unidades
	v_unid = v_cm * FATOR_VEL_ROBO_UNID_POR_CM;
	if (v_unid <= VEL_MAXIMA_ROBO_UNID)
	{
		if (abs(difAngMenos180a180(atang2(dy, dx), ang)) > 90)
		{
			//Robo andando de re';
			v_unid = -v_unid; //frente (etiquetas) para um lado, deslocamento para o outro
		}

		aux = difAngMenos180a180(ang, estadoAnt[NumRobo].angulo);
		// a cada FATOR_ANGULO_DIF_RODAS_UNID_VEL graus significa uma diferenca entre rodas de uma unidade de velocidade
		difVelRodas = abs(aux / FATOR_ANGULO_DIF_RODAS_UNID_VEL);

		if (difVelRodas > VEL_MAXIMA_ROBO_UNID * 2)
			difVelRodas = VEL_MAXIMA_ROBO_UNID * 2;
		if (aux > 0)
		{ //Robo virando para dir.
			ve = v_unid - difVelRodas / 2;
			vd = v_unid + difVelRodas / 2;
			if (ve < -VEL_MAXIMA_ROBO_UNID)
			{
				ve = -VEL_MAXIMA_ROBO_UNID;
				vd = ve + difVelRodas;
			}
			else if (vd > VEL_MAXIMA_ROBO_UNID)
			{
				vd = VEL_MAXIMA_ROBO_UNID;
				ve = vd - difVelRodas;
			}
		}
		else
		{ //Robo virando para esq
			ve = v_unid + difVelRodas / 2;
			vd = v_unid - difVelRodas / 2;
			if (ve > VEL_MAXIMA_ROBO_UNID)
			{
				ve = VEL_MAXIMA_ROBO_UNID;
				vd = ve - difVelRodas;
			}
			else if (vd < -VEL_MAXIMA_ROBO_UNID)
			{
				vd = -VEL_MAXIMA_ROBO_UNID;
				ve = vd + difVelRodas;
			}
		}
	}
	else
	{ //robo muito rapido, possivelmente erro
		ve = (cmdEnviado[0][NumRobo].esq + cmdEnviado[1][NumRobo].esq + cmdEnviado[2][NumRobo].esq + cmdEnviado[3][NumRobo].esq) / 4;
		vd = (cmdEnviado[0][NumRobo].dir + cmdEnviado[1][NumRobo].dir + cmdEnviado[2][NumRobo].dir + cmdEnviado[3][NumRobo].dir) / 4;
	}
	*me = ve;
	*md = vd;
}
campoPot campoPotencial[3];

void calculaCmd(int indJogador, int angObjetivo, int xObjetivo, int yObjetivo, int velObjetivo)
{

	float angRobo, xRobo, yRobo;
	float ang, auxAng;
	int ve, vd, pe, pd, i, j;
	static int cmdAntEsq = 0, cmdAntDir = 0;
	float wObj, vObj, vMaxima;
	int DA;
	float dx, dy, v_CPO[2];
	float d;
	//	position_list *caminhoRoboAux, *q;
	double directionAngle;
	int flagDirection;
	//	caminhoRoboAux = (position_list*) malloc (sizeof(position_list));

	double lim = 180 / 8;

	v_CPO[0] = cos((double)(angObjetivo) / 180 * M_PI);
	v_CPO[1] = sin((double)(angObjetivo) / 180 * M_PI);

	if (xObjetivo > 0 && yObjetivo > 0)
	{
		inicializa_obst_meta(&campoPotencial[indJogador], xObjetivo / DIV_CAMPO, yObjetivo / DIV_CAMPO, indJogador);
		//		if (xObjAnt[indJogador] != xObjetivo && yObjAnt[indJogador] != yObjetivo)
		//			calcula_campo_SOR_CPLO(&campoPotencial[indJogador], v_CPO, xObjetivo, yObjetivo);
		//			calcula_campo_CP(&campoPotencial[indJogador], (float)xObjetivo, (float)yObjetivo, (float)estado[indJogador].x, (float)estado[indJogador].y);
		calcula_campo_SOR(&campoPotencial[indJogador]);
		//			desenha_campo(campoPotencial[0],ret_image, 0, estado[0].x/DIV_CAMPO, estado[0].y/DIV_CAMPO, 0);
		//			desenha_campo(campoPotencial[1],ret_image, WND_X/3, estado[1].x/DIV_CAMPO, estado[1].y/DIV_CAMPO, 1);
		//			desenha_campo(campoPotencial[2],ret_image, 2*WND_X/3, estado[2].x/DIV_CAMPO, estado[2].y/DIV_CAMPO, 2);
		//			imshow("rectangle", ret_image);
		//			waitKey(0);
	}

	xObjAnt[indJogador] = xObjetivo;
	yObjAnt[indJogador] = yObjetivo;
	angRobo = estado[indJogador].angulo;
	xRobo = estado[indJogador].x;
	//	caminhoRoboAux->posX = xRobo;
	yRobo = estado[indJogador].y;
	//	caminhoRoboAux->posY = yRobo;
	//	caminhoRoboAux->nextPosition = NULL;
	dx = xObjetivo - xRobo;
	dy = yObjetivo - yRobo;
	d = sqrt(dx * dx + dy * dy);

	i = xRobo / DIV_CAMPO;
	j = yRobo / DIV_CAMPO;

	directionAngle = atan2((double)(campoPotencial[indJogador].matPot[i][j - 1] - campoPotencial[indJogador].matPot[i][j + 1]), (double)(campoPotencial[indJogador].matPot[i - 1][j] - campoPotencial[indJogador].matPot[i + 1][j])) * 180 / M_PI;

	//	printf("[%d](%f)\n", indJogador, (float)directionAngle);
	//    printf("[%d](%f)\n[        ][%f][        ]\n[%f][        ][%f]\n[        ][%f][        ]\n", indJogador, (float)directionAngle, campoPotencial[indJogador].matPot[i][j+1],campoPotencial[indJogador].matPot[i-1][j],campoPotencial[indJogador].matPot[i+1][j],campoPotencial[indJogador].matPot[i][j-1]);

	ang = (float)directionAngle;

	if (angRobo > 180)
	{
		angRobo -= 360;
	}

	// ver cálculo dos ângulos para que o robô não precise dar uma volta inteira. utilizando flag de dire
	// ção, primeiro e quarto quadrante flag = 1, e segundo e terceiro quadrante flag = -1.

	double dAng = ang - angRobo;
	//	if (dAng > 180){
	//			dAng -= 360;
	//		}
	//	auxAng = dAng;
	//		if ((dAng <= 90 && dAng > 0) || (dAng <= 0 && dAng >= -90)){
	//				flagDirection = 1;
	//		} else
	//			if ((dAng <= 180 && dAng > 90) || (dAng < -90 && dAng >= -180)){
	//				flagDirection = -1;
	//				if (dAng <= 180 && dAng > 90){
	//					dAng = 180 - dAng;
	//				}else{
	//						dAng = -180 - dAng;
	//					}
	//			}
	////	distanciaObj = sqrt(pow(xObjetivo - estado[indJogador].x,2)+pow(yObjetivo - estado[indJogador].x,2));
	//	if ((estado[indJogador].x > xObjetivo - RAIO_DISTANCIA && estado[indJogador].x < xObjetivo + RAIO_DISTANCIA) && (estado[indJogador].y > yObjetivo - RAIO_DISTANCIA && estado[indJogador].y < yObjetivo + RAIO_DISTANCIA)){
	//		vMaxima = (d/RAIO_DISTANCIA)*VELOCIDADE_MAX + (1 - d/RAIO_DISTANCIA)*velObjetivo;
	//	}
	//	vMaxima = VELOCIDADE_MAX;
	//
	//	wObj = (dAng/180)*VELOCIDADE_ANGULAR_MAXIMA;
	//	if (dAng > 0)
	//		vObj = (1 - dAng/180)*vMaxima;
	//		else
	//			vObj = (1 + dAng/180)*vMaxima;
	//
	//	pe = (int)((2*vObj - wObj*DIST_ENTRE_RODAS)/(2*RAIO_DA_RODA))/30;
	//	pe*= flagDirection;
	//	if (pe>7){
	//		pe = 7;
	//	}else if (pe < -7)
	//		pe = -7;
	//	if (pe<0)
	//		pe = -pe + 8;
	//	pd = (int)((2*vObj + wObj*DIST_ENTRE_RODAS)/(2*RAIO_DA_RODA))/30;
	//	pd*= flagDirection;
	//	if (pd>7){
	//		pd = 7;
	//	}else if (pd < -7)
	//		pd = -7;
	//	if (pd<0)
	//		pd = -pd + 8;

	if (dAng < 0)
	{
		dAng += 2 * 180;
	}
	if (dAng < lim)
	{
		pe = 7;
		pd = 7;
	}
	else if (dAng < 180 / 2 - lim)
	{
		pe = 0;
		pd = 7;
	}
	else if (dAng < 180 / 2)
	{
		pe = -7;
		pd = 7;
	}
	else if (dAng < 180 / 2 + lim)
	{
		pe = 7;
		pd = -7;
	}
	else if (dAng < 180 - lim)
	{
		pe = 0;
		pd = -7;
	}
	else if (dAng < 180 + lim)
	{
		pe = -7;
		pd = -7;
	}
	else if (dAng < 3 * 180 / 2 - lim)
	{
		pe = -7;
		pd = 0;
	}
	else if (dAng < 3 * 180 / 2)
	{
		pe = -7;
		pd = 7;
	}
	else if (dAng < 3 * 180 / 2 + lim)
	{
		pe = 7;
		pd = -7;
	}
	else if (dAng < 2 * 180 - lim)
	{
		pe = 7;
		pd = 0;
	}
	else
	{
		pe = 7;
		pd = 7;
	}
	//	q = caminhoRobo[indJogador];
	//
	//	if (q != NULL){
	//		while (q->nextPosition != NULL){
	//			q = q->nextPosition;
	//		}
	//		q->nextPosition = caminhoRoboAux;
	//	}else
	//		caminhoRobo[indJogador] = caminhoRoboAux;

	//	if (cmdEnviado[0][indJogador].esq < pe){
	//		cmdAntEsq++;
	//		cmdEnviado[0][indJogador].esq = cmdAntEsq;
	//	}else if (cmdEnviado[0][indJogador].esq > pe){
	//			cmdAntEsq--;
	//			cmdEnviado[0][indJogador].esq = cmdAntEsq;
	//		}else
	cmdEnviado[0][indJogador].esq = pe * 1;

	//	if (cmdEnviado[0][indJogador].dir < pe){
	//		cmdAntDir++;
	//		cmdEnviado[0][indJogador].dir = cmdAntDir;
	//	}else if (cmdEnviado[0][indJogador].dir > pe){
	//			cmdAntDir--;
	//			cmdEnviado[0][indJogador].dir = cmdAntDir;
	//			}else
	cmdEnviado[0][indJogador].dir = pd * 1;
}
