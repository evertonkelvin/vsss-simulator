/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
// #include "cv.h"
// #include "highgui.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <stdlib.h>
#include <stdio.h>
#endif

typedef unsigned char guint8;

#include "TiposClasses.h"
#include "Auxiliares.h"

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Twist.h"
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <math.h>

//#define RANDOMICO
#define DARKGRAY 192,192,192
#define LIGHTGRAY 224,224,224
#define GRAY 160,160,160
#define WHITE 255,255,255
#define LIGHTMAGENTA 255,128,255
#define GREEN 0,255,0
#define LIGHTRED 255,128,128
#define LIGHTBLUE 128,128,255
#define YELLOW 255,255,0

#define TAM_HISTORIA 50 // tem que ser o maior entre TEMPO_ATRASO e TAM_INERCIA
#define TEMPO_ATRASO 5
#define TAM_INERCIA 3

#define PI 3.14159265359

#define TAM_GOL_X 0.1
#define TAM_GOL_Y 0.4

#define TAM_CAMPO_X_SEM_GOL 1.5
#define TAM_CAMPO_X 1.7
#define TAM_CAMPO_Y 1.3

#define METADE_CAMPO_X_SEM_GOL 0.75
#define METADE_CAMPO_X 0.85
#define METADE_CAMPO_Y 0.65

#define FREEBALL_QUADRANTE_1_X TAM_CAMPO_X_SEM_GOL - (METADE_CAMPO_X_SEM_GOL / 2)
#define FREEBALL_QUADRANTE_1_Y TAM_CAMPO_Y - 0.25
#define FREEBALL_QUADRANTE_2_X METADE_CAMPO_X_SEM_GOL / 2
#define FREEBALL_QUADRANTE_2_Y TAM_CAMPO_Y - 0.25
#define FREEBALL_QUADRANTE_3_X METADE_CAMPO_X_SEM_GOL / 2
#define FREEBALL_QUADRANTE_3_Y 0.25
#define FREEBALL_QUADRANTE_4_X TAM_CAMPO_X_SEM_GOL - (METADE_CAMPO_X_SEM_GOL / 2)
#define FREEBALL_QUADRANTE_4_Y 0.25

#define TEMPO_PARA_FREEBALL 300

struct placar{
	int uni;
	int dez;
	int cen;
	int mil;
} placarTime1, placarTime2;

struct pos{
 float x, y;
} posBolaAnt;

struct var_time{
	int numPenaltis = 0;
	int golsContra = 0;
	int golsDePenaltis = 0;
	int golsRobo[3] = {0, 0, 0};
	int golsContraRobo[3] = {0, 0, 0};
} time1, time2;

bool emJogo = true;

int indGoleiro = 0;
int indVolante = 1;
int indAtacante = 2;
int indGoleiroGuest = 0;
int indVolanteGuest = 1;
int indAtacanteGuest = 2;
bool emPenalti = false;
bool tiroMeta = false;

bool emPenalidade = false;
bool emPosiciona = false;
bool emInicio = false;

bool bola_parada = false;
int count_timer_bola = 0;

FILE *log_Jogo;
bool emPenal = false;
bool possivelPenalTime1 = false;
bool possivelPenalTime2 = false;
bool imprimeEstatistica = false;
bool posiciona = true;
int contPenal = 0;
int numFreeBalls = 0;
int countTotalTempo = 0;
int auxCont = 0;

extern bool voltando;

Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1], estadoPrev[NUM_ROBOS_TIME * 2 + 1];
Estado estadoReal[NUM_ROBOS_TIME * 2 + 1], estadoRealAnt[NUM_ROBOS_TIME * 2 + 1];

Objetivo objetivoRobo[NUM_ROBOS_TIME];

ros::Publisher pubModelPose;

#define DESLINICIOCAMPO 520

CvScalar corNoCV;
CvFont font;
IplImage* image;

void setColor(int r, int g, int b) {
	corNoCV.val[0] = b;
	corNoCV.val[1] = g;
	corNoCV.val[2] = r;
	corNoCV.val[3] = 0;
}

void Inicia(void) {
	cvInitFont(&font, CV_FONT_VECTOR0, 1.0, 1.0);
	setColor(0, 0, 0);
}

void setColor(int i) {
	setColor(255 * (i - 20) / 20, 128, 255 * (20) / 7);
}

#define ESCALA 4

void Circle(float x, float y, float radius) {
	CvPoint pt1 = { x * ESCALA, DESLINICIOCAMPO - y * ESCALA };
	cvCircle(image, pt1, radius * ESCALA, corNoCV);
}

float ultX = 0, ultY = 0;
void MoveTo(float x, float y) {
	ultX = x * ESCALA;
	ultY = DESLINICIOCAMPO - y * ESCALA;
}

void LineTo(float x, float y) {
	CvPoint pt1 = { ultX, ultY };
	ultX = x * ESCALA;
	ultY = DESLINICIOCAMPO - y * ESCALA;
	CvPoint pt2 = { ultX, ultY };
	cvLine(image, pt1, pt2, corNoCV);
}

void Rectangle(float x1, float y1, float x2, float y2) {
	CvPoint pt1 = { x1 * ESCALA, y1 * ESCALA };
	CvPoint pt2 = { x2 * ESCALA, y2 * ESCALA };
	CvScalar cor = corNoCV;
	cor.val[0] = 1 - cor.val[0];
	cor.val[1] = 1 - cor.val[1];
	cor.val[2] = 1 - cor.val[2];
	cvRectangle(image, pt1, pt2, cor, -1);
}

void OutTextXY(float x, float y, char *textstring) {
	CvPoint pt1 = { x * ESCALA, DESLINICIOCAMPO - y * ESCALA };
	cvPutText(image, textstring, pt1, &font, corNoCV);
}

char wndname[] = "Juiz Simulador Very Small Size Soccer";

float xVisao[7][3], yVisao[7][3], angVisao[7][3];

void Visao(void) {
	int x, y, ang;
	int i;
	
	for (i = 0; i < 7; i++) {
		estadoAnt[i] = estado[i];

		if (emJogo) {
			x = estadoReal[i].x; //coordenada obtida da camera //quadrado cinza na tela
			y = estadoReal[i].y;
			ang = estadoReal[i].angulo;

			xVisao[i][2] = xVisao[i][1]; // atraso x, sai sempre o 2 para ser usado
			xVisao[i][1] = xVisao[i][0];
			xVisao[i][0] = x;

			yVisao[i][2] = yVisao[i][1]; // atraso y
			yVisao[i][1] = yVisao[i][0];
			yVisao[i][0] = y;

			angVisao[i][2] = angVisao[i][1]; // atraso ang
			angVisao[i][1] = angVisao[i][0];
			angVisao[i][0] = ang;

			estado[i].angulo = angVisao[i][2]; // estado apresentado na tela como quadrado colorido

			estado[i].dx = xVisao[i][2] - estado[i].x;
			estado[i].dy = yVisao[i][2] - estado[i].y;

			estado[i].x = xVisao[i][2];
			estado[i].y = yVisao[i][2];
		}
	}
}

void DesenhaPlacar(void) {
	char placarTimes[23];
	placarTimes[0] = 'T';
	placarTimes[1] = 'i';
	placarTimes[2] = 'm';
	placarTimes[3] = 'e';
	placarTimes[4] = ' ';
	placarTimes[5] = '1';
	placarTimes[6] = ' ';
	placarTimes[7] = '0' + placarTime1.mil;
	placarTimes[8] = '0' + placarTime1.cen;
	placarTimes[9] = '0' + placarTime1.dez;
	placarTimes[10] = '0' + placarTime1.uni;
	placarTimes[11] = 'x';
	placarTimes[12] = '0' + placarTime2.mil;
	placarTimes[13] = '0' + placarTime2.cen;
	placarTimes[14] = '0' + placarTime2.dez;
	placarTimes[15] = '0' + placarTime2.uni;
	placarTimes[16] = ' ';
	placarTimes[17] = 'T';
	placarTimes[18] = 'i';
	placarTimes[19] = 'm';
	placarTimes[20] = 'e';
	placarTimes[21] = ' ';
	placarTimes[22] = '2';
	placarTimes[23] = 0;

	OutTextXY(TAM_X_CAMPO/2 - 50, -10, placarTimes);
}

void DesenhaCampo(void) {
	Rectangle(0, 0, 180, 140);
	MoveTo(10 + 7, 0);
	LineTo(160 - 7, 0);
	LineTo(160, 0 + 7);
	LineTo(160, 45);
	LineTo(170, 45);
	LineTo(170, 85);
	LineTo(160, 85);
	LineTo(160, 130 - 7);
	LineTo(160 - 7, 130);
	LineTo(10 + 7, 130);
	LineTo(10, 130 - 7);
	LineTo(10, 85);
	LineTo(0, 85);
	LineTo(0, 45);
	LineTo(10, 45);
	LineTo(10, 0 + 7);
	LineTo(10 + 7, 0);
}

void DesenhaJogador(int i) {
	float x, y, ang;
	float d1, d2, x1, y1;
	char num[2];

	x = estadoReal[i].x;
	y = estadoReal[i].y;
	ang = estadoReal[i].angulo;

	d1 = coss(ang) * 8;
	d2 = seno(ang) * 8;

	if (i >= 0 && i <= 2) {
		setColor(LIGHTBLUE);
	} else if (i >= 4 && i <= 6) {
		setColor(YELLOW);
	}

	setColor(WHITE);
	x1 = x - (d1 + d2) / 2;
	y1 = y + (d1 - d2) / 2;

	MoveTo(x1, y1);
	LineTo(x1 + d1, y1 + d2);
	LineTo(x1 + d1 + d2, y1 - d1 + d2);
	LineTo(x1 + d2, y1 - d1);
	LineTo(x1, y1);

	setColor(13 + i);

	MoveTo(x1 + d1, y1 + d2);
	LineTo(x, y);
	LineTo(x1 + d1 + d2, y1 - d1 + d2);

	setColor(WHITE);
}

void DesenhaJogadorVisao(int i) {
	float x, y, ang;
	float d1, d2, x1, y1;

	setColor(LIGHTGRAY);

	x = estado[i].x;
	y = estado[i].y;
	ang = estado[i].angulo;

	d1 = coss(ang) * 8;
	d2 = seno(ang) * 8;

	x1 = x - (d1 + d2) / 2;
	y1 = y + (d1 - d2) / 2;

	MoveTo(x1, y1);
	LineTo(x1 + d1, y1 + d2);
	LineTo(x1 + d1 + d2, y1 - d1 + d2);
	LineTo(x1 + d2, y1 - d1);
	LineTo(x1, y1);

	MoveTo(x1 + d1, y1 + d2);
	LineTo(x, y);
	LineTo(x1 + d1 + d2, y1 - d1 + d2);

	setColor(WHITE);
}

void DesenhaBola(void) {
	setColor(LIGHTRED);
	Circle(estadoReal[3].x, estadoReal[3].y, 3);
	setColor(WHITE);
}

void DesenhaJogo(void) {
	char num[10];
	int i;

	DesenhaCampo();

	for (i = 0; i < 7; i++) {

		if (i != 3) {
			DesenhaJogadorVisao(i);
			DesenhaJogador(i);
			num[0] = '0' + i;
			if (i == indAtacante) {
				num[1] = 'A';
			} else if (i == indGoleiro) {
				num[1] = 'G';
			} else if (i == indVolante) {
				num[1] = 'V';
			}
			num[2] = 0;
			OutTextXY(estadoReal[i].x - 2, estadoReal[i].y + 2, num);
		} else {
			DesenhaBola();
		}
		DesenhaPlacar();
	}
}

int contabiliza_gol(FILE *log_Jogo, Estado estado[7], int golTime){
//	log_Jogo = fopen("output.txt","a+");
	float auxMaisPertoDaBola, maisPertoDaBola;
	int indMaisPertoDaBola;
	maisPertoDaBola = sqrt((estado[3].x - estado[0].x)*(estado[3].x - estado[0].x) + (estado[3].y - estado[0].y)*(estado[3].y - estado[0].y));
	indMaisPertoDaBola = 0;
//	fprintf(log_Jogo,"golTime : %d\n", golTime);
	if (golTime == 1){
		for (int i = 6; i > 0; i--){
			if (i != 3 && i != 0 && i != 4){
				auxMaisPertoDaBola = sqrt((estado[3].x-estado[i].x)*(estado[3].x-estado[i].x) + (estado[3].y-estado[i].y)*(estado[3].y-estado[i].y));
				if (auxMaisPertoDaBola <= maisPertoDaBola){
					maisPertoDaBola = auxMaisPertoDaBola;
					indMaisPertoDaBola = i;
				}
			}
		}
//	fprintf(log_Jogo,"maisPertoDaBola : %f\n", maisPertoDaBola);//X
//	fprintf(log_Jogo,"indMaisPertoDaBola : %d\n", indMaisPertoDaBola);//X

		if (indMaisPertoDaBola > 3){
			time2.golsContra++;
			time2.golsContraRobo[indMaisPertoDaBola-4]++;
		} else
			time1.golsRobo[indMaisPertoDaBola]++;

	} else {
		for (int i = 1; i < 7; i++){
			if (i != 3 && i != 0 && i != 4){
				auxMaisPertoDaBola = sqrt((estado[3].x-estado[i].x)*(estado[3].x-estado[i].x) + (estado[3].y-estado[i].y)*(estado[3].y-estado[i].y));
				if (auxMaisPertoDaBola <= maisPertoDaBola){
					maisPertoDaBola = auxMaisPertoDaBola;
					indMaisPertoDaBola = i;
				}
			}
		}
//		fprintf(log_Jogo,"maisPertoDaBola : %f\n", maisPertoDaBola);
//		fprintf(log_Jogo,"indMaisPertoDaBola : %d\n", indMaisPertoDaBola);//X

		if (indMaisPertoDaBola < 3){
			time1.golsContra++;
			time1.golsContraRobo[indMaisPertoDaBola]++;
		} else
			time2.golsRobo[indMaisPertoDaBola - 4]++;
	}
//	fclose(log_Jogo);
	return 1;
}

void AlteraPosicao(string name, double px, double py, double oz, double ow) {
	gazebo_msgs::ModelState msg;

	msg.model_name = name;
	msg.pose.position.x = px - METADE_CAMPO_X;
	msg.pose.position.y = py - METADE_CAMPO_Y;
	msg.pose.position.z = 0;
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = oz;
	msg.pose.orientation.w = ow;
	msg.twist.linear.x = 0;
	msg.twist.linear.y = 0;
	msg.twist.linear.z = 0;
	msg.twist.angular.x = 0;
	msg.twist.angular.y = 0;
	msg.twist.angular.z = 0;
	msg.reference_frame = "world";

	pubModelPose.publish(msg);
}

void DesenhaPosicoes()
{
	int i;
	char c;

	if (auxCont >= 10000){
		// imprimeEstatistica = true;
		auxCont = 0;
	}

	// if ((estadoReal[3].x >= estadoAnt[3].x - 2 && estadoReal[3].x <= estadoAnt[3].x + 2) 
	// 		&& (estadoReal[3].y >= estadoAnt[3].y - 2 && estadoReal[3].y <= estadoAnt[3].y + 2)
	// 		&& emJogo) {
	if (estadoReal[3].x == estadoRealAnt[3].x
			&& estadoReal[3].y == estadoRealAnt[3].y
			&& emJogo) {
		bola_parada = true;
	} else {
		bola_parada = false;
	}
	if (bola_parada){
		count_timer_bola++;
		printf("\n\n **********************###################################### %d ######################################******************** \n\n", count_timer_bola);
	} else {
		printf("\n\n **********************###################################### COUNT BOLA ZERADO ######################################******************** \n\n");
		count_timer_bola = 0;

	}
	if (emPenal){
		contPenal++;
		if (contPenal >= 1000){
			emPenal = false;
			contPenal = 0;
		}
	}

	Visao();

	printf("\n");
	printf("%3s, %3s, %3s, %3s, %3s | %3s, %3s, %3s, %3s, %3s | %3s, %3s, %3s, %3s, %3s | %3s, %3s, %3s, %3s, %3s | %3s, %3s, %3s, %3s, %3s\n",
		   "RAx", "RAy", "Rdx", "Rdy", "RAa", "eRx", "eRy", "Rdx", "Rdy", "eRa", "eAx", "eAy", "Adx", "Ady", "eAa", "e_x", "e_y", "edx", "edy", "e_a", "ePx", "ePy", "Pdx", "Pdy", "ePa");
	for (i = 0; i < 7; i++) {
		printf("%3.0f, %3.0f, %3.0f, %3.0f, %3.0f | %3.0f, %3.0f, %3.0f, %3.0f, %3.0f | %3.0f, %3.0f, %3.0f, %3.0f, %3.0f | %3.0f, %3.0f, %3.0f, %3.0f, %3.0f | %3.0f, %3.0f, %3.0f, %3.0f, %3.0f\n",
			   estadoRealAnt[i].x, estadoRealAnt[i].y, estadoRealAnt[i].dx, estadoRealAnt[i].dy, estadoRealAnt[i].angulo,
			   estadoReal[i].x, estadoReal[i].y, estadoReal[i].dx, estadoReal[i].dy, estadoReal[i].angulo,
			   estadoAnt[i].x, estadoAnt[i].y, estadoAnt[i].dx, estadoAnt[i].dy, estadoAnt[i].angulo,
			   estado[i].x, estado[i].y, estado[i].dx, estado[i].dy, estado[i].angulo,
			   estadoPrev[i].x, estadoPrev[i].y, estadoPrev[i].dx, estadoPrev[i].dy, estadoPrev[i].angulo);
	}

	posBolaAnt.x = estadoReal[3].x;
	posBolaAnt.y = estadoReal[3].y;
	
	DesenhaJogo();
	cvShowImage(wndname, image);

	indGoleiro = 0;
	indAtacante = 2;
	indVolante = 1;

	fflush(stdout);
	c = cvWaitKey(30); // Se comentar essa linha, a janela do simulador antigo não aparece
	printf("(%d) \n", c);

	if (imprimeEstatistica && c != 'p') {
		c = 'l';
	}

	if (count_timer_bola >= TEMPO_PARA_FREEBALL) {
		c = 'f';
		bola_parada = false;
	}

	if ( // PENALTI A FAVOR DO TIME 2
		(((estadoReal[3].x > 0 && estadoReal[3].x < 25) &&
		  (estadoReal[3].y > 30 && estadoReal[3].y < 100)) &&
		 ((estadoReal[0].x > 0 && estadoReal[0].x < 25) &&
		  (estadoReal[0].y > 30 && estadoReal[0].y < 100)) &&
		 ((estadoReal[1].x > 0 && estadoReal[1].x < 25) &&
		  (estadoReal[1].y > 30 && estadoReal[1].y < 100))) ||
		(((estadoReal[3].x > 0 && estadoReal[3].x < 25) &&
		  (estadoReal[3].y > 30 && estadoReal[3].y < 100)) &&
		 ((estadoReal[0].x > 0 && estadoReal[0].x < 25) &&
		  (estadoReal[0].y > 30 && estadoReal[0].y < 100)) &&
		 ((estadoReal[2].x > 0 && estadoReal[2].x < 25) &&
		  (estadoReal[2].y > 30 && estadoReal[2].y < 100))))
	{
		//			c = 'k';
		possivelPenalTime1 = true;
	}

	if ( // PENALTI A FAVOR DO TIME 1
		(((estadoReal[3].x > TAM_X_CAMPO - 35 && estadoReal[3].x < TAM_X_CAMPO) &&
		  (estadoReal[3].y > 30 && estadoReal[3].y < 100)) &&
		 ((estadoReal[4].x > TAM_X_CAMPO - 35 && estadoReal[4].x < TAM_X_CAMPO) &&
		  (estadoReal[4].y > 30 && estadoReal[4].y < 100)) &&
		 ((estadoReal[5].x > TAM_X_CAMPO - 35 && estadoReal[5].x < TAM_X_CAMPO) &&
		  (estadoReal[5].y > 30 && estadoReal[5].y < 100))) ||
		(((estadoReal[3].x > TAM_X_CAMPO - 35 && estadoReal[3].x < TAM_X_CAMPO) &&
		  (estadoReal[3].y > 30 && estadoReal[3].y < 100)) &&
		 ((estadoReal[4].x > TAM_X_CAMPO - 35 && estadoReal[4].x < TAM_X_CAMPO) &&
		  (estadoReal[4].y > 30 && estadoReal[4].y < 100)) &&
		 ((estadoReal[6].x > TAM_X_CAMPO - 35 && estadoReal[6].x < TAM_X_CAMPO) &&
		  (estadoReal[6].y > 30 && estadoReal[6].y < 100))))
	{
		//			c = 'K';
		possivelPenalTime2 = true;
	}

	if ( //Gol a favor do time 2
		((estadoReal[3].x >= 0 && estadoReal[3].x <= 9) &&
		 (estadoReal[3].y >= 45 && estadoReal[3].y <= 85)))
	{
		if (possivelPenalTime1)
			possivelPenalTime1 = false;
		if (placarTime2.uni == 9)
		{
			placarTime2.uni = 0;
			if (placarTime2.dez == 9)
			{
				placarTime2.dez = 0;
				if (placarTime2.cen == 9)
				{
					placarTime2.cen = 0;
					placarTime2.mil++;
				}
				else
					placarTime2.cen++;
			}
			else
				placarTime2.dez++;
		}
		else
			placarTime2.uni++;
		c = 'p';
		// log_Jogo = fopen("output.txt", "a+");
		// fprintf(log_Jogo, "\n-----------------------------------------------------------\n");
		// fprintf(log_Jogo, "GOOOOL -  TIME 1 %d%d%d%d X %d%d%d%d TIME 2\n", placarTime1.mil, placarTime1.cen, placarTime1.dez, placarTime1.uni,
		// 		placarTime2.mil, placarTime2.cen, placarTime2.dez, placarTime2.uni);
		if (emPenal)
		{
			// fprintf(log_Jogo, "DE PÊNALTI!\n");
			time2.golsDePenaltis++;
		}
		// fprintf(log_Jogo, "TEMPO DE JOGO: %d\n", countTotalTempo);
		// fclose(log_Jogo);
		// contabiliza_gol(log_Jogo, estadoReal, 2);
	}

	if ( //Gol a favor do time 1
		((estadoReal[3].x >= TAM_X_CAMPO - 9 && estadoReal[3].x <= TAM_X_CAMPO) &&
		 (estadoReal[3].y >= 45 && estadoReal[3].y <= 85)))
	{
		if (possivelPenalTime2)
			possivelPenalTime2 = false;
		if (placarTime1.uni == 9)
		{
			placarTime1.uni = 0;
			if (placarTime1.dez == 9)
			{
				placarTime1.dez = 0;
				if (placarTime1.cen == 9)
				{
					placarTime1.cen = 0;
					placarTime1.mil++;
				}
				else
					placarTime1.cen++;
			}
			else
				placarTime1.dez++;
		}
		else
			placarTime1.uni++;
		c = 'P';
		// log_Jogo = fopen("output.txt", "a+");
		// fprintf(log_Jogo, "\n-----------------------------------------------------------\n");
		// fprintf(log_Jogo, "GOOOOL -  TIME 1 %d%d%d%d X %d%d%d%d TIME 2\n", placarTime1.mil, placarTime1.cen, placarTime1.dez, placarTime1.uni,
				// placarTime2.mil, placarTime2.cen, placarTime2.dez, placarTime2.uni);
		if (emPenal)
		{
			// fprintf(log_Jogo, "DE PÊNALTI!\n");
			time1.golsDePenaltis++;
		}
		// fprintf(log_Jogo, "TEMPO DE JOGO: %d\n", countTotalTempo);
		// fclose(log_Jogo);
		// contabiliza_gol(log_Jogo, estadoReal, 1);
	}

	if (possivelPenalTime1)
	{
		if ( // PENALTI A FAVOR DO TIME 2
			((estadoReal[3].x > 0 && estadoReal[3].x < 25) &&
			  (estadoReal[3].y > 30 && estadoReal[3].y < 100)))
		{
			c = 'k';
			possivelPenalTime1 = false;
			emPenal = true;
			time1.numPenaltis++;
		}
	}
	if (possivelPenalTime2)
	{
		if ( // PENALTI A FAVOR DO TIME 1
			((estadoReal[3].x > TAM_X_CAMPO - 35 && estadoReal[3].x < TAM_X_CAMPO) &&
			  (estadoReal[3].y > 30 && estadoReal[3].y < 100)))
		{
			c = 'K';
			possivelPenalTime2 = false;
			emPenal = true;
			time2.numPenaltis++;
		}
	}
	if (posiciona)
	{
		posiciona = false;
		c = 'p';
	}

	switch (c) {
		case 'p': // posiciona jogadores
		case 'P':
			emJogo = false;

			AlteraPosicao("home1", METADE_CAMPO_X - 0.7, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("home2", METADE_CAMPO_X - 0.4, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("home3", METADE_CAMPO_X - 0.2, METADE_CAMPO_Y + 0.12, -0.3, 0.96);
			AlteraPosicao("ball", METADE_CAMPO_X, METADE_CAMPO_Y, 0, 0);
			AlteraPosicao("guest1", METADE_CAMPO_X + 0.7, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("guest2", METADE_CAMPO_X + 0.4, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("guest3", METADE_CAMPO_X + 0.2,  METADE_CAMPO_Y - 0.12, -0.3, 0.96);

			emJogo = false;
			break;

		case 'k':
			emJogo = false;

			AlteraPosicao("home1", METADE_CAMPO_X - 0.7, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("home2", METADE_CAMPO_X + 0.1, METADE_CAMPO_Y + 0.2, 0, 0);
			AlteraPosicao("home3", METADE_CAMPO_X + 0.1, METADE_CAMPO_Y - 0.2, 0, 0);
			AlteraPosicao("ball", METADE_CAMPO_X_SEM_GOL / 2, METADE_CAMPO_Y, 0, 0);
			AlteraPosicao("guest1", METADE_CAMPO_X + 0.7, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("guest2", METADE_CAMPO_X + 0.4, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("guest3", METADE_CAMPO_X_SEM_GOL / 2 + 0.08, METADE_CAMPO_Y, 0, 0);
			break;

		case 'K':
			emJogo = false;

			AlteraPosicao("home1", METADE_CAMPO_X - 0.7, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("home2", METADE_CAMPO_X - 0.4, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("home3", TAM_CAMPO_X_SEM_GOL - (METADE_CAMPO_X_SEM_GOL / 2) - 0.08, METADE_CAMPO_Y, 0, 0);
			AlteraPosicao("ball", TAM_CAMPO_X_SEM_GOL - (METADE_CAMPO_X_SEM_GOL / 2), METADE_CAMPO_Y, 0, 0);
			AlteraPosicao("guest1", METADE_CAMPO_X + 0.7, METADE_CAMPO_Y, -0.71, 0.7);
			AlteraPosicao("guest2", METADE_CAMPO_X - 0.1, METADE_CAMPO_Y + 0.2, -0.71, 0.7);
			AlteraPosicao("guest3", METADE_CAMPO_X - 0.1, METADE_CAMPO_Y - 0.2, 0, 0);
			break;

		case 'j':
		case 'J':
			emJogo = true;
			// indGoleiro = 0;
			// indVolante = 1;
			// indAtacante = 2;
			break;

		case 'f': //freeball
		case 'F':
			emJogo = false;

			if (estadoReal[3].x >= METADE_CAMPO_X * 100 && estadoReal[3].y >= METADE_CAMPO_Y * 100) { // QUADRANTE 1
				AlteraPosicao("home1", METADE_CAMPO_X - 0.7, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("home2", METADE_CAMPO_X - 0.4, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("home3", FREEBALL_QUADRANTE_1_X - 0.2, FREEBALL_QUADRANTE_1_Y, 0, 0);
				AlteraPosicao("ball", FREEBALL_QUADRANTE_1_X, FREEBALL_QUADRANTE_1_Y, 0, 0);
				AlteraPosicao("guest1", METADE_CAMPO_X + 0.7, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("guest2", METADE_CAMPO_X + 0.4, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("guest3", FREEBALL_QUADRANTE_1_X + 0.2, FREEBALL_QUADRANTE_1_Y, 0, 0);
			} else if (estadoReal[3].x < METADE_CAMPO_X  * 100 && estadoReal[3].y > METADE_CAMPO_Y * 100) { // QUADRANTE 2
				AlteraPosicao("home1", METADE_CAMPO_X - 0.7, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("home2", METADE_CAMPO_X - 0.4, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("home3", FREEBALL_QUADRANTE_2_X - 0.2, FREEBALL_QUADRANTE_2_Y, 0, 0);
				AlteraPosicao("ball", FREEBALL_QUADRANTE_2_X, FREEBALL_QUADRANTE_2_Y, 0, 0);
				AlteraPosicao("guest1", METADE_CAMPO_X + 0.7, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("guest2", METADE_CAMPO_X + 0.4, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("guest3", FREEBALL_QUADRANTE_2_X + 0.2, FREEBALL_QUADRANTE_2_Y, 0, 0);
			} else if (estadoReal[3].x <= METADE_CAMPO_X * 100 && estadoReal[3].y <= METADE_CAMPO_Y * 100) { // QUADRANTE 3
				AlteraPosicao("home1", METADE_CAMPO_X - 0.7, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("home2", METADE_CAMPO_X - 0.4, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("home3", FREEBALL_QUADRANTE_3_X - 0.2, FREEBALL_QUADRANTE_3_Y, 0, 0);
				AlteraPosicao("ball", FREEBALL_QUADRANTE_3_X, FREEBALL_QUADRANTE_3_Y, 0, 0);
				AlteraPosicao("guest1", METADE_CAMPO_X + 0.7, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("guest2", METADE_CAMPO_X + 0.4, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("guest3", FREEBALL_QUADRANTE_3_X + 0.2, FREEBALL_QUADRANTE_3_Y, 0, 0);
			} else if (estadoReal[3].x > METADE_CAMPO_X * 100 && estadoReal[3].y < METADE_CAMPO_Y * 100) { // QUADRANTE 4
				AlteraPosicao("home1", METADE_CAMPO_X - 0.7, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("home2", METADE_CAMPO_X - 0.4, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("home3", FREEBALL_QUADRANTE_4_X - 0.2, FREEBALL_QUADRANTE_4_Y, 0, 0);
				AlteraPosicao("ball", FREEBALL_QUADRANTE_4_X, FREEBALL_QUADRANTE_4_Y, 0, 0);
				AlteraPosicao("guest1", METADE_CAMPO_X + 0.7, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("guest2", METADE_CAMPO_X + 0.4, METADE_CAMPO_Y, -0.71, 0.7);
				AlteraPosicao("guest3", FREEBALL_QUADRANTE_4_X + 0.2, FREEBALL_QUADRANTE_4_Y, 0, 0);
			}

			emJogo = true;
			numFreeBalls++;
			c = 0;
			break;
		// case 'l':
		// case 'L':
		// 	break;
		// case 'd':
		// case 'D':
		// 	cout << "debug";
		// 	break;
	}
}

void Position(const gazebo_msgs::ModelStates &msg, int i, int indiceModel)
{
	ignition::math::Quaterniond q(msg.pose[indiceModel].orientation.w, msg.pose[indiceModel].orientation.x, msg.pose[indiceModel].orientation.y, msg.pose[indiceModel].orientation.z);
	// ignition::math::Vector3d ang = q.Euler();
	double yaw = q.Yaw();
	double yaw_degrees = yaw * 180.0 / PI; // conversion to degrees
	if (yaw_degrees < 0)
		yaw_degrees += 360.0; // convert negative to positive angles

	estadoReal[i].angulo = yaw_degrees;

	estadoReal[i].dx = (msg.pose[indiceModel].position.x + METADE_CAMPO_X) * 100 - estadoReal[i].x;
	estadoReal[i].dy = (msg.pose[indiceModel].position.y + METADE_CAMPO_Y) * 100 - estadoReal[i].y;

	estadoReal[i].x = (msg.pose[indiceModel].position.x + METADE_CAMPO_X) * 100;
	estadoReal[i].y = (msg.pose[indiceModel].position.y + METADE_CAMPO_Y) * 100;
}

void PoseCallback(const gazebo_msgs::ModelStates& msg)
{
	int i;

	for (i = 0; i < 7; i++)
	{
		estadoRealAnt[i].angulo = estadoReal[i].angulo;

		estadoRealAnt[i].dx = estadoReal[i].dx;
		estadoRealAnt[i].dy = estadoReal[i].dy;

		estadoRealAnt[i].x = estadoReal[i].x;
		estadoRealAnt[i].y = estadoReal[i].y;

		if (i < 3) {
			Position(msg, i, i+3); // Recebe posições dos robôs da casa (local)
		} else if (i == 3) {
			Position(msg, i, 2); // Recebe posição da bola
		} else if (i > 3) {
			Position(msg, i, i+2); // Recebe posições dos robôs visitantes (adversários)
		}
	}

	DesenhaPosicoes();
	// estrategia();
}

int main(int argc, char** argv) {
    	
	// image = cvCreateImage(cvSize(680, 520), 8, 3);
	image = cvCreateImage(cvSize(680, 580), 8, 3);
	
	// Create a window
	cvNamedWindow(wndname, 1);
	cvZero(image);
	cvShowImage(wndname, image);

	placarTime1.uni = 0;
	placarTime1.dez = 0;
	placarTime1.cen = 0;
	placarTime1.mil = 0;

	placarTime2.uni = 0;
	placarTime2.dez = 0;
	placarTime2.cen = 0;
	placarTime2.mil = 0;

	Inicia();

	ros::init(argc, argv, "vsss_referee");
	ros::NodeHandle n;

	pubModelPose = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 7);

	ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, PoseCallback);

	ros::Rate loop_rate(33);

	int count = 0;

	while (ros::ok()) {

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	cvReleaseImage(&image);
	cvDestroyWindow(wndname);

	return 0;
}

#ifdef _EiC
main(1,"drawing.c");
#endif
