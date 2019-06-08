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
#include "Estrategia.h"

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

#define RAIO_DA_RODA_M 0.017 // 1.7cm   
#define DIST_ENTRE_RODAS_M 0.069 // 6.9cm

#define VELOCIDADE_ANGULAR_MAXIMA 117.9947 // 117.9947 rad/s = 112(cmd)

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

// #define TEMPO_PARA_FREEBALL 300
#define TEMPO_PARA_FREEBALL 2000

// #define VELOCIDADE_MAX 107.1		   //(vMaxRe+vMaxRd)*RAIO_RODA/2 //Vel3 = 107.1 //Vel4 = 142.8 //Vel5 = 178.5
// #define RAIO_DISTANCIA 5
// #define VEL_MAX 2

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
int Goleiro1 = 0;
int Volante1 = 1;
int Atacante1 = 2;
bool emPenalti = false;
bool tiroMeta = false;

bool emPenalidade = false;
bool emPosiciona = false;
bool emInicio = false;

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
Estado estado1[NUM_ROBOS_TIME * 2 + 1], estado1Ant[NUM_ROBOS_TIME * 2 + 1], estado1Prev[NUM_ROBOS_TIME * 2 + 1];
Estado estadoReal[NUM_ROBOS_TIME * 2 + 1], estadoRealAnt[NUM_ROBOS_TIME * 2 + 1];

Objetivo objetivoRobo[NUM_ROBOS_TIME];

ros::Publisher pubAtacanteGuest;
ros::Publisher pubVolanteGuest;
ros::Publisher pubGoleiroGuest;

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

char wndname[] = "Controle Time 2 VSSS";


float xVisao[7][3], yVisao[7][3], angVisao[7][3];

void Visao(void) {
	int x, y, ang;
	int i;
	// ------------- nosso time
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

	for (i = 0; i < 7; i++) { // somente quando for jogar com o time da direita
		estado1[i].x = 170 - (estadoReal[6 - i].x); //+random(3)-1);
		estado1[i].y = 130 - (estadoReal[6 - i].y); //+random(3)-1);
		estado1[i].dx = estado1[i].x - estado1Ant[i].x;
		estado1[i].dy = estado1[i].y - estado1Ant[i].y;
		estado1[i].angulo = atang2(estado1[i].dy, estado1[i].dx);
	}

}

void PosicionaRobos1(void) {
	int i, j;
	for (i = 0; i < 3; i++) {
		for (j = i + 1; j < 3; j++) { //repete para os tres primeiros
			if (estado1[j].x < estado1[i].x) {
				Estado Tmp = estado1[j];
				estado1[j] = estado1[i];
				estado1[i] = Tmp;
			}
		}
	}
	for (i = 0; i < 3; i++) {
		estado1Ant[i] = estado1Prev[i] = estado1[i];
	}
	Goleiro1 = 0;
	Volante1 = 1;
	Atacante1 = 2;
}

int cnvCmd(int b) {
	if (b & 0x80)
		return -((b & 0x70) >> 4);
	else
		return b >> 4;
}

void EnviaCmd2Gazebo(int indiceRobo, float ve, float vd) {
	geometry_msgs::Twist msg;

	// float vel_linear_robo = (((vd + ve) / 2 * coss(ang)) + ((vd + ve) / 2 * seno(ang))) / 2;

	// float vel_linear_robo = (RAIO_DA_RODA_M / 2) * (vd + ve);
	// float vel_angular_robo = (RAIO_DA_RODA_M / DIST_ENTRE_RODAS_M) * (vd - ve);
	float vel_linear_robo = (vd + ve) / 2 * 30 / 100;
	float vel_angular_robo = atan2((vd - ve), DIST_ENTRE_RODAS_M);

	msg.linear.x = vel_linear_robo * -1;
	msg.angular.z = vel_angular_robo;
	// msg.angular.z = (ang * (PI / 180));

	cout << "  /////   pub[" << indiceRobo << "] x=" << msg.linear.x << " z=" << msg.angular.z << "\n";

	if (indiceRobo == 0) {
		pubGoleiroGuest.publish(msg);
	}
	else if (indiceRobo == 1) {
		pubVolanteGuest.publish(msg);
	}
	else if (indiceRobo == 2) {
		pubAtacanteGuest.publish(msg);
	}
}

static float vdAnt[3][TAM_HISTORIA], veAnt[3][TAM_HISTORIA];
static float fdang[3] = { 0, 0, 0 };
static float fdx[3] = { 0, 0, 0 }, fdy[3] = { 0, 0, 0 };

void enviaDados(unsigned char b1, unsigned char b2, unsigned char b3, unsigned char b4, unsigned char b5, unsigned char b6) {
	float vd, ve, dang, ang;
	float v;
	int i, j;
	float vd_Ant, ve_Ant;
	unsigned char b[6];
	//----------- b1
	if (b1 == 0x81) { //0x81 // nao usado na comunicacao real,
		memset(vdAnt, 0, sizeof vdAnt); // usado aqui para limpar a historia
		memset(veAnt, 0, sizeof vdAnt);
	}

	b[0] = (b1 >> 4) & 0xf;
	b[1] = (b2 >> 4) & 0xf;
	b[2] = (b3 >> 4) & 0xf;
	b[3] = (b4 >> 4) & 0xf;
	b[4] = (b5 >> 4) & 0xf;
	b[5] = (b6 >> 4) & 0xf;

	printf("\n");
	printf("                      %3s, %3s, %3s, %3s, %3s, %3s\n", "GRe", "GRd", "VRe", "VRd", "ARe", "ARd");
	// printf("ORI %3d, %3d, %3d, %3d, %3d, %3d\n", b1, b2, b3, b4, b5, b6);
	// printf("CMD RECEBIDO SERIAL_: %3d, %3d, %3d, %3d, %3d, %3d\n", (int)b1, (int)b2, (int)b3, (int)b4, (int)b5, (int)b6);
	// printf("CMD_TRANSFORMADO_1a7: %3d, %3d, %3d, %3d, %3d, %3d\n", cnvCmd(b1), cnvCmd(b2), cnvCmd(b3), cnvCmd(b4), cnvCmd(b5), cnvCmd(b6));
	printf("CMD_ENVIADO_SERIAL__: %3d, %3d, %3d, %3d, %3d, %3d\n", cnvCmd(b1), cnvCmd(b2), cnvCmd(b3), cnvCmd(b4), cnvCmd(b5), cnvCmd(b6));
	
	for (i = 0; i < 3; i++) {
		printf("\n----------INDICE: %d------------", i);
		if (b[i * 2] & 0x8) {
			ve = -(b[i * 2] & 0x7);
			// printf("\nve = -(b[i * 2] & 0x7): %0.2f", ve); // -1.00
		}
		else { 
			ve = b[i * 2] & 0x7;
			// printf("\nve = b[i * 2] & 0x7: %0.2f", ve);
		}
		if (b[i * 2 + 1] & 0x8) {
			vd = -(b[i * 2 + 1] & 0x7);
			// printf("\nvd = -(b[i * 2 + 1] & 0x7): %0.2f", vd); // 0.00
		}
		else {
			vd = b[i * 2 + 1] & 0x7;
			// printf("\nvd = -(b[i * 2 + 1] & 0x7): %0.2f", vd);
		}
		for (j = TAM_HISTORIA - 1; j > 0; j--) {
			vdAnt[i][j] = vdAnt[i][j - 1];
			veAnt[i][j] = veAnt[i][j - 1];
		}

		vdAnt[i][0] = vd * VEL_MAXIMA_ROBO_CM / VEL_MAXIMA_ROBO_UNID / 30;
		// printf("\nvdAnt[i][0] = vd * VEL_MAXIMA_ROBO_CM / VEL_MAXIMA_ROBO_UNID / 30: %0.2f", vdAnt[i][0]); //  0.00
		veAnt[i][0] = ve * VEL_MAXIMA_ROBO_CM / VEL_MAXIMA_ROBO_UNID / 30;
		// printf("\nvdAnt[i][0] = ve * VEL_MAXIMA_ROBO_CM / VEL_MAXIMA_ROBO_UNID / 30: %0.2f", veAnt[i][0]); // -0.71

		vd = vdAnt[i][TEMPO_ATRASO]; //atraso entre a visao e a realizacao do comando
		// printf("\nvd = vdAnt[i][TEMPO_ATRASO]: %0.2f", vd); // 0.00
		ve = veAnt[i][TEMPO_ATRASO];
		// printf("\nve = vdAnt[i][TEMPO_ATRASO]: %0.2f", ve); // -0.71

		float somaVd = 0, somaVe = 0, cont = 0;
		for (j = TEMPO_ATRASO; j < TAM_INERCIA + TEMPO_ATRASO; j++) {
			somaVd += vdAnt[i][j];
			somaVe += veAnt[i][j];
			cont++;
		}

		vd = somaVd / cont;
		// printf("\nvd = somaVd / cont: %0.2f", vd); // 0.00
		ve = somaVe / cont;
		// printf("\nve = somaVe / cont: %0.2f", ve); // -0.71

		// //estadoReal[i].dAngulo = dang = (vd - ve) / DIST_ENTRE_RODAS * 180 / 3.14;***
		// estadoReal[i].dAngulo = dang = atan2((vd - ve), DIST_ENTRE_RODAS) * 180 / 3.14;
		dang = atan2((vd - ve), DIST_ENTRE_RODAS) * 180 / 3.14;
		// printf("\ndang = atan2((vd - ve), DIST_ENTRE_RODAS): %0.2f", dang); // 0.10
		ang = (estadoReal[i].angulo += dang);
		// printf("\nang = (estadoReal[i].angulo += dang): %0.2f", ang); // 180.11
		// estadoReal[i].dx = (vd + ve) / 2 * coss(ang);
		// printf("\nestadoReal[i].dx = (vd + ve) / 2 * coss(ang): %0.2f", (vd + ve) / 2 * coss(ang)); // 0.36
		// estadoReal[i].dy = (vd + ve) / 2 * seno(ang);
		// printf("\nestadoReal[i].dx = (vd + ve) / 2 * seno(ang): %0.2f", (vd + ve) / 2 * seno(ang)); // 0.00

		// if (estadoReal[i].dx > 5)
		// 	estadoReal[i].dx = 5;
		// else if (estadoReal[i].dx < -5)
		// 	estadoReal[i].dx = -5;

		// if (estadoReal[i].dy > 5)
		// 	estadoReal[i].dy = 5;
		// else if (estadoReal[i].dy < -5)
		// 	estadoReal[i].dy = -5;

		// while (estadoReal[i].angulo < 0)
		// 	estadoReal[i].angulo += 360;
		// while (estadoReal[i].angulo > 360)
		// 	estadoReal[i].angulo -= 360;
		// // estadoReal[i].x += estadoReal[i].dx;
		// // estadoReal[i].y += estadoReal[i].dy;

		if (emJogo) {
			EnviaCmd2Gazebo(i, ve, vd);
		}
	}

	// geometry_msgs::Twist msg;

	// // msg.linear.x = 0.0;
	// // msg.angular.z = 0;
	// // msg.angular.z = ang*3.14/180;


	// float vel_angular_esquerda = (int)b1 * VELOCIDADE_ANGULAR_MAXIMA / 112;
	// printf("\nG vel_angular_esquerda = (int)b1 * VELOCIDADE_ANGULAR_MAXIMA / 112: %0.2f", vel_angular_esquerda);
	// float vel_angular_direita = (int)b2 * VELOCIDADE_ANGULAR_MAXIMA / 112;
	// printf("\nG vel_angular_direita = (int)b2 * VELOCIDADE_ANGULAR_MAXIMA / 112: %0.2f", vel_angular_direita);

	// float vel_linear_esquerda = vel_angular_esquerda * RAIO_DA_RODA_M;
	// printf("\nG vel_linear_esquerda = vel_angular_esquerda * RAIO_DA_RODA_M: %0.2f", vel_linear_esquerda);
	// float vel_linear_direita = vel_angular_direita * RAIO_DA_RODA_M;
	// printf("\nG vel_linear_direita = vel_angular_direita * RAIO_DA_RODA_M: %0.2f", vel_linear_direita);

	// float vel_linear_robo = vel_linear_direita + vel_linear_esquerda / 2;
	// printf("\nG vel_linear_robo = vel_linear_direita + vel_linear_esquerda / 2: %0.2f", vel_linear_robo);
	// // float vel_angular_robo = vel_linear_direita - vel_linear_esquerda / DIST_ENTRE_RODAS_M;
	// float vel_angular_robo = atan2((vel_linear_direita - vel_linear_esquerda), DIST_ENTRE_RODAS_M);
	// printf("\nG vel_angular_robo = vel_linear_direita - vel_linear_esquerda / DIST_ENTRE_RODAS_M: %0.2f\n", vel_angular_robo);

	// msg.linear.x = vel_linear_robo / 10;
	// msg.angular.z = vel_angular_robo / 10;

	// pubGoleiroHome.publish(msg);
	// cout << "pubG" << " x=" << msg.linear.x << " z="<<msg.angular.z << "\n";


	// vel_angular_esquerda = (int)b3 * VELOCIDADE_ANGULAR_MAXIMA / 112;
	// vel_angular_direita = (int)b4 * VELOCIDADE_ANGULAR_MAXIMA / 112;

	// vel_linear_esquerda = vel_angular_esquerda * RAIO_DA_RODA_M;
	// vel_linear_direita = vel_angular_direita * RAIO_DA_RODA_M;

	// vel_linear_robo = vel_linear_direita + vel_linear_esquerda / 2;
	// // vel_angular_robo = vel_linear_direita - vel_linear_esquerda / DIST_ENTRE_RODAS_M;
	// vel_angular_robo = atan2((vel_linear_direita - vel_linear_esquerda), DIST_ENTRE_RODAS_M);

	// printf("\nV vel_angular_esquerda = (int)b1 * VELOCIDADE_ANGULAR_MAXIMA / 112: %0.2f", vel_angular_esquerda);
	// printf("\nV vel_angular_direita = (int)b2 * VELOCIDADE_ANGULAR_MAXIMA / 112: %0.2f", vel_angular_direita);
	// printf("\nV vel_linear_esquerda = vel_angular_esquerda * RAIO_DA_RODA_M: %0.2f", vel_linear_esquerda);
	// printf("\nV vel_linear_direita = vel_angular_direita * RAIO_DA_RODA_M: %0.2f", vel_linear_direita);
	// printf("\nV vel_linear_robo = vel_linear_direita + vel_linear_esquerda / 2: %0.2f", vel_linear_robo);
	// printf("\nV vel_angular_robo = vel_linear_direita - vel_linear_esquerda / DIST_ENTRE_RODAS_M: %0.2f\n", vel_angular_robo);

	// msg.linear.x = vel_linear_robo / 10;
	// msg.angular.z = vel_angular_robo / 10;

	// pubVolanteHome.publish(msg);
	// cout << "pubV" << " x=" << msg.linear.x << " z="<<msg.angular.z << "\n";


	// vel_angular_esquerda = (int)b5 * VELOCIDADE_ANGULAR_MAXIMA / 112;
	// vel_angular_direita = (int)b6 * VELOCIDADE_ANGULAR_MAXIMA / 112;

	// vel_linear_esquerda = vel_angular_esquerda * RAIO_DA_RODA_M;
	// vel_linear_direita = vel_angular_direita * RAIO_DA_RODA_M;

	// vel_linear_robo = vel_linear_direita + vel_linear_esquerda / 2;
	// // vel_angular_robo = vel_linear_direita - vel_linear_esquerda / DIST_ENTRE_RODAS_M;
	// vel_angular_robo = atan2((vel_linear_direita - vel_linear_esquerda), DIST_ENTRE_RODAS_M);

	// printf("\nA vel_angular_esquerda = (int)b1 * VELOCIDADE_ANGULAR_MAXIMA / 112: %0.2f", vel_angular_esquerda);
	// printf("\nA vel_angular_direita = (int)b2 * VELOCIDADE_ANGULAR_MAXIMA / 112: %0.2f", vel_angular_direita);
	// printf("\nA vel_linear_esquerda = vel_angular_esquerda * RAIO_DA_RODA_M: %0.2f", vel_linear_esquerda);
	// printf("\nA vel_linear_direita = vel_angular_direita * RAIO_DA_RODA_M: %0.2f", vel_linear_direita);
	// printf("\nA vel_linear_robo = vel_linear_direita + vel_linear_esquerda / 2: %0.2f", vel_linear_robo);
	// printf("\nA vel_angular_robo = vel_linear_direita - vel_linear_esquerda / DIST_ENTRE_RODAS_M: %0.2f\n", vel_angular_robo);

	// msg.linear.x = vel_linear_robo / 10;
	// msg.angular.z = vel_angular_robo / 10;

	// pubAtacanteHome.publish(msg);
	// cout << "pubA" << " x=" << msg.linear.x << " z="<<msg.angular.z << "\n";
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

void DesenhaObjetivos(int i) {
	int j;
	int x, y, ang;
	int d1, d2, x1, y1;
	char num[2];

	num[0] = '0' + i;
	num[1] = 0;
	OutTextXY(objetivoRobo[i].x - 2, objetivoRobo[i].y + 2, num);

	MoveTo(objetivoRobo[i].x, objetivoRobo[i].y);
	x = objetivoRobo[i].x;
	y = objetivoRobo[i].y;
	ang = objetivoRobo[i].angulo;

	d1 = coss(ang) * 10;
	d2 = seno(ang) * 10;

	setColor(GREEN);
	MoveTo(x, y);
	LineTo((x + d1), (y + d2));

	setColor(i);

	Circle(x, y, 4);

	setColor(WHITE);
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

void DesenhaJogadorPrev(int i) {
	float x, y, ang;
	float d1, d2, x1, y1;

	setColor(GREEN);

	x = estadoPrev[i].x;
	y = estadoPrev[i].y;
	ang = estadoPrev[i].angulo;

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

void DesenhaBolaPrev(void) {
	setColor(DARKGRAY);
	Circle(estadoPrev[3].x, estadoPrev[3].y, 3);
	setColor(WHITE);
}

void DesenhaJogo(void) {
	char num[10];
	int i;

	DesenhaCampo();

	for (i = 0; i < 7; i++) {

		if (i != 3) {
			DesenhaJogadorVisao(i);
			DesenhaJogadorPrev(i);
			DesenhaJogador(i);
			DesenhaObjetivos(i);
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
			DesenhaBolaPrev();
		}
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

float CnvtCm2M(float numCm) {
	return numCm / 100;
}

void resetaVelRobos() {
	int i=0;
	for (i; i < 3; i++) {
		EnviaCmd2Gazebo(i, 0, 0);
	}
}

void DesenhaPosicoes()
{
	int i;
	char c;

	if (auxCont >= 10000){
		// imprimeEstatistica = true;
		auxCont = 0;
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

	estrategia();
	
	DesenhaJogo();
	cvShowImage(wndname, image);

	indGoleiro = 0;
	indAtacante = 2;
	indVolante = 1;

	fflush(stdout);
	c = cvWaitKey(30); // Se comentar essa linha, a janela do simulador antigo não aparece
	printf("(%d) \n", c);

	switch (c) {
		case 'p': // posiciona jogadores
		case 'P':
			emJogo = false;

			resetaVelRobos();

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

			resetaVelRobos();

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

			resetaVelRobos();

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

			resetaVelRobos();

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

	// estadoReal[i].dx = (1.7 - (((msg.pose[indiceModel].position.x + METADE_CAMPO_X) * cos(180 * PI / 180)) - ((msg.pose[indiceModel].position.y + METADE_CAMPO_Y)) * sin(180 * PI / 180))) * 100 - estadoReal[i].x;
	// estadoReal[i].dy = (1.3 - (((msg.pose[indiceModel].position.x + METADE_CAMPO_X) * sin(180 * PI / 180)) + ((msg.pose[indiceModel].position.y + METADE_CAMPO_Y)) * cos(180 * PI / 180))) * 100 - estadoReal[i].y;

	// estadoReal[i].x = (1.7 - (((msg.pose[indiceModel].position.x + METADE_CAMPO_X) * cos(180 * PI / 180)) - ((msg.pose[indiceModel].position.y + METADE_CAMPO_Y)) * sin(180 * PI / 180))) * 100;
	// estadoReal[i].y = (1.3 - (((msg.pose[indiceModel].position.x + METADE_CAMPO_X) * sin(180 * PI / 180)) + ((msg.pose[indiceModel].position.y + METADE_CAMPO_Y)) * cos(180 * PI / 180))) * 100;
	
	estadoReal[i].dx = (1.7 - (msg.pose[indiceModel].position.x + METADE_CAMPO_X)) * 100 - estadoReal[i].x;
	estadoReal[i].dy = (1.3 - (msg.pose[indiceModel].position.y + METADE_CAMPO_Y)) * 100 - estadoReal[i].y;

	estadoReal[i].x = (1.7 - (msg.pose[indiceModel].position.x + METADE_CAMPO_X)) * 100;
	estadoReal[i].y = (1.3 - (msg.pose[indiceModel].position.y + METADE_CAMPO_Y)) * 100;
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
			Position(msg, i, i+6); // Recebe posições dos robôs da casa (local)
		} else if (i == 3) {
			Position(msg, i, 2); // Recebe posição da bola
		} else if (i > 3) {
			Position(msg, i, i-1); // Recebe posições dos robôs visitantes (adversários)
		}
	}

	DesenhaPosicoes();
	// estrategia();
}

int main(int argc, char** argv) {
    	
	image = cvCreateImage(cvSize(680, 520), 8, 3);
	// image = cvCreateImage(cvSize(680, 580), 8, 3);
	
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

	ros::init(argc, argv, "vsss_team2_control");
	ros::NodeHandle n;

	pubAtacanteGuest = n.advertise<geometry_msgs::Twist>("/cmd_vel_guest3", 1);
	pubVolanteGuest = n.advertise<geometry_msgs::Twist>("/cmd_vel_guest2", 1);
	pubGoleiroGuest = n.advertise<geometry_msgs::Twist>("/cmd_vel_guest1", 1);

	pubModelPose = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 7);

	ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, PoseCallback);

	ros::Rate loop_rate(33);

	int count = 0;

	while (ros::ok()) {
		// printf("\nros::ok() %d", ros::ok());

		ros::spinOnce();

		loop_rate.sleep();
		++count;
		// printf("\n++count %d", count);
	}

	cvReleaseImage(&image);
	cvDestroyWindow(wndname);

	return 0;
}

#ifdef _EiC
main(1,"drawing.c");
#endif
