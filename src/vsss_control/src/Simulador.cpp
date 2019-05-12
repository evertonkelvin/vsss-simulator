/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <pthread.h>
#include <unistd.h>

#ifdef _CH_
#pragma package < opencv >
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
#include "Estrategia.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"

//#define RANDOMICO
#define DARKGRAY 192, 192, 192
#define LIGHTGRAY 224, 224, 224
#define GRAY 160, 160, 160
#define WHITE 255, 255, 255
#define LIGHTMAGENTA 255, 128, 255
#define GREEN 0, 255, 0
#define LIGHTRED 255, 128, 128
#define LIGHTBLUE 128, 128, 255
#define YELLOW 255, 255, 0

#define TAM_HISTORIA 50 // tem que ser o maior entre TEMPO_ATRASO e TAM_INERCIA
#define TEMPO_ATRASO 5
#define TAM_INERCIA 3

bool emJogo = false;
struct placar
{
	int uni;
	int dez;
	int cen;
	int mil;
} placarTime1, placarTime2;
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

int quadro;

Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1], estadoPrev[NUM_ROBOS_TIME * 2 + 1];
Estado estado1[NUM_ROBOS_TIME * 2 + 1], estado1Ant[NUM_ROBOS_TIME * 2 + 1], estado1Prev[NUM_ROBOS_TIME * 2 + 1];
Estado estadoReal[NUM_ROBOS_TIME * 2 + 1], estadoRealAnt[NUM_ROBOS_TIME * 2 + 1];

Objetivo objetivoRobo[NUM_ROBOS_TIME];

#define DESLINICIOCAMPO 520

CvScalar corNoCV;
CvFont font;
IplImage *image;

void setColor(int r, int g, int b)
{
	corNoCV.val[0] = b;
	corNoCV.val[1] = g;
	corNoCV.val[2] = r;
	corNoCV.val[3] = 0;
}

void Inicia(void)
{
	cvInitFont(&font, CV_FONT_VECTOR0, 1.0, 1.0);
	setColor(0, 0, 0);
}

void setColor(int i)
{
	setColor(255 * (i - 20) / 20, 128, 255 * (20) / 7);
}

#define ESCALA 4

void Circle(float x, float y, float radius)
{
	CvPoint pt1 = {x * ESCALA, DESLINICIOCAMPO - y * ESCALA};
	cvCircle(image, pt1, radius * ESCALA, corNoCV);
}

float ultX = 0, ultY = 0;
void MoveTo(float x, float y)
{
	ultX = x * ESCALA;
	ultY = DESLINICIOCAMPO - y * ESCALA;
}

void LineTo(float x, float y)
{
	CvPoint pt1 = {ultX, ultY};
	ultX = x * ESCALA;
	ultY = DESLINICIOCAMPO - y * ESCALA;
	CvPoint pt2 = {ultX, ultY};
	cvLine(image, pt1, pt2, corNoCV);
}

void Rectangle(float x1, float y1, float x2, float y2)
{
	CvPoint pt1 = {x1 * ESCALA, y1 * ESCALA};
	CvPoint pt2 = {x2 * ESCALA, y2 * ESCALA};
	CvScalar cor = corNoCV;
	cor.val[0] = 1 - cor.val[0];
	cor.val[1] = 1 - cor.val[1];
	cor.val[2] = 1 - cor.val[2];
	cvRectangle(image, pt1, pt2, cor, -1);
}

void OutTextXY(float x, float y, char *textstring)
{
	CvPoint pt1 = {x * ESCALA, DESLINICIOCAMPO - y * ESCALA};
	cvPutText(image, textstring, pt1, &font, corNoCV);
}

char wndname[] = "Simulador";

void PosicionaRobos(void)
{
}

float xVisao[7][3], yVisao[7][3], angVisao[7][3];
void Visao(void)
{
	int x, y, ang;
	int i;
	// ------------- nosso time
	for (i = 0; i < 7; i++)
	{
		estadoAnt[i] = estado[i];
		if (emJogo)
		{
#ifdef RANDOMICO
			x = estadoReal[i].x + random() % 3 - 1; //coordenada obtida da camera
			y = estadoReal[i].y + random() % 3 - 1;
			ang = estadoReal[i].angulo + random() % 3 - 1;
#else
			x = estadoReal[i].x; //coordenada obtida da camera
			y = estadoReal[i].y;
			ang = estadoReal[i].angulo;
#endif
			xVisao[i][2] = xVisao[i][1];
			xVisao[i][1] = xVisao[i][0];
			xVisao[i][0] = x;

			yVisao[i][2] = yVisao[i][1];
			yVisao[i][1] = yVisao[i][0];
			yVisao[i][0] = y;

			angVisao[i][2] = angVisao[i][1];
			angVisao[i][1] = angVisao[i][0];
			angVisao[i][0] = ang;

			estado[i].angulo = angVisao[i][2];

			estado[i].dx = xVisao[i][2] - estado[i].x;
			estado[i].dy = yVisao[i][2] - estado[i].y;

			estado[i].x = xVisao[i][2];
			estado[i].y = yVisao[i][2];
		}
	}

	for (i = 0; i < 7; i++)
	{
		estado1[i].x = 170 - (estadoReal[6 - i].x); //+random(3)-1);
		estado1[i].y = 130 - (estadoReal[6 - i].y); //+random(3)-1);
		estado1[i].dx = estado1[i].x - estado1Ant[i].x;
		estado1[i].dy = estado1[i].y - estado1Ant[i].y;
		estado1[i].angulo = atang2(estado1[i].dy, estado1[i].dx);
	}
	//	printf("%5d\n", quadro++);
}

void PosicionaRobos1(void)
{
	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = i + 1; j < 3; j++)
		{ //repete para os tres primeiros
			if (estado1[j].x < estado1[i].x)
			{
				Estado Tmp = estado1[j];
				estado1[j] = estado1[i];
				estado1[i] = Tmp;
			}
		}
	}
	for (i = 0; i < 3; i++)
	{
		estado1Ant[i] = estado1Prev[i] = estado1[i];
	}
	Goleiro1 = 0;
	Volante1 = 1;
	Atacante1 = 2;
}

int cnvCmd(int b)
{
	if (b & 0x80)
		return -((b & 0x70) >> 4);
	else
		return b >> 4;
}
static float vdAnt[7][TAM_HISTORIA], veAnt[7][TAM_HISTORIA];
static float fdang[3] = {0, 0, 0};
static float fdx[3] = {0, 0, 0}, fdy[3] = {0, 0, 0};
void enviaDados(unsigned char b1, unsigned char b2, unsigned char b3,
				unsigned char b4, unsigned char b5, unsigned char b6, unsigned char b7, unsigned char b8, unsigned char b9,
				unsigned char b10, unsigned char b11, unsigned char b12)
{
	float vd, ve, dang, ang;
	float v;
	int i, j, ind_i;
	float vd_Ant, ve_Ant;
	unsigned char b[12];
	//----------- b1
	if (b1 == 0x81)
	{									//0x81 nao usado na comunicacao real,
		memset(vdAnt, 0, sizeof vdAnt); //usado aqui para limpar a historia
		memset(veAnt, 0, sizeof vdAnt);
	}
	printf("%3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d\n", b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12);
	b[0] = (b1 >> 4) & 0xf;
	b[1] = (b2 >> 4) & 0xf;
	b[2] = (b3 >> 4) & 0xf;
	b[3] = (b4 >> 4) & 0xf;
	b[4] = (b5 >> 4) & 0xf;
	b[5] = (b6 >> 4) & 0xf;

	b[6] = (b7 >> 4) & 0xf;
	b[7] = (b8 >> 4) & 0xf;
	b[8] = (b9 >> 4) & 0xf;
	b[9] = (b10 >> 4) & 0xf;
	b[10] = (b11 >> 4) & 0xf;
	b[11] = (b12 >> 4) & 0xf;

	for (i = 0; i < 7; i++)
	{
		if (i == 3)
			i++;
		if (i < 3)
		{
			ind_i = i * 2;
		}
		else
		{
			ind_i = i * 2 - 2;
		}

		if (b[ind_i] & 0x8)
			ve = -(b[ind_i] & 0x7);
		else
			ve = b[ind_i] & 0x7;

		if (b[ind_i + 1] & 0x8)
			vd = -(b[ind_i + 1] & 0x7);
		else
			vd = b[ind_i + 1] & 0x7;

		for (j = TAM_HISTORIA - 1; j > 0; j--)
		{
			vdAnt[i][j] = vdAnt[i][j - 1];
			veAnt[i][j] = veAnt[i][j - 1];
		}

		vdAnt[i][0] = vd * VEL_MAXIMA_ROBO_CM / VEL_MAXIMA_ROBO_UNID / 30;
		veAnt[i][0] = ve * VEL_MAXIMA_ROBO_CM / VEL_MAXIMA_ROBO_UNID / 30;

		vd = vdAnt[i][TEMPO_ATRASO]; //atraso entre a visao e a realizacao do comando
		ve = veAnt[i][TEMPO_ATRASO];

		float somaVd = 0, somaVe = 0, cont = 0;
		for (j = TEMPO_ATRASO; j < TAM_INERCIA + TEMPO_ATRASO; j++)
		{
			somaVd += vdAnt[i][j];
			somaVe += veAnt[i][j];
			cont++;
		}

		vd = somaVd / cont;
		ve = somaVe / cont;

		estadoReal[i].dAngulo = dang = (vd - ve) / DIST_ENTRE_RODAS * 180 / 3.14;
		ang = (estadoReal[i].angulo += dang);
		estadoReal[i].dx = (vd + ve) / 2 * coss(ang);
		estadoReal[i].dy = (vd + ve) / 2 * seno(ang);

		if (estadoReal[i].dx > 5)
			estadoReal[i].dx = 5;
		else if (estadoReal[i].dx < -5)
			estadoReal[i].dx = -5;

		if (estadoReal[i].dy > 5)
			estadoReal[i].dy = 5;
		else if (estadoReal[i].dy < -5)
			estadoReal[i].dy = -5;

		while (estadoReal[i].angulo < 0)
			estadoReal[i].angulo += 360;
		while (estadoReal[i].angulo > 360)
			estadoReal[i].angulo -= 360;
		//			estadoReal[i].x += estadoReal[i].dx;
		//			estadoReal[i].y += estadoReal[i].dy;
	}
}
//void enviaDados(unsigned char b1, unsigned char b2, unsigned char b3) {
//	float vd, ve, dang, ang;
//	float v;
//	int i;
//	float vd_Ant, ve_Ant;
//	unsigned char b[3];
//	//----------- b1
//	if (b1 == 0x81) { //0x81 nao usado na comunicacao real,
//		memset(vdAnt, 0, sizeof vdAnt); //usado aqui para limpar a historia
//		memset(veAnt, 0, sizeof vdAnt);
//	}
//	printf("%3x, %3x, %3x\n", b1, b2, b3);
//	b[0] = b1;
//	b[1] = b2;
//	b[2] = b3;
//	for (i = 0; i < 3; i++) {
//
//		//    vd_Ant=(vdAnt[0][i]+vdAnt[1][i]+vdAnt[2][i])/3;
//		//    vdAnt[2][i]=vdAnt[1][i]; vdAnt[1][i]=vdAnt[0][i];
//		//    ve_Ant=(veAnt[0][i]+veAnt[1][i]+veAnt[2][i])/3;
//		//    veAnt[2][i]=veAnt[1][i]; veAnt[1][i]=veAnt[0][i];
//
//		if (b[i] & 0x8)
//			vd = -(b[i] & 0x7);
//		else
//			vd = b[i] & 0x7;
//		vd = vd * 5 / 7; //a unidade de comando significa 0,7142857cm de deslocamento
//		/*    if (abs(vd_Ant-vd)>1) {
//		 if (vd_Ant>vd)
//		 vd=vd_Ant-1;	//esta diminuindo a velocidade
//		 else
//		 vd=vd_Ant+1;	//esta aumentando a velocidade
//		 }*/
//		vdAnt[0][i] = vd;
//
//		if (b[i] & 0x80)
//			ve = -((b[i] >> 4) & 7);
//		else
//			ve = (b[i] >> 4) & 7;
//		ve = ve * 5 / 7; //a unidade de comando significa 0,7142857cm de deslocamento
//		/*    if (abs(ve_Ant-ve)>1) {
//		 if (ve_Ant>ve)
//		 ve=ve_Ant-1;	//esta diminuindo a velocidade
//		 else
//		 ve=ve_Ant+1;	//esta aumentando a velocidade
//		 }*/
//		veAnt[0][i] = ve;
//
//		vd = vdAnt[0][i]; //atraso entre a visao e a realizacao do comando
//		ve = veAnt[0][i];
//
//		v = (vd + ve) / 2; //velocidade a frente
//		dang = fdang[i] += (float) (vd - ve) * 5 / 3;
//		fdang[i] -= dang;
//		if (dang < 0)
//			dang += 360;
//		ang = estadoReal[i].angulo + dang;
//		if (ang < 0)
//			ang += 360;
//		else if (ang > 360)
//			ang -= 360;
//		estadoReal[i].angulo = ang;
//
//		estadoReal[i].dx = fdx[i] += coss(dang) * v * coss(ang);
//		fdx[i] -= estadoReal[i].dx;
//		if (estadoReal[i].dx > 5)
//			estadoReal[i].dx = 5;
//		else if (estadoReal[i].dx < -5)
//			estadoReal[i].dx = -5;
//
//		estadoReal[i].dy = fdy[i] += coss(dang) * v * seno(ang);
//		fdy[i] -= estadoReal[i].dy;
//		if (estadoReal[i].dy > 5)
//			estadoReal[i].dy = 5;
//		else if (estadoReal[i].dy < -5)
//			estadoReal[i].dy = -5;
//
//		//  estadoReal[0].Angulo=dang=DifAng(estadoReal[0].Angulo, dang);
//	}
//}

void CorrigeLimites(int i)
{
	if (i != 3)
	{
		if (41 < estadoReal[i].y && estadoReal[i].y < 89)
		{
			if (estadoReal[i].x < 4)
			{
				estadoReal[i].x = 4;
			}
			else if (estadoReal[i].x > 166)
			{
				estadoReal[i].x = 166;
			}
		}
		else
		{
			if (estadoReal[i].x < 14)
			{
				estadoReal[i].x = 14;
			}
			else if (estadoReal[i].x > 156)
			{
				estadoReal[i].x = 156;
			}
			if (estadoReal[i].y < 4)
			{
				estadoReal[i].y = 4;
			}
			else if (estadoReal[i].y > 126)
			{
				estadoReal[i].y = 126;
			}
		}
	}
	else
	{ // é a bola
		int dx = estadoReal[i].dx;
		int dy = estadoReal[i].dy;
		if (42 < estadoReal[i].y && estadoReal[i].y < 87)
		{ // bola na direcao (y) dos limites do gol?
			if (estadoReal[i].x < 3)
			{ //bola no fundo do gol?
				estadoReal[i].x = 3;
				estadoReal[i].dx = -dx - 1;
			}
			else if (estadoReal[i].x > 167)
			{ //bola no fundo do gol?
				estadoReal[i].x = 167;
				estadoReal[i].dx = -dx + 1;
			}
		}
		else
		{													//bola fora da direcao do gol
			if (estadoReal[i].x - estadoReal[i].y > 151 ||  // canto inferior direito
				estadoReal[i].x - estadoReal[i].y < -110 || // canto superio esquerdo
				estadoReal[i].x + estadoReal[i].y > 280 ||  // canto superio direito
				estadoReal[i].x + estadoReal[i].y < 20)
			{ // canto inferior esquerdo
				while (estadoReal[i].x - estadoReal[i].y > 151)
				{
					estadoReal[i].x--;
					estadoReal[i].y++;
				}
				while (estadoReal[i].x - estadoReal[i].y < -110)
				{
					estadoReal[i].x++;
					estadoReal[i].y--;
				}
				while (estadoReal[i].x + estadoReal[i].y > 280)
				{
					estadoReal[i].x--;
					estadoReal[i].y--;
				}
				while (estadoReal[i].x + estadoReal[i].y < 20)
				{
					estadoReal[i].x++;
					estadoReal[i].y++;
				}
				if (abs(dx) > abs(dy))
				{
					dx > 0 ? dx-- : dx++;
				}
				else if (abs(dx) < abs(dy))
				{
					dy > 0 ? dy-- : dy++;
				}
				else if (abs(dx) > 0)
				{
					dx > 0 ? dx-- : dx++;
					dy > 0 ? dy-- : dy++;
				}
				if (estadoReal[i].x + estadoReal[i].y > 280 || // canto superio direito
					estadoReal[i].x + estadoReal[i].y < 20)
				{
					estadoReal[i].dx = -dy;
					estadoReal[i].dy = -dx;
				}
				else
				{
					estadoReal[i].dx = dy;
					estadoReal[i].dy = dx;
				}
				return;
			}
			if (estadoReal[i].x < 13)
			{ //linha de fundo?
				estadoReal[i].x = 13;
				estadoReal[i].dx = -dx - 1;
			}
			else if (estadoReal[i].x > 157)
			{ //linha de fundo?
				estadoReal[i].x = 157;
				estadoReal[i].dx = -dx + 1;
			}
			if (estadoReal[i].y < 3)
			{ //na lateral?
				estadoReal[i].y = 3;
				estadoReal[i].dy = -dy - 1;
			}
			else if (estadoReal[i].y > 127)
			{ //na lateral?
				estadoReal[i].y = 127;
				estadoReal[i].dy = -dy + 1;
			}
		}
	}
}

void AtualizaJogador(int i)
{
	estadoRealAnt[i] = estadoReal[i];
	//  estadoReal[i].Dx+=random(3)-1;
	//  estadoReal[i].Dy+=random(3)-1;
	if (estadoReal[i].dx > 7)
		estadoReal[i].dx = 7;
	if (estadoReal[i].dy > 7)
		estadoReal[i].dy = 7;
	estadoReal[i].x += estadoReal[i].dx;
	estadoReal[i].y += estadoReal[i].dy;

	CorrigeLimites(i);

	estadoReal[i].dx = estadoReal[i].x - estadoRealAnt[i].x;
	estadoReal[i].dy = estadoReal[i].y - estadoRealAnt[i].y;
	//  estadoReal[i].Angulo=iatan2(estadoReal[i].Dy, estadoReal[i].Dx);
}

void AtualizaBola(void)
{
	int i = 3;
	estadoRealAnt[i] = estadoReal[i];
	if (estadoReal[i].dx > 7)
		estadoReal[i].dx = 7;
	if (estadoReal[i].dy > 7)
		estadoReal[i].dy = 7;
	estadoReal[i].x += estadoReal[i].dx;
	estadoReal[i].y += estadoReal[i].dy;

	CorrigeLimites(i);

	estadoReal[i].angulo = atang2(estadoReal[i].dy, estadoReal[i].dx);
}

void DesenhaCampo(void)
{
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
void DesenhaPlacar()
{
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

	OutTextXY(TAM_X_CAMPO / 2 - 50, -10, placarTimes);
}
void DesenhaObjetivos(int i)
{
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

int VerificaColisao(int r1, int r2)
{
	int x1r1, x2r1, x1r2, x2r2;
	int y1r1, y2r1, y1r2, y2r2;
	int Colisaox = 0, Colisaoy = 0;
	int d;

	if (r1 == r2)
		return 0;

	if (r1 == 3 || r2 == 3)
		d = 6;
	else
		d = 8;

	if (estadoReal[r1].x > estadoRealAnt[r1].x)
	{ //teste pelo cruzamento
		x1r1 = estadoRealAnt[r1].x;
		x2r1 = estadoReal[r1].x;
	}
	else
	{
		x1r1 = estadoReal[r1].x;
		x2r1 = estadoRealAnt[r1].x;
	}
	if (estadoReal[r2].x > estadoRealAnt[r2].x)
	{
		x1r2 = estadoRealAnt[r2].x;
		x2r2 = estadoReal[r2].x;
	}
	else
	{
		x1r2 = estadoReal[r2].x;
		x2r2 = estadoRealAnt[r2].x;
	}
	if (x1r1 < x2r2)
	{
		if (x2r1 > x2r2 || x2r1 > x1r2)
		{
			Colisaox = 1;
		}
	}
	if (estadoReal[r1].y > estadoRealAnt[r1].y)
	{ //teste pelo cruzamento
		y1r1 = estadoRealAnt[r1].y;
		y2r1 = estadoReal[r1].y;
	}
	else
	{
		y1r1 = estadoReal[r1].y;
		y2r1 = estadoRealAnt[r1].y;
	}
	if (estadoReal[r2].y > estadoRealAnt[r2].y)
	{
		y1r2 = estadoRealAnt[r2].y;
		y2r2 = estadoReal[r2].y;
	}
	else
	{
		y1r2 = estadoReal[r2].y;
		y2r2 = estadoRealAnt[r2].y;
	}
	if (y1r1 < y2r2)
	{
		if (y2r1 > y2r2 || y2r1 > y1r2)
		{
			Colisaoy = 1;
		}
	}

	if (abs(x2r1 - x2r2) < d && abs(y2r1 - y2r2) < d)
	{ //teste pela distancia
		Colisaox = 1;
		Colisaoy = 1;
	}
	return Colisaox && Colisaoy;
}

void CorrigePosicao(int r1, int r2)
{ //corrige colisao
	int xc, yc;
	int d1, d2, dx, dy, h;

	if (r1 != 3 && r2 != 3)
	{
		xc = (estadoRealAnt[r1].x + estadoRealAnt[r2].x) / 2;
		yc = (estadoRealAnt[r1].y + estadoRealAnt[r2].y) / 2;
		d1 = (abs(estadoRealAnt[r1].dx) + abs(estadoRealAnt[r1].dy)) / 2;
		if (d1 != 0)
		{
			estadoReal[r1].x = xc - estadoRealAnt[r1].dx * 2 / d1;
			estadoReal[r1].y = yc - estadoRealAnt[r1].dy * 2 / d1;
		}
		d2 = (abs(estadoRealAnt[r2].dx) + abs(estadoRealAnt[r2].dy)) / 2;
		if (d2 != 0)
		{
			estadoReal[r2].x = xc - estadoRealAnt[r2].dx * 2 / d2;
			estadoReal[r2].y = yc - estadoRealAnt[r2].dy * 2 / d2;
		}
		estadoReal[r1].dx = 0;
		estadoReal[r1].dy = 0;
		estadoReal[r2].dx = 0;
		estadoReal[r2].dy = 0;

		dx = estadoReal[r1].x - estadoReal[r2].x;
		dy = estadoReal[r1].y - estadoReal[r2].y;
		h = sqrt(dx * dx + dy * dy);
		if (h <= 10)
		{
			if (h == 0)
				h = 1;
			dx = 11 * dx / h;
			dy = 11 * dy / h;
			estadoReal[r1].x = xc + dx / 2;
			estadoReal[r1].y = yc + dy / 2;
			estadoReal[r2].x = xc - dx / 2;
			estadoReal[r2].y = yc - dy / 2;
		}
	}
	else
	{
		if (r2 == 3)
		{
			int tmp = r1;
			r1 = r2;
			r2 = tmp;
		}
		xc = (estadoRealAnt[r1].x + estadoRealAnt[r2].x) / 2;
		yc = (estadoRealAnt[r1].y + estadoRealAnt[r2].y) / 2;
		d1 = (abs(estadoRealAnt[r1].dx) + abs(estadoRealAnt[r1].dy)) / 2;
		if (d1 != 0)
		{
			estadoReal[r1].x = xc - estadoRealAnt[r1].dx / d1;
			estadoReal[r1].y = yc - estadoRealAnt[r1].dy / d1;
		}
		d2 = (abs(estadoRealAnt[r2].dx) + abs(estadoRealAnt[r2].dy)) / 2;
		if (d2 != 0)
		{
			estadoReal[r2].x = xc - estadoRealAnt[r2].dx * 2 / d2;
			estadoReal[r2].y = yc - estadoRealAnt[r2].dy * 2 / d2;
		}
		estadoReal[r1].dx = estadoRealAnt[r2].dx - estadoRealAnt[r1].dx / 2;
		estadoReal[r1].dy = estadoRealAnt[r2].dy - estadoRealAnt[r1].dy / 2;
		estadoReal[r2].dx /= 2;
		estadoReal[r2].dy /= 2;

		dx = estadoReal[r1].x - estadoReal[r2].x;
		dy = estadoReal[r1].y - estadoReal[r2].y;
		h = sqrt(dx * dx + dy * dy);
		if (h <= 8)
		{
			if (h == 0)
				h = 1;
			dx = 9 * dx / h;
			dy = 9 * dy / h;
			estadoReal[r1].x = xc + dx / 2;
			estadoReal[r1].y = yc + dy / 2;
			estadoReal[r2].x = xc - dx / 2;
			estadoReal[r2].y = yc - dy / 2;
		}
	}

	CorrigeLimites(r1);
	CorrigeLimites(r2);
}

void DesenhaJogador(int i)
{
	int j;
	float x, y, ang;
	float d1, d2, x1, y1;
	char num[2];

	for (j = 0; j < 7; j++)
	{
		if (VerificaColisao(i, j))
		{
			setColor(LIGHTMAGENTA);
			CorrigePosicao(i, j);
			break;
		}
	}

	x = estadoReal[i].x;
	y = estadoReal[i].y;
	ang = estadoReal[i].angulo;

	d1 = coss(ang) * 8;
	d2 = seno(ang) * 8;

	if (i >= 0 && i <= 2)
	{
		setColor(LIGHTBLUE);
	}
	else if (i >= 4 && i <= 6)
	{
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

void DesenhaJogadorPrev(int i)
{
	int j;
	float x, y, ang;
	float d1, d2, x1, y1;

	setColor(GRAY);

	x = estadoPrev[i].x;
	y = estadoPrev[i].y;
	ang = estadoReal[i].angulo;

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

void DesenhaBola(void)
{
	setColor(LIGHTRED);
	Circle(estadoReal[3].x, estadoReal[3].y, 3);
	setColor(WHITE);
}

void DesenhaBolaPrev(void)
{
	setColor(DARKGRAY);
	Circle(estadoPrev[3].x, estadoPrev[3].y, 3);
	setColor(WHITE);
}

void DesenhaJogo(void)
{
	char num[10];
	int i;

	//  cleardevice();
	//  bar(
	DesenhaCampo();
	for (i = 0; i < 7; i++)
	{
		if (i != 3)
		{
			DesenhaJogadorPrev(i);
			DesenhaJogador(i);
			DesenhaObjetivos(i);
			num[0] = '0' + i;
			if (i == indAtacante)
			{
				num[1] = 'A';
			}
			else if (i == indGoleiro)
			{
				num[1] = 'G';
			}
			else if (i == indVolante)
			{
				num[1] = 'V';
			}
			num[2] = 0;
			OutTextXY(estadoReal[i].x - 2, estadoReal[i].y + 2, num);
		}
		else
		{
			DesenhaBola();
			DesenhaBolaPrev();
		}
		DesenhaPlacar();
	}
}

//-----------------------------------------------------------------------------------------------------------------------------

void *threadFunc(void *);

struct Thread
{
	pthread_t thread;
	int iret;

public:
	Thread()
	{
		iret = pthread_create(&thread, NULL, threadFunc, (void *)this);
		//		printf("iret: %d\n", iret);
	}

	//	static void *threadFunc(void *This) {
	//		printf("$$\n");
	//		((Thread*) This)->run();
	//	}

	virtual void run()
	{
	}
};

void *threadFunc(void *This)
{
	//	printf("$$\n");
	((Thread *)This)->run();
	return NULL;
}

void error(char *msg)
{
	perror(msg);
}

struct DatagramSocket
{
	int sock;
	struct sockaddr_in from;
	long int time_out;

	DatagramSocket(int numSocket)
	{
		int length;
		struct sockaddr_in server;
		time_out = 0;
		sock = socket(AF_INET, SOCK_DGRAM, 0);
		if (sock < 0)
		{
			error("Opening socket");
			exit(0);
		}
		length = sizeof(server);
		memset(&server, 0, length);
		server.sin_family = AF_INET;
		server.sin_addr.s_addr = INADDR_ANY;
		server.sin_port = htons(numSocket);
		if (bind(sock, (struct sockaddr *)&server, length) < 0)
			error("binding");
	}

	void setSoTimeout(int miliSegs)
	{
		time_out = miliSegs;
	}

	bool timeout()
	{
		if (time_out <= 0)
			return false;
		fd_set socks;
		struct timeval t;
		FD_ZERO(&socks);
		FD_SET(sock, &socks);
		t.tv_sec = time_out / 1000;
		t.tv_usec = (time_out % 1000) * 1000;
		int res = select(sock + 1, &socks, NULL, NULL, &t);
		if (res < 0)
		{
			error("select");
			return true;
		}
		bool to = !FD_ISSET(sock, &socks);
		return to;
	}

	bool receive(void *dados, int tam)
	{
		int n;
		if (!timeout())
		{
			socklen_t fromlen = sizeof(struct sockaddr_in);
			//			printf("y\n");
			n = recvfrom(sock, dados, tam, 0, (struct sockaddr *)&from, &fromlen);
			if (n < 0)
			{
				error("recvfrom");
				return false;
			}
			else
				return true;
		}
		return false;
	}

	bool send(void *dados, int tam)
	{
		socklen_t fromlen = sizeof(struct sockaddr_in);
		int n;
		n = sendto(sock, dados, tam, 0, (struct sockaddr *)&from, fromlen);
		if (n < 0)
		{
			//			error("sendto");
			return false;
		}
		return true;
	}
};

struct ComandosTeam1 : public Thread
{
	volatile bool continua;
	unsigned char cmd[6];
	int socket_t;
	DatagramSocket *sockTeam;

	struct EstadoComun
	{
		float angulo;
		float x, y;
		float dAngulo;
		float dx, dy;
	};

	ComandosTeam1(DatagramSocket *_sockTeam, int i)
	{
		continua = true;
		socket_t = i;
		sockTeam = _sockTeam;
	}

	virtual void run()
	{
		EstadoComun est[3];
		printf("Thread %d Iniciada\n", socket_t);
		while (continua)
		{

			if (sockTeam->receive(&cmd, sizeof(cmd)))
			{
				//				printf("RECEBE");
				//				printf("%d = %s", socket_t, cmd);
				sockTeam->send(estado, sizeof(estado));
				//				sockTeam->receive(&cmd, sizeof(cmd));
				//				printf("%d = %s", socket_t, cmd);
			}
		}
	}

	~ComandosTeam1()
	{
		continua = false;
	}
};
struct pos
{
	float x, y;
} posBolaAnt;

struct var_time
{
	int numPenaltis = 0;
	int golsContra = 0;
	int golsDePenaltis = 0;
	int golsRobo[3] = {0, 0, 0};
	int golsContraRobo[3] = {0, 0, 0};
} time1, time2;

void termina_jogo_imprime_estatisticas(FILE *log_Jogo, int countTotalTempo, int numFreeBalls)
{
	log_Jogo = fopen("output.txt", "a+");
	fprintf(log_Jogo, "TERMINA O JOGO AQUI NO ALLIANZ PARQUE -> CPH(SEGUE A BOLA) X CPH(SEGUE A BOLA)");
	fprintf(log_Jogo, "\n--------------------------------------|--------------------------------------\n");
	fprintf(log_Jogo, "PLACAR				| TIME 1 %d%d%d%d 		  X 		%d%d%d%d TIME 2\n", placarTime1.mil, placarTime1.cen, placarTime1.dez, placarTime1.uni,
			placarTime2.mil, placarTime2.cen, placarTime2.dez, placarTime2.uni); //X
	fprintf(log_Jogo, "GOLS CONTRA			| TIME 1     %d 		  X 		   %d 	 TIME 2\n", time1.golsContra, time2.golsContra);
	fprintf(log_Jogo, "PÊNALTIS COMETIDOS	| TIME 1     %d 		  X 		   %d 	 TIME 2\n", time1.numPenaltis, time2.numPenaltis);		  //X
	fprintf(log_Jogo, "GOLS DE PÊNALTIS		| TIME 1     %d 		  X 		   %d 	 TIME 2\n", time1.golsDePenaltis, time2.golsDePenaltis); //X
	fprintf(log_Jogo, "GOLS DO ROBÔ 1 Time 1	| A FAVOR    %d 		  | CONTRA	   %d 	 \n", time1.golsRobo[0], time1.golsContraRobo[0]);
	fprintf(log_Jogo, "GOLS DO ROBÔ 2 Time 1	| A FAVOR    %d 		  | CONTRA	   %d 	 \n", time1.golsRobo[1], time1.golsContraRobo[1]);
	fprintf(log_Jogo, "GOLS DO ROBÔ 3 Time 1	| A FAVOR    %d 		  | CONTRA	   %d 	 \n", time1.golsRobo[2], time1.golsContraRobo[2]);
	fprintf(log_Jogo, "GOLS DO ROBÔ 1 Time 2	| A FAVOR    %d 		  | CONTRA	   %d 	 \n", time2.golsRobo[0], time2.golsContraRobo[0]);
	fprintf(log_Jogo, "GOLS DO ROBÔ 2 Time 2	| A FAVOR    %d 		  | CONTRA	   %d 	 \n", time2.golsRobo[1], time2.golsContraRobo[1]);
	fprintf(log_Jogo, "GOLS DO ROBÔ 3 Time 2	| A FAVOR    %d 		  | CONTRA	   %d 	 \n", time2.golsRobo[2], time2.golsContraRobo[2]);
	fprintf(log_Jogo, "TEMPO DE JOGO			| %d\n", countTotalTempo); //X
	fprintf(log_Jogo, "NÚMERO DE FREEBALLS	| %d\n", numFreeBalls);	//X
	fclose(log_Jogo);
}
//-----------------------------------------------------------------------------------------------------------------------------
int contabiliza_gol(FILE *log_Jogo, Estado estado[7], int golTime)
{
	//	log_Jogo = fopen("output.txt","a+");
	float auxMaisPertoDaBola, maisPertoDaBola;
	int indMaisPertoDaBola;
	maisPertoDaBola = sqrt((estado[3].x - estado[0].x) * (estado[3].x - estado[0].x) + (estado[3].y - estado[0].y) * (estado[3].y - estado[0].y));
	indMaisPertoDaBola = 0;
	//	fprintf(log_Jogo,"golTime : %d\n", golTime);
	if (golTime == 1)
	{
		for (int i = 6; i > 0; i--)
		{
			if (i != 3 && i != 0 && i != 4)
			{
				auxMaisPertoDaBola = sqrt((estado[3].x - estado[i].x) * (estado[3].x - estado[i].x) + (estado[3].y - estado[i].y) * (estado[3].y - estado[i].y));
				if (auxMaisPertoDaBola <= maisPertoDaBola)
				{
					maisPertoDaBola = auxMaisPertoDaBola;
					indMaisPertoDaBola = i;
				}
			}
		}
		//	fprintf(log_Jogo,"maisPertoDaBola : %f\n", maisPertoDaBola);//X
		//	fprintf(log_Jogo,"indMaisPertoDaBola : %d\n", indMaisPertoDaBola);//X

		if (indMaisPertoDaBola > 3)
		{
			time2.golsContra++;
			time2.golsContraRobo[indMaisPertoDaBola - 4]++;
		}
		else
			time1.golsRobo[indMaisPertoDaBola]++;
	}
	else
	{
		for (int i = 1; i < 7; i++)
		{
			if (i != 3 && i != 0 && i != 4)
			{
				auxMaisPertoDaBola = sqrt((estado[3].x - estado[i].x) * (estado[3].x - estado[i].x) + (estado[3].y - estado[i].y) * (estado[3].y - estado[i].y));
				if (auxMaisPertoDaBola <= maisPertoDaBola)
				{
					maisPertoDaBola = auxMaisPertoDaBola;
					indMaisPertoDaBola = i;
				}
			}
		}
		//		fprintf(log_Jogo,"maisPertoDaBola : %f\n", maisPertoDaBola);
		//		fprintf(log_Jogo,"indMaisPertoDaBola : %d\n", indMaisPertoDaBola);//X

		if (indMaisPertoDaBola < 3)
		{
			time1.golsContra++;
			time1.golsContraRobo[indMaisPertoDaBola]++;
		}
		else
			time2.golsRobo[indMaisPertoDaBola - 4]++;
	}
	//	fclose(log_Jogo);
	return 1;
}

void chatterCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    int i = 0;

    ROS_INFO("I heard: %d", i);

    // ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	FILE *log_Jogo;
	bool emPenal = false;
	bool possivelPenalTime1 = false, possivelPenalTime2 = false, imprimeEstatistica = true, posiciona = true;
	int contPenal = 0, numFreeBalls = 0, countTotalTempo = 0, auxCont = 0;
	// log_Jogo = fopen("output.txt", "w");
	// fprintf(log_Jogo, "COMEÇA O JOGO -> CPH(SEGUE A BOLA) X CPH(SEGUE A BOLA)");
	// fclose(log_Jogo);
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

	char c = 'p';
	int i;

	Inicia();

	DatagramSocket *sockTeam1 = new DatagramSocket(7878);
	DatagramSocket *sockTeam2 = new DatagramSocket(7879);

	//	EnviaSerial *es = new EnviaSerial();
	ComandosTeam1 *cr1 = new ComandosTeam1(sockTeam1, 1);
	ComandosTeam1 *cr2 = new ComandosTeam1(sockTeam2, 2);

	//	cr1->run();
	//	cr2->run();
	// Inicia PosiÆo jogadores
	usleep(5000000);
	bool bola_parada = false;
	int count_timer_bola = 0;

	// ros::init(argc, argv, "listener");

	// ros::NodeHandle n;

	// ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, chatterCallback);
	// ROS_INFO("IN ");
	// ros::Duration(1, 0).sleep();
	// ROS_INFO("OUT ");

	while (c != 27)
	{
		countTotalTempo++;
		auxCont++;
		if (auxCont >= 10000)
		{
			imprimeEstatistica = true;
			auxCont = 0;
		}
		if ((((estadoReal[3].x >= posBolaAnt.x - 2 && estadoReal[3].x <= posBolaAnt.x + 2) &&
			  (estadoReal[3].y >= posBolaAnt.y - 2 && estadoReal[3].y <= posBolaAnt.y + 2)) ||
			 (estadoReal[3].x < 20 && estadoReal[3].y < 10) ||
			 (estadoReal[3].x < 20 && estadoReal[3].y > TAM_Y_CAMPO - 10) ||
			 (estadoReal[3].x > TAM_X_CAMPO - 20 && estadoReal[3].y < 10) ||
			 (estadoReal[3].x > TAM_X_CAMPO - 20 && estadoReal[3].y > TAM_Y_CAMPO - 10)

				 ) &&
			emJogo == 1)
		{
			bola_parada = true;
		}
		else
		{
			bola_parada = false;
		}
		if (bola_parada)
		{
			count_timer_bola++;
		}
		else
		{
			count_timer_bola = 0;
		}
		if (emPenal)
		{
			contPenal++;
			if (contPenal >= 1000)
			{
				emPenal = false;
				contPenal = 0;
			}
		}

		Visao();

		for (i = 0; i < 7; i++)
		{
			if (i != 3)
			{
				AtualizaJogador(i);
			}
			else
			{
				AtualizaBola();
			}
		}

		if (quadro == 54)
			cout << "debug";
		//estrategia_connect();
		posBolaAnt.x = estadoReal[3].x;
		posBolaAnt.y = estadoReal[3].y;
		estrategia(cr1->cmd, cr2->cmd);

		for (i = 0; i < 7; i++)
		{
			estado[i].x = estadoReal[i].x;
			estado[i].y = estadoReal[i].y;
			estado[i].dx = estadoReal[i].dx;
			estado[i].dy = estadoReal[i].dy;
			estado[i].angulo = estadoReal[i].angulo;
		}
		//CHAMA ROTINA QUE CONECTA COM CLIENTE
		//		Estrategia1(estado1, estado1Ant, estado1Prev);

		for (i = 0; i < 7; i++)
		{
			//			printf("%3.0f, %3.0f, %3.0f, %3.0f, %3.0f|%3.0f, %3.0f, %3.0f, %3.0f, %3.0f|%3.0f, %3.0f, %3.0f, %3.0f, %3.0f\n", estadoReal[i].x, estadoReal[i].y, estadoReal[i].dx, estadoReal[i].dy, estadoReal[i].angulo, estado[i].x, estado[i].y,
			//					estado[i].dx, estado[i].dy, estado[i].angulo, estadoPrev[i].x, estadoPrev[i].y, estadoPrev[i].dx, estadoPrev[i].dy, estadoPrev[i].angulo);
		}

		DesenhaJogo();
		cvShowImage(wndname, image);

		c = cvWaitKey(33);

		if (imprimeEstatistica && c != 'p')
		{
			c = 'l';
		}
		if (count_timer_bola >= 300)
		{
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
			(((estadoReal[3].x > 0 && estadoReal[3].x < 25) &&
			  (estadoReal[3].y > 30 && estadoReal[3].y < 100)) &&
			 ((estadoReal[4].x > 0 && estadoReal[4].x < 25) &&
			  (estadoReal[4].y > 30 && estadoReal[4].y < 100)) &&
			 ((estadoReal[5].x > 0 && estadoReal[5].x < 25) &&
			  (estadoReal[5].y > 30 && estadoReal[5].y < 100))) ||
			(((estadoReal[3].x > TAM_X_CAMPO - 25 && estadoReal[3].x < TAM_X_CAMPO) &&
			  (estadoReal[3].y > 30 && estadoReal[3].y < 100)) &&
			 ((estadoReal[4].x > TAM_X_CAMPO - 25 && estadoReal[4].x < TAM_X_CAMPO) &&
			  (estadoReal[4].y > 30 && estadoReal[4].y < 100)) &&
			 ((estadoReal[6].x > TAM_X_CAMPO - 25 && estadoReal[6].x < TAM_X_CAMPO) &&
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
			log_Jogo = fopen("output.txt", "a+");
			fprintf(log_Jogo, "\n-----------------------------------------------------------\n");
			fprintf(log_Jogo, "GOOOOL -  TIME 1 %d%d%d%d X %d%d%d%d TIME 2\n", placarTime1.mil, placarTime1.cen, placarTime1.dez, placarTime1.uni,
					placarTime2.mil, placarTime2.cen, placarTime2.dez, placarTime2.uni);
			if (emPenal)
			{
				fprintf(log_Jogo, "DE PÊNALTI!\n");
				time2.golsDePenaltis++;
			}
			fprintf(log_Jogo, "TEMPO DE JOGO: %d\n", countTotalTempo);
			fclose(log_Jogo);
			contabiliza_gol(log_Jogo, estadoReal, 2);
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
			log_Jogo = fopen("output.txt", "a+");
			fprintf(log_Jogo, "\n-----------------------------------------------------------\n");
			fprintf(log_Jogo, "GOOOOL -  TIME 1 %d%d%d%d X %d%d%d%d TIME 2\n", placarTime1.mil, placarTime1.cen, placarTime1.dez, placarTime1.uni,
					placarTime2.mil, placarTime2.cen, placarTime2.dez, placarTime2.uni);
			if (emPenal)
			{
				fprintf(log_Jogo, "DE PÊNALTI!\n");
				time1.golsDePenaltis++;
			}
			fprintf(log_Jogo, "TEMPO DE JOGO: %d\n", countTotalTempo);
			fclose(log_Jogo);
			contabiliza_gol(log_Jogo, estadoReal, 1);
		}

		if (possivelPenalTime1)
		{
			if ( // PENALTI A FAVOR DO TIME 2
				!((estadoReal[3].x > 0 && estadoReal[3].x < 25) &&
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
				!((estadoReal[3].x > TAM_X_CAMPO - 25 && estadoReal[3].x < TAM_X_CAMPO) &&
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

		//		if (countTotalTempo >= 560000){
		//			c = 'i';
		//		}
		switch (c)
		{
		case 'p':
			emJogo = 0;
			estadoReal[0].x = 15;
			estadoReal[0].y = 65;
			estadoReal[0].angulo = 270;
			estadoReal[0].dx = 0;
			estadoReal[0].dy = 0;
			estadoReal[1].x = 47.5;
			estadoReal[1].y = 65;
			estadoReal[1].angulo = 270;
			estadoReal[1].dx = 0;
			estadoReal[1].dy = 0;
			estadoReal[2].x = TAM_X_CAMPO / 2 - 8;
			estadoReal[2].y = 65;
			estadoReal[2].angulo = 0;
			estadoReal[2].dx = 0;
			estadoReal[2].dy = 0;
			estadoReal[3].x = TAM_X_CAMPO / 2;
			estadoReal[3].y = 65;
			estadoReal[3].angulo = 0;
			estadoReal[3].dx = 0;
			estadoReal[3].dy = 0;
			estadoReal[4].x = TAM_X_CAMPO - 15;
			estadoReal[4].y = 65;
			estadoReal[4].angulo = 90;
			estadoReal[4].dx = 0;
			estadoReal[4].dy = 0;
			estadoReal[5].x = 47.5 + 75;
			estadoReal[5].y = 65;
			estadoReal[5].angulo = 90;
			estadoReal[5].dx = 0;
			estadoReal[5].dy = 0;
			estadoReal[6].x = TAM_X_CAMPO / 2 + 20;
			estadoReal[6].y = 65;
			estadoReal[6].angulo = 180;
			estadoReal[6].dx = 0;
			estadoReal[6].dy = 0;

			for (i = 0; i < 7; i++)
			{
				estado[i].x = estadoReal[i].x;
				estado[i].y = estadoReal[i].y;
				estado[i].dx = estadoReal[i].dx;
				estado[i].dy = estadoReal[i].dy;
				estado[i].angulo = estadoReal[i].angulo;
			}
			memcpy(estadoAnt, estado, sizeof(Estado[7]));
			memcpy(estadoPrev, estado, sizeof(Estado[7]));
			memcpy(estadoRealAnt, estadoReal, sizeof(estadoReal[7]));

			memset(vdAnt, 0, sizeof(float[3][3]));
			memset(veAnt, 0, sizeof(float[3][3]));
			memset(fdang, 0, sizeof(float[3]));
			memset(fdx, 0, sizeof(float[3]));
			memset(fdy, 0, sizeof(float[3]));

			PosicionaRobos();
			PosicionaRobos1();

			for (i = 0; i < 7; i++)
			{
				int j;
				for (j = 0; j < 3; j++)
				{
					xVisao[i][j] = estado[i].x;
					yVisao[i][j] = estado[i].y;
					angVisao[i][j] = estado[i].angulo;
				}
			}
			//         srand(1);
			quadro = 0;
			emJogo = 1;
			c = 0;
			break;
		case 'P':
			emJogo = 0;
			estadoReal[0].x = 15;
			estadoReal[0].y = 65;
			estadoReal[0].angulo = 270;
			estadoReal[0].dx = 0;
			estadoReal[0].dy = 0;
			estadoReal[1].x = 47.5;
			estadoReal[1].y = 65;
			estadoReal[1].angulo = 270;
			estadoReal[1].dx = 0;
			estadoReal[1].dy = 0;
			estadoReal[2].x = TAM_X_CAMPO / 2 - 20;
			estadoReal[2].y = 65;
			estadoReal[2].angulo = 0;
			estadoReal[2].dx = 0;
			estadoReal[2].dy = 0;
			estadoReal[3].x = TAM_X_CAMPO / 2;
			estadoReal[3].y = 65;
			estadoReal[3].angulo = 0;
			estadoReal[3].dx = 0;
			estadoReal[3].dy = 0;
			estadoReal[4].x = TAM_X_CAMPO - 15;
			estadoReal[4].y = 65;
			estadoReal[4].angulo = 90;
			estadoReal[4].dx = 0;
			estadoReal[4].dy = 0;
			estadoReal[5].x = 47.5 + 75;
			estadoReal[5].y = 65;
			estadoReal[5].angulo = 90;
			estadoReal[5].dx = 0;
			estadoReal[5].dy = 0;
			estadoReal[6].x = TAM_X_CAMPO / 2 + 8;
			estadoReal[6].y = 65;
			estadoReal[6].angulo = 180;
			estadoReal[6].dx = 0;
			estadoReal[6].dy = 0;

			for (i = 0; i < 7; i++)
			{
				estado[i].x = estadoReal[i].x;
				estado[i].y = estadoReal[i].y;
				estado[i].dx = estadoReal[i].dx;
				estado[i].dy = estadoReal[i].dy;
				estado[i].angulo = estadoReal[i].angulo;
			}
			memcpy(estadoAnt, estado, sizeof(Estado[7]));
			memcpy(estadoPrev, estado, sizeof(Estado[7]));
			memcpy(estadoRealAnt, estadoReal, sizeof(estadoReal[7]));

			memset(vdAnt, 0, sizeof(float[3][3]));
			memset(veAnt, 0, sizeof(float[3][3]));
			memset(fdang, 0, sizeof(float[3]));
			memset(fdx, 0, sizeof(float[3]));
			memset(fdy, 0, sizeof(float[3]));

			PosicionaRobos();
			PosicionaRobos1();

			for (i = 0; i < 7; i++)
			{
				int j;
				for (j = 0; j < 3; j++)
				{
					xVisao[i][j] = estado[i].x;
					yVisao[i][j] = estado[i].y;
					angVisao[i][j] = estado[i].angulo;
				}
			}
			//         srand(1);
			quadro = 0;
			emJogo = 1;
			c = 0;
			break;
		case 'j':
		case 'J':
			emJogo = 1;
			indGoleiro = 0;
			indAtacante = 2;
			indVolante = 1;
			c = 0;
			break;
		case 'l':
		case 'L':
			termina_jogo_imprime_estatisticas(log_Jogo, countTotalTempo, numFreeBalls);
			imprimeEstatistica = false;
			c = 0;
			break;
		case 'k':
			emJogo = 0;
			estadoReal[0].x = 15;
			estadoReal[0].y = 65;
			estadoReal[0].angulo = 270;
			estadoReal[0].dx = 0;
			estadoReal[0].dy = 0;
			estadoReal[1].x = TAM_X_CAMPO / 2 + 10;
			estadoReal[1].y = 80;
			estadoReal[1].angulo = 270;
			estadoReal[1].dx = 0;
			estadoReal[1].dy = 0;
			estadoReal[2].x = TAM_X_CAMPO / 2 + 10;
			estadoReal[2].y = 65;
			estadoReal[2].angulo = 0;
			estadoReal[2].dx = 0;
			estadoReal[2].dy = 0;
			estadoReal[3].x = 47.5;
			estadoReal[3].y = 65;
			estadoReal[3].angulo = 0;
			estadoReal[3].dx = 0;
			estadoReal[3].dy = 0;
			estadoReal[4].x = TAM_X_CAMPO - 15;
			estadoReal[4].y = 65;
			estadoReal[4].angulo = 90;
			estadoReal[4].dx = 0;
			estadoReal[4].dy = 0;
			estadoReal[5].x = TAM_X_CAMPO / 2 + 20;
			estadoReal[5].y = 65;
			estadoReal[5].angulo = 180;
			estadoReal[5].dx = 0;
			estadoReal[5].dy = 0;
			estadoReal[6].x = 47.5 + 8;
			estadoReal[6].y = 65;
			estadoReal[6].angulo = 180;
			estadoReal[6].dx = 0;
			estadoReal[6].dy = 0;
			emJogo = 1;

			for (i = 0; i < 7; i++)
			{
				estado[i].x = estadoReal[i].x;
				estado[i].y = estadoReal[i].y;
				estado[i].dx = estadoReal[i].dx;
				estado[i].dy = estadoReal[i].dy;
				estado[i].angulo = estadoReal[i].angulo;
			}

			memcpy(estadoAnt, estado, sizeof(Estado[7]));
			memcpy(estadoPrev, estado, sizeof(Estado[7]));
			memcpy(estadoRealAnt, estadoReal, sizeof(estadoReal[7]));

			memset(vdAnt, 0, sizeof(float[3][3]));
			memset(veAnt, 0, sizeof(float[3][3]));
			memset(fdang, 0, sizeof(float[3]));
			memset(fdx, 0, sizeof(float[3]));
			memset(fdy, 0, sizeof(float[3]));

			PosicionaRobos();
			PosicionaRobos1();

			for (i = 0; i < 7; i++)
			{
				int j;
				for (j = 0; j < 3; j++)
				{
					xVisao[i][j] = estado[i].x;
					yVisao[i][j] = estado[i].y;
					angVisao[i][j] = estado[i].angulo;
				}
			}
			c = 0;
			break;
		case 'K':
			emJogo = 0;
			estadoReal[0].x = 15;
			estadoReal[0].y = 65;
			estadoReal[0].angulo = 270;
			estadoReal[0].dx = 0;
			estadoReal[0].dy = 0;
			estadoReal[1].x = TAM_X_CAMPO / 2 - 20;
			estadoReal[1].y = 80;
			estadoReal[1].angulo = 270;
			estadoReal[1].dx = 0;
			estadoReal[1].dy = 0;
			estadoReal[2].x = TAM_X_CAMPO - 55.5;
			estadoReal[2].y = 65;
			estadoReal[2].angulo = 0;
			estadoReal[2].dx = 0;
			estadoReal[2].dy = 0;
			estadoReal[3].x = TAM_X_CAMPO - 47.5;
			estadoReal[3].y = 65;
			estadoReal[3].angulo = 0;
			estadoReal[3].dx = 0;
			estadoReal[3].dy = 0;
			estadoReal[4].x = TAM_X_CAMPO - 15;
			estadoReal[4].y = 65;
			estadoReal[4].angulo = 90;
			estadoReal[4].dx = 0;
			estadoReal[4].dy = 0;
			estadoReal[5].x = TAM_X_CAMPO / 2 - 10;
			estadoReal[5].y = 65;
			estadoReal[5].angulo = 180;
			estadoReal[5].dx = 0;
			estadoReal[5].dy = 0;
			estadoReal[6].x = TAM_X_CAMPO / 2 - 10;
			estadoReal[6].y = 80;
			estadoReal[6].angulo = 180;
			estadoReal[6].dx = 0;
			estadoReal[6].dy = 0;
			emJogo = 1;

			for (i = 0; i < 7; i++)
			{
				estado[i].x = estadoReal[i].x;
				estado[i].y = estadoReal[i].y;
				estado[i].dx = estadoReal[i].dx;
				estado[i].dy = estadoReal[i].dy;
				estado[i].angulo = estadoReal[i].angulo;
			}

			memcpy(estadoAnt, estado, sizeof(Estado[7]));
			memcpy(estadoPrev, estado, sizeof(Estado[7]));
			memcpy(estadoRealAnt, estadoReal, sizeof(estadoReal[7]));

			memset(vdAnt, 0, sizeof(float[3][3]));
			memset(veAnt, 0, sizeof(float[3][3]));
			memset(fdang, 0, sizeof(float[3]));
			memset(fdx, 0, sizeof(float[3]));
			memset(fdy, 0, sizeof(float[3]));

			PosicionaRobos();
			PosicionaRobos1();

			for (i = 0; i < 7; i++)
			{
				int j;
				for (j = 0; j < 3; j++)
				{
					xVisao[i][j] = estado[i].x;
					yVisao[i][j] = estado[i].y;
					angVisao[i][j] = estado[i].angulo;
				}
			}
			c = 0;
			break;
			//freeball7
		case 'f':
		case 'F':
			emJogo = 0;
			if (estadoReal[3].x <= 75 && estadoReal[3].y <= 65)
			{
				estadoReal[0].x = 15;
				estadoReal[0].y = 50;
				estadoReal[0].angulo = 270;
				estadoReal[0].dx = 0;
				estadoReal[0].dy = 0;
				estadoReal[1].x = 47.5;
				estadoReal[1].y = 65;
				estadoReal[1].angulo = 270;
				estadoReal[1].dx = 0;
				estadoReal[1].dy = 0;
				estadoReal[2].x = 47.5 - 20;
				estadoReal[2].y = 25;
				estadoReal[2].angulo = 0;
				estadoReal[2].dx = 0;
				estadoReal[2].dy = 0;
				estadoReal[3].x = 47.5;
				estadoReal[3].y = 25;
				estadoReal[3].angulo = 0;
				estadoReal[3].dx = 0;
				estadoReal[3].dy = 0;
				estadoReal[4].x = TAM_X_CAMPO - 15;
				estadoReal[4].y = 65;
				estadoReal[4].angulo = 90;
				estadoReal[4].dx = 0;
				estadoReal[4].dy = 0;
				estadoReal[5].x = 47.5 + 75;
				estadoReal[5].y = 65;
				estadoReal[5].angulo = 180;
				estadoReal[5].dx = 0;
				estadoReal[5].dy = 0;
				estadoReal[6].x = 47.5 + 20;
				estadoReal[6].y = 25;
				estadoReal[6].angulo = 180;
				estadoReal[6].dx = 0;
				estadoReal[6].dy = 0;
			}
			else if (estadoReal[3].x <= TAM_X_CAMPO - 10 && estadoReal[3].y < 65)
			{
				estadoReal[0].x = 15;
				estadoReal[0].y = 65;
				estadoReal[0].angulo = 270;
				estadoReal[0].dx = 0;
				estadoReal[0].dy = 0;
				estadoReal[1].x = 47.5;
				estadoReal[1].y = 65;
				estadoReal[1].angulo = 270;
				estadoReal[1].dx = 0;
				estadoReal[1].dy = 0;
				estadoReal[2].x = TAM_X_CAMPO - 47.5 - 20;
				estadoReal[2].y = 25;
				estadoReal[2].angulo = 0;
				estadoReal[2].dx = 0;
				estadoReal[2].dy = 0;
				estadoReal[3].x = 47.5 + 75;
				estadoReal[3].y = 25;
				estadoReal[3].angulo = 0;
				estadoReal[3].dx = 0;
				estadoReal[3].dy = 0;
				estadoReal[4].x = TAM_X_CAMPO - 15;
				estadoReal[4].y = 50;
				estadoReal[4].angulo = 90;
				estadoReal[4].dx = 0;
				estadoReal[4].dy = 0;
				estadoReal[5].x = 47.5 + 75;
				estadoReal[5].y = 65;
				estadoReal[5].angulo = 180;
				estadoReal[5].dx = 0;
				estadoReal[5].dy = 0;
				estadoReal[6].x = TAM_X_CAMPO - 47.5 + 20;
				estadoReal[6].y = 25;
				estadoReal[6].angulo = 180;
				estadoReal[6].dx = 0;
				estadoReal[6].dy = 0;
			}
			else if (estadoReal[3].x < 75 && estadoReal[3].y <= TAM_Y_CAMPO)
			{
				estadoReal[0].x = 15;
				estadoReal[0].y = 80;
				estadoReal[0].angulo = 270;
				estadoReal[0].dx = 0;
				estadoReal[0].dy = 0;
				estadoReal[1].x = 47.5;
				estadoReal[1].y = 65;
				estadoReal[1].angulo = 270;
				estadoReal[1].dx = 0;
				estadoReal[1].dy = 0;
				estadoReal[2].x = 47.5 - 20;
				estadoReal[2].y = TAM_Y_CAMPO - 25;
				estadoReal[2].angulo = 0;
				estadoReal[2].dx = 0;
				estadoReal[2].dy = 0;
				estadoReal[3].x = 47.5;
				estadoReal[3].y = TAM_Y_CAMPO - 25;
				estadoReal[3].angulo = 0;
				estadoReal[3].dx = 0;
				estadoReal[3].dy = 0;
				estadoReal[4].x = TAM_X_CAMPO - 15;
				estadoReal[4].y = 65;
				estadoReal[4].angulo = 90;
				estadoReal[4].dx = 0;
				estadoReal[4].dy = 0;
				estadoReal[5].x = 47.5 + 75;
				estadoReal[5].y = 65;
				estadoReal[5].angulo = 180;
				estadoReal[5].dx = 0;
				estadoReal[5].dy = 0;
				estadoReal[6].x = 47.5 + 20;
				estadoReal[6].y = TAM_Y_CAMPO - 25;
				estadoReal[6].angulo = 180;
				estadoReal[6].dx = 0;
				estadoReal[6].dy = 0;
			}
			else
			{
				estadoReal[0].x = 15;
				estadoReal[0].y = 65;
				estadoReal[0].angulo = 270;
				estadoReal[0].dx = 0;
				estadoReal[0].dy = 0;
				estadoReal[1].x = 47.5;
				estadoReal[1].y = 65;
				estadoReal[1].angulo = 270;
				estadoReal[1].dx = 0;
				estadoReal[1].dy = 0;
				estadoReal[2].x = TAM_X_CAMPO - 47.5 - 20;
				estadoReal[2].y = TAM_Y_CAMPO - 25;
				estadoReal[2].angulo = 0;
				estadoReal[2].dx = 0;
				estadoReal[2].dy = 0;
				estadoReal[3].x = TAM_X_CAMPO - 47.5;
				estadoReal[3].y = TAM_Y_CAMPO - 25;
				estadoReal[3].angulo = 0;
				estadoReal[3].dx = 0;
				estadoReal[3].dy = 0;
				estadoReal[4].x = TAM_X_CAMPO - 15;
				estadoReal[4].y = 80;
				estadoReal[4].angulo = 90;
				estadoReal[4].dx = 0;
				estadoReal[4].dy = 0;
				estadoReal[5].x = 47.5 + 75;
				estadoReal[5].y = 65;
				estadoReal[5].angulo = 180;
				estadoReal[5].dx = 0;
				estadoReal[5].dy = 0;
				estadoReal[6].x = TAM_X_CAMPO - 47.5 + 20;
				estadoReal[6].y = TAM_Y_CAMPO - 25;
				estadoReal[6].angulo = 180;
				estadoReal[6].dx = 0;
				estadoReal[6].dy = 0;
			}
			for (i = 0; i < 7; i++)
			{
				estado[i].x = estadoReal[i].x;
				estado[i].y = estadoReal[i].y;
				estado[i].dx = estadoReal[i].dx;
				estado[i].dy = estadoReal[i].dy;
				estado[i].angulo = estadoReal[i].angulo;
			}
			emJogo = 1;
			numFreeBalls++;
			memcpy(estadoAnt, estado, sizeof(Estado[7]));
			memcpy(estadoPrev, estado, sizeof(Estado[7]));
			memcpy(estadoRealAnt, estadoReal, sizeof(estadoReal[7]));

			memset(vdAnt, 0, sizeof(float[3][3]));
			memset(veAnt, 0, sizeof(float[3][3]));
			memset(fdang, 0, sizeof(float[3]));
			memset(fdx, 0, sizeof(float[3]));
			memset(fdy, 0, sizeof(float[3]));

			PosicionaRobos();
			PosicionaRobos1();

			for (i = 0; i < 7; i++)
			{
				int j;
				for (j = 0; j < 3; j++)
				{
					xVisao[i][j] = estado[i].x;
					yVisao[i][j] = estado[i].y;
					angVisao[i][j] = estado[i].angulo;
				}
			}
			c = 0;
			break;
		case 'i':
		case 'I':
			emJogo = 0;
			estadoReal[0].x = 15;
			estadoReal[0].y = 65;
			estadoReal[0].angulo = 270;
			estadoReal[0].dx = 0;
			estadoReal[0].dy = 0;
			estadoReal[1].x = 47.5;
			estadoReal[1].y = 65;
			estadoReal[1].angulo = 270;
			estadoReal[1].dx = 0;
			estadoReal[1].dy = 0;
			estadoReal[2].x = TAM_X_CAMPO / 2 - 8;
			estadoReal[2].y = 65;
			estadoReal[2].angulo = 0;
			estadoReal[2].dx = 0;
			estadoReal[2].dy = 0;
			estadoReal[3].x = TAM_X_CAMPO / 2;
			estadoReal[3].y = 65;
			estadoReal[3].angulo = 0;
			estadoReal[3].dx = 0;
			estadoReal[3].dy = 0;
			estadoReal[4].x = TAM_X_CAMPO - 15;
			estadoReal[4].y = 65;
			estadoReal[4].angulo = 90;
			estadoReal[4].dx = 0;
			estadoReal[4].dy = 0;
			estadoReal[5].x = 47.5 + 75;
			estadoReal[5].y = 65;
			estadoReal[5].angulo = 90;
			estadoReal[5].dx = 0;
			estadoReal[5].dy = 0;
			estadoReal[6].x = TAM_X_CAMPO / 2 + 20;
			estadoReal[6].y = 65;
			estadoReal[6].angulo = 180;
			estadoReal[6].dx = 0;
			estadoReal[6].dy = 0;

			for (i = 0; i < 7; i++)
			{
				estado[i].x = estadoReal[i].x;
				estado[i].y = estadoReal[i].y;
				estado[i].dx = estadoReal[i].dx;
				estado[i].dy = estadoReal[i].dy;
				estado[i].angulo = estadoReal[i].angulo;
			}
			memcpy(estadoAnt, estado, sizeof(Estado[7]));
			memcpy(estadoPrev, estado, sizeof(Estado[7]));
			memcpy(estadoRealAnt, estadoReal, sizeof(estadoReal[7]));

			memset(vdAnt, 0, sizeof(float[3][3]));
			memset(veAnt, 0, sizeof(float[3][3]));
			memset(fdang, 0, sizeof(float[3]));
			memset(fdx, 0, sizeof(float[3]));
			memset(fdy, 0, sizeof(float[3]));

			PosicionaRobos();
			PosicionaRobos1();

			for (i = 0; i < 7; i++)
			{
				int j;
				for (j = 0; j < 3; j++)
				{
					xVisao[i][j] = estado[i].x;
					yVisao[i][j] = estado[i].y;
					angVisao[i][j] = estado[i].angulo;
				}
			}
			//         srand(1);
			quadro = 0;
			c = 0;
			break;
		case 'd':
		case 'D':
			cout << "debug";
			c = 0;
			break;
		}
	}

	// ros::spin();

	cvReleaseImage(&image);
	cvDestroyWindow(wndname);
	pthread_join(cr1->thread, NULL);
	pthread_join(cr2->thread, NULL);
	//	pthread_join(es->thread, NULL);
	return 0;
}

#ifdef _EiC
main(1, "drawing.c");
#endif
