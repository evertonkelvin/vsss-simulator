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
#include "Estrategia.h"

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

bool emJogo = false;

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
extern bool voltando;

Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1], estadoPrev[NUM_ROBOS_TIME * 2 + 1];
Estado estado1[NUM_ROBOS_TIME * 2 + 1], estado1Ant[NUM_ROBOS_TIME * 2 + 1], estado1Prev[NUM_ROBOS_TIME * 2 + 1];
Estado estadoReal[NUM_ROBOS_TIME * 2 + 1], estadoRealAnt[NUM_ROBOS_TIME * 2 + 1];

Objetivo objetivoRobo[NUM_ROBOS_TIME];

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

char wndname[] = "Simulador";

void PosicionaRobos(void) {
}

float xVisao[7][3], yVisao[7][3], angVisao[7][3];
void Visao(void) {
	int x, y, ang;
	int i;
	// ------------- nosso time
	for (i = 0; i < 7; i++) {
		estadoAnt[i] = estado[i];
		if (emJogo) {
#ifdef RANDOMICO
			x=estadoReal[i].x+random()%3-1; //coordenada obtida da camera
			y=estadoReal[i].y+random()%3-1;
			ang=estadoReal[i].angulo+random()%3-1;
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

	for (i = 0; i < 7; i++) {
		estado1[i].x = 170 - (estadoReal[6 - i].x); //+random(3)-1);
		estado1[i].y = 130 - (estadoReal[6 - i].y); //+random(3)-1);
		estado1[i].dx = estado1[i].x - estado1Ant[i].x;
		estado1[i].dy = estado1[i].y - estado1Ant[i].y;
		estado1[i].angulo = atang2(estado1[i].dy, estado1[i].dx);
	}
	printf("%5d\n", quadro++);
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
	if (b1 == 0x81) { //0x81 nao usado na comunicacao real,
		memset(vdAnt, 0, sizeof vdAnt); //usado aqui para limpar a historia
		memset(veAnt, 0, sizeof vdAnt);
	}
	printf("%3d, %3d, %3d, %3d, %3d, %3d\n", cnvCmd(b1), cnvCmd(b2), cnvCmd(b3), cnvCmd(b4), cnvCmd(b5), cnvCmd(b6));
	b[0] = (b1 >> 4) & 0xf;
	b[1] = (b2 >> 4) & 0xf;
	b[2] = (b3 >> 4) & 0xf;
	b[3] = (b4 >> 4) & 0xf;
	b[4] = (b5 >> 4) & 0xf;
	b[5] = (b6 >> 4) & 0xf;
	for (i = 0; i < 3; i++) {
		if (b[i * 2] & 0x8)
			ve = -(b[i * 2] & 0x7);
		else
			ve = b[i * 2] & 0x7;

		if (b[i * 2 + 1] & 0x8)
			vd = -(b[i * 2 + 1] & 0x7);
		else
			vd = b[i * 2 + 1] & 0x7;

		for (j = TAM_HISTORIA - 1; j > 0; j--) {
			vdAnt[i][j] = vdAnt[i][j - 1];
			veAnt[i][j] = veAnt[i][j - 1];
		}

		vdAnt[i][0] = vd * VEL_MAXIMA_ROBO_CM / VEL_MAXIMA_ROBO_UNID / 30;
		veAnt[i][0] = ve * VEL_MAXIMA_ROBO_CM / VEL_MAXIMA_ROBO_UNID / 30;

		vd = vdAnt[i][TEMPO_ATRASO]; //atraso entre a visao e a realizacao do comando
		ve = veAnt[i][TEMPO_ATRASO];

		float somaVd = 0, somaVe = 0, cont = 0;
		for (j = TEMPO_ATRASO; j < TAM_INERCIA + TEMPO_ATRASO; j++) {
			somaVd += vdAnt[i][j];
			somaVe += veAnt[i][j];
			cont++;
		}

		vd = somaVd / cont;
		ve = somaVe / cont;

		//estadoReal[i].dAngulo = dang = (vd - ve) / DIST_ENTRE_RODAS * 180 / 3.14;***
		estadoReal[i].dAngulo = dang = atan2((vd - ve), DIST_ENTRE_RODAS) * 180 / 3.14;
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
//		estadoReal[i].x += estadoReal[i].dx;
//		estadoReal[i].y += estadoReal[i].dy;

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

void CorrigeLimites(int i) {
	if (i != 3) {
		if (41 < estadoReal[i].y && estadoReal[i].y < 89) {
			if (estadoReal[i].x < 4) {
				estadoReal[i].x = 4;
			} else if (estadoReal[i].x > 166) {
				estadoReal[i].x = 166;
			}
		} else {
			if (estadoReal[i].x < 14) {
				estadoReal[i].x = 14;
			} else if (estadoReal[i].x > 156) {
				estadoReal[i].x = 156;
			}
			if (estadoReal[i].y < 4) {
				estadoReal[i].y = 4;
			} else if (estadoReal[i].y > 126) {
				estadoReal[i].y = 126;
			}
		}
	} else { // é a bola
		int dx = estadoReal[i].dx;
		int dy = estadoReal[i].dy;
		if (42 < estadoReal[i].y && estadoReal[i].y < 87) { // bola na direcao (y) dos limites do gol?
			if (estadoReal[i].x < 3) { //bola no fundo do gol?
				estadoReal[i].x = 3;
				estadoReal[i].dx = -dx - 1;
			} else if (estadoReal[i].x > 167) { //bola no fundo do gol?
				estadoReal[i].x = 167;
				estadoReal[i].dx = -dx + 1;
			}
		} else { //bola fora da direcao do gol
			if (estadoReal[i].x - estadoReal[i].y > 151 || // canto inferior direito
					estadoReal[i].x - estadoReal[i].y < -110 || // canto superio esquerdo
					estadoReal[i].x + estadoReal[i].y > 280 || // canto superio direito
					estadoReal[i].x + estadoReal[i].y < 20) { // canto inferior esquerdo
				while (estadoReal[i].x - estadoReal[i].y > 151) {
					estadoReal[i].x--;
					estadoReal[i].y++;
				}
				while (estadoReal[i].x - estadoReal[i].y < -110) {
					estadoReal[i].x++;
					estadoReal[i].y--;
				}
				while (estadoReal[i].x + estadoReal[i].y > 280) {
					estadoReal[i].x--;
					estadoReal[i].y--;
				}
				while (estadoReal[i].x + estadoReal[i].y < 20) {
					estadoReal[i].x++;
					estadoReal[i].y++;
				}
				if (abs(dx) > abs(dy)) {
					dx > 0 ? dx-- : dx++;
				} else if (abs(dx) < abs(dy)) {
					dy > 0 ? dy-- : dy++;
				} else if (abs(dx) > 0) {
					dx > 0 ? dx-- : dx++;
					dy > 0 ? dy-- : dy++;
				}
				if (estadoReal[i].x + estadoReal[i].y > 280 || // canto superio direito
						estadoReal[i].x + estadoReal[i].y < 20) {
					estadoReal[i].dx = -dy;
					estadoReal[i].dy = -dx;
				} else {
					estadoReal[i].dx = dy;
					estadoReal[i].dy = dx;
				}
				return;
			}
			if (estadoReal[i].x < 13) { //linha de fundo?
				estadoReal[i].x = 13;
				estadoReal[i].dx = -dx - 1;
			} else if (estadoReal[i].x > 157) { //linha de fundo?
				estadoReal[i].x = 157;
				estadoReal[i].dx = -dx + 1;
			}
			if (estadoReal[i].y < 3) { //na lateral?
				estadoReal[i].y = 3;
				estadoReal[i].dy = -dy - 1;
			} else if (estadoReal[i].y > 127) { //na lateral?
				estadoReal[i].y = 127;
				estadoReal[i].dy = -dy + 1;
			}
		}
	}
}

void AtualizaJogador(int i) {
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

void AtualizaBola(void) {
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

int VerificaColisao(int r1, int r2) {
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

	if (estadoReal[r1].x > estadoRealAnt[r1].x) { //teste pelo cruzamento
		x1r1 = estadoRealAnt[r1].x;
		x2r1 = estadoReal[r1].x;
	} else {
		x1r1 = estadoReal[r1].x;
		x2r1 = estadoRealAnt[r1].x;
	}
	if (estadoReal[r2].x > estadoRealAnt[r2].x) {
		x1r2 = estadoRealAnt[r2].x;
		x2r2 = estadoReal[r2].x;
	} else {
		x1r2 = estadoReal[r2].x;
		x2r2 = estadoRealAnt[r2].x;
	}
	if (x1r1 < x2r2) {
		if (x2r1 > x2r2 || x2r1 > x1r2) {
			Colisaox = 1;
		}
	}
	if (estadoReal[r1].y > estadoRealAnt[r1].y) { //teste pelo cruzamento
		y1r1 = estadoRealAnt[r1].y;
		y2r1 = estadoReal[r1].y;
	} else {
		y1r1 = estadoReal[r1].y;
		y2r1 = estadoRealAnt[r1].y;
	}
	if (estadoReal[r2].y > estadoRealAnt[r2].y) {
		y1r2 = estadoRealAnt[r2].y;
		y2r2 = estadoReal[r2].y;
	} else {
		y1r2 = estadoReal[r2].y;
		y2r2 = estadoRealAnt[r2].y;
	}
	if (y1r1 < y2r2) {
		if (y2r1 > y2r2 || y2r1 > y1r2) {
			Colisaoy = 1;
		}
	}

	if (abs(x2r1 - x2r2) < d && abs(y2r1 - y2r2) < d) { //teste pela distancia
		Colisaox = 1;
		Colisaoy = 1;
	}
	return Colisaox && Colisaoy;
}

void CorrigePosicao(int r1, int r2) { //corrige colisao
	int xc, yc;
	int d1, d2, dx, dy, h;

	if (r1 != 3 && r2 != 3) {
		xc = (estadoRealAnt[r1].x + estadoRealAnt[r2].x) / 2;
		yc = (estadoRealAnt[r1].y + estadoRealAnt[r2].y) / 2;
		d1 = (abs(estadoRealAnt[r1].dx) + abs(estadoRealAnt[r1].dy)) / 2;
		if (d1 != 0) {
			estadoReal[r1].x = xc - estadoRealAnt[r1].dx * 2 / d1;
			estadoReal[r1].y = yc - estadoRealAnt[r1].dy * 2 / d1;
		}
		d2 = (abs(estadoRealAnt[r2].dx) + abs(estadoRealAnt[r2].dy)) / 2;
		if (d2 != 0) {
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
		if (h <= 10) {
			if (h == 0)
				h = 1;
			dx = 11 * dx / h;
			dy = 11 * dy / h;
			estadoReal[r1].x = xc + dx / 2;
			estadoReal[r1].y = yc + dy / 2;
			estadoReal[r2].x = xc - dx / 2;
			estadoReal[r2].y = yc - dy / 2;
		}
	} else {
		if (r2 == 3) {
			int tmp = r1;
			r1 = r2;
			r2 = tmp;
		}
		xc = (estadoRealAnt[r1].x + estadoRealAnt[r2].x) / 2;
		yc = (estadoRealAnt[r1].y + estadoRealAnt[r2].y) / 2;
		d1 = (abs(estadoRealAnt[r1].dx) + abs(estadoRealAnt[r1].dy)) / 2;
		if (d1 != 0) {
			estadoReal[r1].x = xc - estadoRealAnt[r1].dx / d1;
			estadoReal[r1].y = yc - estadoRealAnt[r1].dy / d1;
		}
		d2 = (abs(estadoRealAnt[r2].dx) + abs(estadoRealAnt[r2].dy)) / 2;
		if (d2 != 0) {
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
		if (h <= 8) {
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

void DesenhaJogador(int i) {
	int j;
	float x, y, ang;
	float d1, d2, x1, y1;
	char num[2];

	for (j = 0; j < 7; j++) {
		if (VerificaColisao(i, j)) {
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
	int j;
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
	int j;
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

	//  cleardevice();
	//  bar(
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

int main(int argc, char** argv) {
    	image = cvCreateImage(cvSize(680, 520), 8, 3);
	// Create a window
	cvNamedWindow(wndname, 1);
	cvZero(image);
	cvShowImage(wndname, image);

	char c;
	int i;

	Inicia();

	// Inicia Posicao jogadores
	c = 0;
	while (c != 27) {
		int indiceMovimentoTeclado = 3;
		Visao();

		for (i = 0; i < 7; i++) {
			if (i != 3) {
				AtualizaJogador(i);
			} else {
				AtualizaBola();
			}
		}

		if (voltando)
   			cout << "voltando";
		else
   			cout << "atacando";
		if (quadro == 110)
       			cout << "debug";
		estrategia();
		//		Estrategia1(estado1, estado1Ant, estado1Prev);

		for (i = 0; i < 7; i++) {
			printf("%3.0f, %3.0f, %3.0f, %3.0f, %3.0f|%3.0f, %3.0f, %3.0f, %3.0f, %3.0f|%3.0f, %3.0f, %3.0f, %3.0f, %3.0f\n", estadoReal[i].x, estadoReal[i].y, estadoReal[i].dx, estadoReal[i].dy, estadoReal[i].angulo, estado[i].x, estado[i].y,
					estado[i].dx, estado[i].dy, estado[i].angulo, estadoPrev[i].x, estadoPrev[i].y, estadoPrev[i].dx, estadoPrev[i].dy, estadoPrev[i].angulo);
		}

		for (;;) {
			DesenhaJogo();
			cvShowImage(wndname, image);
			fflush(stdout);
			c = cvWaitKey();
			printf("(%d)", c);
			switch (c) {
			case '0':
			case '1':
			case '2':
			case '3':
				indiceMovimentoTeclado = c - '0';
				continue;
			case '.':
				estadoReal[indiceMovimentoTeclado].dx = 0;
				estadoReal[indiceMovimentoTeclado].dy = 0;
				break;
			case 81: //esquerda
				estadoReal[indiceMovimentoTeclado].x -= 10;
				continue;
			case 82: //sobe
				estadoReal[indiceMovimentoTeclado].y += 10;
				continue;
			case 83: //direita
				estadoReal[indiceMovimentoTeclado].x += 10;
				continue;
			case 84: //desce
				estadoReal[indiceMovimentoTeclado].y -= 10;
				continue;
			}
			printf("[%d]", c);
			break;
		}
		switch (c) {
		case 'p':
		case 'P':
			emJogo = 0;

			//	 estadoReal[0].X=85; estadoReal[0].y=65; estadoReal[0].Angulo=45;
   			estadoReal[0].x = 17;
			estadoReal[0].y = 66;
			estadoReal[0].angulo = 274;
			estadoReal[1].x = 100;
			estadoReal[1].y = 100;
			estadoReal[1].angulo = 90;
			estadoReal[2].x = 150;
			estadoReal[2].y = 30;
			estadoReal[2].angulo = 180;
			estadoReal[3].x = 150;
			estadoReal[3].y = 55;
			estadoReal[3].angulo = 180;
			estadoReal[4].x = 120;
			estadoReal[4].y = 45;
			estadoReal[4].angulo = 90;
			estadoReal[5].x = 135;
			estadoReal[5].y = 90;
			estadoReal[5].angulo = 90;
			estadoReal[6].x = 155;
			estadoReal[6].y = 65;
			estadoReal[6].angulo = 90;
			/*estadoReal[4].x = 180;
			 estadoReal[4].y = 5;
			 estadoReal[4].angulo = 90;
			 estadoReal[5].x = 180;
			 estadoReal[5].y = 5;
			 estadoReal[5].angulo = 90;
			 estadoReal[6].x = 180;
			 estadoReal[6].y = 5;
			 estadoReal[6].angulo = 90;*/
			estadoReal[0].dx = 0;
			estadoReal[0].dy = 0;
			estadoReal[1].dx = 0;
			estadoReal[1].dy = 0;
			estadoReal[2].dx = 0;
			estadoReal[2].dy = 0;
			//	 estadoReal[3].Dx=0.5; estadoReal[3].Dy=0;
			estadoReal[3].dx = -1;
			estadoReal[3].dy = 1;
			estadoReal[4].dx = 0;
			estadoReal[4].dy = 0;
			estadoReal[5].dx = 0;
			estadoReal[5].dy = 0;
			estadoReal[6].dx = 0;
			estadoReal[6].dy = 0;

			for (i = 0; i < 7; i++) {
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

			for (i = 0; i < 7; i++) {
				int j;
				for (j = 0; j < 3; j++) {
					xVisao[i][j] = estado[i].x;
					yVisao[i][j] = estado[i].y;
					angVisao[i][j] = estado[i].angulo;
				}
			}
			//         srand(1);
			quadro = 0;
			break;
		case 'j':
		case 'J':
			emJogo = 1;
			indGoleiro = 0;
			indAtacante = 2;
			indVolante = 1;
			break;
		case 'l':
		case 'L':
			break;
		case 'd':
		case 'D':
			cout << "debug";
			break;
		}
	}
	cvReleaseImage(&image);
	cvDestroyWindow(wndname);

	return 0;
}

#ifdef _EiC
main(1,"drawing.c");
#endif
