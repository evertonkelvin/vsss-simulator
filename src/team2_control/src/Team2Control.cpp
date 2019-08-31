/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#include "TiposClasses.h"
#include "Auxiliares.h"
#include "Estrategia.h"

#include "opencv2/highgui.hpp"

#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include <ignition/math/Quaternion.hh>

typedef unsigned char guint8;

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

#define RAIO_DA_RODA_M 0.017 // 1.7cm
#define DIST_ENTRE_RODAS_M 0.069 // 6.9cm

#define DESLINICIOCAMPO 520

char wndname[] = "Controle Time 2 SCC VSSS";
char interrupcao;

bool emPenalti = false;
bool tiroMeta = false;

bool emPenalidade = false;
bool emPosiciona = false;
bool emInicio = false;
bool emJogo = true;

int indGoleiro = 0;
int indVolante = 1;
int indAtacante = 2;

int loop_count = 0;
int modelIndexes[6];

ros::Publisher pubAtacanteGuest;
ros::Publisher pubVolanteGuest;
ros::Publisher pubGoleiroGuest;

ros::Publisher pubModelPose;

Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1], estadoPrev[NUM_ROBOS_TIME * 2 + 1];
Estado estadoReal[NUM_ROBOS_TIME * 2 + 1], estadoRealAnt[NUM_ROBOS_TIME * 2 + 1];

Objetivo objetivoRobo[NUM_ROBOS_TIME];

CvScalar corNoCV;
CvFont font;
IplImage *image;

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

}

int ConverteCmd(int b) {
	if (b & 0x80)
		return -((b & 0x70) >> 4);
	else
		return b >> 4;
}

void EnviaCmd2Gazebo(int indiceRobo, float ve, float vd) {
	geometry_msgs::Twist msg;

	float vel_linear_robo = (vd + ve) / 2 * 30 / 100;
	float vel_angular_robo = (vd - ve) * (VEL_MAXIMA_ROBO_CM / 100 / 7) / DIST_ENTRE_RODAS_M;

	msg.linear.x = vel_linear_robo * -1;
	msg.angular.z = vel_angular_robo;

	printf("\nROBO[%d] - Vel.Linear: %f | Vel.Ang: %f\n", indiceRobo, msg.linear.x, msg.angular.z);

	if (indiceRobo == 0) {
		pubGoleiroGuest.publish(msg);
	} else if (indiceRobo == 1) {
		pubVolanteGuest.publish(msg);
	} else if (indiceRobo == 2) {
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
	printf("         %3s, %3s, %3s, %3s, %3s, %3s\n", "GRe", "GRd", "VRe", "VRd", "ARe", "ARd");
	printf("CMD_1a7: %3d, %3d, %3d, %3d, %3d, %3d\n", ConverteCmd(b1), ConverteCmd(b2), ConverteCmd(b3), ConverteCmd(b4), ConverteCmd(b5), ConverteCmd(b6));

	for (i = 0; i < 3; i++) {
		if (b[i * 2] & 0x8) {
			ve = -(b[i * 2] & 0x7);
		}
		else {
			ve = b[i * 2] & 0x7;
		}
		if (b[i * 2 + 1] & 0x8) {
			vd = -(b[i * 2 + 1] & 0x7);
		}
		else {
			vd = b[i * 2 + 1] & 0x7;
		}
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

		dang = atan2((vd - ve), DIST_ENTRE_RODAS) * 180 / M_PI;

		if (emJogo) {
			EnviaCmd2Gazebo(i, ve, vd);
		}
	}
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
			DesenhaJogadorPrev(i);
			DesenhaJogador(i);
			DesenhaObjetivos(i);
			num[0] = '0' + i;
			if (i == indAtacante) {
				num[1] = 'A';
			} else if (i == indVolante) {
				num[1] = 'V';
			} else if (i == indGoleiro) {
				num[1] = 'G';
			} else {
				num[1] = 'R';
			}
			num[2] = 0;
			OutTextXY(estadoReal[i].x - 2, estadoReal[i].y + 2, num);
		} else {
			DesenhaBola();
			DesenhaBolaPrev();
		}
	}
}

void AlteraPosicaoGazebo(string name, double px, double py, double oz, double ow) {
	gazebo_msgs::ModelState msg;

	msg.model_name = name;
	msg.pose.position.x = (px - MEIO_CAMPO_X) / 100;
	msg.pose.position.y = (py - MEIO_CAMPO_Y) / 100;
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

void resetaVelRobos() {
	int i = 0;
	for (i; i < 3; i++) {
		EnviaCmd2Gazebo(i, 0, 0);
	}
}

void VerificaInterrupcao() {

	// EXIBE POSIÇÕES RECEBIDAS
	printf("\n");
	printf("%3s, %3s, %3s, %3s, %3s | %3s, %3s, %3s, %3s, %3s\n",
		   "RAx", "RAy", "Rdx", "Rdy", "RAa", "eRx", "eRy", "Rdx", "Rdy", "eRa");
	for (int i = 0; i < 7; i++) {
		printf("%3.0f, %3.0f, %3.0f, %3.0f, %3.0f | %3.0f, %3.0f, %3.0f, %3.0f, %3.0f\n",
			   estadoRealAnt[i].x, estadoRealAnt[i].y, estadoRealAnt[i].dx, estadoRealAnt[i].dy, estadoRealAnt[i].angulo,
			   estadoReal[i].x, estadoReal[i].y, estadoReal[i].dx, estadoReal[i].dy, estadoReal[i].angulo);
	}

	switch (interrupcao) {
	case 'f': // FREEBALL
	case 'F':
		emJogo = false;
		resetaVelRobos();

		if (estadoReal[3].x >= MEIO_CAMPO_X && estadoReal[3].y >= MEIO_CAMPO_Y) { // QUADRANTE 1
			AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home3", FREEBALL_QUADRANTE_1_X - 20, FREEBALL_QUADRANTE_1_Y, 0, 0);
			AlteraPosicaoGazebo("ball", FREEBALL_QUADRANTE_1_X, FREEBALL_QUADRANTE_1_Y, 0, 0);
			AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X + 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest3", FREEBALL_QUADRANTE_1_X + 20, FREEBALL_QUADRANTE_1_Y, 0, 0);

		} else if (estadoReal[3].x < MEIO_CAMPO_X && estadoReal[3].y > MEIO_CAMPO_Y) { // QUADRANTE 2
			AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home3", FREEBALL_QUADRANTE_2_X - 20, FREEBALL_QUADRANTE_2_Y, 0, 0);
			AlteraPosicaoGazebo("ball", FREEBALL_QUADRANTE_2_X, FREEBALL_QUADRANTE_2_Y, 0, 0);
			AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X + 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest3", FREEBALL_QUADRANTE_2_X + 20, FREEBALL_QUADRANTE_2_Y, 0, 0);

		} else if (estadoReal[3].x <= MEIO_CAMPO_X && estadoReal[3].y <= MEIO_CAMPO_Y) { // QUADRANTE 3
			AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home3", FREEBALL_QUADRANTE_3_X - 20, FREEBALL_QUADRANTE_3_Y, 0, 0);
			AlteraPosicaoGazebo("ball", FREEBALL_QUADRANTE_3_X, FREEBALL_QUADRANTE_3_Y, 0, 0);
			AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X + 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest3", FREEBALL_QUADRANTE_3_X + 20, FREEBALL_QUADRANTE_3_Y, 0, 0);

		} else if (estadoReal[3].x > MEIO_CAMPO_X && estadoReal[3].y < MEIO_CAMPO_Y) { // QUADRANTE 4
			AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home3", FREEBALL_QUADRANTE_4_X - 20, FREEBALL_QUADRANTE_4_Y, 0, 0);
			AlteraPosicaoGazebo("ball", FREEBALL_QUADRANTE_4_X, FREEBALL_QUADRANTE_4_Y, 0, 0);
			AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X + 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest3", FREEBALL_QUADRANTE_4_X + 20, FREEBALL_QUADRANTE_4_Y, 0, 0);
		}
		break;

	case 'K': // PÈNALTI PARA TIME 1
		emJogo = false;
		resetaVelRobos();

		AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);
		AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);
		AlteraPosicaoGazebo("home3", TAM_X_CAMPO_SEM_GOL - (TAM_X_CAMPO_SEM_GOL / 4) - 8, MEIO_CAMPO_Y, 0, 0);
		AlteraPosicaoGazebo("ball", TAM_X_CAMPO_SEM_GOL - (TAM_X_CAMPO_SEM_GOL / 4), MEIO_CAMPO_Y, 0, 0);
		AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);
		AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X - 10, MEIO_CAMPO_Y + 20, -0.71, 0.7);
		AlteraPosicaoGazebo("guest3", MEIO_CAMPO_X - 10, MEIO_CAMPO_Y - 20, 0, 0);
		break;

	case 'k': // PÈNALTI PARA TIME 2
		emJogo = false;
		resetaVelRobos();

		AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);
		AlteraPosicaoGazebo("home2", MEIO_CAMPO_X + 10, MEIO_CAMPO_Y + 20, 0, 0);
		AlteraPosicaoGazebo("home3", MEIO_CAMPO_X + 10, MEIO_CAMPO_Y - 20, 0, 0);
		AlteraPosicaoGazebo("ball", TAM_X_CAMPO_SEM_GOL / 4, MEIO_CAMPO_Y, 0, 0);
		AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);
		AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X + 40, MEIO_CAMPO_Y, -0.71, 0.7);
		AlteraPosicaoGazebo("guest3", (TAM_X_CAMPO_SEM_GOL / 4) + 8, MEIO_CAMPO_Y, 0, 0);
		break;

	case 'p': // POSIÇÕES INICIAIS
	case 'P':
		emJogo = false;
		resetaVelRobos();

		AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);		 // GOLEIRO DO TIME 1
		AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);		 // VOLANTE DO TIME 1
		AlteraPosicaoGazebo("home3", MEIO_CAMPO_X - 20, MEIO_CAMPO_Y + 12, -0.3, 0.96);  // ATACANTE DO TIME 1
		AlteraPosicaoGazebo("ball", MEIO_CAMPO_X, MEIO_CAMPO_Y, 0, 0);					 // BOLA
		AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);		 // GOLEIRO DO TIME 2
		AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X + 40, MEIO_CAMPO_Y, -0.71, 0.7);		 // VOLANTE DO TIME 2
		AlteraPosicaoGazebo("guest3", MEIO_CAMPO_X + 20, MEIO_CAMPO_Y - 12, -0.3, 0.96); // ATACANTE DO TIME 2
		break;
	}
	emJogo = true;
}

void Posicoes(const gazebo_msgs::ModelStates &msg, int i, int indiceModel) {
	ignition::math::Quaterniond q(msg.pose[indiceModel].orientation.w, msg.pose[indiceModel].orientation.x, msg.pose[indiceModel].orientation.y, msg.pose[indiceModel].orientation.z);

	double yaw = q.Yaw();
	double yaw_degrees = yaw * 180.0 / M_PI; // conversion to degrees
	if (yaw_degrees < 0)
		yaw_degrees += 360.0; // convert negative to positive angles

	estadoReal[i].angulo = yaw_degrees;

	estadoReal[i].dx = 170 - ((msg.pose[indiceModel].position.x * 100) + MEIO_CAMPO_X) - estadoReal[i].x;
	estadoReal[i].dy = 130 - ((msg.pose[indiceModel].position.y * 100) + MEIO_CAMPO_Y) - estadoReal[i].y;

	estadoReal[i].x = 170 - ((msg.pose[indiceModel].position.x * 100) + MEIO_CAMPO_X);
	estadoReal[i].y = 130 - ((msg.pose[indiceModel].position.y * 100) + MEIO_CAMPO_Y);
}

int getIndex(std::vector<std::string> input, std::string searched) {
	int wordIndex;

	for (int i = 0; i < input.size(); i++) {
		if (input[i] == searched) {
			return i;
		}
	}
}

void PoseCallback(const gazebo_msgs::ModelStates &msg)
{
	int i;

	// array para identificar os índices dos robôs e bola do model_states
	string modelNames[] = {"guest1", "guest2", "guest3", "ball", "home1", "home2", "home3"};

	for (i = 0; i < 7; i++)
	{
		estadoRealAnt[i].angulo = estadoReal[i].angulo;

		estadoRealAnt[i].dx = estadoReal[i].dx;
		estadoRealAnt[i].dy = estadoReal[i].dy;

		estadoRealAnt[i].x = estadoReal[i].x;
		estadoRealAnt[i].y = estadoReal[i].y;

		if (loop_count == 1) {
			modelIndexes[i] = getIndex(msg.name, modelNames[i]);
			cout << "\n" << modelIndexes[i] << "\n";
		}

		Posicoes(msg, i, modelIndexes[i]);
	}

	Visao();
	estrategia();
	DesenhaJogo();

	cvShowImage(wndname, image);
	fflush(stdout);
	interrupcao = cvWaitKey(30);

	VerificaInterrupcao();
}

int main(int argc, char **argv) {

	image = cvCreateImage(cvSize(680, 520), 8, 3);

	// Create a window
	cvNamedWindow(wndname, 1);
	cvZero(image);
	cvShowImage(wndname, image);

	Inicia();

	ros::init(argc, argv, "vsss_team2_control");
	ros::NodeHandle n;

	pubGoleiroGuest = n.advertise<geometry_msgs::Twist>("/cmd_vel_guest1", 1);
	pubVolanteGuest = n.advertise<geometry_msgs::Twist>("/cmd_vel_guest2", 1);
	pubAtacanteGuest = n.advertise<geometry_msgs::Twist>("/cmd_vel_guest3", 1);

	pubModelPose = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 7);

	// PoseCallback: função que recebe as posições do gazebo e inicia o loop do jogo
	ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, PoseCallback);

	ros::Rate loop_rate(33);

	while (interrupcao != 'q' && ros::ok()) {
		ros::spinOnce();

		loop_rate.sleep();
		++loop_count;
	}

	resetaVelRobos();
	cvReleaseImage(&image);
	cvDestroyWindow(wndname);

	return 0;
}
