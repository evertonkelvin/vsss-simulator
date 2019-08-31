/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#include "TiposClasses.h"

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

#define TEMPO_PARA_FREEBALL 230

#define DESLINICIOCAMPO 520

struct EstatisticasJogo
{
	int tempo;
	int freeballs = 0;
	int tempo_bola_parada = 0;
} jogo;

struct EstatisticasTimes
{
	int penaltis = 0;
	int gols = 0;
} time1, time2;

struct PosicaoBolaParada
{
	float x = 0;
	float y = 0;
} bola_parada;

char wndname[] = "Árbitro - Simulador Very Small Size Soccer";
char interrupcao;

int indGoleiro = 0;
int indVolante = 1;
int indAtacante = 2;

int loop_registrado = 0;
int loop_count = 0;
int model_indexes[6];

ros::Publisher pubModelPose;

Estado estadoReal[NUM_ROBOS_TIME * 2 + 1], estadoRealAnt[NUM_ROBOS_TIME * 2 + 1];

CvScalar corNoCV;
CvFont font;
IplImage* image;

inline float seno(float x) {
	return sin(x * M_PI / 180);
}

inline float coss(float x) {
	return cos(x * M_PI / 180);
}

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

void DesenhaBola(void) {
	setColor(LIGHTRED);
	Circle(estadoReal[3].x, estadoReal[3].y, 3);
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

void DesenhaBallTimer(void)
{
	char ball_timer[5];

	sprintf(ball_timer, "%d", jogo.tempo_bola_parada);

	OutTextXY(13, -30, "Timer Bola: ");
	OutTextXY(60, -30, ball_timer);
	// OutTextXY(60, -30, "9999");
}

void DesenhaNumFreeball(void)
{
	char freeball_count[5];

	sprintf(freeball_count, "%d", jogo.freeballs);

	// setTextStyle(8, 0, 3);
	OutTextXY(MEIO_CAMPO_X, -30, "|");
	OutTextXY(MEIO_CAMPO_X + 10, -30, "Freeballs: ");
	OutTextXY(MEIO_CAMPO_X + 50, -30, freeball_count);
	// OutTextXY(MEIO_CAMPO_X + 50, -30, "9999");
}

void DesenhaNumPenalti(void)
{
	char penaltis_time1[5];
	char penaltis_time2[5];

	sprintf(penaltis_time1, "%d", time1.penaltis);
	sprintf(penaltis_time2, "%d", time2.penaltis);

	OutTextXY(MEIO_CAMPO_X - 40, -20, penaltis_time1);
	// OutTextXY(MEIO_CAMPO_X - 40, -20, "9999");
	OutTextXY(MEIO_CAMPO_X - 15, -20, "penaltis");
	OutTextXY(MEIO_CAMPO_X + 20, -20, penaltis_time2);
	// OutTextXY(MEIO_CAMPO_X + 20, -20, "9999");
}

void DesenhaPlacar(void)
{

	char gols_time1[5];
	char gols_time2[5];

	sprintf(gols_time1, "%d", time1.gols);
	sprintf(gols_time2, "%d", time2.gols);

	OutTextXY(10, -10, "Time 1 ");
	OutTextXY(MEIO_CAMPO_X - 33, -10, gols_time1);
	// OutTextXY(MEIO_CAMPO_X - 33, -10, "9999");
	OutTextXY(MEIO_CAMPO_X - 7, -10, "gols");
	OutTextXY(MEIO_CAMPO_X + 15, -10, gols_time2);
	// OutTextXY(MEIO_CAMPO_X + 15, -10, "9999");
	OutTextXY(TAM_X_CAMPO - 40, -10, " Time 2");
}

void DesenhaCampo(void) {
	Rectangle(0, 0, 180, 180); // Alterar esses valores caso queira adicionar mais informações no placar
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

void DesenhaJogo(void) {
	char num[10];
	int i;

	DesenhaCampo();
	DesenhaPlacar();
	DesenhaNumPenalti();
	DesenhaBallTimer();
	DesenhaNumFreeball();

	for (i = 0; i < 7; i++) {

		if (i != 3) {
			DesenhaJogador(i);
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
			if (estadoReal[3].x >= MEIO_CAMPO_X && estadoReal[3].y >= MEIO_CAMPO_Y) { // QUADRANTE 1
				AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);
				AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);
				AlteraPosicaoGazebo("home3", FREEBALL_QUADRANTE_1_X - 20, FREEBALL_QUADRANTE_1_Y, 0, 0);
				AlteraPosicaoGazebo("ball", FREEBALL_QUADRANTE_1_X, FREEBALL_QUADRANTE_1_Y, 0, 0);
				AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);
				AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X + 40, MEIO_CAMPO_Y, -0.71, 0.7);
				AlteraPosicaoGazebo("guest3", FREEBALL_QUADRANTE_1_X + 20, FREEBALL_QUADRANTE_1_Y, 0, 0);

			} else if (estadoReal[3].x < MEIO_CAMPO_X  && estadoReal[3].y > MEIO_CAMPO_Y) { // QUADRANTE 2
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
			AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("home3", TAM_X_CAMPO_SEM_GOL - (TAM_X_CAMPO_SEM_GOL / 4) - 8, MEIO_CAMPO_Y, 0, 0);
			AlteraPosicaoGazebo("ball", TAM_X_CAMPO_SEM_GOL - (TAM_X_CAMPO_SEM_GOL / 4), MEIO_CAMPO_Y, 0, 0);
			AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);
			AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X - 10, MEIO_CAMPO_Y + 20, -0.71, 0.7);
			AlteraPosicaoGazebo("guest3", MEIO_CAMPO_X - 10, MEIO_CAMPO_Y - 20, 0, 0);
			break;

		case 'k': // PÈNALTI PARA TIME 2
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
			AlteraPosicaoGazebo("home1", MEIO_CAMPO_X - 70, MEIO_CAMPO_Y, -0.71, 0.7);		 // GOLEIRO DO TIME 1
			AlteraPosicaoGazebo("home2", MEIO_CAMPO_X - 40, MEIO_CAMPO_Y, -0.71, 0.7);		 // VOLANTE DO TIME 1
			AlteraPosicaoGazebo("home3", MEIO_CAMPO_X - 20, MEIO_CAMPO_Y + 12, -0.3, 0.96);  // ATACANTE DO TIME 1
			AlteraPosicaoGazebo("ball", MEIO_CAMPO_X, MEIO_CAMPO_Y, 0, 0);					 // BOLA
			AlteraPosicaoGazebo("guest1", MEIO_CAMPO_X + 70, MEIO_CAMPO_Y, -0.71, 0.7);		 // GOLEIRO DO TIME 2
			AlteraPosicaoGazebo("guest2", MEIO_CAMPO_X + 40, MEIO_CAMPO_Y, -0.71, 0.7);		 // VOLANTE DO TIME 2
			AlteraPosicaoGazebo("guest3", MEIO_CAMPO_X + 20, MEIO_CAMPO_Y - 12, -0.3, 0.96); // ATACANTE DO TIME 2
			break;
	}
}

void VerificaPosicionamento() {

	// VERIFICA INÍCIO DA PARTIDA
	if (loop_count == 1) {
		interrupcao = 'p';
	}

	// VERIFICA GOL DO TIME 1
	if (((estadoReal[3].x >= TAM_X_CAMPO - (TAM_X_DO_GOL - 2) && estadoReal[3].x <= TAM_X_CAMPO) &&
		 (estadoReal[3].y >= 45 && estadoReal[3].y <= 85))) {

		if (loop_count > loop_registrado + 10) {
			loop_registrado = loop_count;
			time1.gols++;
			interrupcao = 'p';
		}
	}

	// VERIFICA GOL DO TIME 2
	if (((estadoReal[3].x >= 0 && estadoReal[3].x <= TAM_X_DO_GOL - 2) &&
		 (estadoReal[3].y >= 45 && estadoReal[3].y <= 85))) {

		if (loop_count > loop_registrado + 10) {
			loop_registrado = loop_count;
			time2.gols++;
			interrupcao = 'p';
		}
	}

	// VERIFICA FREEBALL
	if (estadoReal[3].x == estadoRealAnt[3].x && estadoReal[3].y == estadoRealAnt[3].y && jogo.tempo_bola_parada == 0) {
		bola_parada.x = estadoRealAnt[3].x;
		bola_parada.y = estadoRealAnt[3].y;
	}

	if ((estadoReal[3].x >= bola_parada.x - 5 && estadoReal[3].x <= bola_parada.x + 5)
			&& (estadoReal[3].y >= bola_parada.y - 5 && estadoReal[3].y <= bola_parada.y + 5)) {
		jogo.tempo_bola_parada++;
		if (jogo.tempo_bola_parada >= TEMPO_PARA_FREEBALL) {
			jogo.tempo_bola_parada = 0;
			jogo.freeballs++;
			interrupcao = 'f';
		}
	} else {
		jogo.tempo_bola_parada = 0;
	}

	// VERIFICA PENALTI A FAVOR DO TIME 2
	if ((((estadoReal[3].x > 0 && estadoReal[3].x < 25) &&
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
		if (loop_count > loop_registrado + 10) {
			loop_registrado = loop_count;
			time2.penaltis++;
			interrupcao = 'k';
		}
	}

	// VERIFICA PENALTI A FAVOR DO TIME 1
	if ((((estadoReal[3].x > TAM_X_CAMPO - 35 && estadoReal[3].x < TAM_X_CAMPO) &&
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
		if (loop_count > loop_registrado + 10) {
			loop_registrado = loop_count;
			time1.penaltis++;
			interrupcao = 'K';
		}
	}
}

void Posicoes(const gazebo_msgs::ModelStates &msg, int i, int indiceModel) {
	ignition::math::Quaterniond q(msg.pose[indiceModel].orientation.w, msg.pose[indiceModel].orientation.x, msg.pose[indiceModel].orientation.y, msg.pose[indiceModel].orientation.z);
	
	double yaw = q.Yaw();
	double yaw_degrees = yaw * 180.0 / M_PI; // conversion to degrees
	if (yaw_degrees < 0)
		yaw_degrees += 360.0; // convert negative to positive angles

	estadoReal[i].angulo = yaw_degrees;

	estadoReal[i].dx = (msg.pose[indiceModel].position.x * 100) + MEIO_CAMPO_X - estadoReal[i].x;
	estadoReal[i].dy = (msg.pose[indiceModel].position.y * 100) + MEIO_CAMPO_Y - estadoReal[i].y;

	estadoReal[i].x = (msg.pose[indiceModel].position.x * 100) + MEIO_CAMPO_X;
	estadoReal[i].y = (msg.pose[indiceModel].position.y * 100) + MEIO_CAMPO_Y;
}

int getIndex(std::vector<std::string> input, std::string searched) {
	int wordIndex;

	for (int i = 0; i < input.size(); i++) {
		if (input[i] == searched) {
			return i;
		}
	}
}

void PoseCallback(const gazebo_msgs::ModelStates& msg) {
	int i;

	// array para identificar os índices dos robôs e bola do model_states
	string modelNames[] = {"home1", "home2", "home3", "ball", "guest1", "guest2", "guest3"};

	for (i = 0; i < 7; i++) {
		estadoRealAnt[i].angulo = estadoReal[i].angulo;

		estadoRealAnt[i].dx = estadoReal[i].dx;
		estadoRealAnt[i].dy = estadoReal[i].dy;

		estadoRealAnt[i].x = estadoReal[i].x;
		estadoRealAnt[i].y = estadoReal[i].y;

		if (loop_count == 1) {
			model_indexes[i] = getIndex(msg.name, modelNames[i]);
			cout << "\n" << model_indexes[i] << "\n";
		}

		Posicoes(msg, i, model_indexes[i]);
	}

	DesenhaJogo();

	cvShowImage(wndname, image);
	fflush(stdout);
	interrupcao = cvWaitKey(30);

	VerificaPosicionamento();
	VerificaInterrupcao();
}

int main(int argc, char** argv) 
{    	
	image = cvCreateImage(cvSize(680, 650), 8, 3);
	
	// Create a window
	cvNamedWindow(wndname, 1);
	cvZero(image);
	cvShowImage(wndname, image);

	Inicia();

	ros::init(argc, argv, "vsss_referee");
	ros::NodeHandle n;

	pubModelPose = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 7);

	// PoseCallback: função que recebe as posições do gazebo e inicia o loop do jogo
	ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, PoseCallback);

	ros::Rate loop_rate(33);

	while (interrupcao != 'q' && ros::ok()) {
		ros::spinOnce();

		loop_rate.sleep();
		++loop_count;
	}

	cvReleaseImage(&image);
	cvDestroyWindow(wndname);

	return 0;
}
