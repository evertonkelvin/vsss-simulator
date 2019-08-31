/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#ifndef TIPOSCLASSES_H_
#define TIPOSCLASSES_H_

#include <map>
#include <cmath>

using namespace std;

typedef unsigned char guint8;

#define NUM_CAMERAS 1

#define TAM_X_DO_GOL 10 // centímetros
#define TAM_Y_DO_GOL 40 // centímetros

#define TAM_X_CAMPO_SEM_GOL 150	// centímetros
#define TAM_X_CAMPO (TAM_X_CAMPO_SEM_GOL + (TAM_X_DO_GOL * 2)) // centímetros
#define TAM_Y_CAMPO 130	// centímetros
#define MEIO_CAMPO_X TAM_X_CAMPO / 2 // centímetros
#define MEIO_CAMPO_Y TAM_Y_CAMPO / 2 // centímetros

#define CENTRO_X_GOL (TAM_X_CAMPO - TAM_X_DO_GOL) // centímetros
#define CENTRO_Y_GOL TAM_Y_CAMPO / 2 // centímetros

#define TAM_X_AREA 15
#define TAM_Y_AREA 70

#define POS_X_GOLEIRO (TAM_X_DO_GOL + TAM_ROBO / 2 + 2) // 2 cm a mais para o robo ter espaco para andar
#define DIST_GIRO TAM_ROBO * 2
#define TAM_ROBO 8 //tamanho do lado do robo em cm
#define TAM_BOLA 4 //tamanho do raio da bola
//numero de robos em cada time
#define NUM_ROBOS_TIME 3

// posicionamento da bola em cada quadrante do freeball
#define FREEBALL_QUADRANTE_1_X TAM_X_CAMPO_SEM_GOL - (TAM_X_CAMPO_SEM_GOL / 4) // centímetros
#define FREEBALL_QUADRANTE_1_Y TAM_Y_CAMPO - 25	// centímetros
#define FREEBALL_QUADRANTE_2_X TAM_X_CAMPO_SEM_GOL / 4 // centímetros
#define FREEBALL_QUADRANTE_2_Y TAM_Y_CAMPO - 25	// centímetros
#define FREEBALL_QUADRANTE_3_X TAM_X_CAMPO_SEM_GOL / 4 // centímetros
#define FREEBALL_QUADRANTE_3_Y 25 // centímetros
#define FREEBALL_QUADRANTE_4_X TAM_X_CAMPO_SEM_GOL - (TAM_X_CAMPO_SEM_GOL / 4) // centímetros
#define FREEBALL_QUADRANTE_4_Y 25 // centímetros

#define IND_BOLA NUM_ROBOS_TIME

//numero de quadros de atraso entre a visao e o comando ser recebido pelos robos
#define N_IMAGENS_ATRASO 5
#define N_IMAGENS_INERCIA 3

/** custo com alta penalidade indicando que nao existe
 * usado também -NAO_EXISTE, para agilizar comparacao <0, pode ser mais rapido que !=NAO_EXISTE (especialmente em reais (float)
 */
#define NAO_EXISTE INT_MAX / 100

#define VEL_MAXIMA_ROBO_CM 150.0											   // velocidade maxima de robo em cm
#define VEL_MAXIMA_ROBO_UNID 7												   // velocidade que se pode atribuir ao robo
#define FATOR_VEL_ROBO_UNID_POR_CM (VEL_MAXIMA_ROBO_UNID / VEL_MAXIMA_ROBO_CM) // fator de conversao entre a velocidade em cm para unidades
#define DIST_ENTRE_RODAS 7
#define FATOR_ANGULO_DIF_RODAS_UNID_VEL 5.83 // atang2(VEL_MAXIMA_ROBO_CM/7/30, DIST_ENTRE_RODAS) a cada 5.83 graus significa uma diferenca entre rodas de uma unidade de velocidade em 1/30s

#if NUM_ROBOS_TIME == 3
#define MAX_CONT_COMB 6
#elif NUM_ROBOS_TIME == 4
#define MAX_CONT_COMB 24
#elif NUM_ROBOS_TIME == 5
#define MAX_CONT_COMB 120
#else
Erro, precisa rever a criacao das combinacoes
#endif

enum IndCores
{
	COR_BOLA,
	COR_TIME,
	COR_CONTRA,
	COR_AUX1,
	COR_AUX2,
	COR_AUX3,
	MAX_IND_CORES
};
#define SEM_COR MAX_IND_CORES

struct Cor
{
	unsigned char b, g, r;
};

class Props : public map<string, string>
{
	string nomeArq;

public:
	Props(string nomeArq);
	void carrega();
	string get(string chave, string padrao = "");
	int getInt(string chave, int padrao = 0);
	int getFloat(string chave, float padrao = 0.0);
	void put(string chave, string valor);
	void put(string chave, int valor);
	void put(string chave, float valor);
	void salva(string header = "");
	~Props();
};

enum Lado
{
	ESQUERDA = 1,
	DIREITA
};

struct Estado
{
	float angulo;
	float dAngulo;
	float x, y;
	float dx, dy;
};

struct PosXYAux
{
	float x, y, xAux, yAux;
};

struct PosXY
{
	float x, y;
};

struct Objetivo
{
	float angulo;
	float x, y;
	float vel;
};

struct LimitesCampo
{
	int minXSemGol, maxXSemGol;
	int minX, maxX, minY, maxY;
};

struct CmdEnviado
{
	signed char esq, dir, vel; //v e' a velocidade esperada
};

void erro(string msg);

#endif /* TIPOSCLASSES_H_ */