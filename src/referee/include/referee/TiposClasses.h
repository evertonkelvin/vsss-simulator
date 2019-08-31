/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#ifndef TIPOSCLASSES_H_
#define TIPOSCLASSES_H_

using namespace std;

typedef unsigned char guint8;

#define TAM_X_DO_GOL 10 // centímetros
#define TAM_Y_DO_GOL 40 // centímetros

#define TAM_X_CAMPO_SEM_GOL 150	// centímetros
#define TAM_X_CAMPO (TAM_X_CAMPO_SEM_GOL + (TAM_X_DO_GOL * 2)) // centímetros
#define TAM_Y_CAMPO 130	// centímetros
#define MEIO_CAMPO_X TAM_X_CAMPO / 2 // centímetros
#define MEIO_CAMPO_Y TAM_Y_CAMPO / 2 // centímetros

#define CENTRO_X_GOL (TAM_X_CAMPO - TAM_X_DO_GOL) // centímetros
#define CENTRO_Y_GOL TAM_Y_CAMPO / 2 // centímetros

#define NUM_ROBOS_TIME	3

// posicionamento da bola em cada quadrante do freeball
#define FREEBALL_QUADRANTE_1_X TAM_X_CAMPO_SEM_GOL - (TAM_X_CAMPO_SEM_GOL / 4) // centímetros
#define FREEBALL_QUADRANTE_1_Y TAM_Y_CAMPO - 25 // centímetros
#define FREEBALL_QUADRANTE_2_X TAM_X_CAMPO_SEM_GOL / 4 // centímetros
#define FREEBALL_QUADRANTE_2_Y TAM_Y_CAMPO - 25 // centímetros
#define FREEBALL_QUADRANTE_3_X TAM_X_CAMPO_SEM_GOL / 4 // centímetros
#define FREEBALL_QUADRANTE_3_Y 25 // centímetros
#define FREEBALL_QUADRANTE_4_X TAM_X_CAMPO_SEM_GOL - (TAM_X_CAMPO_SEM_GOL / 4) // centímetros
#define FREEBALL_QUADRANTE_4_Y 25 // centímetros

struct Estado {
	float angulo;
	float dAngulo;
	float x, y;
	float dx, dy;
};

#endif /* TIPOSCLASSES_H_ */
