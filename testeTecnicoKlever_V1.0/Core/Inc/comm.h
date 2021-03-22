/*
 * comm.h
 *
 *  Created on: Mar 20, 2021
 *      Author: Gomes
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_
#include "stdint.h"
/* Funções externalizadas */
void MaquinaEstadosUART2(char dado);
void SendLoopBack(void);
void SendACK(void);
void SendPacoteIncorreto(void);
void SendLeituraADC(uint32_t adcValue);
/* ESTADOS DE COMUNICAÇÃO SERIAL */
typedef enum{
	SERIAL_BYTE_INICIO = 1,
	SERIAL_BYTE_COMANDO,
	SERIAL_BYTE_LEN,
	SERIAL_BYTE_PARAM,
	SERIAL_BYTE_CHECKSUM
}ESTADOS_SERIAL;
/* TIPOS DE COMANDOS A SEREM RECEBIDOS NA UART */
typedef enum ComandosUart{
	CMD_LED = 1,
	CMD_ADC,
	CMD_SERIAL
}CMD_UART;

/* TIPOS DE COMANDOS A SEREM ENVIADOS PARA TASK UART */
typedef enum{
	LIGAR_LED = 1,
	DESLIGAR_LED,
	TOOGLE_LED,
	LOOP_BACK,
	LER_AD,
	PACOTE_INCORRETO
}CMD_TASK;

#endif /* INC_COMM_H_ */
