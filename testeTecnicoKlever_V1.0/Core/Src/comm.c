/*
 * comm.c
 *
 *  Created on: Mar 20, 2021
 *      Author: Gomes
 */
#include "comm.h"
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
/* Func Prototype */
void ResetMaquinaEstadosUART2(void);

/* Variáveis */
ESTADOS_SERIAL UART2Estado = SERIAL_BYTE_INICIO;
CMD_UART CMDUartISR= 0;
CMD_TASK CMDTaskISR = 0;
int8_t ParamLen = 0;
int16_t CheckSum = 0;

void SendLeituraADC(uint32_t adcValue){
	unsigned char Buffer[8] = {0x01,'A', 0x04, adcValue >> 24, adcValue >> 16, adcValue >> 8, adcValue, 0x00};
	int16_t CheckSum = 0;
	for(int i = 0; i < sizeof(Buffer) - 1; i++){
		CheckSum += Buffer[i];
	}

	Buffer[7] = CheckSum % 256;
	Comm_TX(Buffer, sizeof(Buffer));
}

void SendPacoteIncorreto(void){
	unsigned char Buffer[4] = {0x01,0xFF, 0x00, 0x00};
	int16_t CheckSum = 0;
	for(int i = 0; i < sizeof(Buffer) - 1; i++){
		CheckSum += Buffer[i];
	}

	Buffer[3] = CheckSum % 256;
	Comm_TX(Buffer, sizeof(Buffer));
}

void SendLoopBack(void){
	unsigned char Buffer[4] = {0x01,'S',0x00, 0x00};
	int16_t CheckSum = 0;
	for(int i = 0; i < sizeof(Buffer) - 1; i++){
		CheckSum += Buffer[i];
	}

	Buffer[3] = CheckSum % 256;
	Comm_TX(Buffer, sizeof(Buffer));
}

void SendACK(void){
	unsigned char Buffer[5] = {0x01,'S',0x00,'S', 0x00};
	int16_t CheckSum = 0;
	for(int i = 0; i < sizeof(Buffer) - 1; i++){
		CheckSum += Buffer[i];
	}

	Buffer[4] = CheckSum % 256;
	Comm_TX(Buffer, sizeof(Buffer));
}

void Comm_TX(unsigned char *s, int bufferSize)
{
  while(bufferSize)
	{
	  while( !(USART2->SR & 0x00000040) ){}; // Espera ate registrador ficar limpo
		HAL_UART_Transmit(&huart2,s, 1, 0xFFFF);
		s++;
		bufferSize--;
	}
}
void MaquinaEstadosUART2(char dado){

	switch(UART2Estado){
		case SERIAL_BYTE_INICIO:
			if(dado == 0x01){
				CheckSum+=dado;
				UART2Estado = SERIAL_BYTE_COMANDO;
			}else{
				CMDTaskISR = PACOTE_INCORRETO;
				/* Envia alerta de pacote incorreto para task uart */
				xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
				/* Reinicia a máquina de estados da UART */
				ResetMaquinaEstadosUART2();
			}
			break;
		case SERIAL_BYTE_COMANDO:
			if(dado == 'L'){
				CheckSum+=dado;
				CMDUartISR = CMD_LED;
				UART2Estado = SERIAL_BYTE_LEN;
			}else if(dado == 'S'){
				CheckSum+=dado;
				CMDUartISR = CMD_SERIAL;
				CMDTaskISR = LOOP_BACK;
				UART2Estado = SERIAL_BYTE_LEN;
			}else if(dado == 'A'){
				CheckSum+=dado;
				CMDUartISR = CMD_ADC;
				CMDTaskISR = LER_AD;
				UART2Estado = SERIAL_BYTE_LEN;
			}else{
				CMDTaskISR = PACOTE_INCORRETO;
				/* Envia alerta de pacote incorreto para task uart */
				xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
				/* Reinicia a máquina de estados da UART */
				ResetMaquinaEstadosUART2();
			}
		break;
		case SERIAL_BYTE_LEN:
			ParamLen = dado;
			if(ParamLen <= 1){
				if(ParamLen > 0){
					if(CMDUartISR == CMD_LED){
						CheckSum+=dado;
						UART2Estado = SERIAL_BYTE_PARAM;
					}else{
						/* CMD_ADC e CMD_SERIAL não recebem parâmetros */
						CMDTaskISR = PACOTE_INCORRETO;
						/* Envia alerta de pacote incorreto para task uart */
						xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
						/* Reinicia a máquina de estados da UART */
						ResetMaquinaEstadosUART2();
					}
				}else{
					/* int8_t não aceita números negativos*/
					/* Else representa se dado = 0 */
					if(CMDUartISR == CMD_ADC || CMDUartISR == CMD_SERIAL){
						CheckSum+=dado;
						UART2Estado = SERIAL_BYTE_CHECKSUM;
					}else{
						/* CMD_LED sempre tem um parâmetro */
						CMDTaskISR = PACOTE_INCORRETO;
						/* Envia alerta de pacote incorreto para task uart */
						xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
						/* Reinicia a máquina de estados da UART */
						ResetMaquinaEstadosUART2();
					}
				}
			}else{
				/* Parametro só pode ter tamanho de no máximo 1 */
				CMDTaskISR = PACOTE_INCORRETO;
				/* Envia alerta de pacote incorreto para task uart */
				xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
				/* Reinicia a máquina de estados da UART */
				ResetMaquinaEstadosUART2();
			}
		break;
		case SERIAL_BYTE_PARAM:
			if(dado == 'L'){
				CheckSum+=dado;
				CMDTaskISR = LIGAR_LED;
				UART2Estado = SERIAL_BYTE_CHECKSUM;
			}else if(dado == 'D'){
				CheckSum+=dado;
				CMDTaskISR = DESLIGAR_LED;
				UART2Estado = SERIAL_BYTE_CHECKSUM;
			}else if(dado == 'T'){
				CheckSum+=dado;
				CMDTaskISR = TOOGLE_LED;
				UART2Estado = SERIAL_BYTE_CHECKSUM;
			}else{
				/* ENVIA ALERTA PARA TASK UART */
				CMDTaskISR = PACOTE_INCORRETO;
				/* Envia alerta de pacote incorreto para task uart */
				xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
				/* Reinicia a máquina de estados da UART */
				ResetMaquinaEstadosUART2();
			}
		break;
		case SERIAL_BYTE_CHECKSUM:
			if(dado == CheckSum % 256){
				/* Envia comando recebido pela serial e que será tratado na Task */
				xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
			}else{
				CMDTaskISR = PACOTE_INCORRETO;
				/* Envia alerta de pacote incorreto para task uart */
				xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
			}
			/* Fim de comunicação*/
			/* Reinicia a máquina de estados da UART */
			ResetMaquinaEstadosUART2();
		break;
	}
}

inline void ResetMaquinaEstadosUART2(void){
	UART2Estado = SERIAL_BYTE_INICIO;
	CMDUartISR= 0;
	CMDTaskISR = 0;
	CheckSum = 0;
}
