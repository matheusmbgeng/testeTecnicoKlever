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
void Comm_TX(unsigned char *s, int bufferSize);
void ResetMaquinaEstadosUART2(void);

char CalcCheckSum(unsigned char * Buffer, int dataLen);
/* Variáveis */
ESTADOS_SERIAL UART2Estado = SERIAL_BYTE_INICIO;
CMD_UART CMDUartISR= 0;
CMD_TASK CMDTaskISR = 0;
int8_t ParamLen = 0;
int16_t CheckSum = 0;
/*----------------------------------------------------------------------------------------------
SendLeituraADC: Envia a leitura do ADC, recebida pelo parâmetro adcValue. Dado é enviado em formato
Big-Endian. Envia na resposta o comando de leitura de AD.
----------------------------------------------------------------------------------------------*/
void SendLeituraADC(uint32_t adcValue){
	unsigned char Buffer[8] = {0x01,ADC_CMD_BYTE, 0x04, adcValue >> 24, adcValue >> 16, adcValue >> 8, adcValue, 0x00};
	int16_t CheckSum = 0;
	for(int i = 0; i < sizeof(Buffer) - 1; i++){
		CheckSum += Buffer[i];
	}

	Buffer[7] = CheckSum % 256;
	Comm_TX(Buffer, sizeof(Buffer));
}
/*----------------------------------------------------------------------------------------------
SendLoopBack: Envia um ACK de pacote fora do padrão. O comando desse ACK é 0xFF.
----------------------------------------------------------------------------------------------*/
void SendPacoteIncorreto(void){
	unsigned char Buffer[4] = {0x01,FAIL_CMD_BYTE, 0x00, 0x00};

	Buffer[3] = CalcCheckSum(Buffer, sizeof(Buffer) - 1);
	/* Envia pela serial */
	Comm_TX(Buffer, sizeof(Buffer));
}
/*----------------------------------------------------------------------------------------------
SendLoopBack: Envia o loopback pela Serial. Envia o comando serial do protocolo na resposta.
----------------------------------------------------------------------------------------------*/
void SendLoopBack(void){
	unsigned char Buffer[4] = {0x01,SERIAL_CMD_BYTE,0x00, 0x00};

	Buffer[3] = CalcCheckSum(Buffer, sizeof(Buffer) - 1);
	Comm_TX(Buffer, sizeof(Buffer));
}
/*----------------------------------------------------------------------------------------------
SendACK: Envia ACK de acordo com o comando 'cmd' recebido.
----------------------------------------------------------------------------------------------*/
void SendACK(char cmd){
	/* Envia ACK de acordo com o comando recebido*/
	/* Possui 1 Parâmetro 'S', indicando sucesso na comunicação */
	unsigned char Buffer[5] = {0x01, cmd,0x01,'S', 0x00};

	Buffer[4] = CalcCheckSum(Buffer, sizeof(Buffer) - 1);
	Comm_TX(Buffer, sizeof(Buffer));
}
/*----------------------------------------------------------------------------------------------
Comm_TX: Função de envio na serial da UART2. Recebe como parâmetro o buffer de dados a ser enviado
e o tamanho do pacote.
----------------------------------------------------------------------------------------------*/
void Comm_TX(unsigned char *s, int bufferSize)
{
  while(bufferSize)
	{
	  /* Espera ate registrador ficar limpo */
	  while( !(USART2->SR & 0x00000040) ){};
		HAL_UART_Transmit(&huart2,s, 1, 0xFFFF);
		s++;
		bufferSize--;
	}
}

/*----------------------------------------------------------------------------------------------
CalcCheckSum: Calcula o CheckSum do buffer pacoteDados. Parâmetro dataLen representa a
quantidade de bytes que devem ser somados do buffer. Retorna o byte LSB da soma simples dos bytes.
----------------------------------------------------------------------------------------------*/
char CalcCheckSum(unsigned char * pacoteDados, int dataLen){
	int16_t CheckSum = 0;
	for(int i = 0; i < dataLen; i++){
		CheckSum += pacoteDados[i];
	}

	return CheckSum % 256;
}
/*----------------------------------------------------------------------------------------------
MaquinaEstadosUART2: Máquina de estados que recebe byte a byte da ISR de serial o protocolo
estabelecido. Cada posição do protocolo corresponde a um estado definido no header comm.h.
Recebe como parâmetro um char correspondente ao byte recebido na ISR.
----------------------------------------------------------------------------------------------*/
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
			if(dado == LED_CMD_BYTE){
				CheckSum+=dado;
				CMDUartISR = CMD_LED;
				UART2Estado = SERIAL_BYTE_LEN;
			}else if(dado == SERIAL_CMD_BYTE){
				CheckSum+=dado;
				CMDUartISR = CMD_SERIAL;
				CMDTaskISR = LOOP_BACK;
				UART2Estado = SERIAL_BYTE_LEN;
			}else if(dado == ADC_CMD_BYTE){
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
					/* Este else representa se dado = 0 */
					if(CMDUartISR == CMD_ADC || CMDUartISR == CMD_SERIAL){
						CheckSum+=dado;
						UART2Estado = SERIAL_BYTE_CHECKSUM;
					}else{
						/* Este else represente se CMD recebido não for nem ADC nem Serial */
						/* CMD_LED sempre tem um parâmetro */
						CMDTaskISR = PACOTE_INCORRETO;
						/* Envia alerta de pacote incorreto para task uart */
						xQueueSendFromISR(xUartTaskQueue,&CMDTaskISR,NULL);
						/* Reinicia a máquina de estados da UART */
						ResetMaquinaEstadosUART2();
					}
				}
			}else{
				/* Parametros dos pacotes recebidos só tem no máximo  1 byte */
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
			/* Reinicia a máquina de estados da UART */
			ResetMaquinaEstadosUART2();
			/* Fim de comunicação*/
		break;
	}
}
/*----------------------------------------------------------------------------------------------
ResetMaquinaEstadosUART2: Reinicia os parâmetros da máquina de estados que recebe o pacote
	byte a byte pela interrupção da serial. Função inline para garantir que o compilador
	trate o corpo da função como uma continuação de onde foi chamada, pois se trata de uma ISR.
----------------------------------------------------------------------------------------------*/
inline void ResetMaquinaEstadosUART2(void){
	UART2Estado = SERIAL_BYTE_INICIO;
	CMDUartISR= 0;
	CMDTaskISR = 0;
	CheckSum = 0;
}
