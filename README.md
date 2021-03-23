# Teste Tecnico Klever 

O desafio técnico consiste em desenvolver um sistema de tempo real completo que recebe comandos via serial e os executa.


Os comandos são:

    Ligar LED
    Desligar LED
    Inverter (toggle) LED
    Loop back (enviar de volta o mesmo pacote recebido)
    Ler conversor AD (valor mais recente, em 12 bits) 

# Requisitos

- Disponibilize o código no Github

- Utilize o sistema operacional FreeRTOS

- Divida a aplicação em tarefas:

    uma para ler o conversor ADC periodicamente
    outra para acionar o LED
    outra para tratar os comandos da serial


- Utilize filas (Queue) para transferir os dados de uma tarefa para outra

- Faça o recebimento dos dados da serial por interrupção.

# Protocolo Comunicação Serial

 Os comandos e as respostas devem ser enviados seguindo um protocolo proprietário, em que:

    1º byte é 0x01
    2º byte: comando ('L' para LED, 'A' para ADC e 'S' para LoopBack)
    3º byte: quantidade de bytes de parâmetro (0 se não houver)
    4º byte em diante: parâmetros (quando aplicável)
    último byte: checksum (byte menos significativo da soma simples de todos os bytes anteriores)


- Os comandos dos LEDs (que não são de leitura) devem retornar uma mensagem de ACK (confirmação de recebimento). 
- Para o comando de LED ('L'), o parâmetro 'L' representa acionar led, parâmetro 'D' para desligar o led e parâmetro 'T' para inverter o estado lógico do LED.

# Configurações

- O microcontrolador utilizado é o STM32F401RE (tomar como referência a placa NUCLEO-F401RE).

- Devem ser utilizados os recursos de hardware:

	- USART2 @ 115200,N,8,1 nos pinos PA2 e PA3

	- Entrada analógica no pino PA0

	- LED no PA5