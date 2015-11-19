// Include the servo library
#include <Servo.h>

// Criando os dois objetos que representam os dois servos, o da base e o do elo
Servo BaseServo, LinkServo;

void setup() {
	BaseServo.attach(10);
	LinkServo.attach(9);
	BaseServo.write(0);
	LinkServo.write(0);
	Serial.begin(9600);
}

void loop() {

int angulo1, angulo2;

	while (Serial.available() < 2);		/*Espera chegar dados na porta */
	angulo1 = Serial.parseInt();		/*Pega o primeiro dado*/

	while (Serial.available() < 2);		/*Espera chegar dados na porta */
	angulo2 = Serial.parseInt();		/*Pega o segundo dado*/

	BaseServo.write(angulo1);			/*Escreve a nova posicao do angulo*/
	LinkServo.write(angulo2);			/*Escreve a nova posicao do angulo*/

	Serial.println("\n\nAngulo1: ");	/*Retorna pelo para o computador o angulo lido pelo sensor*/
	Serial.print(angulo1);				/*Retorna para o computador o angulo lido pelo sensor*/
	Serial.println("\nAngulo2: ");		/*Retorna para o computador o angulo lido pelo sensor*/
	Serial.print(angulo2);				/*Retorna para o computador o angulo lido pelo sensor*/
}
