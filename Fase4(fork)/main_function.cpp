#include "bib.cpp"

int main (int argc, char *argv[]){

	Robo Manipulador;
	double x, y, vel;

	//Tratamento de argumentos recebidos pela linha de comando
	tratamento_argumentos (argc, argv);

	while (!feof(arq)){
		//Entrada de dados
		entrada_dados (&x, &y, &vel);

		//Movimenta
		Manipulador.moveto(Posicao(x, y, 'c'), vel);
	}
}
