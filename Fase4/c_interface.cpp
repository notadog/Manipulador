#include "c_bib.cpp"

int main (int argc, char *argv[]){

	Servo PosAtual, PosFutura;
	double velocidade;					//0 - 1 (percentual da velocidade máxima)

	/*Adicionando parâmetros para o arquivo*/
	tratamento_argumentos (argc, argv);

	if (entrada_arquivo == false)
		arq = stdin;

	/*LOOP DE FUNCIONAMENTO DO PROGRAMA. A SAÍDA É QUANDO ACABAR OS DADOS DO ARQUIVO OU O USUÁRIO INFORMAR CRTL+Z NA LINHA DE COMANDO*/
	while (true){

		/*Acabando o arquivo, fecha o prog*/
		if (feof(arq))
			exit(0);

		if (entrada_arquivo == false)
			printf ("\n --> Informe a proxima posicao: ");

		fscanf (arq, "%lf %lf", &PosFutura.x, &PosFutura.y);

		if(getc(arq)==' ')
			scanf ("%lf", &velocidade);
		else
			velocidade = 1;

		/*Calcule a cinemática inversa, pedindo para arredondar e a direta sem pedir para arredondar*/
		PosFutura.CinematicaInversa(true);
		PosFutura.CinematicaDireta(false);

		/*Se a posição calculada não for realizável ignore o comando*/
		if (PosFutura.pos_valida == false)
			continue;

		/*Posiciona o sistema*/
		posiciona_sistema(&PosFutura, &PosAtual, velocidade);

		/*Se não entrou naquele if, então a solucao eh viavel */
		//if (entrada_arquivo == false){
			printf ("Posicao Atual\n\t(x,y)\t\t = (%.2f, %.2f)\n", PosFutura.x, PosFutura.y);
			printf ("\t(teta1,teta2)\t = (%.2f, %.2f)\n\n", PosFutura.teta1, PosFutura.teta2);
		//}
	}
}

