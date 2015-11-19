#include "c_bib.cpp"

int main (){

	Servo PosAtual;
	Servo PosFutura;
	double velocidade;		//0 - 1 (percentual da velocidade máxima)

	porta = NULL;
	tem_arduino = false; //Ainda nao sei se tem arduino na USB

	while (true){

		/*Entrada de dados*/
        printf ("\n --> Informe a proxima posicao: ");
		scanf ("%lf %lf", &PosFutura.x, &PosFutura.y);

		if(getchar()==' ')
			scanf ("%lf", &velocidade);
		else
			velocidade = 1;

		/*Calcule a cinemática inversa, pedindo Servoa arredondar e a direta sem pedir Servoa arredondar*/
		PosFutura.CinematicaInversa(true);
		PosFutura.CinematicaDireta(false);

		/*Se a posição calculada não for realizável ignore o comando*/
		if (PosFutura.pos_valida == false)
			continue;

		/*Se não entrou naquele if, então a solucao eh viavel */
		printf ("----------------------------------------\n");
		printf ("Posicao Atual (x,y) = (%.2f, %.2f)\n", PosAtual.x, PosAtual.y);
		printf ("Posicao Atual (teta1,teta2) = (%.2f, %.2f)\n\n", PosAtual.teta1, PosAtual.teta2);

		printf ("Posicao futura (x,y) = (%.2f, %.2f)\n", PosFutura.x, PosFutura.y);
		printf ("Posicao futura (teta1,teta2) = (%.2f, %.2f)\n", PosFutura.teta1, PosFutura.teta2);

//		posiciona_sistema(&PosFutura, &PosAtual, velocidade);
		posiciona_sistema_pol(&PosFutura, &PosAtual, velocidade);
	}
}

