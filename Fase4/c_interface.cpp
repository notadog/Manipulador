#include "c_bib.cpp"

int main (int argc, char *argv[]){

	/*TRATAMENTO DE ARGUMENTOS RECEBIDOS PELA LINHA DE COMANDO*/
	tratamento_argumentos (argc, argv);

	Servo PosAtual, PosFutura;				// VARIÁVEIS PARA SEGURAR AS EXTREMIDADES DO MOVIMENTO
	double x, y, velocidade;				// VARIÁVEIS PARA SEGURAR A ENTRADA DE DADOS

	/*LOOP DE FUNCIONAMENTO DO PROGRAMA. A SAÍDA É QUANDO ACABAR OS DADOS DO ARQUIVO OU O USUÁRIO INFORMAR CRTL+Z NA LINHA DE COMANDO*/
	while (!feof(arq)) {

		/*LÊ UMA INSTRUÇÃO*/
		entrada_dados(&x, &y, &velocidade);

		/*SE DER ALGUM BUG, ESSA INFORMAÇÃO EH IMPORTANTE*/
		if (debug == true){
			printf ("\n------------------------------ Entrada de dados ------------------------------\n");
			printf ("x = %2.2lf\ty = %2.2lf\tvel = %2.2lf\n", x, y, velocidade);
			getchar();
		}

		/*CALCULE A CINEMÁTICA INVERSA, PEDINDO PARA ARREDONDAR*/
		PosFutura.CinematicaInversa(x, y, true);

		/*SE A POSIÇÃO CALCULADA NÃO FOR LEGAL, IGNORE ESSA INSTRUÇÃO*/
		if (PosFutura.pos_valida == false)
			continue;

		/*CALCULA A CINEMÁTICA DIRETA, SEM PEDIR PARA ARREDONDAR*/
		PosFutura.CinematicaDireta(false);

		/*VALIDA A TRAJETÓRIA ANTES DE MOVIMENTAR*/
		if (valida_trajetoria(PosAtual, PosFutura) == false)
			continue;

		/*ANTES DE MOVIMENTAR, INFORMAR O QUE QUE ESTÁ ROLANDO*/
		if (entrada_arquivo == false){
			printf ("---------- Movimento ----------\n");
			printf ("\t(x,y)\t\t= (%5.2lf, %5.2lf)\t->\t(x,y)\t\t = (%5.2lf, %5.2lf) \n", PosAtual.x, PosAtual.y, PosFutura.x, PosFutura.y);
			printf ("\t(teta1,teta2)\t= (%5.2lf, %5.2lf)\t->\t(teta1,teta2)\t = (%5.2lf, %5.2lf)\n\n", PosAtual.teta1, PosAtual.teta2, PosFutura.teta1, PosFutura.teta2);
		}

		/*MOVIMENTA O SISTEMA DA POSIÇÃO INICIAL PARA A POSIÇÃO FINAL*/
		if (PosFutura.pos_valida == true)
			posiciona_sistema(&PosFutura, &PosAtual, velocidade);
	}
}
