/** ----------------------------------------------- BIBLIOTECAS --------------------------------------------------- **/
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define PI 3.141592653589793

/** ------------------------------------------- PARÂMETROS DO PROJETO --------------------------------------------- **/
double L1 = 10;
double L2 = 10;
int REFRESH_TIME = 30;
FILE *porta;
bool tem_arduino;
/** ---------------------------------------------- DEFINICAO DE PAR ----------------------------------------------- **/
class Par {
	public:

		/**Variáveis*/
		double x, y;
        double teta1, teta2;
		double pos_valida;
		double degrees;				//true -> graus (0,180)  false -> rad (-pi,pi)

        /**Construtor para inicializar a classe*/
        Par () {
			x=20;
			y=0;
			teta1=0;
			teta2=0;
			pos_valida=true;
			degrees=false;
		};

        /**Funções*/
		void convert_degrees(){
			if (degrees == false){
				teta1 = teta1*180/PI; teta2=teta2*180/PI;
				degrees = true;
			}
		};

		void convert_rads(){
			if (degrees == true){
				teta1 = teta1*PI/180 ; teta2=teta2*PI/180;
				degrees = false;
			}
		};

		void CinematicaInversa (bool arredondar){

			/*Circulo Externo, a solução 'calculada' eh a posicao atual para nao se mexer*/
			if ( x*x + y*y > (L1+L2)*(L1+L2)){
				fprintf (stderr, " *** Nao foi possivel encontrar uma solucao ***\n");
				pos_valida = false;
				return;
			}

			pos_valida = true;

			/*Calcule teta2 e se for negativo, temos que trocar o sinal para pegar a outra solução*/
			if ( (teta2=AnguloLink()) < 0)
				teta2 *= -1;

			/*Calcule teta1*/
			teta1 = AnguloBase();

			/*As funções calculam em radianos, temos que converter para graus*/
			degrees = false;
			convert_degrees();

			/*Se pedir arredondamento, arredonde*/
			if (arredondar == true){
				teta1 = round(teta1);
				teta2 = round(teta2);
			}
		}

		void CinematicaDireta (bool arredondar){

			/*Circulo Externo, a solução 'calculada' eh a posicao atual para nao se mexer*/
			if (teta1 < 0 || teta2<0){
				fprintf (stderr, " *** Nao foi possivel posicionar o sistema ***\n");
				pos_valida = false;
				return;
			}

			/*Calcula a cinemática direta*/
			convert_rads();
			x = L1*cos(teta1) + L2*cos(teta1-teta2);
			y = L1*sin(teta1) + L2*sin(teta1-teta2);
			convert_degrees();

			/*Se pedir para arredondar, arredonde*/
			if (arredondar == true) {
				x = round(10*x)/10;
				y = round (10*y)/10;
			}
		}

		double AnguloLink() {
			return acos((x*x + y*y -L1*L1 -L2*L2)/(2*L1*L2));
		};

		double AnguloBase() {
			return atan2( y*(L1 +L2*cos(-1*teta2))-x*L2*sin(-1*teta2) , x*(L1 +L2*cos(-1*teta2)) + y*L2*sin(-1*teta2) );
		};
};

/** -------------------------------------------------- FUNCOES ---------------------------------------------------- **/

void delay (int ms){

	long ticks = ms * CLOCKS_PER_SEC/1000;
	clock_t comeco, atual;

	for (comeco = atual = clock(); atual - comeco < ticks ; atual = clock() )
		;
}

char* itoa(int i, char b[]){
    char const digit[] = "0123456789";
    char* p = b;
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}

void comunicaSerial ( Par p1 ){
	char str1[5], str2[5];

	/*Se não tiver comunicação estabelecida, tenta abrir alguma das quatro portas*/
	if ( tem_arduino==false &&
		(porta = fopen("/dev/ttyACM0", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM1", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM2", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM3", "w+"))==NULL )
		return;
	tem_arduino = true;

	itoa(( round(p1.teta1)), str1);
	itoa(( round(p1.teta2)), str2);

	fprintf (porta, "%s %s\n", str1, str2);
//	printf ("Par de angulos (%s , %s) \n", str1, str2);
	delay (REFRESH_TIME);
}

int signal (double a){
	if (a != 0)
		return a/abs(a);
	return 1;
}

void posiciona_sistema ( Par *PosFutura, Par *PosAtual, double velocidade ){

	//MOVIMENTAÇÃO
	double d_teta1, d_teta2;				//Direrença de ângulo
	double STEP1, STEP2;				//Deslocamento teta na janela atual
	int dir1, dir2;						//Esquerda ou direita

	/*Converte para graus*/
	PosAtual->convert_degrees();
	PosFutura->convert_degrees();

	/*Calcule o deslocamento para cada junta*/
	d_teta1 = PosFutura->teta1 - PosAtual->teta1;
	d_teta2 = PosFutura->teta2 - PosAtual->teta2;

	/*Calcule o deslocamento por janela*/
	if (d_teta1 == 0 || d_teta2==0){
		STEP1 = STEP2 = velocidade;
	}
	else if (fabs(d_teta1) > fabs(d_teta2)){
		STEP1 = velocidade;
		STEP2 = velocidade * fabs(d_teta2/d_teta1);
	}
	else{
		STEP2 = velocidade;
		STEP1 = velocidade * fabs(d_teta1/d_teta2);
	}


	/*Verificando o sentido do deslocamento, dir=1 (crescente) e dir=-1 (decrescente)*/
	dir1 = signal(d_teta1);
	dir2 = signal(d_teta2);

	/*Enquanto nao chegarmos à posição final (à menos de detalhes de resolução)*/
	while (fabs(d_teta1) + abs(d_teta2) > 0.1){

		printf (" -> Angulo atual (%.2lf %.2lf)\n", PosAtual->teta1, PosAtual->teta2);
//		printf ("Delta1: %.2lf \t Delta2: %.2lf\n", dir1*d_teta1, dir2*d_teta2);

		//Se existe uma diferença em teta1, diminua um pouco
		if (d_teta1 != 0)
			PosAtual->teta1 += dir1*STEP1;

		//Se existe uma diferença em teta2, diminua um pouco
		if (d_teta2 != 0)
			PosAtual->teta2 += dir2*STEP2;

		//Se der um overshoot, a gente compensa
		if ((PosAtual->teta1 - PosFutura->teta1)*dir1 >0)
			PosAtual->teta1 = PosFutura->teta1;
		if ((PosAtual->teta2 - PosFutura->teta2)*dir2 >0)
			PosAtual->teta2 = PosFutura->teta2;

		//Passa a nova posição desejada para o arduino
		comunicaSerial (*PosAtual);

		//Atualiza o delta
		d_teta1 = PosFutura->teta1 - PosAtual->teta1;
		d_teta2 = PosFutura->teta2 - PosAtual->teta2;
	}
	printf (" -> Angulo atual (%.2lf %.2lf)\n", PosAtual->teta1, PosAtual->teta2);
	PosAtual->x = PosFutura->x;
	PosAtual->y = PosFutura->y;
}

void posiciona_sistema_pol ( Par *PosFutura, Par *PosAtual, double velocidade ){

	//MOVIMENTAÇÃO
	double d_teta1, d_teta2;				//Direrença de ângulo
	double STEP1, STEP2, param, xi = PosAtual->x, yi = PosAtual->y;					//Deslocamento teta na janela atual
	int dir1, dir2;							//Esquerda ou direita

	/*Converte para graus*/
	PosAtual->convert_degrees();
	PosFutura->convert_degrees();

	double teta1_ini=PosAtual->teta1;
	double teta2_ini=PosAtual->teta2;

	/*Calcule o deslocamento para cada junta*/
	d_teta1 = PosFutura->teta1 - PosAtual->teta1;
	d_teta2 = PosFutura->teta2 - PosAtual->teta2;

	/*Verificando o sentido do deslocamento, dir=1 (crescente) e dir=-1 (decrescente)*/
	dir1 = signal(d_teta1);
	dir2 = signal(d_teta2);


	int quant_passos = 50;


	/*Enquanto nao chegarmos à posição final (à menos de detalhes de resolução)*/
	for (int i=1; i<=quant_passos; i++) {

		printf (" -> Angulo atual (%.2lf %.2lf)\n", PosAtual->teta1, PosAtual->teta2);
//		printf ("Delta1: %.2lf \t Delta2: %.2lf\n", dir1*d_teta1, dir2*d_teta2);

		param = (-2*pow((1.0*i/quant_passos),3) + 3*pow(1.0*i/quant_passos, 2));
		PosAtual->x = xi + param * (PosFutura->x - xi);
		PosAtual->y = yi + param * (PosFutura->y - yi);
		PosAtual->CinematicaInversa(false);

		//Passa a nova posição desejada para o arduino
		comunicaSerial (*PosAtual);
	}

	printf (" -> Angulo atual (%.2lf %.2lf)\n", PosAtual->teta1, PosAtual->teta2);
	PosAtual->x = PosFutura->x;
	PosAtual->y = PosFutura->y;
}
