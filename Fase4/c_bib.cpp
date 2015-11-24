/** ----------------------------------------------- BIBLIOTECAS --------------------------------------------------- **/
#include <cmath>							//sen(), cos(), acos(), atan2()
#include <stdio.h>							//printf(), fprintf(), snprintf(), fscanf(), stdin, FILE
#include <stdlib.h>							//exit()
#include <time.h>							//clock()
#include <string.h>							//strcmp()

#ifdef _WIN32
	#include "windows_serial.cpp"
#endif

#define PI 3.141592653589793

/** ------------------------------------------- PARÂMETROS DO PROJETO --------------------------------------------- **/
double L1 = 10;
double L2 = 10;
int REFRESH_TIME = 30;

/** --------------------------------------- PROTOTIPOS DE FUNÇÕES E CLASSES --------------------------------------- **/
class Servo;
void tratamento_argumentos (int argc, char* argv[]);
void entrada_dados (double *x, double *y, double *velocidade);
bool valida_trajetoria (Servo PosAtual, Servo PosFutura);
void posiciona_sistema ( Servo *PosFutura, Servo *PosAtual, double velocidade );
void comunicaSerial (Servo p1);
void delay (int ms);
void error_log (int codigo);

/** --------------------------------------------- VARIÁVEIS GLOBAIS ----------------------------------------------- **/
FILE *porta=NULL;
FILE *arq=NULL;
FILE *dump_file=NULL;

bool tem_arduino=false;
bool debug=false;
bool dump;
bool entrada_arquivo = false;

/** ---------------------------------------------- DEFINICAO DE PAR ----------------------------------------------- **/
class Servo {
	public:

		/**Variáveis*/
		double x, y;
        double teta1, teta2;
		bool pos_valida;
		bool degrees;				//true -> graus (-180,180)  false -> rad (-pi,pi)

        /**Construtor para inicializar a classe*/
        Servo () {
			x=20;
			y=teta1=teta2=0;
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

			/*Circulo Externo, pontos inalcançáveis*/
			if ( x*x + y*y > (L1+L2)*(L1+L2)){
				error_log(0);
				pos_valida = false;
				return;
			}

			/*Calcule teta2 e se for negativo, temos que trocar o sinal para pegar a outra solução. Depois calcula teta1*/
			if ( (teta2=acos((x*x + y*y -L1*L1 -L2*L2)/(2*L1*L2))) < 0)
				teta2 *= -1;
			teta1 = atan2( y*(L1 +L2*cos(-1*teta2))-x*L2*sin(-1*teta2) , x*(L1 +L2*cos(-1*teta2)) + y*L2*sin(-1*teta2) );

			/*Circulo interno, pontos ilegais com a montagem atual*/
			if (teta1 < 0 || teta2<0){
				error_log(1);
				pos_valida = false;
				return;
			}

			/*Se passar em todos os testes antes daqui, eh valida*/
			pos_valida = true;

			/*As funções calculam em radianos, temos que converter para graus*/
			degrees = false;
			convert_degrees();

			/*Se pedir arredondamento, arredonde*/
			if (arredondar == true){
				teta1 = round(teta1);
				teta2 = round(teta2);
			}
		}

		void CinematicaInversa (double x_new, double y_new, bool arredondar){
			x = x_new;
			y = y_new;

			CinematicaInversa(arredondar);
		}

		void CinematicaDireta (bool arredondar){

			/*Circulo interno, pontos ilegais com a montagem atual*/
			if (teta1 < 0 || teta2<0){
				error_log(1);
				pos_valida = false;
				return;
			}

			/*Calcula a cinemática direta*/
			convert_rads();
			x = L1*cos(teta1) + L2*cos(teta1-teta2);
			y = L1*sin(teta1) + L2*sin(teta1-teta2);
			convert_degrees();

			/*Circulo Externo, pontos inalcançáveis*/
			if ( x*x + y*y > (L1+L2)*(L1+L2)){
				error_log(0);
				pos_valida = false;
				return;
			}

			/*Se passar em todos os testes antes daqui, eh valida*/
			pos_valida = true;

			/*Se pedir para arredondar, arredonde*/
			if (arredondar == true) {
				x = round(10*x)/10;
				y = round(10*y)/10;
			}
		}

		void CinematicaDireta (double teta1_new, double teta2_new, bool arredondar){
			teta1 = teta1_new;
			teta2 = teta2_new;
			CinematicaDireta(arredondar);
		}
};

/** -------------------------------------------------- FUNCOES ---------------------------------------------------- **/

void delay (int ms){

	long ticks = ms * CLOCKS_PER_SEC/1000;
	clock_t comeco, atual;

	for (comeco = atual = clock(); atual - comeco < ticks ; atual = clock() );
}

/*Função em Linux para fazer a comunicação serial*/
#ifdef __linux__
void comunicaSerial (Servo p1 ){
	char str1[5], str2[5];

	/*Se não tiver comunicação estabelecida, tenta abrir alguma das portas*/
	if ( tem_arduino==false &&
		(porta = fopen("/dev/ttyACM0", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM1", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM2", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM3", "w+"))==NULL )
		return;
	tem_arduino = true;

	snprintf(str1, sizeof(str1), "%d", (int) round(p1.teta1));
	snprintf(str2, sizeof(str2), "%d", (int) round(p1.teta2));
	fprintf (porta, "%s %s\n", str1, str2);
	delay (REFRESH_TIME);
}
#endif // __linux__

/*Função em Windows para fazer a comunicação serial*/
#ifdef _WIN32
Serial *SP;
void comunicaSerial (Servo p1 ){
	char str1[5], str2[5];

	/*Se não houver comunicação com um Arduíno, tente abrir*/
	if(tem_arduino == false){
		SP = new Serial("\\\\.\\COM10");    // adjust as needed
	}

	/*Se não funcionar, deixa pra lá*/
	if (SP->IsConnected() == false)
		return;

	/*Se funcionar, faça a comunicação*/
	tem_arduino = true;
	snprintf(str1, sizeof(str1), "%d", (int) round(p1.teta1));
	snprintf(str2, sizeof(str2), "%d", (int) round(p1.teta2));

	delay (REFRESH_TIME);
}
#endif

void posiciona_sistema ( Servo *PosFutura, Servo *PosAtual, double velocidade ){

	//MOVIMENTAÇÃO
	double xi = PosAtual->x, yi = PosAtual->y;				//Segurando a posicao inicial
	double dx = PosFutura->x - xi, dy = PosFutura->y - yi;	//Segurando o deltax e deltay
	int quant_passos = sqrt(dx*dx + dy*dy)/0.1;					//10 cm -> 100 passos

	/*Converte para graus*/
	PosAtual->convert_degrees();
	PosFutura->convert_degrees();

	/*Enquanto nao chegarmos à posição final*/
	for (int i=1; i<=quant_passos; i++) {

		if (dump == true)
			fprintf (dump_file, "%5.2lf\t%5.2lf\t%5.2lf\t%5.2lf\t\n", PosAtual->teta1, PosAtual->teta2, PosAtual->x, PosAtual->y);

		if (debug == true)
			printf (" -> Angulo atual (%5.2lf %5.2lf)\n", PosAtual->teta1, PosAtual->teta2);

		PosAtual->x = xi + dx*i/quant_passos;
		PosAtual->y = yi + dy*i/quant_passos;

		//Dado o novo (x,y), calcule o (teta1, teta2)
		PosAtual->CinematicaInversa(false);

		//Passa a nova posição desejada para o arduino
		comunicaSerial (*PosAtual);
	}

	PosAtual->x = PosFutura->x;
	PosAtual->y = PosFutura->y;
}

void error_log (int codigo){

	switch(codigo){
		case 0:
			fprintf (stderr, " *** Nao foi possivel encontrar uma solucao ***\n");
			break;

		case 1:
			fprintf (stderr, " *** Nao foi possivel posicionar o sistema ***\n");
			break;

		case 100:
			fprintf (stderr, " *** Nao foi possivel encontrar o arquivo fonte *** \n");
			break;

		case 101:
			fprintf (stderr, " *** Diretiva nao conhecida *** \n");
			break;

		case 200:
			fprintf (stderr, " *** Trajetoria passa por uma regiao ilegal *** \n");
			break;
	};
}

/*TRATAMENTO DE ARGUMENTOS PARA O PROGRAMA*/
void tratamento_argumentos (int argc, char* argv[]){

	/*Senão tiver argumentos, a entrada de dados eh pelo teclado e pode retornar*/
	if (argc == 1){
		arq = stdin;
		return;
	}

	/*Para cada argumento, execute*/
	for (int i=1; i<argc; i++){

		if (*argv[i]=='-'){
			if (!strcmp(argv[i], "-debug")){
				debug = true;
				continue;
			}

			if (!strcmp(argv[i], "-dump")){
				dump = true;
				dump_file = fopen("dump.txt", "w+");
				continue;
			}

			error_log(101);
			continue;
		}

		//Um argumento pode ser um endereço de arquivo fonte
		if ((entrada_arquivo == false) && (arq=fopen(argv[i],"r")) != NULL){
			entrada_arquivo = true;
		}
		else{
			error_log (100);
			exit(0);
		}
	}
}

void entrada_dados (double *x, double *y, double *velocidade){

	if (entrada_arquivo == false)
		printf ("\n --> Informe a proxima posicao: ");

	fscanf (arq, "%lf %lf", x, y);

	if(getc(arq)==' ')
		scanf ("%lf", velocidade);
	else
		*velocidade = 1;
}

/*
Calcula os parametros a, b e c da reta em geometria analítica que passa pela posição inicial e final
a*(x) + b*(y) + c = 0
Depois calcula a distância mínima entre essa reta e um ponto x0, y0 definido como (-L1, 0)

Se a distância entre a reta desejada e o ponto for menor que 10 ele está passando pela região ilegal
nesse caso, dê a mensagem de erro e não movimente os motores e não atualize a posição inicial
*/
bool valida_trajetoria (Servo PosAtual, Servo PosFutura){

	double delta_x = PosFutura.x - PosAtual.x;
	double delta_y = PosFutura.y - PosAtual.y;

	double a, b, c;

	if (delta_x != 0){
		a = -1*delta_y/delta_x;
		b = 1;
		c = -1*(PosAtual.y * delta_x - PosAtual.x * delta_y)/delta_x;
	}
	else{
		a = 0;
		b = 1;
		c = 0;
	}

	//DIST = MENOR DISTÂNCIA DO PONTO (-10, 0) ATÉ A RETA QUE PASSA POR POSATUAL E POSFUTURA
//	double dist = fabs(a*(x) + b*(y) c)/sqrt(a*a + b*b);
	double dist = fabs(a*(-1*L1) + c)/sqrt(a*a + b*b);

	//SE PASSAR MUITO PERTO DO CIRCULO DE RAIO L2 LOCALIZADO NO PONTO (-L1, 0), SIGNIFICA QUE
	// A TRAJETÓRIA ESTÁ PASSANDO PELA REGIÃO PROIBIDA
	if (PosFutura.x<0 && dist < L2){
		error_log(200);
		return false;
	}
	return true;
}
