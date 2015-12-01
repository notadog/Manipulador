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

/** --------------------------------------- PROTOTIPOS DE FUNÇÕES E CLASSES --------------------------------------- **/
class Servo;
void tratamento_argumentos (int argc, char* argv[]);
void entrada_dados (double *x, double *y, double *velocidade);
bool valida_trajetoria (Servo PosAtual, Servo PosFutura);
void posiciona_sistema ( Servo *PosFutura, Servo *PosAtual, double velocidade );
void comunicaSerial (Servo p1);
void delay (int ms);
void error_log (int codigo);

double L1 = 10;
double L2 = 10;

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
			teta2 = fabs(acos((x*x + y*y -L1*L1 -L2*L2)/(2*L1*L2)));
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

/** ------------------------------------------- PARÂMETROS DO PROJETO --------------------------------------------- **/
int REFRESH_TIME = 30;

/** --------------------------------------------- VARIÁVEIS GLOBAIS ----------------------------------------------- **/
FILE *porta=NULL;
FILE *arq=NULL;
FILE *dump_file=NULL;

bool tem_arduino=false;
bool debug=false;
bool dump=false;
bool entrada_arquivo=false;

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
	fprintf (porta, " %s\n %s\n", str1, str2);
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
	int quant_passos = sqrt(dx*dx + dy*dy)/0.05;					//10 cm -> 100 passos

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
		PosAtual->CinematicaInversa(true);

		//Passa a nova posição desejada para o arduino
		comunicaSerial (*PosAtual);
	}

	PosAtual->x = PosFutura->x;
	PosAtual->y = PosFutura->y;
}

/*Função que dado um código de erro, manda para o stderr mais informações sobre o erro detectado
	  0- 99 -> erros relacionados à posicionar o servo
	100-199 -> erros relacionados à CLI
	200-299 -> erros relacionados à trajetória
*/
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
			fprintf (stderr, " *** Diretiva nao conhecida ***\n");
			break;

		case 102:
			fprintf (stderr, "*** Apenas um arquivo fonte por favor ***\n");
			break;

		case 200:
			fprintf (stderr, " *** Trajetoria passa por uma regiao ilegal ***\n");
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
			if (!strcmp(argv[i], "-debug")){									/*Programa roda avisando para o usuário as diversas informações relevantes*/
				debug = true;
				continue;
			}

			if (!strcmp(argv[i], "-dump")){										/*Programa roda e coloca todos os valores calculados relevantes em dump.txt, para ser plottado*/
				dump = true;
				dump_file = fopen("dump.txt", "w+");
				continue;
			}

			error_log(101);
			continue;
		}

		//Um argumento sem - soh pode ser um endereço de arquivo fonte
		if ((entrada_arquivo == false) && (arq=fopen(argv[i],"r")) != NULL)
			entrada_arquivo = true;
		else if (entrada_arquivo == false){										/*Não consegui abrir, arquivo não encontrado*/
			error_log (100);
			exit(0);
		}																		/*Se tem arquivo aberto e tentou abrir um segundo, informe que um arquivo por vez*/
		else{
			error_log (102);
			exit(0);
		}
	}

	if(entrada_arquivo == false)
		arq = stdin;
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

	if (debug == true){
		printf ("\n---------- Parametros da linha ----------\n");
		printf ("a = %5.2lf\tb = %5.2lf\tb = %5.2lf\n", a, b, c);
		printf ("Dist: %5.2lf\n", dist);
	}

	//SE PASSAR MUITO PERTO DO CIRCULO DE RAIO L2 LOCALIZADO NO PONTO (-L1, 0), SIGNIFICA QUE
	// A TRAJETÓRIA ESTÁ PASSANDO PELA REGIÃO PROIBIDA
	if (PosFutura.x<=0 && dist < L2){
		error_log(200);
		return false;
	}
	return true;
}

/*
1) Calcula uma trajetória r(t) = (x(t),y(t)) que leva da posicao atual ateh a posicao final

x(t) = x_i + delta_x * t ; t pertence a [0,1]
y(t) = y_i + delta_y * t ; t pertence a [0,1]

2) Dado um ponto P0 = (x0,y0) = (-10,0)

3)Defino a funcao:

d_quad = |P_atual - P0| ^2 = (P_atual.x -x0)^2 + (P_atual.y - y0)^2 ; t pertence a [0,1]

4) Encontro o ponto de mínimo da função e retorno true se min{d_quad} < 100

---- mat
Como P_atual eh (t), d_quad eh funcao de t:

d_quad(t) = (x(t) -x0)^2 + (y(t) - y0)^2
d_quad(t) = (x_i + delta_x*t -x0)^2 + (y_i + delta_y*t - y0)^2
d_quad(t) = {(x_i -x0)^2 + (y_i -y0)^2} + t * 2{delta_x*(x_i - x0) + delta_y(y_i - y0)} + t^2 * {delta_x^2 + delta_y^2}

Derivando e igualando a zero para encontrar os pontos críticos:

dot(d_quad(t_crit)) = 0 -> delta_x*(x_i - x0) + delta_y(y_i - y0) = -t_crit * {delta_x^2 + delta_y^2}
dot(d_quad(t_crit)) = 0 -> delta_x*(x_i - x0) + delta_y(y_i - y0) = -t_crit * {delta_x^2 + delta_y^2}

t_crit = - [delta_x*(x_i - x0) + delta_y(y_i - y0)] / [delta_x^2 + delta_y^2]


*/
bool valida_trajetoria_2 (Servo PosAtual, Servo PosFutura){

	double delta_x = PosFutura.x - PosAtual.x;
	double delta_y = PosFutura.y - PosAtual.y;

	double t_crit = -1*(delta_x*(PosAtual.x + 10) + delta_y*PosAtual.y)/(pow(delta_x,2) + pow(delta_y, 2));
	double d_quad_crit;

	double x_i = PosAtual.x;
	double y_i = PosAtual.y;
	double x0 = -10;
	double y0 = 0;

	double d_quad_ini = pow(x_i -x0, 2) + pow(y_i -y0, 2);
	double d_quad_fim = pow(x_i -x0, 2) + pow(y_i -y0, 2) + 2*(delta_x*(x_i - x0) + delta_y*(y_i - y0)) + pow(delta_x,2) + pow(delta_y, 2);

	/*Ponto crítico dentro da trajetória*/
	if ((t_crit > 0) and (t_crit < 1))
		d_quad_crit = pow(x_i -x0, 2) + pow(y_i -y0, 2) + t_crit * 2*(delta_x*(x_i - x0) + delta_y*(y_i - y0)) + t_crit*t_crit * (pow(delta_x,2) + pow(delta_y, 2));

	/*Ponto crítico fora da trajetória*/
	else
		d_quad_crit = INFINITY;

	/*Se a menor dessas três distâncias for algo maior que 100, retorne true e caso contrário retorne falso*/
	return fmin (d_quad_ini, fmin(d_quad_fim, d_quad_crit)) - 100 > 0;
}
