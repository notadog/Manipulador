#include <math.h>			//cos, sin, atan2
#include <stdio.h>			//FILE, printf, scanf
#include <string.h>
#include <algorithm>		//max, min

#ifdef __unix__
	#include <unistd.h>
#endif // __unix__

using namespace std;

//Protótipos das classes
class Posicao;			//Classe que representa uma posição (x,y) <-> (teta1, teta2)
class Robo;				//Classe que representa o robô como um todo

//Protótipos das funções próprias
void tratamento_argumentos (int argc, char* argv[]);
void entrada_dados (double *x, double *y, double *velocidade);

//Variáveis globais
bool entrada_arquivo=false;
FILE *arq=NULL;
bool debug=false;
bool dump=false;
FILE *dump_file=NULL;

#include "class_posicao.cpp"
#include "class_robo.cpp"

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
				fprintf (dump_file, "%%Teta1 Ideal\tTeta1 Real\tTeta2 Ideal\tTeta2 Real\tX Ideal\t\tX real\t\tY Ideal\t\tY Real\n");
				continue;
			}

			fprintf (stderr, " *** Diretiva nao conhecida ***\n");
			continue;
		}

		//Um argumento sem - soh pode ser um endereço de arquivo fonte
		if ((entrada_arquivo == false) && (arq=fopen(argv[i],"r")) != NULL)
			entrada_arquivo = true;
		else if (entrada_arquivo == false){										/*Não consegui abrir, arquivo não encontrado*/
			fprintf (stderr, " *** Nao foi possivel encontrar o arquivo fonte *** \n");
			exit(0);
		}																		/*Se tem arquivo aberto e tentou abrir um segundo, informe que um arquivo por vez*/
		else{
			fprintf (stderr, "*** Apenas um arquivo fonte por favor ***\n");
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
