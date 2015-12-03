class Robo {
	public:
		//Construtores
		Robo();

		//Usuário soh pode pedir para o robor se mover, não tem controle interno nenhum
		void moveto(Posicao p, double vel);

	private:

		//Informações básicas
		Posicao Patual;			//Posição no plano de trabalho (em coordenadas cartesianas ou polares)
		Posicao Pdesejada;		//Posição de interesse no plano de trabalho
		double velocidade;		//Velocidade para atingir o ponto desejado
		double teta1, teta2;	//Juntas do robô

		//Validação das entradas do usuário
		bool pos_valida;
		bool trajetoria_valida;

		//Estados do robo
		bool arduino_detectado;
		bool in_degrees;

		//Comprimento dos elos
		static const double L1 = 10;
		static const double L2 = 10;

		//Tempo entre a emissão de comando para o Arduíno
		static const double FRAME_WINDOW = 40;					//em ms
		static const double passos_per_cm = 5;

		//Funções para mapear volume de trabalho e posição de juntas
		Posicao CinDir(double teta1_in, double teta2_in, char c);
		void bestround(Posicao p, double *teta1_pointer, double *teta2_pointer, Posicao *p_round);
		double AnguloLink();
		double AnguloLink(Posicao p);
		double AnguloBase();
		double AnguloBase(Posicao p, double teta2);

		//Conversão de unidade radial (graus) <-> radianos
		void converteToGraus();
		void converteToRad();
		static const double degrees_to_rad = pi/180;
		static const double rad_to_degrees = 180/pi;

		//Validação de trajetória para garantir que o caminho para a próxima posição é válido
		bool validaTrajetoria(Posicao p);
		double distTrajetoriaPonto(Posicao Pi, Posicao Pf, Posicao p0);

		//Comunicação com o robô
		void abreComunicacao();
		void comunicacaoRobo(int teta1, int teta2);
		FILE *porta;
};

/*Inicialização do robô*/
Robo::Robo(){

	/*Posição inicial (20,0)*/
	Patual = Posicao(20, 0);
	Pdesejada = Posicao(20,0);
	velocidade = 0;
	teta1 = 0;
	teta2 = 0;
	in_degrees = false;
	pos_valida = true;
	arduino_detectado = false;
	trajetoria_valida = true;

	//Abre a comunicação com o arduíno (se estiver conectado) e posiciona na posição inicial
	abreComunicacao();
	comunicacaoRobo(teta1, teta2);
}

/*Calcula o ângulo do elo com base em uma posição p = (x0,y0) dada com base na cinemática inversa própria do robô*/
double Robo::AnguloLink(Posicao p){
	return fabs(acos((p.x*p.x + p.y*p.y -L1*L1 -L2*L2)/(2*L1*L2)));
}

/*Calcula o ângulo do elo com base em na posição 'Patual' dada com base na cinemática inversa própria do robô*/
double Robo::AnguloLink(){
	return AnguloLink(Patual);
}

/*Calcula o ângulo do elo com base em uma posição p = (x0,y0) dada e um ângulo do link em radianos com base na cinemática inversa própria do robô*/
double Robo::AnguloBase(Posicao p, double teta2){
	return atan2( p.y*(L1 +L2*cos(teta2))+p.x*L2*sin(teta2) , p.x*(L1+L2*cos(teta2)) - p.y*L2*sin(teta2) );
}

/*Calcula o ângulo do elo com base em uma posição 'Patual' dada e um ângulo 'Teta2'*/
double Robo::AnguloBase(){
	converteToRad();
	return atan2( Patual.y*(L1 +L2*cos(teta2))+Patual.x*L2*sin(teta2) , Patual.x*(L1+L2*cos(teta2)) - Patual.y*L2*sin(teta2) );
}

void Robo::converteToGraus(){
	if (in_degrees == false){
			teta1 *= rad_to_degrees;
			teta2 *= rad_to_degrees;
			in_degrees = !in_degrees;
	}
}

void Robo::converteToRad(){
	if (in_degrees == true){
		teta1 *= degrees_to_rad;
		teta2 *= degrees_to_rad;
		in_degrees = !in_degrees;
	}
}

/*Passa para o Robô pela porta serial */
void Robo::comunicacaoRobo(int teta1, int teta2){
/*Função em Linux para fazer a comunicação serial os dois ângulos das juntas como texto ASCII int=180 -> char[]="180" */
	#ifdef __linux__
	char str1[4], str2[4];
	snprintf(str1, sizeof(str1), "%d", teta1);
	snprintf(str2, sizeof(str2), "%d", teta2);
	fprintf (porta, "%s %s\n", str1, str2);
	usleep( FRAME_WINDOW*1000);
	#endif // __linux__
}

void Robo::abreComunicacao (){
	#ifdef __linux__
	/*Se não tiver comunicação estabelecida, tenta abrir alguma das portas*/
	if ( arduino_detectado==false &&
		(porta = fopen("/dev/ttyACM0", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM1", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM2", "w+"))==NULL &&
		(porta = fopen("/dev/ttyACM3", "w+"))==NULL ){
		porta = stdin;
		return;
	}
	arduino_detectado = true;
	#endif // __linux__
}

/*Baseado em um par de ângulos, retorna */
Posicao Robo::CinDir(double teta1_in, double teta2_in, char c) {

	//teta1 e teta2 nem em graus e nem em radianos
	if (c != 'g' and c != 'r'){
		fprintf (stderr, "WTF? Angulo em graus ('g') ou radianos ('r').");
		exit(1);
	}

	//teta1 e teta2 em graus, passa para radianos
	if (c == 'g'){
		teta1_in *= degrees_to_rad;
		teta2_in *= degrees_to_rad;
	}

	return Posicao(L1*cos(teta1_in) + L2*cos(teta1_in-teta2_in), L1*sin(teta1_in) + L2*sin(teta1_in-teta2_in), 'c' );
}

/*Calcula os ângulos para atingir a posição 'p' e depois arredonda para o ângulo que melhor atinge o objetivo.
Retorna os ângulos nos ponteiros recebidos como argumentos.

Argumentos:
	posição de interesse 'p' (x,y)
	ponteiro para guardar o teta1 que melhor aproxima
	ponteiro para quardar o teta2 que melhor aproxima
	ponteiro para guardar 'p_round' a posição resultante do melhor p
*/
void Robo::bestround(Posicao p, double *teta1_pointer, double *teta2_pointer, Posicao *p_round) {

	//Calculo os ângulos que geram a posição final desejada
	double teta2_loc = AnguloLink(p);
	double teta1_loc = AnguloBase(p, teta2_loc);

	teta1_loc *= rad_to_degrees;
	teta2_loc *= rad_to_degrees;

	//Tenho quatro opcoes de arredondamento: up or down, vezes 2 ângulos
	Posicao opcoes[4];
	opcoes[0] = CinDir(floor(teta1_loc), floor(teta2_loc), 'g');
	opcoes[1] = CinDir(floor(teta1_loc), ceil (teta2_loc), 'g');
	opcoes[2] = CinDir(ceil (teta1_loc), floor(teta2_loc), 'g');
	opcoes[3] = CinDir(ceil (teta1_loc), ceil (teta2_loc), 'g');

	//Calcula qual das quatro opcoes de arredondamento tem a menor distancia para a posicao 'p' de interesse
	int j=0;
	for (int i=0; i<4; i++){
		if (opcoes[i].dist(p) < opcoes[j].dist(p))
			j = i;
	}

	//Passa a melhor opção para os ponteiros
	*p_round = opcoes[j];
	switch (j){
		case 0:
			*teta1_pointer = floor(teta1_loc);
			*teta2_pointer = floor(teta2_loc);
			break;
		case 1:
			*teta1_pointer = floor(teta1_loc);
			*teta2_pointer = ceil(teta2_loc);
			break;
		case 2:
			*teta1_pointer = ceil(teta1_loc);
			*teta2_pointer = floor(teta2_loc);
			break;
		case 3:
			*teta1_pointer = ceil(teta1_loc);
			*teta2_pointer = ceil(teta2_loc);
			break;
	};
}

/*Com base em um ponto inicial e um ponto final, calcula a distância mínima da trajetória linear pi -> pf até o ponto p0

	calcular o min d(t) = (x(t) - x0)^2 + (y(t) - y0)^2
	x(t) = x0 + delta_x*t ; t pertence a [0,1]
	y(t) = y0 + delta_y*t ; t pertence a [0,1]

	-> deriva e iguala a zero
	-> x(t) e y(t) lineares, dot(d(t)) fica linear -> 1 ponto crítico
		*avaliar no ponto crítico dot(d(t_crit))=0
		*avaliar nos limites do domínio t=0 e t=1
*/
double Robo::distTrajetoriaPonto(Posicao Pi, Posicao Pf, Posicao p0){
	double delta_x = Pf.x - Pi.x;
	double delta_y = Pf.y - Pi.y;
	double t_crit = -1*(delta_x*(Pi.x + 10) + delta_y*Pi.y)/(pow(delta_x,2) + pow(delta_y,2));

	double d_i    = pow(Pi.x-p0.x,2) + pow(Pi.y-p0.y,2);
	double d_crit = pow(Pi.x-p0.x,2) + pow(Pi.y-p0.y,2) + t_crit*2*(delta_x*(Pi.x-p0.x) + delta_y*(Pi.y-p0.y)) + pow(t_crit,2)*(pow(delta_x,2) + pow(delta_y,2));
	double d_f    = pow(Pi.x-p0.x,2) + pow(Pi.y-p0.y,2) +      1*2*(delta_x*(Pi.x-p0.x) + delta_y*(Pi.y-p0.y)) + 			  (pow(delta_x,2) + pow(delta_y,2));

	/*Ponto crítico fora da trajetória eh ignorado, min (a, INFINITY) = a*/
	if ( (t_crit < 0) or (t_crit > 1) )
		d_crit = INFINITY;

	/*Retorne a menor das três distâncias*/
	return sqrt(min(d_i, min(d_f, d_crit)));
}

/*Recebe uma posição para se mover, verifica a viabilidade da posição, depois da trajetória e por fim chama a 'move' para executar a movimentação*/
bool Robo::validaTrajetoria(Posicao p){

	//Proibindo se a posição final for muito perto da origem
	if (p.dist(Posicao(  0, 0, 'c')) < 5){
		pos_valida = false;
		return false;
	}

	//Proibindo se a posição final for muito longe da origem
	if (p.dist(Posicao(  0, 0, 'c')) > L1+L2){
		pos_valida = false;
		return false;
	}

	//Proibindo se a posição final for na região proibida acima de x=0
	if (p.dist(Posicao(-L1, 0, 'c')) < L2){
		pos_valida = false;
		return false;
	}

	//Proibindo se a posição final for na região proibida abaix0 de x=0
	if ( (p.x < 0) and (p.dist(Posicao(L1, 0, 'c')) > 5) ){
		pos_valida = false;
		return false;
	}


	//Se algum ponto da trajetória passar muito perto da região proibida
	if (distTrajetoriaPonto(Patual, p, Posicao(-10,0,'c')) < L2 ){
		trajetoria_valida = false;
		return false;
	}

	return true;
}

/*Função para fazer a movimentação do robô, o usuário informa a próxima posição e a gente verifica se pode fazer a movimentação
e se possível, fazemos a movimentação
*/
void Robo::moveto(Posicao p, double vel){

	/*Se a trajetória não for válida, avise o usuário */
	if (validaTrajetoria(p) == false){
		fprintf (stderr, " *** Movimentação ilegal ***\n");
		return;
	}

	/*Jah sei que a trajetória eh válida, agora eh fazer a movimentação*/

	/*Opcao1: linear*/

	/*x(t) = x0 + delta_x *t */
	/*y(t) = y0 + delta_y *t */

	Posicao Pini=Patual, Pideal;
	double t;
	double delta_x = p.x - Patual.x;
	double delta_y = p.y - Patual.y;
	double teta1_arduino, teta2_arduino;
	int quant_passos = round(passos_per_cm*vel*Patual.dist(p));

	//Se o usuário me trollar e mandar eu ficar aonde estou, não execute o loop a seguir
	if (quant_passos == 0)
		return;

	for (int i=0; i<= quant_passos; i++){

		t = 1.0*i/quant_passos;														// calcula o tempo, indo de 0 a 1

		Pideal = Posicao(Pini.x + t*delta_x, Pini.y + t*delta_y, 'c');				// calcula a posição que a equação sugere para o arduino

		/*Recalculo os ângulos da cinemática inversa para a nova posição atual*/
        teta2 = AnguloLink(Pideal);
        teta1 = AnguloBase(Pideal, teta2);
        in_degrees = false;

        /*Calculo os melhores angulos e posicoes para mandar para o arduino*/
        bestround(Pideal, &teta1_arduino, &teta2_arduino, &Patual);					// calcula os angulos que melhor se aproximam da posição sugerida pela equação


		if (debug == true) {
			converteToGraus();
			printf ("\t\tIdealmente:\t\tNa prática:\n");
			printf ("Angulos\t\t(%6.2lf, %6.2lf)\t(%6.2lf, %6.2lf)\n", teta1, teta2, teta1_arduino, teta2_arduino);
			printf ("Posições\t(%6.2lf, %6.2lf)\t(%6.2lf, %6.2lf)\n\n", Pideal.x, Pideal.y, Patual.x, Patual.y);
		}

		if (dump == true){
			converteToGraus();
			fprintf (dump_file, "%6.2lf\t\t%6.2lf\t\t%6.2lf\t\t%6.2lf\t\t", teta1, teta1_arduino, teta2, teta2_arduino);
			fprintf (dump_file, "%5.2lf\t\t%5.2lf\t\t%5.2lf\t\t%5.2lf\n", Pideal.x, Patual.x, Pideal.y, Patual.y);
		}

		comunicacaoRobo(teta1_arduino, teta2_arduino);

	}

}
