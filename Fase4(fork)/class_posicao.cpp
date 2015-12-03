/**
CLASSE PARA REPRESENTAR UMA POSIÇÃO EM UM PLANO.

ESSA CLASSE PERMITE REPRESENTAR UM PAR ORDENADO EM UMA ESTRUTURA APENAS, FACILITANDO A ORGANIZAÇÃO DA CLASSE ROBÔ QUE PODE CONTER VÁRIAS POSIÇÕES
*/

#define pi 3.141592653589793

class Posicao {
	public:

		/**Variáveis*/
		double x, y;					//coordenadas cartesianas
        double r, teta;					//coordenadas polares
		bool in_degrees;				//true -> teta1 (-180,180) ; false -> rad (-pi,pi)

        /**Construtores para inicializar a classe*/
        /* Três situações:
			char = 'c' (cartesiano), inicializa como (x,y)
			char = 'p' (polares), inicializa como (r,teta), com teta1 e teta2 sendo ângulos em graus
			char = 'r' (polares, angulo em radianos), inicializa como (teta1, teta2), com teta1 e teta2 sendo ângulos em radianos
		*/
        Posicao ();										// Sem parâmetros, inicializa com tudo nulo
        Posicao (double x, double y);					// Com dois parâmetros, inicializa como o usuário informando (x,y)
        Posicao (double x, double y, char texto_input);

		/**Funções*/
		void convertDegrees ();
		void convertRad();
		double dist(Posicao p);

    private:
		static const double degrees_to_rad = pi/180;
		static const double rad_to_degrees = 180/pi;
		void setValues(double a, double b, char texto_input);
};

Posicao::Posicao(){
	setValues(0, 0, 'c');
}

Posicao::Posicao(double x_input, double y_input){
	setValues(x_input,y_input, 'c');
}

Posicao::Posicao(double x_input, double y_input, char texto_input){
	setValues(x_input,y_input, texto_input);
}

void Posicao::setValues(double a, double b, char texto_input){

	if (texto_input == 'c'){
		x = a;
		y = b;
		teta = atan2(b,a);
		r = sqrt(x*x + y*y);
		in_degrees = false;
	}

	if (texto_input == 'p'){
		r = a;
		teta = b;
		x = r*cos(teta*pi/180);
		y = r*sin(teta*pi/180);
		in_degrees = true;
	}

	if (texto_input == 'r'){
		r = a;
		teta = b;
		x = r*cos(teta);
		y = r*sin(teta);
		in_degrees = false;
	}
}

void Posicao::convertDegrees(){
	if (in_degrees == false)
			teta *= rad_to_degrees;
}

void Posicao::convertRad(){
	if (in_degrees == true)
		teta *= degrees_to_rad;
}

double Posicao::dist(Posicao p){
	return sqrt(pow(p.x - x,2)+pow(p.y - y,2));
}
