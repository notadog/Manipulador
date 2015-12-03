clc;
clear;
close;

load("dump.txt");
teta1_ideal = dump([1:end],1);
teta2_ideal = dump([1:end],3);
teta1_real  = dump([1:end],2);
teta2_real  = dump([1:end],4);
x_ideal     = dump([1:end],5);
y_ideal	    = dump([1:end],7);
x_real      = dump([1:end],6);
y_real	    = dump([1:end],8);

%Assim, alterando o terceiro parâmetro, é possível fazer três gráficos, um acima do outro
%subplot(3,1,1);
%plot([teta1_real,teta2_real]);
%title("Espaço de juntas");
%xlabel ("Amostras");
%ylabel ("Posição");
%%text (pi, 0.7, "arbitrary text");
%legend ("Teta1(t)", "Teta2(t)");


figure (1)
plot(x_real,y_real, "k+");
title("Pontos Percorridos");
xlabel ("X");
ylabel ("Y");

%Gráfico dos erros, mostrando o quanto erra em x e o quanto em y

erro_x = x_real - x_ideal;
erro_y = y_real - y_ideal;
erro_max = max(erro_x.^2 + erro_y.^2);
maior_diff = max(max([erro_x,erro_y]));
range = 1.5*maior_diff;



figure (2)
plot(erro_x, erro_y, "k+");
title("Erro");
xlabel ("Erro em X");
ylabel ("Erro em Y");
axis([-range, range, -range, range]);
