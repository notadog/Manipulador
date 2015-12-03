clc;
clear;
close;

load("dump.txt");
teta1 = dump([1:end],1);
teta2 = dump([1:end],2);
x     = dump([1:end],3);
y     = dump([1:end],4);

%subplot(3,1,1);
%plot([teta1,teta2]);
%title("Espaço de juntas");
%xlabel ("Amostras");
%ylabel ("Posição");
%%text (pi, 0.7, "arbitrary text");
%legend ("Teta1(t)", "Teta2(t)");

%subplot(3,1,2);
%plot([x,y]);
%title("Espaço cartesiano")
%xlabel ("Amostras");
%ylabel ("Posição");
%legend ("X(t)", "Y(t)");

%Plot
%subplot(3,1,3);
plot(x,y, "k+");
title("Pontos Percorridos")
xlabel ("X");
ylabel ("Y");
