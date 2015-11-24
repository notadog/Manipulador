clc;
clear;
close;

load("dump.txt");
teta1 = dump([1:end],1);
teta2 = dump([1:end],2);
x     = dump([1:end],3);
y     = dump([1:end],4);

subplot(2,1,1);
plot([teta1,teta2]);
title("Espaço de juntas");
xlabel ("Amostras");
ylabel ("Posição");
%text (pi, 0.7, "arbitrary text");
legend ("Teta1", "Teta2");

subplot(2,1,2);
plot([x,y]);
title("Espaço cartesiano")
xlabel ("Amostras");
ylabel ("Posição");

%text (pi, 0.7, "arbitrary text");
legend ("X", "Y");