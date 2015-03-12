clc 
clear all

min_sp = 64;
max_sp = 128;

posFader = 0:1023;
tau = 256;
destino = 1023;

fadSpeed1 = min_sp + (max_sp-min_sp)*(1-exp(-abs(destino-posFader)./tau));        
%fadSpeed2 = min_sp + (max_sp-min_sp)*(1-exp(-abs(posFader)./tau));

%subplot(2,1,1)
plot(fadSpeed1)
%subplot(2,1,2)
%plot(fadSpeed2)