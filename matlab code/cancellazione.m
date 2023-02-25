
clear all;
close all;

%% DEFINIZIONE DEL SISTEMA INNER LOOP
p=20; %in realta il valore può variare tra 20 e 35
G1 = zpk([],[-p,-220],2200);
bode (G1); grid on; legend ();
[num_G1,den_G1] = tfdata(G1,'v');


%% CONTROLLO IN CASCATA
%per attenuare il disturbo d1 a 1r/s uso un guadagno statico
%per avere l'inseguimento di u e quindi errore nullo mi basta il polo 
%nell'origine già presente in G.

%% INNER LOOP
%PROGETTO DEL REGOLATORE STATICO
mu1 = 283.79;
mu2 = 1.15; %mu2 aggiunto per via dell'attenuazione dovuta alla rete 
%ritardatrice che avviene anche nell'intervallo in cui esiste il disturbo
%d1
mu = mu1*mu2;
G1e = G1*mu;
bode (G1e); grid on; legend ();


%PROGETTO DEL REGOLATORE DINAMICO
s=tf('s');
alpha = 0.059;
tau = 0.566;
Rdi1 = (alpha*tau*s + 1)/(tau*s + 1);


Li1 = Rdi1*G1e;
bode(Li1, 'black'); grid on; legend ();
hold on;
hold off;
%Serve un polo di fiisca realizzabilità per moderare la Q ad elevate
%frequenze
Rdi2 = 1/((1/800)*s+1);
Rdi = Rdi1*Rdi2;
[num_Rdi,den_Rdi] = tfdata(Rdi,'v');

Li = Rdi*G1e;
margin(Li); grid on; legend ();
hold on;
hold off;

Qi = (Rdi*mu)/(Li+1); %definisco la Q dell'inner loop
bode (Qi, 'blue'); grid on; legend ();

Ri = Rdi*mu; %rinomino il regolatore dell'inner completo cosi per poterlo 
%in modo compatto più avanti
[num_Ri,den_Ri] = tfdata(Ri,'v');

%% DEFINIZIONE DEL SISTEMA OUTER LOOP
G2 = zpk([],[0,-7],7);
bode (G2); grid on; legend ();
[num_G2,den_G2] = tfdata(G2,'v');

%% PROGETTO DEL REGOLATORE DINAMICO
%con le formule di inversione trovo il regolatore dinamico che mi risolve
%lo scenario B

%proviamo a non usare il mu ma solo una rete anticipatrice
%[alpha2, tau2] = ProgettaRA (G2, 15, 50)

alpha2 = 0.024;  
tau2 = 5.575; 
Rdo1 = (tau2*s + 1)/(alpha2*tau2*s + 1);
bode (Rdo1); grid on; legend ();
Lo1 = Rdo1*G2;
margin(Lo1); grid on; legend ();

%Serve un polo di fiisca realizzabilità per moderare la Q ad elevate
%frequenze
Rdo2 = 1/((1/90)*s+1);

Rdo = Rdo1*Rdo2;
[num_Rdo,den_Rdo] = tfdata(Rdo,'v');

Lo = Rdo*G2;
margin(Lo); grid on; legend ();
%valuto la Q del sistema complessivo 

Ro = Rdo; %Il regolatore dell'outer presenta solo la parte dinamica

R = Rdo*Ri; %regolatore complessivo
G = G1*G2; %sistema complessivo 
Q_tot = R/(1+G*R+G1*Ri);
bode (Q_tot, 'blue'); grid on; legend ();

%simulo il sistema e tolgo momentaneamente il prefiltro che regolo in
%seguito
num_Rpf = 1;
den_Rpf = 1;

open('sis_PID');
sim('sis_PID');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on; 

pzmap (Rdo);
%noto forti code di assestamento dovute ad uno zero di Rdo praticamente
%attaccato all'orgine. Provo allora un progetto in cancellazione,
%eliminando il polo in -7 della G2
tau_z = 1/7; %presuppongo che la posizione del polo che questo zero cancella
%possa variare di circa il 20% cioè [5.6;8.4]
Rz = 1+tau_z*s;

G2z = Rz*G2;
bode (G2z); grid on; legend ();

%calibro il mu per far attraversare il sistema dove voglio io
mu_o = 14.75;
Ro_1 = mu_o*Rz;
Lo_1 = Ro_1*G2;
margin (Lo_1); grid on; legend ();

%2 poli di fisica realizzabilità
tau_p = 1/50;
Rp = 1/(1+tau_p*s)^2;
Ro = mu_o*Rz*Rp;
Lo = Ro*G2;
margin (Lo); grid on; legend ();
[num_Rdo,den_Rdo] = tfdata(Ro,'v');

R = Ro*Ri; %regolatore complessivo
G = G1*G2; %sistema complessivo 
Q_tot = R/(1+G*R+G1*Ri);
bode (Q_tot, 'blue'); grid on; legend ();

sim('sis_PID');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on; 

%applichiamo un preflitro per togliere la sovraelongazione
tauf = 0.65/11.5;
Rpf= 1/(1+tauf*s)^2; %filtro del secondo ordine con omega_star=9
[num_Rpf,den_Rpf] = tfdata(Rpf,'v');


sim('sis_PID');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on; 


