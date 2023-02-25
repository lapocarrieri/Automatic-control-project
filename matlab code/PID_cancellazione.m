
%L'idea del progetto in cancellazione nasce dal fatto che la risposta del
%sistema controllato presenta evidenti sovraelongazioni probabilmente
%causate dalla vicinanza all'origine dei alcuni zeri dei regolatori

clear all;
close all;
 
%% DEFINIZIONE DEL SISTEMA INNER LOOP
p=35; %in relta il valore può variare tra 20 e 35
G1 = zpk([],[-p,-220],2200)
bode (G1); grid on; legend ();
[num_G1,den_G1] = tfdata(G1,'v');
pzmap(G1);
 
%% CONTROLLO IN CASCATA
%per attenuare il disturbo d1 a 1r/s uso un guadagno statico
%per avere l'inseguimento di u e quindi errore nullo mi basta il polo 
%nell'origine già presente in G.
 
%% INNER LOOP
%PROGETTO DEL REGOLATORE PI
s = tf('s');
% il polo che da problemi e quello a 35 poiche in alcuni versioni
% fa attreversare la L prima di 100rad/s
G1e = G1/s;
bode (G1e); grid on; legend ();

%calibro lo zero per avere lo sfasamento necessario a omega_c = 150r/s 
[MAG_G1e,PHASE_G1e] = bode(G1e,150);
Mf = 50;

%cancellazione del polo instabile, dove 27.5 è una media tra 20 e 35. Nel
%caso senza cancellazione il polo del PI è stato messo circa a 35 quindi
%non troppo distante da questo
tau_z=1/27.5;
Rz=(1+tau_z*s);
bode(Rz); grid on; legend ();

Gee=Rz*G1e;
bode(Gee); grid on; legend ();

[MAG_Gee,PHASE_Gee] = bode(Gee,150);
MAG_Gee_db = -20*log10(MAG_Gee);
%faccio una media dei due MAG_Gee_db in caso di p=20 e p=35 cioe 53.976

mu = 10^(53.976/20);
R_pi = (mu*Rz)/s;

%introduco un polo di fisica realizzabilità
Rf = (1/(1+s/800));

Rdi = (Rf*R_pi);
[num_Rdi,den_Rdi] = tfdata(Rdi,'v');

L_pi = Rdi*G1;
margin(L_pi); grid on; legend ();

Qi = (Rdi*mu)/(L_pi+1); %definisco la Q dell'inner loop
bode (Qi, 'blue'); grid on; legend ();


Ri = Rdi; %rinomino il regolatore dell'inner completo cosi per poterlo 
%in modo compatto più avanti
[num_Ri,den_Ri] = tfdata(Ri,'v');

 
%% DEFINIZIONE DEL SISTEMA OUTER LOOP
G2 = zpk([],[0,-7],7);
bode (G2); grid on; legend ();
[num_G2,den_G2] = tfdata(G2,'v');
 
%% PROGETTO DEL REGOLATORE DINAMICO
 
%devo dare tanta fase, non basta un PI ma serve un PID

%il regolatore PID senza la cancellazione piazza uno zero a circa 8.5,
%dunque decido di spostare questo zero a 7 per eliminare il polo prensente
%a quella frequenza 
tau2_1 = 7;
Rz2_1 = (1+tau2_1*s);

G2e = G2*Rz2_1/s;
bode (G2e); grid on; legend ();
[MAG_G2e,PHASE_G2e] = bode(G2e,15);
%devo dare tanta fase, non basta un PI ma serve un PID, quindi calibro lo
%zero e il polo che mi rimangono
%STEP 1
Mf2 = 55;
phi2 = -180+Mf2-PHASE_G2e;
%STEP 2
rho2 = tand(phi2);
tau2_2 = rho2/15;
Rz2_2 = (1+tau2_2*s);
%STEP 3
MAG_G2e_db = 20*log10(MAG_G2e);
%guardando il grafico per trovare A* trovo --> A*= sqrt(1+rho^2) (NON IN DB)
A2 = 1.294; %vedendo il grafico basato su rho 
%(attenzione perche troppo alto fa attraversare troppo basso
%troppo alto non fa rispettare la fase)
mu2 = 10^(-(A2+MAG_G2e_db)/20);
%STEP 4
%posizione il polo che avanza una decade oltre 
tau_p = 1/(10*15);
Rp = (1+tau_p*s);
%definisco il regolatore PID
R_pid = (mu2*Rz2_1*Rz2_2)/(s*Rp);
bode(R_pid); grid on; legend ();
%aggiungo un polo di fisica realizzabilità perche mi sono tenuto un po
% largo con la fase
Rf2 = (1/(1+s/150));
Rdo =Rf2*R_pid;
[num_Rdo,den_Rdo] = tfdata(Rdo,'v');
%definisco la L 
L_pid = Rdo*G2;
margin(L_pid); grid on; legend ();

R = Rdo*Rdi; %regolatore complessivo
G = G1*G2; %sistema complessivo 
Q_tot = R/(1+G*R+G1*Rdi);
bode (Q_tot, 'blue'); grid on; legend ();

%evito problemi con simulink e il prefiltro ancora non definito
num_Rpf = 1;
den_Rpf = 1;
sim('sis_PID');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on;
%rispetto al progetto senza cascata si nota una sovraelongazione un poco
%migliore ma devo comunque ricorrere ad un prefiltro passo basso 

%applichiamo un preflitro per togliere la sovraelongazione
tauf = 0.081;
Rpf= 1/(1+tauf*s)^2; %filtro del secondo ordine con omega_star=8
[num_Rpf,den_Rpf] = tfdata(Rpf,'v');

sim('sis_PID');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on;



