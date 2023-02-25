clear all;
close all;
 
%% DEFINIZIONE DEL SISTEMA INNER LOOP
p=20; %in relta il valore può variare tra 20 e 35
G1 = zpk([],[-p,-220],2200);
bode (G1); grid on; legend ();
[num_G1,den_G1] = tfdata(G1,'v');
 
 
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
MAG_G1e_db = 20*log10(MAG_G1e);
%STEP 1
phi = -180+Mf-PHASE_G1e;
rho = tand(phi);
%STEP 2
tau_z = rho/150;
Rz = (1+tau_z*s);
pzmap (Rz);
bode(G1e*Rz); grid on; legend ();
%guardando il grafico per trovare A* trovo --> A* = sqrt(1+rho^2) (NON IN DB)
A = 11.6; %vedendo il grafico basato su rho (valore in DB), il valore 
%è in realtà è circa un media tra quando p = 20 e p = 35 facendo attenzione
%a rispettare la fase in entrambi i casi. Ovviamento l'attraversamento si
%modifica in base al polo
%(attenzione perche troppo alto fa attraversare troppo basso
%troppo alto non fa rispettare la fase)
%STEP 3
mu = 10^(-((-68.85+A)/20)); %-68.85 è una media di MAG_G1e_db nei casi dei
%valori che il polo instabile può assumere
R_pi = (mu*Rz)/s;

Rf = (1/(1+s/1200)); %aggiungo un polo di fisica realizzabilità
Rdi = (Rf*R_pi);
[num_Rdi,den_Rdi] = tfdata(Rdi,'v');
L_pi = Rdi*G1;
margin(L_pi); grid on; legend ();

Qi = (Rdi*mu)/(L_pi+1); %definisco la Q dell'inner loop
bode (Qi, 'blue'); grid on; legend ();
 
Ri = R_pi; %rinomino il regolatore dell'inner completo cosi per poterlo 
%in modo compatto più avanti
[num_Ri,den_Ri] = tfdata(Ri,'v');

% hold off
% 
% rho=tand(phi/2)
% tau_z=rho/150
% Rz_pid=(1+tau_z*s)^2
% bode(Rz_pid)
% %guardando il grafico per trovare A* trovo:
% A=2.2
% mu=10^(-(A+A+MAG_G1e_db)/20)
% bode(mu*s)
% tau_p=1/(10*150)
% 
% Rp=(1+tau_p*s)
% R_pid=(Rz_pid*Rz_pid)/(s*Rp)
% bode(R_pid)
% Rf=(1/(1+s/800))
% Ri_pid=Rf*R_pid
% L_pid=Ri_pid*G1
% bode(L_pid)
% hold on
% Qi2 = (Ri_pid)/(L_pid+1); %definisco la Q dell'inner loop
% bode (Qi2, 'blue'); grid on; legend ();
%% DEFINIZIONE DEL SISTEMA OUTER LOOP

G2 = zpk([],[0,-7],7);
bode (G2); grid on; legend ();
[num_G2,den_G2] = tfdata(G2,'v');
 
%% PROGETTO DEL REGOLATORE DINAMICO
 
%devo dare tanta fase, non basta un PI ma serve un PID
G2e = G2/s;
bode (G2e); grid on; legend ();
[MAG_G2e,PHASE_G2e] = bode(G2e,15);
%devo dare tanta fase, non basta un PI ma serve un PID
Mf2 = 55;
%STEP 1
MAG_G2e_db = 20*log10(MAG_G2e);
phi2 = -180+Mf2-PHASE_G2e;
rho2 = tand(phi2/2);
%STEP 2 
tau2_z = rho2/15;
Rz2 = (1+tau2_z*s)^2;
bode(Rz2); grid on; legend ();
bode(Rz2*G2e); grid on; legend ();
%guardando il grafico per trovare A* trovo --> A*= sqrt(1+rho^2) (NON IN DB)
A2 = 6.02;%vedendo il grafico basato su rho 
%(attenzione perche troppo alto fa attraversare troppo basso
%troppo alto non fa rispettare la fase)

%STEP 3
mu2 = 10^(-(A2+A2+MAG_G2e_db)/20);
%STEP 4
tau_p = 1/(10*13); %non esattamente una decade dopo perchè ho un po di fase 
% in più che posso sfruttare
Rp = (1+tau_p*s);

R_pid = (mu2*Rz2)/(s*Rp);
bode(R_pid*G2); grid on; legend ();
pzmap (R_pid);
%per moderare ulteriormente la Q aggiungo un polo di fisica realizzabilità
Rf2 = (1/(1+s/110));
Rdo = Rf2*R_pid;
bode(Rdo); grid on; legend ();
[num_Rdo,den_Rdo] = tfdata(Rdo,'v');
L_pid = Rdo*G2;
margin(L_pid); grid on; legend ();


R = Rdo*Rdi; %regolatore complessivo
G = G1*G2; %sistema complessivo 
Q_tot = R/(1+G*R+G1*Rdi);
bode (Q_tot, 'blue'); grid on; legend ();

%evito problemi con simulink e il prefiltro ancora non definito
num_Rpf = 1;
den_Rpf = 1;

open ('sis_PID');
sim('sis_PID');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on;

%applichiamo un preflitro per togliere la sovraelongazione
tauf = 0.10;
Rpf= 1/(1+tauf*s)^2; %filtro del secondo ordine con omega_star=6.5
[num_Rpf,den_Rpf] = tfdata(Rpf,'v');

sim('sis_PID');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on;




