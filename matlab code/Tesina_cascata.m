
clear all;
close all;

%% DEFINIZIONE DEL SISTEMA INNER LOOP
p=35; %in relta il valore può variare tra 20 e 35
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

%% DEFINIZIONE DEL SISTEMA OUTER LOOP
G2 = zpk([],[0,-7],7);
bode (G2); grid on; legend ();
[num_G2,den_G2] = tfdata(G2,'v');

%% PROGETTO DEL REGOLATORE DINAMICO
%con le formule di inversione trovo il regolatore dinamico che mi risolve
%lo scenario B

%proviamo a non usare il mu ma solo una rete anticipatrice
%[alpha2, tau2] = ProgettaRA (G2, 15, 50)
%noto che il sistema cosi progettato ha delle code di assestamento decido
%quindi di usare anche il guadagno per provare a risolvere questo problema

%scelgo di mettere uno zero in 5.43r/s ottenendo un alpha=0.131 e un 
%tau=0.184 che mi danno un sistema complessivo con poca sovraelongazione ma
%una Q e una u migliorabili.
%Quindi richiedo meno fase guadagnata e
%uso le formule fase_guadagata=asin((1-alpha)/(1+alpha)) e wb=1/(tau*sqrt(alpha))
%formule a [slide 5,25] 
alpha2 = 0.406;  
tau2 = 0.104; %lo zero in 9.61 (meglio anche per le code di assestamento)
Rdo1 = (tau2*s + 1)/(alpha2*tau2*s + 1);
bode (Rdo1); grid on; legend ();
Lo1 = Rdo1*G2;
bode(Lo1, 'black'); grid on; legend ();
%MODIFICA
mu3 = 22.64;

%MODIFICA
Lo1 = Rdo1*G2*mu3;
bode(Lo1, 'black'); grid on; legend ();
hold on;
hold off;
%Serve un polo di fiisca realizzabilità per moderare la Q ad elevate
%frequenze
%MODIFICA DA 85 A 150
Rdo2 = 1/((1/150)*s+1);
%MODIFICA
Rdo = Rdo1*Rdo2*mu3;
[num_Rdo,den_Rdo] = tfdata(Rdo,'v');

Lo = Rdo*G2;
margin(Lo); grid on; legend ();
%valuto la Q del sistema complessivo 

Ro = Rdo; %Il regolatore dell'outer presenta solo la parte dinamica

R = Rdo*Ri; %regolatore complessivo
G = G1*G2; %sistema complessivo 
Q_tot = R/(1+G*R+G1*Ri);
bode (Q_tot, 'blue'); grid on; legend ();

%applichiamo un preflitro per togliere la sovraelongazione
tauf = 0.65/9;
Rpf= 1/(1+tauf*s)^2; %filtro del secondo ordine con omega_star=9
[num_Rpf,den_Rpf] = tfdata(Rpf,'v');

%pongo i filtri anti-aliasing a 1 per evitare che alterino la simulazione
%del sistema
num_Raa_i = 1;
den_Raa_i = 1;
num_Raa_o = 1;
den_Raa_o = 1;
%pongo ad 1 anche i componenti discreti e i tempi di campionamento che per il 
% momento non ho definito e non mi interessano
NUMpf = 1;
DENpf = 1;
NUMdo = 1;
DENdo = 1;
NUMdi = 1;
DENdi = 1;
Ts_o = 1;
Ts_i = 1;

sim('sis_controllo_cascata');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on; 

pzmap (Li);
pzmap (Lo);

%% DISCRETIZZAZIONE DEL REGOLATORE INNER E OUTER
%INNER LOOP
%La pulsazione di attraversamento dell'inner loop è omega_c = 156r/s
omega_s_i = 10000; %6435
Ts_i = 2*pi/omega_s_i;

% filtro anti-aliasing con w_aa=1686r/s
tau_aa_i = 1/2000;%1686;
Raa_i = 1/((s*tau_aa_i)+1);
bode (Raa_i); grid on; legend ();
[num_Raa_i,den_Raa_i] = tfdata(Raa_i,'v');

[num_Ri,den_Ri] = tfdata(Ri,'v');
[NUMdi,DENdi] = c2dm (num_Ri,den_Ri,Ts_i,'tustin');


%OUTER LOOP
%La pulsazione di attraversamento dell'outer loop è omega_c = 15r/s
omega_s_o = 1000;%618.75 
Ts_o = 2*pi/omega_s_o;

%filtro anti-aliasing con w_aa=162/s
tau_aa_o = 1/250;%162;
Raa_o = 1/((s*tau_aa_o)+1);
bode (Raa_o); grid on; legend ();
[num_Raa_o,den_Raa_o] = tfdata(Raa_o,'v');

[num_Ro,den_Ro] = tfdata(Ro,'v');
[NUMdo,DENdo] = c2dm (num_Ro,den_Ro,Ts_o,'tustin');
[NUMpf,DENpf] = c2dm (num_Rpf,den_Rpf,Ts_o,'tustin');
