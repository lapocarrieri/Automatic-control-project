clear all;
close all;

num_Raa = 1;
den_Raa = 1;
NUMd = 1;
DENd = 1;
NUMpf = 1;
DENpf = 1;
Ts = 1;
%% DEFINIZIONE DEL SISTEMA
p=35; %in relta il valore può variare tra 20 e 35
G1 = zpk([],[-p,-220],2200)
bode (G1); grid on; legend ();
[num_G1,den_G1] = tfdata(G1,'v');

G2 = zpk([],[0,-7],7)
hold on;
bode (G2); grid on; legend ();
hold off;
[num_G2,den_G2] = tfdata(G2,'v')

G = G1*G2;
bode (G); grid on; legend ();
%% CONTROLLO IL SISTEMA SENZA SFRUTTTARE IL CONTROLLO IN CASCATA
%per attenuare il disturbo d1 a 1r/s uso un guadagno statico
%per avere l'inseguimento di u e quindi errore nullo mi basta il polo 
%nell'origine già presente in G.

%PROGETTO DEL REGOLATORE STATICO
mu = 283.79;
Ge = G*mu;
bode (Ge); grid on; legend ();

%PROGETTO DEL REGOLATORE DINAMICO
s=tf('s');
alpha = 0.05;
tau = 0.046;
Rd1 = ((tau*s + 1)/(alpha*tau*s + 1))^2; % 2 reti anticipatrici progettate
% con il tuning pratico
Rd2 = 1/(((s/400)+1)*((s/1800)+1)); % polo di fisica realizzabilita per attenuare la Q
%Rd2 = zpk([],[400],400) metodo alternativo per Rd2
Rd = Rd1*Rd2;
[num_Rd,den_Rd] = tfdata(Rd,'v');

L = Rd*Ge;
bode(L,'black'); margin (L); grid on; legend ();
hold on;

%Definisco tutte le funzione di sensitività del sistema
F = L/(L+1);
S = 1/(L+1);
Q = (Rd*mu)/(L+1);

%Plot di tutte le funzione di sensitività nello stesso bode
bode (F, 'green');
hold on;
bode (S, 'red');
hold on;
bode (Q, 'blue');
hold on; grid on; legend ();
hold off;
%Richiamo lo schema di Simulink per guardare quale è la risposta al gradino
%del sistema con Regolatore. Si nota subito un overshoot eccessivo e
%un tempo di assestamento che invece va bene (0.95 - 1.05) anzi anche 
%meglio del dovuto

%Valuto quali sono i valori temporali del sistema inziale, prima che venga
%controllato
%per stabilizzare usiamo il root locus
dcgain (s*G);
rltool(G);
sim('sistema_iniziale');
plot(out.yi); grid on;
plot(out.ui); grid on;
plot(out.ei); grid on;

%Applico un filtro passabasso per attenuare overshoot e moderare il
%controllo
%omega_c=12r/s e quindi tauf=1/omega_c per filtro di 1º ordine
tauf = 0.083; % tau-filtro. Definito se si usa regolatori del primo 
%o secondo ordine non Buttterwhorth
%Rpf = 1/((s/15)^2+1.414*(s/15)+1); %filtro di Butterwhorth con

Rpf= 1/(1+tauf*s)^2; %filtro del secondo ordine con omega_star=21
[num_Rpf,den_Rpf] = tfdata(Rpf,'v');
%Grazie al pre-filtro ottengo un sistema privo di overshoot e con una
%variabile di controllo decisamente più controllata. 


%Simulazione dello schema simulink per tenere sotto controllo le uscite di
%interesse
sim('sistema_controllato');
plot(out.y); grid on;
plot(out.u); grid on;
plot(out.e); grid on;

%% DISCRETIZZAZIONE DEL REGOLATORE
%Il valore della omega_c è 45r/s
omega_s = 3000; %1850 valore che causa un peggioramento della u, se metto 
%invece 3300 la u migliora ancora di più
Ts = 2*pi/omega_s;

% filtro anti-aliasing con w_aa=485r/s. Abbassando la w_aa si può
% modificare sostaziosamente la u. 485 è il valor medio che si ha se
% omega_s = 1850
tau_aa=1/485;
Raa=1/((s*tau_aa)+1);
margin (Raa); grid on; legend ();
[num_Raa,den_Raa] = tfdata(Raa,'v');


R = Rd*mu;
[num_R,den_R] = tfdata(R,'v');
[NUMd,DENd] = c2dm (num_R,den_R,Ts,'tustin');
[NUMpf,DENpf] = c2dm (num_Rpf,den_Rpf,Ts,'tustin');

%sostituire i componenti continui con quelli discreti
sim('sistema_controllato');
plot(out.y); grid on;
plot(out.u); grid on;

