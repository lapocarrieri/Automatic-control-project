% Funzione ProgettaRA: Data in ingresso la funzione di trasferimento
% del sistema non compensato, la pulsazione di attraversamento desiderata 
% e il margine di fase deiderato, viene implementata la procedura di sintesi
% di una rete Anticipatrice mediante diagrammi di Bode con imposizione
% del margine di fase
function [alpha,tau]=ProgettaRA(Ge, Wcd, MFd)

% Troviamo il valore del modulo e dell'argomento del sistema esteso non
% compensato alla frequenza di attraversamento
[M,P,W]=bode(Ge);
[V,i]=min(abs(W-Wcd));
GeWcd=M(i);
ArgGeWcd=P(i);
Pd=-180+MFd-ArgGeWcd;
Md=1/GeWcd;
% Conversione in radianti di Pd
Pd=Pd*pi/180;

% Check sulla realizzabilita` della rete
if (Md<1 || Pd<0 || cos(Pd)<1/Md)
   disp('Attenzione: sintesi non possibile con una rete anticipatrice');
    return;
end     
% Calcolo dell'alpha e tau della rete sulla base delle formule di inversione
tau=(Md-cos(Pd))/(Wcd*sin(Pd));
alpha=(cos(Pd)-1/Md)/(Wcd*sin(Pd))/tau;
% Tracciamento del diagramma di bode della rete e del sistema compensato
%hold on 
%R=tf([tau,1],[tau*alpha,1]);
%bode(R);
%bode(R*Ge);
%hold off


