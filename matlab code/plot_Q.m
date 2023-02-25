
load('all_Q.mat')
bode (Q_tesina_cascata, 'blue'); grid on; legend ();
hold on;
bode (Q_senza_cascata, 'green'); grid on; legend ();
bode (Q_cancellazione, 'red'); grid on; legend ();
bode (Q_PID_cancellazione, 'black'); grid on; legend ();
bode (Q_cascata_PI_PID, 'yellow'); grid on; legend ();
bode (Q_canc_al_limite, 'red'); grid on; legend ();


