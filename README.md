# Automatic-control-project
Project realized for the course of Automatic Control 2 at the Università di Bologna.
The project is realized by Edoardo Panichi, Lapo Carrieri, Giulio Rosadi and Jhonny.

## Problem
Gven the following block scheme
![image](https://user-images.githubusercontent.com/56505429/221368417-04963794-7db4-41be-bfe8-929debe20d5a.png)
the goal is to regulate in Cascade with both open and closed loop considering static and dynamic design specifications. Everything taking in to account p that is an uncertain parameter.
## Summary

![image](https://user-images.githubusercontent.com/56505429/221368535-881c8f67-6379-4478-b691-a06e0d4826f7.png)
![image](https://user-images.githubusercontent.com/56505429/221368544-b4ee26b7-241e-460a-9df0-1680a74a31d9.png)
## Simulink scheme
![image](https://user-images.githubusercontent.com/56505429/221368573-9b6ac865-42fa-4145-a6e8-92c899a7518d.png)

## Resolution
To solve the problem we tried multiple possibility using PID cancellation and other techniques. The 2 final solutions are with the cascade control and without:
### Control without cascade
https://github.com/lapocarrieri/Automatic-control-project/blob/main/matlab%20code/Tesina_senza_cascata.m
### Control with cascade

https://github.com/lapocarrieri/Automatic-control-project/blob/main/matlab%20code/Tesina_cascata.m
