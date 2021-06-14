gcc -pg main.c solver.c motor.c pid.c controllers.c -Wall -o motor_sim  -lm -DLOG
