gcc -pg main.c solver.c motor.c pid.c controllers.c -Wall -o ../pmsm_server  -lm -DLOG
