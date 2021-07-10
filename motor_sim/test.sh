gcc -pg test.c solver.c motor.c pid.c controllers.c -Wall -o ../test_server  -lm -DLOG -LOG
