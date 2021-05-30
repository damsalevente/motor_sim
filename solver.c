#include "solver.h"

typedef struct solver_input{
    float u;
    float v;
    float w;
} solver_input;

typedef struct solver_output{
    float x;
    float y;
    float z;
} solver_output;

typedef struct solver_data{
    solver_input *input;
    solver_output *output;
} solver_data;

void rk4();
void dl();
