#include <stdio.h>
#include <iostream>
#include <cstddef>
#include "Devola.hpp"

int main()
{ 
    Devola devola;
    devola.march(1.34);
    devola.report();
    devola.march(2.45);
    devola.report();
    printf("Finished\n");
    return 0;
}
