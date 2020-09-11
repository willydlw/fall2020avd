#include <stdio.h>

//#define DEBUG

int main(void)
{
    #ifdef DEBUG
        printf("DEBUG defined\n");
    #endif 
    printf("this always executes\n");
    return 0;
}