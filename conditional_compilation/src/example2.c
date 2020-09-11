#include <stdio.h>

#define DEBUG 0

int main(void)
{
    #if DEBUG
        printf("DEBUG defined\n");
    #endif 
    printf("this always executes\n");
    return 0;
}