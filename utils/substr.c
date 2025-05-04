#include <stdio.h>
#include <stdlib.h>

char* substr(char* str, int start, int length) {
    char* result = malloc(length + 1);
    
    for (int i = 0; i < length; i++) {
        result[i] = str[start + i];
    }

    result[length] = '\0';

    return result;
}

int main() {
    char* str = "abcde";

    char* result = substr(str, 1, 3);
    printf("result: %s\n", result);

    free(result);

    return 0;
}