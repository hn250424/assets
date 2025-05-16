#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define SIZE 100

struct Item {
    char* item;
};

struct Stack {
    int idx;
    struct Item* items[SIZE];
};

struct Stack* create() {
    struct Stack* stack = malloc(sizeof(struct Stack));
    if (! stack) return NULL;
    
    stack->idx = -1;
    for (int i = 0; i < SIZE; i++) {
        stack->items[i] = NULL;
    }

    return stack;
}

bool is_empty(struct Stack* stack) {
    return stack->idx == - 1;
}

bool is_full(struct Stack* stack) {
    return stack->idx == SIZE - 1;
}

bool push_back(struct Stack* stack, char* value) {
    if (is_full(stack)) return false;

    struct Item* new_item = malloc(sizeof(struct Item));
    if (! new_item) return false;

    new_item->item = strdup(value);
    if (! new_item->item) {
        free(new_item);
        return false;
    }

    stack->idx++;
    stack->items[stack->idx] = new_item;
    return true;
}

bool pop(struct Stack* stack) {
    if (is_empty(stack)) return false;
    struct Item* top = stack->items[stack->idx];
    free(top->item);
    free(top);
    stack->idx--;
    return true;
}

bool top(struct Stack* stack, char** out) {
    if (is_empty(stack)) return false;
    *out = stack->items[stack->idx]->item;
    return true;
}

void destroy(struct Stack* stack) {
    while (! is_empty(stack)) {
        struct Item* top = stack->items[stack->idx];
        free(top->item);
        free(top);
        stack->idx--;
    }
    free(stack);
}

int main() {
    char* out;

    struct Stack* stack = create();

    push_back(stack, "abc");
    if (top(stack, &out)) printf("out: %s\n", out);

    push_back(stack, "def");
    if (top(stack, &out)) printf("out: %s\n", out);

    pop(stack);
    if (top(stack, &out)) printf("out: %s\n", out);

    pop(stack);
    if (top(stack, &out)) printf("out: %s\n", out);
    else printf("Stack is empty\n");

    destroy(stack);

    return 0;
}