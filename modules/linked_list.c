#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct LinkedList {
    int value;
    struct LinkedList* next;
} LinkedList;

void free_linkedList(LinkedList* list) {
    LinkedList* cur = list->next;
    while (cur != NULL) {
        LinkedList* next = cur->next;
        free(cur);
        cur = next;
    }
}

bool init_linkedList(LinkedList* list, int size, int defaultValue) {
    list->value = defaultValue;
    list->next = NULL;

    if (size < 0) return false;
    
    LinkedList* current = list;

    for (int i = 1; i < size; i++) {
        current->next = malloc(sizeof(LinkedList));
        if (! current->next) {
            free_linkedList(list);
            return false;
        }

        current = current->next;
        current->value = defaultValue;
        current->next = NULL;
    }

    return true;
}

bool get(LinkedList* list, int index, int* out) {
    if (index < 0) return false;
    LinkedList* cur = list;
    for (int i = 0; i < index; i++) {
        if (cur->next) cur = cur->next;
        else return false;
    }
    *out = cur->value;
    return true;
}

bool set(LinkedList* list, int index, int value) {
    if (index < 0) return false;
    LinkedList* cur = list;
    for (int i = 0; i < index; i++) {
        if (cur->next) cur = cur->next;
        else return false;
    }
    cur->value = value;
    return true;
}

void print_linkedList(LinkedList* list) {
    LinkedList* cur = list;
    while (cur != NULL) {
        printf("%d ", cur->value);
        cur = cur->next;
    }
    printf("\n");
}

int main() {
    LinkedList* list = malloc(sizeof(LinkedList));
    if (! init_linkedList(list, 10, 0) ) free(list);

    set(list, 5, 55);
    set(list, 6, 66);

    int out;
    if ( get(list, 5, &out) ) printf("out index-5: %d\n", out);
    if ( get(list, 6, &out) ) printf("out index-6: %d\n", out);

    print_linkedList(list);

    free_linkedList(list);
    free(list);
    
    return 0;
}
