#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define TABLE_SIZE 100

struct Entry {
    char* key;
    int value;
    struct Entry* next;
};

struct HashMap {
    struct Entry* table[TABLE_SIZE];
};

// Hash function: djb2 by Dan Bernstein
unsigned int hash(const char* key) {
    unsigned long hash = 5381;
    int c;
    while ((c = *key++))
        hash = ((hash << 5) + hash) + c;
    return hash % TABLE_SIZE;
}

struct HashMap* create_map() {
    struct HashMap* map = malloc(sizeof(struct HashMap));
    if (! map) return NULL;

    for (int i = 0; i < TABLE_SIZE; i++) {
        map->table[i] = NULL;
    }

    return map;
}

void put_map(struct HashMap* map, const char* key, const int value) {
    unsigned int idx = hash(key);
    struct Entry* entry = map->table[idx];

    while (entry != NULL) {
        if (strcmp(entry->key, key) == 0) {
            entry->value = value;
            return;
        }

        entry = entry->next;
    }

    struct Entry* new_entry = malloc(sizeof(struct Entry));
    new_entry->key = strdup(key);
    new_entry->value = value;
    new_entry->next = map->table[idx];
    map->table[idx] = new_entry;
}

bool get_map(struct HashMap* map, const char* key, int* out) {
    unsigned int idx = hash(key);
    struct Entry* entry = map->table[idx];

    while (entry != NULL) {
        if (strcmp(entry->key, key) == 0) {
            *out = entry->value;
            return true;
        }

        entry = entry->next;
    }

    return false;
}

void delete_map(struct HashMap* map, const char* key) {
    unsigned int idx = hash(key);
    struct Entry* current = map->table[idx];
    struct Entry* pre = NULL;

    while (current != NULL) {
        if (strcmp(current->key, key) == 0) {
            if (pre == NULL) map->table[idx] = current->next;
            else pre->next = current->next;
            
            free(current->key);
            free(current);
            return;
        }

        pre = current;
        current = current->next;
    }
}

void free_map(struct HashMap* map) {
    for (int i = 0; i < TABLE_SIZE; i++) {
        struct Entry* entry = map->table[i];

        while (entry != NULL) {
            struct Entry* temp = entry;
            entry = entry->next;
            free(temp->key);
            free(temp);
            
        }
    }
    free(map);
}

int main() {
    struct HashMap* map = create_map();
    put_map(map, "apple", 11);
    put_map(map, "orange", 22);

    int out = 0;
    if (get_map(map, "orange", &out)) printf("out: %d\n", out);
    else printf("Key is not exists\n");

    delete_map(map, "orange");
    if (get_map(map, "orange", &out)) printf("out: %d\n", out);
    else printf("Key is not exists\n");

    if (get_map(map, "apple", &out)) printf("out: %d\n", out);
    else printf("Key is not exists\n");

    free_map(map);

    return 0;
}