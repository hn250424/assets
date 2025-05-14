#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define SIZE 5

struct Queue {
    int items[SIZE];
    int front;
    int rear;
};

void initQueue(struct Queue* q) {
    q->front = -1;
    q->rear = -1;
}

bool isEmpty(struct Queue* q) {
    return q->front == -1;
}

bool isFull(struct Queue* q) {
    return q->front == (q->rear + 1) % SIZE;
}

bool enqueue(struct Queue* q, int value) {
    if (isFull(q)) return false;

    q->rear = (q->rear + 1) % SIZE;
    q->items[q->rear] = value;
    if (q->front == - 1) q->front = 0;

    return true;
}

bool dequeue(struct Queue* q) {
    if (isEmpty(q)) return false;

    if (q->front == q->rear) {
        q->front = -1;
        q->rear = -1;
    } else {
        q->front = (q->front + 1) % SIZE;
    }

    return true;
}

int peek(struct Queue* q) {
    // You must check isEmpty before call this.
    if (isEmpty(q)) {
        printf("Error: Queue is empty. Cannot peek.\n");
        exit(1);
    }

    return q->items[q->front];
}

void displayQueue(struct Queue* q) {
    if (isEmpty(q)) {
        printf("(queue is empty)\n");
        return;
    }

    int i = q->front;
    while (1) {
        printf("%d ", q->items[i]);
        if (i == q->rear) break;
        i = (i + 1) % SIZE;
    }
    printf("\n");
}

int main() {
    struct Queue q;
    initQueue(&q);

    enqueue(&q, 111);
    enqueue(&q, 222);
    enqueue(&q, 333);
    enqueue(&q, 444);
    enqueue(&q, 555);

    dequeue(&q);
    enqueue(&q, 666);
    dequeue(&q);
    enqueue(&q, 777);

    displayQueue(&q);

    return 0;
}