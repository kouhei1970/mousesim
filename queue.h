#ifndef _QUEUE_H_INCLUDED_
#define _QUEUE_H_INCLUDED_

#define QUEUE_SIZE (30)

#include <stdio.h>

enum status
{
  EMPTY,
  AVAILABLE,
  FULL
};

typedef struct
{
  u_int8_t dataX[QUEUE_SIZE];
  u_int8_t dataY[QUEUE_SIZE];
  int16_t head;
  int16_t tail;
  int8_t flag;
} Queue_t;

void printQueue(Queue_t *pQueue);
void initQueue(Queue_t *pQueue);
void enqueue(Queue_t *pQueue, u_int8_t x, u_int8_t y);
void dequeue(Queue_t *pQueue);


#endif