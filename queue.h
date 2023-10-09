#ifndef _QUEUE_H_INCLUDED_
#define _QUEUE_H_INCLUDED_

#define QUEUE_SIZE (30)

#include <stdio.h>
#include <stdint.h>


enum status
{
  EMPTY,
  AVAILABLE,
  FULL
};

typedef struct
{
  uint8_t dataX[QUEUE_SIZE];
  uint8_t dataY[QUEUE_SIZE];
  int16_t head;
  int16_t tail;
  int8_t flag;
} Queue_t;

void printQueue(Queue_t *pQueue);
void initQueue(Queue_t *pQueue);
void enqueue(Queue_t *pQueue, uint8_t x, uint8_t y);
void dequeue(Queue_t *pQueue);


#endif