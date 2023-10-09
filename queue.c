#include "queue.h"

//キューの中身をprint出力
void printQueue(Queue_t *pQueue)
{
  int i;
  for (i = 0; i <= QUEUE_SIZE - 1; i++)
  {
    printf("%d %d ", pQueue->dataX[i], pQueue->dataY[i]);
  }
  printf("\n");
}

//キューの初期化
void initQueue(Queue_t *pQueue)
{
  int i;
  //キューの中身を0埋め
  for (i = 0; i <= QUEUE_SIZE - 1; i++)
  {
    pQueue->dataX[i] = 0;
    pQueue->dataY[i] = 0;
  }
  //初期化
  pQueue->head = 0;
  pQueue->tail = 0;
  pQueue->flag = EMPTY;
  printQueue(pQueue);
}

// enqueue関数
void enqueue(Queue_t *pQueue, uint8_t x, uint8_t y)
{
  printf("enQ(%d %d)\n", x, y);
  //キューがFullの処理
  if (pQueue->flag == FULL)
  {
    printf("Full\n");
    return;
  }
  //キューがFullでないので、enqueue操作
  pQueue->dataX[pQueue->tail] = x;
  pQueue->dataY[pQueue->tail] = y;

  //リングバッファのため、tailが配列の終端だったら0にする
  if (pQueue->tail == QUEUE_SIZE - 1)
  {
    pQueue->tail = 0;
    //終端でなければ、tailをインクリメント
  }
  else
  {
    pQueue->tail++;
  }
  //フラグの更新
  if (pQueue->tail == pQueue->head)
  {
    pQueue->flag = FULL;
  }
  else
  {
    pQueue->flag = AVAILABLE;
  }
  printQueue(pQueue);
}

// dequeue関数
void dequeue(Queue_t *pQueue)
{
  printf("deQ\n");
  //キューがEmptyの処理
  if (pQueue->flag == EMPTY)
  {
    printf("Empty\n");
    return;
  }
  //キューがEmptyでなければ、dequeue操作
  pQueue->dataX[pQueue->head] = 0;
  pQueue->dataY[pQueue->head] = 0;

  //リングバッファのため、headが配列の終端だったら0にする
  if (pQueue->head == QUEUE_SIZE - 1)
  {
    pQueue->head = 0;
    //終端でなければ、headをインクリメント
  }
  else
  {
    pQueue->head++;
  }
  //フラグの更新
  if (pQueue->tail == pQueue->head)
  {
    pQueue->flag = EMPTY;
  }
  else
  {
    pQueue->flag = AVAILABLE;
  }
  printQueue(pQueue);
}
