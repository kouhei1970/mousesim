#include <stdio.h>
#include <cairo/cairo.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include "vector.h"

#define UNIT_LENGTH (180.0)
#define MAZE_SIZE (16)
#define SECTION (180)
#define START_X (0)
#define START_Y (0)
#define GOAL_X (7)
#define GOAL_Y (7)
#define WIDTH (2892)
#define HEIGHT (2892)
//#define WIDTH (180*3+12)
//#define HEIGHT (180*3+12)
#define CW (1)
#define CCW (-1)
#define STEP (1e-6)
#define VELOCITY (500.0)
#define TURN_VELOCITY (211.222)
#define ACC0 (9800.0)
#define ANGLE_ACC0 (M_PI * 5)
#define ANGLE1 (0.785398)
//#define ANGLE1 (30.0*M_PI/180)
#define STACK_SIZE (9)
#define NUM_MAX (100)


#define SLA_OUTPUT



enum sensor_num
{
  LEFT_FRONT,
  LEFT_SIDE,
  RIGHT_FRONT,
  RIGHT_SIDE
};

typedef struct
{
  double t;
  double x;
  double y;
  double angle;
  double velocity;
  double omega;
  double red;
  double green;
  double blue;
} data_t;

typedef struct
{
  point2d_t pos;//センサのロボット上での位置
  double angle;//センサのロボット上での角度
  point2d_t pos_g;//センサのグルーバル位置
  double angle_g;//センサのグルーバル角度
} sensor_t;


typedef struct
{
  double t;
  double x;
  double y;
  double angle;
  double velocity;
  double omega;
  double h;
  double acc;
  double angle_acc;
  double angle1;
  double turn_velocity;
  char dir;
  sensor_t sensor[4];
} robot_t;

typedef struct
{
  u_int8_t *pX;
  u_int8_t *pY;
  int16_t tail;
  u_int16_t thisNum;
} Stack_t;




typedef struct
{
  point2d_t sen[4];
  //point2d_t left_front;
  //point2d_t left_side;
  //point2d_t right_front;
  //point2d_t right_side;
}sensor_detect_point_t;

void make_mapdata(void);
void reset_map(void);
void round_dir(char d);
char update_mouse_position(char dir);
char get_map(char x, char y, char dir);
void set_map(char x, char y, char dir, char state);
void draw_v_wall(int x, int y);
void draw_h_wall(int x, int y);
void draw_mouse(double x, double y, double angle);
void erase_mouse(double x, double y, double angle);
void draw_maze(int width, int height);
void eom(double *xdot, double *x, double t, double *u);
void rk4(void (*dxdt)(double *, double *, double, double *), double *x, double t, double *u, double h, int n);
void one_step(int dir, double x, double y, double angle, double velocity, double framerate);
void r_turn(double x, double y, double angle, double velocity, double framerate);
void l_turn(double x, double y, double angle, double velocity, double framerate);
void straight(void);
void right_turn(void);
void left_turn(void);
void turn180(void);
void mode0(void);
void calc_trajectry(void);
void output_to_img(int cnt, data_t *data);
void draw_traj(int cnt, data_t *data);
void erase_traj(int cnt, data_t *data);
void erase_mouse2(double x, double y, double angle);
void slalom_opt(void);
void make_contourmap(u_int8_t _map[16][16], u_int8_t sx, u_int8_t sy, u_int8_t gx, u_int8_t gy);
void make_contourmap2(u_int8_t _map[16][16], u_int8_t sx, u_int8_t sy, u_int8_t gx, u_int8_t gy);
void run(void);
void output_cmap(void);
int8_t initStack(Stack_t *pStack);
int8_t deleteStack(Stack_t *pStack);
void printStack(Stack_t *pStack);
void copy_mapdata(u_int8_t _map[16][16]);
point2d_t find_cross_point(double a1, double b1, double a2, double b2);
void setup_micromouse(robot_t *robot);
sensor_detect_point_t calc_sensor_ray(robot_t *robot, u_int8_t _map[16][16]);


// 2019+alpha Clasic mouse expart final maze
char maze[33][66] = {
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+", //    0
    "|                                                               |", // 15  1
    "+   +---+---+---+---+---+---+---+---+---+---+---+---+---+   +---+", //    2
    "|                               |       |       |   |           |", // 14  3
    "+---+---+---+---+---+---+---+   +   +   +   +   +   +   +   +   +", //    4
    "|               |       |       |   |       |           |   |   |", // 13  5
    "+   +---+---+   +   +   +   +   +   +---+   +---+   +---+---+   +", //    6
    "|   |       |       |       |   |       |               |       |", // 12  7
    "+   +   +   +---+---+---+---+   +---+   +---+---+---+---+   +---+", //    8
    "|   |   |   |   |   |   |       |       |       |       |       |", // 11  9
    "+   +   +   +   +   +   +   +---+   +---+   +   +   +   +---+   +", //   10
    "|   |   |                   |       |       |       |       |   |", // 10 11
    "+   +   +---+   +   +   +---+   +---+   +---+---+---+---+   +   +", //   12
    "|   |       |   |   |   |       |           |       |           |", // 9 13
    "+   +---+   +---+---+---+   +---+---+   +---+   +   +   +---+---+", //   14
    "|                   |                   |       |   |           |", // 8 15
    "+---+   +---+   +---+---+   +   +   +---+   +   +   +---+---+   +", //   16
    "|                       |   |       |       |   |               |", // 7 17
    "+   +---+   +---+   +   +   +---+---+   +---+---+---+---+---+---+", //   18
    "|                   |   |   |   |   |                           |", // 6 19
    "+---+   +---+   +   +   +---+   +   +---+---+---+---+---+---+   +", //   20
    "|   |           |           |                   |       |       |", // 5 21
    "+   +---+   +   +   +---+   +   +---+   +   +   +   +   +   +---+", //   22
    "|   |       |       |       |   |       |   |       |           |", // 4 23
    "+   +   +   +   +---+   +---+   +   +---+   +---+---+---+---+---+", //   24
    "|       |       |       |       |   |           |       |       |", // 3 25
    "+   +   +   +---+   +---+   +---+   +---+   +   +   +   +   +   +", //   26
    "|   |       |       |       |       |       |       |       |   |", // 2 27
    "+   +   +---+   +---+   +---+   +   +   +   +   +   +   +   +   +", //   28
    "|           |                   |       |       |       |       |", // 1 29
    "+   +   +   +---+---+---+---+---+---+---+---+---+---+---+---+   +", //   30
    "|   |   |                                                       |", // 0 31
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"  //    32 
    // 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
};

u_int8_t mapdata[16][16];
u_int8_t map[16][16];
u_int16_t cmap[16][16]; //等高線マップ
u_int8_t smap[16][16];  //既探索マップ
char mouse_x = 0;
char mouse_y = 0;
char mouse_dir = 0;
short Index = 0;

double Mx = 90.0;
double My = 90.0;
double Mangle = 0.0;
double Mv = VELOCITY;
double Momega = 1.5 * M_PI;
int Framerate = 30;
char folder[] = "imgx";
char TurnFlag = 1;

cairo_surface_t *CS;
cairo_t *C_robot;
cairo_t *C_maze;
robot_t Micromouse;



void setup_robot(robot_t *robot)
{
  robot->x = 90.0;
  robot->y = 90.0;
  robot->angle = 90.0*M_PI/180;

  robot->sensor[LEFT_FRONT].pos.x =  32.0;
  robot->sensor[LEFT_FRONT].pos.y =  32.5;
  robot->sensor[LEFT_FRONT].angle = 0.0;

  robot->sensor[LEFT_SIDE].pos.x =  40.0;
  robot->sensor[LEFT_SIDE].pos.y =  15.0;
  robot->sensor[LEFT_SIDE].angle = 60.0*M_PI/180;

  robot->sensor[RIGHT_FRONT].pos.x =  32.0;
  robot->sensor[RIGHT_FRONT].pos.y = -32.5;
  robot->sensor[RIGHT_FRONT].angle = 0.0;

  robot->sensor[RIGHT_SIDE].pos.x =  40.0;
  robot->sensor[RIGHT_SIDE].pos.y = -15.0;
  robot->sensor[RIGHT_SIDE].angle = -60.0*M_PI/180;

}

void hoge(robot_t *robot, uint8_t sensor_id)
{
  point2d_t gpos;//センサ位置

  //left_front_sensor ray equation

    gpos = rotate2d(robot->sensor[sensor_id].pos, robot->angle);
    gpos = move2d(gpos, robot_pos);
    angle = robot->sensor[sensor_id].angle + robot->angle;
    //センサ光線の傾きと切片を算出
    a = tan(angle);
    //y=a(x - x1)+ y1 -> y = ax - a x1 + y1
    //y=ax+b -> y=a(x + b/a) y/a = x +b/a
    b = -a * gpos.x + gpos.y; 

    //光線と壁の交点を求める
    north = find_cross_point(a, b, wa_n, wb_n);
    east  = find_cross_point(a, b, wa_e, wb_e);
    south = find_cross_point(a, b, wa_s, wb_s);
    west  = find_cross_point(a, b, wa_w, wb_w);

    //光線のベクトルを求める
    north = vec_sub2d(north, gpos);
    east  = vec_sub2d(east,  gpos);
    south = vec_sub2d(south, gpos);
    west  = vec_sub2d(west,  gpos);
    
    //光線ベクトルとロボットの方向ベクトルの内積を求め、なす角が鋭角か鈍角を決定
    north_dot = dot_product2d(north, robot_vec);
    east_dot  = dot_product2d(east,  robot_vec);
    south_dot = dot_product2d(south, robot_vec);
    west_dot  = dot_product2d(west,  robot_vec);

    //直近の壁にぶつかる光線だけを選定
    if      (north_dot > 0.0 && x_w <= north.x && north.x <= x_e) detect_point.sen[sensor_id]=north;
    else if (east_dot  > 0.0 && y_s <=  east.y &&  east.y <= y_n) detect_point.sen[sensor_id]=east;
    else if (south_dot > 0.0 && x_w <= south.x && south.x <= x_e) detect_point.sen[sensor_id]=south;
    else if (west_dot  > 0.0 && y_s <=  west.x &&  west.x <= y_n) detect_point.sen[sensor_id]=west;

}

sensor_detect_point_t calc_sensor_ray(robot_t *robot, u_int8_t _map[16][16])
{
  //次にやること
  //mapを見て光線のあたった位置に反映させる
  //直近1区画までを範囲とする

  sensor_detect_point_t detect_point;
  point2d_t gpos;//センサ位置
  point2d_t robot_pos;//ロボット位置
  point2d_t north, east, south, west;//光線のぶつかった位置
  point2d_t robot_vec;//ロボットの方向ベクトル

  double north_dot, east_dot, south_dot, west_dot;
  double x, y, angle;
  double a,b;
  double wa_n, wa_e, wa_s, wa_w;
  double wb_n, wb_e, wb_s, wb_w;
  double y_n, y_s, x_e, x_w;

  robot_pos.x = robot->x;
  robot_pos.y = robot->y;

  //ロボットの方向ベクトル
  robot_vec.x = cos(robot->angle);
  robot_vec.y = sin(robot->angle);


  //直近の壁の方程式をすべて見出して、センサ光線との交点を求める
  x_w = (int)(robot_pos.x/UNIT_LENGTH) * UNIT_LENGTH;
  x_e = x_w + UNIT_LENGTH;
  y_s = (int)(robot_pos.y/UNIT_LENGTH) * UNIT_LENGTH;
  y_n = y_s + UNIT_LENGTH;

  //壁の傾きと切片
  wa_n = 0.0;
  wb_n = y_n;
  wa_e = tan(90*M_PI/180);
  wb_e = wa_e * x_e;
  wa_s = 0.0;
  wb_s = y_s;
  wa_w = tan(90*M_PI/180);
  wb_w = wa_w * x_w;

  //left_front_sensor ray equation

    gpos = rotate2d(robot->sensor[LEFT_FRONT].pos, robot->angle);
    gpos = move2d(gpos, robot_pos);
    angle = robot->sensor[LEFT_FRONT].angle + robot->angle;
    //センサ光線の傾きと切片を算出
    a = tan(angle);
    //y=a(x - x1)+ y1 -> y = ax - a x1 + y1
    //y=ax+b -> y=a(x + b/a) y/a = x +b/a
    b = -a * gpos.x + gpos.y; 

    //光線と壁の交点を求める
    north = find_cross_point(a, b, wa_n, wb_n);
    east  = find_cross_point(a, b, wa_e, wb_e);
    south = find_cross_point(a, b, wa_s, wb_s);
    west  = find_cross_point(a, b, wa_w, wb_w);

    //光線のベクトルを求める
    north = vec_sub2d(north, gpos);
    east  = vec_sub2d(east,  gpos);
    south = vec_sub2d(south, gpos);
    west  = vec_sub2d(west,  gpos);
    
    //光線ベクトルとロボットの方向ベクトルの内積を求め、なす角が鋭角か鈍角を決定
    north_dot = dot_product2d(north, robot_vec);
    east_dot  = dot_product2d(east,  robot_vec);
    south_dot = dot_product2d(south, robot_vec);
    west_dot  = dot_product2d(west,  robot_vec);

    //直近の壁にぶつかる光線だけを選定
    if      (north_dot > 0.0 && x_w <= north.x && north.x <= x_e) detect_point.sen[LEFT_FRONT]=north;
    else if (east_dot  > 0.0 && y_s <=  east.y &&  east.y <= y_n) detect_point.sen[LEFT_FRONT]=east;
    else if (south_dot > 0.0 && x_w <= south.x && south.x <= x_e) detect_point.sen[LEFT_FRONT]=south;
    else if (west_dot  > 0.0 && y_s <=  west.x &&  west.x <= y_n) detect_point.sen[LEFT_FRONT]=west;

  //left_side_sensor ray equation
    gpos = rotate2d(robot->sensor[LEFT_SIDE].pos, robot->angle);
    gpos = move2d(gpos, robot_pos);
    angle = robot->sensor[LEFT_FRONT].angle + robot->angle;
    //センサ光線の傾きと切片を算出
    a = tan(angle);
    //y=a(x - x1)+ y1 -> y = ax - a x1 + y1
    //y=ax+b -> y=a(x + b/a) y/a = x +b/a
    b = -a * gpos.x + gpos.y; 

    //光線と壁の交点を求める
    north = find_cross_point(a, b, wa_n, wb_n);
    east  = find_cross_point(a, b, wa_e, wb_e);
    south = find_cross_point(a, b, wa_s, wb_s);
    west  = find_cross_point(a, b, wa_w, wb_w);

    //光線のベクトルを求める
    north = vec_sub2d(north, gpos);
    east  = vec_sub2d(east,  gpos);
    south = vec_sub2d(south, gpos);
    west  = vec_sub2d(west,  gpos);

    //光線ベクトルとロボットの方向ベクトルの内積を求め、なす角が鋭角か鈍角を決定   
    north_dot = dot_product2d(north, robot_vec);
    east_dot  = dot_product2d(east,  robot_vec);
    south_dot = dot_product2d(south, robot_vec);
    west_dot  = dot_product2d(west,  robot_vec);

    //直近の壁にぶつかる光線だけを選定
    if      (north_dot > 0.0 && x_w <= north.x && north.x <= x_e) detect_point.sen[LEFT_SIDE]=north;
    else if (east_dot  > 0.0 && y_s <=  east.y &&  east.y <= y_n) detect_point.sen[LEFT_SIDE]=east;
    else if (south_dot > 0.0 && x_w <= south.x && south.x <= x_e) detect_point.sen[LEFT_SIDE]=south;
    else if (west_dot  > 0.0 && y_s <=  west.x &&  west.x <= y_n) detect_point.sen[LEFT_SIDE]=west;

  //right_front_sensor ray equation
    gpos = rotate2d(robot->sensor[RIGHT_FRONT].pos, robot->angle);
    gpos = move2d(gpos, robot_pos);
    angle = robot->sensor[RIGHT_FRONT].angle + robot->angle;
    //センサ光線の傾きと切片を算出
    a = tan(angle);
    //y=a(x - x1)+ y1 -> y = ax - a x1 + y1
    //y=ax+b -> y=a(x + b/a) y/a = x +b/a
    b = -a * gpos.x + gpos.y; 

    north = find_cross_point(a, b, wa_n, wb_n);
    east  = find_cross_point(a, b, wa_e, wb_e);
    south = find_cross_point(a, b, wa_s, wb_s);
    west  = find_cross_point(a, b, wa_w, wb_w);

    north = vec_sub2d(north, gpos);
    east  = vec_sub2d(east,  gpos);
    south = vec_sub2d(south, gpos);
    west  = vec_sub2d(west,  gpos);
    
    north_dot = dot_product2d(north, robot_vec);
    east_dot  = dot_product2d(east,  robot_vec);
    south_dot = dot_product2d(south, robot_vec);
    west_dot  = dot_product2d(west,  robot_vec);

    if      (north_dot > 0.0 && x_w <= north.x && north.x <= x_e) detect_point.sen[RIGHT_FRONT]=north;
    else if (east_dot  > 0.0 && y_s <=  east.y &&  east.y <= y_n) detect_point.sen[RIGHT_FRONT]=east;
    else if (south_dot > 0.0 && x_w <= south.x && south.x <= x_e) detect_point.sen[RIGHT_FRONT]=south;
    else if (west_dot  > 0.0 && y_s <=  west.x &&  west.x <= y_n) detect_point.sen[RIGHT_FRONT]=west;

  //right_side_sensor ray equation
    gpos = rotate2d(robot->sensor[RIGHT_SIDE].pos, robot->angle);
    gpos = move2d(gpos, robot_pos);
    angle = robot->sensor[RIGHT_SIDE].angle + robot->angle;
    //センサ光線の傾きと切片を算出
    a = tan(angle);
    //y=a(x - x1)+ y1 -> y = ax - a x1 + y1
    //y=ax+b -> y=a(x + b/a) y/a = x +b/a
    b = -a * gpos.x + gpos.y; 

    north = find_cross_point(a, b, wa_n, wb_n);
    east  = find_cross_point(a, b, wa_e, wb_e);
    south = find_cross_point(a, b, wa_s, wb_s);
    west  = find_cross_point(a, b, wa_w, wb_w);

    north = vec_sub2d(north, gpos);
    east  = vec_sub2d(east,  gpos);
    south = vec_sub2d(south, gpos);
    west  = vec_sub2d(west,  gpos);
    
    north_dot = dot_product2d(north, robot_vec);
    east_dot  = dot_product2d(east,  robot_vec);
    south_dot = dot_product2d(south, robot_vec);
    west_dot  = dot_product2d(west,  robot_vec);

    if      (north_dot > 0.0 && x_w <= north.x && north.x <= x_e) detect_point.sen[RIGHT_SIDE]=north;
    else if (east_dot  > 0.0 && y_s <=  east.y &&  east.y <= y_n) detect_point.sen[RIGHT_SIDE]=east;
    else if (south_dot > 0.0 && x_w <= south.x && south.x <= x_e) detect_point.sen[RIGHT_SIDE]=south;
    else if (west_dot  > 0.0 && y_s <=  west.x &&  west.x <= y_n) detect_point.sen[RIGHT_SIDE]=west;

  return detect_point;
}


int8_t initStack(Stack_t *pStack)
{
  u_int16_t i;

  pStack->pX = (u_int8_t *)malloc(sizeof(u_int8_t) * STACK_SIZE);
  pStack->pY = (u_int8_t *)malloc(sizeof(u_int8_t) * STACK_SIZE);

  if (pStack->pX == NULL || pStack->pY == NULL)
  {
    printf("malloc error\n");
    return -1;
  }
  //スタックの中身を0埋め
  for (i = 0; i < STACK_SIZE; i++)
  {
    pStack->pX[i] = 0;
    pStack->pY[i] = 0;
  }
  //初期化時は空なのでtailは-1とする
  pStack->tail = -1;
  //初期化時はスタック1個分なので1とする
  pStack->thisNum = 1;
  printf("init stack\n");
  // printStack(pStack);
  return 0;
}

//終了関数
int8_t deleteStack(Stack_t *pStack)
{
  free(pStack->pX);
  free(pStack->pY);
  return 0;
}

//スタックの中身をprint出力
void printStack(Stack_t *pStack)
{
  int i;
  for (i = 0; i < STACK_SIZE * pStack->thisNum; i++)
  {
    printf("%d %d\n", pStack->pX[i], pStack->pY[i]);
  }
  printf("\n");
}

// push関数
int push(Stack_t *pStack, u_int8_t valueX, u_int8_t valueY)
{
  u_int8_t *pTmpX;
  u_int8_t *pTmpY;
  int i;
  //スタックがFullの処理
  if (pStack->tail >= STACK_SIZE * NUM_MAX - 1)
  {
    printf("Full\n");
    printStack(pStack);
    return 0;
  }

  //スタックの拡張
  if (pStack->tail >= STACK_SIZE * pStack->thisNum - 1)
  {
    printf("Upsize stack\n");
    pStack->thisNum++;
    pTmpX = (u_int8_t *)realloc(pStack->pX, sizeof(u_int8_t) * STACK_SIZE * pStack->thisNum);
    pTmpY = (u_int8_t *)realloc(pStack->pY, sizeof(u_int8_t) * STACK_SIZE * pStack->thisNum);
    if (pTmpX == NULL || pTmpY == NULL)
    {
      printf("realloc error\n");
      return -1;
    }
    else
    {
      pStack->pX = pTmpX;
      pStack->pY = pTmpY;
      for (i = pStack->tail + 1; i < STACK_SIZE * pStack->thisNum; i++)
      {
        pStack->pX[i] = 0;
        pStack->pY[i] = 0;
      }
    }
  }
  // Push操作
  pStack->tail++;
  pStack->pX[pStack->tail] = valueX;
  pStack->pY[pStack->tail] = valueY;

  printf("Push\n");
  printStack(pStack);

  return 0;
}

// pop関数
int8_t pop(Stack_t *pStack)
{
  u_int8_t *pTmpX;
  u_int8_t *pTmpY;

  //スタックがEmptyの処理
  if (pStack->tail <= -1)
  {
    printf("Empty\n");
    return 0;
  }

  //スタックの縮小
  if (pStack->thisNum > 1 && pStack->tail <= STACK_SIZE * (pStack->thisNum - 1))
  {
    printf("Downsize stack\n");
    pStack->thisNum--;
    pTmpX = (u_int8_t *)realloc(pStack->pX, sizeof(u_int8_t) * STACK_SIZE * pStack->thisNum);
    pTmpY = (u_int8_t *)realloc(pStack->pY, sizeof(u_int8_t) * STACK_SIZE * pStack->thisNum);
    if (pTmpX == NULL || pTmpY == NULL)
    {
      printf("realloc error\n");
      return -1;
    }
    else
    {
      pStack->pX = pTmpX;
      pStack->pY = pTmpY;
    }
  }
  // Pop操作
  pStack->pX[pStack->tail] = 0;
  pStack->pY[pStack->tail] = 0;
  pStack->tail--;
  printf("Pop stack\n");
  printStack(pStack);

  return 0;
}

void copy_mapdata(u_int8_t _map[16][16])
{
  for (u_int8_t y = 0; y < 16; y++)
  {
    for (u_int8_t x = 0; x < 16; x++)
    {
      map[x][y] = _map[x][y];
    }
  }
}

void reset_contourmap(void)
{
  for (u_int8_t y = 0; y < 16; y++)
  {
    for (u_int8_t x = 0; x < 16; x++)
    {
      cmap[x][y] = 0xffff;
    }
  }
}

void make_contourmap(u_int8_t _map[16][16], u_int8_t sx, u_int8_t sy, u_int8_t gx, u_int8_t gy)
{

  // output_cmap();

  u_int16_t tmp_cnt = 0;
  u_int16_t tmp_max = 0;
  unsigned short step = 0;
  cmap[gx][gy] = step;

  while (cmap[sx][sy] == 0xffff)
  {
    tmp_cnt = 0;
    for (unsigned char y = 0; y < 16; y++)
    {
      for (unsigned char x = 0; x < 16; x++)
      {
        if (cmap[x][y] == step)
        {
          tmp_cnt++;
          // North
          if (!(_map[x][y] & 0x01) && cmap[x][y + 1] == 0xffff && y < 15)
            cmap[x][y + 1] = step + 1;
          // East
          if (!(_map[x][y] & 0x02) && cmap[x + 1][y] == 0xffff && x < 15)
            cmap[x + 1][y] = step + 1;
          // South
          if (!(_map[x][y] & 0x04) && cmap[x][y - 1] == 0xffff && y > 0)
            cmap[x][y - 1] = step + 1;
          // West
          if (!(_map[x][y] & 0x08) && cmap[x - 1][y] == 0xffff && x > 0)
            cmap[x - 1][y] = step + 1;
        }
      }
    }
    if (tmp_max < tmp_cnt)
      tmp_max = tmp_cnt;
    // output_cmap();
    step++;
  }

  output_cmap();
  printf("%d\n", tmp_max);
}

void make_contourmap2(u_int8_t _map[16][16], u_int8_t sx, u_int8_t sy, u_int8_t gx, u_int8_t gy)
{
  Queue_t cell;

  initQueue(&cell);

  // output_cmap();

  u_int16_t tmp_cnt = 0;
  u_int16_t tmp_max = 0;
  u_int8_t x, y;
  unsigned short step = 0;

  cmap[gx][gy] = step;
  enqueue(&cell, gx, gy);

  while (
      cmap[sx][sy] == 0xffff ||
      (cmap[sx][sy + 1] == 0xffff && !(map[sx][sy] & 0x01) && sy < 15) ||
      (cmap[sx + 1][sy] == 0xffff && !(map[sx][sy] & 0x02) && sx < 15) ||
      (cmap[sx][sy - 1] == 0xffff && !(map[sx][sy] & 0x04) && sy > 0) ||
      (cmap[sx - 1][sy] == 0xffff && !(map[sx][sy] & 0x08) && sx > 0))
  {
    x = cell.dataX[cell.head];
    y = cell.dataY[cell.head];
    step = cmap[x][y];
    dequeue(&cell);

    // tmp_cnt++;
    // North
    if (!(_map[x][y] & 0x01) && cmap[x][y + 1] == 0xffff && y < 15)
    {
      cmap[x][y + 1] = step + 1;
      enqueue(&cell, x, y + 1);
    }
    // East
    if (!(_map[x][y] & 0x02) && cmap[x + 1][y] == 0xffff && x < 15)
    {
      cmap[x + 1][y] = step + 1;
      enqueue(&cell, x + 1, y);
    }
    // South
    if (!(_map[x][y] & 0x04) && cmap[x][y - 1] == 0xffff && y > 0)
    {
      cmap[x][y - 1] = step + 1;
      enqueue(&cell, x, y - 1);
    }
    // West
    if (!(_map[x][y] & 0x08) && cmap[x - 1][y] == 0xffff && x > 0)
    {
      cmap[x - 1][y] = step + 1;
      enqueue(&cell, x - 1, y);
    }
     output_cmap();
  }

  output_cmap();
  //deleteStack(&cell);
}

void output_cmap(void)
{

  for (char y = 15; y >= 0; y--)
  {
    for (char x = 0; x < 16; x++)
    {
      printf("%05d ", cmap[x][y]);
    }
    printf("\n");
  }
  printf("\n");
}

void plot(double *x, double *y, int n)
{

  for (int i = 1; i < n; i++)
  {
  }
}

void output_to_img(int cnt, data_t *data)
{
  char str[50];
  printf("---------+---------+\n");
  for (int i = 0; i < cnt; i++)
  {
    sprintf(str, "%s/test%05d.png", folder, Index++);
    // draw_traj(i, data);
    draw_mouse(data[i].x, data[i].y, data[i].angle);
    cairo_surface_write_to_png(CS, str);
    erase_mouse2(data[i].x, data[i].y, data[i].angle);
    // erase_traj(i, data);
    printf(" %2d%%\r", (int)(i * 100 / cnt));
    fflush(stdout);
  }
  printf("\nFinish!\n");
}

void calc_trajectry(void)
{
  double state[5];
  double u[2];
  double acc0 = ACC0;
  double angle_acc0 = ANGLE_ACC0;
  double acc = 0.0;
  double angle_acc = 0.0;
  double x = 180.0;
  double y = 90.0;
  double angle = 0.0;
  double v = VELOCITY;
  double omega = 0.0;
  double t = 0.0;
  double h = STEP;
  double fin = 2.0;
  double framerate = 30.0;
  double sample_time = 0.0;
  int logcnt = 0;

  double vs = VELOCITY;
  double vt = TURN_VELOCITY;
  double angle1 = 30.0;
  double angle2 = 90.0 - angle1;

  data_t data[1000];

  int mode = 0;

  // cairo_set_source_rgb( C, 0, 0, 0 );
  // cairo_rectangle( C, 0, 0, WIDTH, HEIGHT );
  // cairo_fill( C );
  vt = 190.0;
  // vt=TURN_VELOCITY h=1e-6においての最適値

  while (vt < 192.0)
  {
    logcnt = 0;
    mode = 0;
    acc = -acc0;
    t = 0.0;
    sample_time = 0.0;
    x = 180.0;
    y = 90.0;
    angle = 0.0;
    v = VELOCITY;
    omega = 0.0;

    while (mode != 999 && logcnt < 999)
    {
      // sampling data
      if (t >= sample_time)
      {
        // printf("%d %f %12.8f %12.8f %f %f %f %f\n", logcnt, t, x, y, angle*180/M_PI, v, omega, vt);
        data[logcnt].t = sample_time;
        data[logcnt].x = x;
        data[logcnt].y = y;
        data[logcnt].angle = angle;
        data[logcnt].velocity = v;
        data[logcnt].omega = omega;
        data[logcnt].red = 0.5;
        data[logcnt].green = 0.5;
        data[logcnt].blue = 0.5;
        sample_time = sample_time + 1 / framerate;
        logcnt++;
      }

      switch (mode)
      {
      case 0: //減速
      {
        if (v <= vt)
        {
          acc = 0.0;
          angle_acc = angle_acc0;
          mode = 1;
        }
        break;
      }
      case 1: //クロソイド
      {
        if (angle >= M_PI * angle1 / 180)
        {
          angle_acc = 0.0;
          mode = 2;
        }
        break;
      }
      case 2: //円弧
      {
        if (angle >= M_PI * angle2 / 180)
        {
          angle_acc = -angle_acc0;
          mode = 3;
        }
        break;
      }
      case 3: //クロソイド
      {
        if (angle >= M_PI * 0.5)
        {
          acc = acc0;
          angle_acc = 0.0;
          mode = 4;
        }
        break;
      }
      case 4: //加速
      {
        if (v >= vs)
        {
          acc = 0.0;
          mode = 999;
        }
        break;
      }
      }
      // printf("%d %f %12.8f %12.8f %f %f %f %f\n", logcnt, t, x, y, angle*180/M_PI, v, omega, vt);

      // Fhysical Simulation
      state[0] = x;
      state[1] = y;
      state[2] = angle;
      state[3] = v;
      state[4] = omega;
      u[0] = acc;
      u[1] = angle_acc;
      rk4(eom, state, t, u, h, 5);
      x = state[0];
      y = state[1];
      angle = state[2];
      v = state[3];
      omega = state[4];
      t = t + h;
    }
    double errx = (x - 180.0) - 90.0;
    double erry = (y - 90.0) - 90.0;
    printf("%d %13.9f %12.8f %12.8f %f %f %f %f %f\n", logcnt, t, (x - 180.0), (y - 90.0), sqrt(errx * errx + erry * erry), angle * 180 / M_PI, v, omega, vt);
    // printf("-------------------------\n");
    fflush(stdout);
    data[logcnt].t = sample_time;
    data[logcnt].x = x;
    data[logcnt].y = y;
    data[logcnt].angle = angle;
    data[logcnt].velocity = v;
    data[logcnt].omega = omega;
    data[logcnt].red = 0.5;
    data[logcnt].green = 0.5;
    data[logcnt].blue = 0.5;
    logcnt++;

    // output_to_img(logcnt, data);
    vt = vt + 0.1;
  }
}

#if 1
void slalom_phy(robot_t *robot, char dir)
{
  double state[5];
  double u[2];
  double x, y, angle, v, omega;
  double acc;
  double angle_acc;
  double acc0 = ACC0;
  double angle_acc0 = ANGLE_ACC0;
  double t;
  double framerate = 30.0;
  double h = STEP;
  double sample_time = 0.0;
  double vs = VELOCITY;
  double vt = robot->turn_velocity; // TURN_VELOCITY;
  double angle1 = robot->angle1;
  double angle2 = M_PI / 2 - angle1;
  double angle0;
  double errx, erry;
  data_t data[1000];

  int logcnt = 0;
  int mode = 0;

  t = 0.0;
  acc = -acc0;
  angle_acc = 0.0;
  sample_time = 0.0;
  x = 0.0; // robot.x;
  y = 0.0; // robot.y;
  angle = robot->angle;
  angle0 = angle;
  v = robot->velocity;
  omega = robot->omega;
  // omega = 0.0;

  errx = x - 90.0;
  erry = y - 90.0;

#ifdef SLA_OUTPUT
  printf("SLALOM Start:%d %13.9f %12.8f %12.8f %f %f %f %f\n", logcnt, t, robot->x, robot->y, angle * 180 / M_PI, v, omega, vt);
#endif

  while (mode != 999 && logcnt < 999)
  {
    // sampling data
    if (t >= sample_time)
    {
      errx = x - 90.0;
      erry = y - 90.0;

#ifdef SLA_OUTPUT
      printf("%d %13.9f %12.8f %12.8f %f %f %f %f %f\n", logcnt, t, x, y, sqrt(errx * errx + erry * erry), angle * 180 / M_PI, v, omega, vt);
#endif
      data[logcnt].t = sample_time;
      data[logcnt].x = robot->x + x;
      data[logcnt].y = robot->y + y;
      data[logcnt].angle = angle;
      data[logcnt].velocity = v;
      data[logcnt].omega = omega;
      data[logcnt].red = 0.5;
      data[logcnt].green = 0.5;
      data[logcnt].blue = 0.5;
      sample_time = sample_time + 1 / framerate;
      logcnt++;
    }

    switch (mode)
    {
    case 0: //減速
    {
      if (v <= vt)
      {
        acc = 0.0;
        angle_acc = -dir * angle_acc0;
        mode = 1;
      }
      break;
    }
    case 1: //クロソイド
    {
      if (fabs(angle - angle0) >= angle1)
      {
        angle_acc = 0.0;
        mode = 2;
      }
      break;
    }
    case 2: //円弧
    {
      if (fabs(angle - angle0) >= angle2)
      {
        angle_acc = dir * angle_acc0;
        mode = 3;
      }
      break;
    }
    case 3: //クロソイド
    {
      if (dir == CW)
      {
        if (omega >= 0)
        {
          acc = acc0;
          angle_acc = 0.0;
          mode = 4; /*printf("Hit!\n");*/
        }
      }
      else
      {
        if (omega <= 0)
        {
          acc = acc0;
          angle_acc = 0.0;
          mode = 4; /*printf("Hit!\n");*/
        }
      }
      break;
    }
    case 4: //加速
    {
      if (v >= vs)
      {
        acc = 0.0;
        mode = 5;
        dir = (robot->dir + dir) & 3;
      }
      break;
    }
    case 5: //位置合わせ
    {
      switch (dir)
      {
      case 0:
      {
        if (y >= 90.0)
        {
          mode = 999;
        }
        break;
      }
      case 1:
      {
        if (x >= 90.0)
        {
          mode = 999;
        }
        break;
      }
      case 2:
      {
        if (y <= -90.0)
        {
          mode = 999;
        }
        break;
      }
      case 3:
      {
        if (x <= -90.0)
        {
          mode = 999;
        }
        break;
      }
      }
      break;
    }
    }

    // Fhysical Simulation
    state[0] = x;
    state[1] = y;
    state[2] = angle;
    state[3] = v;
    state[4] = omega;
    u[0] = acc;
    u[1] = angle_acc;
    rk4(eom, state, t, u, h, 5);
    x = state[0];
    y = state[1];
    angle = state[2];
    v = state[3];
    omega = state[4];
    t = t + h;
  }

  errx = x - 90.0;
  erry = y - 90.0;
  // printf("%d %13.9f %12.8f %12.8f %f %f %f %f %f\n", logcnt, t, x, y, sqrt(errx*errx+erry*erry), angle*180/M_PI, v, omega, vt);
  // printf("-------------------------\n");
  // fflush(stdout);

  data[logcnt].t = sample_time;
  data[logcnt].x = x;
  data[logcnt].y = y;
  data[logcnt].angle = angle;
  data[logcnt].velocity = v;
  data[logcnt].omega = omega;
  data[logcnt].red = 0.5;
  data[logcnt].green = 0.5;
  data[logcnt].blue = 0.5;

#ifdef SLA_OUTPUT
  output_to_img(logcnt, data);
#endif

  // Robot State Update
  robot->t = robot->t + t;
  robot->x = robot->x + x;
  robot->y = robot->y + y;
  robot->angle = angle;
  robot->velocity = v;
  // omega=0.0;
  robot->omega = omega;
  robot->dir = dir;

  errx = x - 90.0;
  erry = y - 90.0;

#ifdef SLA_OUTPUT
  printf("SLALOM End:%d %13.9f %12.8f %12.8f %f %f %f %f\n",
         logcnt, t, robot->x, robot->y, angle * 180 / M_PI, v, omega, vt);
#endif
}

void straight_phy(robot_t *robot)
{
  double state[5];
  double u[2];
  double x, y, angle, v, omega;
  double acc;
  double angle_acc;
  double acc0 = ACC0;
  double angle_acc0 = ANGLE_ACC0;
  double t;
  double framerate = 30.0;
  double h = STEP;
  double sample_time = 0.0;
  double vs = VELOCITY;
  double vt = TURN_VELOCITY;
  double angle1 = ANGLE1; // 30.0*M_PI/180;
  double angle2 = M_PI / 2 - angle1;
  double angle0;
  data_t data[1000];
  int logcnt = 0;
  int mode = 0;
  int dir;

  printf("Start:%f %f %f %d\n", robot->x, robot->y, 180 * robot->angle / M_PI, robot->dir);

  t = 0.0;
  acc = 0.0;
  angle_acc = 0.0;
  sample_time = 0.0;
  x = 0.0; // robot.x;
  y = 0.0; // robot.y;
  angle = robot->angle;
  angle0 = angle;
  v = robot->velocity;
  omega = robot->omega;
  // omega = 0.0;
  dir = robot->dir;

  while (mode != 999 && logcnt < 999)
  {
    // sampling data
    if (t >= sample_time)
    {
      printf("%d %f %f %f %f %f\n", logcnt, x, y, angle * 180 / M_PI, v, omega);
      data[logcnt].t = sample_time;
      data[logcnt].x = robot->x + x;
      data[logcnt].y = robot->y + y;
      data[logcnt].angle = angle;
      data[logcnt].velocity = v;
      data[logcnt].omega = omega;
      data[logcnt].red = 0.5;
      data[logcnt].green = 0.5;
      data[logcnt].blue = 0.5;
      sample_time = sample_time + 1 / framerate;
      logcnt++;
    }

    switch (mode)
    {
    case 0:
    {
      switch (dir)
      {
      case 0:
      {
        if (y >= 180.0)
        {
          mode = 999;
        }
      }
      case 1:
      {
        if (x >= 180.0)
        {
          mode = 999;
        }
      }
      case 2:
      {
        if (y <= -180.0)
        {
          mode = 999;
        }
      }
      case 3:
      {
        if (x <= -180.0)
        {
          mode = 999;
        }
      }
      }
      break;
    }
    }

    // Fhysical Simulation
    state[0] = x;
    state[1] = y;
    state[2] = angle;
    state[3] = v;
    state[4] = omega;
    u[0] = acc;
    u[1] = angle_acc;
    rk4(eom, state, t, u, h, 5);
    x = state[0];
    y = state[1];
    angle = state[2];
    v = state[3];
    omega = state[4];
    t = t + h;
  }

  double errx = x - 90.0;
  double erry = y - 90.0;
  printf("%d %13.9f %12.8f %12.8f %f %f %f %f %f\n", logcnt, t, x, y, sqrt(errx * errx + erry * erry), angle * 180 / M_PI, v, omega, vt);
  // printf("-------------------------\n");
  fflush(stdout);

  data[logcnt].t = sample_time;
  data[logcnt].x = x;
  data[logcnt].y = y;
  data[logcnt].angle = angle;
  data[logcnt].velocity = v;
  data[logcnt].omega = omega;
  data[logcnt].red = 0.5;
  data[logcnt].green = 0.5;
  data[logcnt].blue = 0.5;

  output_to_img(logcnt, data);

  // Robot State Update
  robot->t = robot->t + t;
  robot->x = robot->x + x;
  robot->y = robot->y + y;
  robot->angle = angle;
  robot->velocity = v;
  robot->omega = omega;
  robot->dir = dir;
  // printf("*** %f,%f,%f,%d",robot.x,robot.y,robot.angle*180/M_PI, robot.dir);
  printf("End  :%f %f %f %d\n", robot->x, robot->y, 180 * robot->angle / M_PI, robot->dir);
}

void turn180_phy(robot_t *robot)
{
  double state[5];
  double u[2];
  double x, y, angle, v, omega;
  double acc;
  double angle_acc;
  double acc0 = ACC0;
  double angle_acc0 = ANGLE_ACC0;
  double t;
  double framerate = 30.0;
  double h = STEP;
  double sample_time = 0.0;
  double vs = VELOCITY;
  double angle1 = M_PI;
  double angle0;
  data_t data[1000];
  int logcnt = 0;
  int mode = 0;
  int dir;

  printf("180Start:%f %f %f %d\n", robot->x, robot->y, 180 * robot->angle / M_PI, robot->dir);

  t = 0.0;
  angle_acc = 0.0;
  sample_time = 0.0;
  x = 0.0; // robot.x;
  y = 0.0; // robot.y;
  angle = robot->angle;
  angle0 = angle;
  v = robot->velocity;
  omega = robot->omega;
  // omega = 0.0;
  dir = robot->dir;
  acc = -v * v / 2 / 90.0;

  while (mode != 999 && logcnt < 999)
  {
    // sampling data
    if (t >= sample_time)
    {
      printf("%d %13.9f %12.8f %12.8f %f %f %f %d\n", logcnt, t, x, y, angle * 180 / M_PI, v, omega, dir);
      data[logcnt].t = sample_time;
      data[logcnt].x = robot->x + x;
      data[logcnt].y = robot->y + y;
      data[logcnt].angle = angle;
      data[logcnt].velocity = v;
      data[logcnt].omega = omega;
      data[logcnt].red = 0.5;
      data[logcnt].green = 0.5;
      data[logcnt].blue = 0.5;
      sample_time = sample_time + 1 / framerate;
      logcnt++;
    }

    switch (mode)
    {
    case 0: //減速
    {
      if (v <= 0)
      {
        acc = 0.0;
        angle_acc = (TurnFlag)*angle_acc0;
        mode = 1;
      }
      break;
    }
    case 1: //旋回加速
    {
      if (fabs(angle - angle0) >= angle1 / 2)
      {
        angle_acc = -(TurnFlag)*angle_acc0;
        mode = 2;
      }
      break;
    }
    case 2: //旋回減速
    {
      if (TurnFlag == CW)
      {
        if (omega <= 0)
        {
          acc = acc0;
          angle_acc = 0.0;
          mode = 3;
          printf("Hit!\n");
        }
      }
      else
      {
        if (omega >= 0)
        {
          acc = acc0;
          angle_acc = 0.0;
          mode = 3;
          printf("Hit!\n");
        }
      }
      break;
    }
    case 3: //加速
    {
      if (v >= vs)
      {
        acc = 0.0;
        mode = 4;
        dir = (robot->dir + 2) & 3;
      }
      break;
    }
    case 4: //位置合わせ
    {
      switch (dir)
      {
      case 0:
      {
        if (y >= 0.0)
        {
          mode = 999;
        }
        break;
      }
      case 1:
      {
        if (x >= 0.0)
        {
          mode = 999;
        }
        break;
      }
      case 2:
      {
        if (y <= 0.0)
        {
          mode = 999;
        }
        break;
      }
      case 3:
      {
        if (x <= 0.0)
        {
          mode = 999;
        }
        break;
      }
      }
      break;
    }
    }

    // Fhysical Simulation
    state[0] = x;
    state[1] = y;
    state[2] = angle;
    state[3] = v;
    state[4] = omega;
    u[0] = acc;
    u[1] = angle_acc;
    rk4(eom, state, t, u, h, 5);
    x = state[0];
    y = state[1];
    angle = state[2];
    v = state[3];
    omega = state[4];
    t = t + h;
  }

  printf("%d %13.9f %12.8f %12.8f %f %f %f %d\n", logcnt, t, x, y, angle * 180 / M_PI, v, omega, dir);
  // printf("-------------------------\n");
  fflush(stdout);

  data[logcnt].t = sample_time;
  data[logcnt].x = x;
  data[logcnt].y = y;
  data[logcnt].angle = angle;
  data[logcnt].velocity = v;
  data[logcnt].omega = omega;
  data[logcnt].red = 0.5;
  data[logcnt].green = 0.5;
  data[logcnt].blue = 0.5;

  output_to_img(logcnt, data);

  // Robot State Update
  robot->t = robot->t + t;
  robot->x = robot->x + x;
  robot->y = robot->y + y;
  robot->angle = angle;
  robot->velocity = v;
  robot->omega = omega;
  robot->dir = dir;
  // printf("*** %f,%f,%f,%d",robot.x,robot.y,robot.angle*180/M_PI, robot.dir);
  printf("180End:%d %13.9f %12.8f %12.8f %f %f %f %d\n", logcnt, t, robot->x, robot->y, angle * 180 / M_PI, v, omega, dir);

  TurnFlag = -TurnFlag;
}

void stop_phy(robot_t *robot)
{
  double state[5];
  double u[2];
  double x, y, angle, v, omega;
  double acc;
  double angle_acc;
  double acc0 = ACC0;
  double angle_acc0 = ANGLE_ACC0;
  double t;
  double framerate = 30.0;
  double h = STEP;
  double sample_time = 0.0;
  double vs = VELOCITY;
  double angle1 = M_PI;
  double angle0;
  data_t data[1000];
  int logcnt = 0;
  int mode = 0;
  int dir;

  printf("180Start:%f %f %f %d\n", robot->x, robot->y, 180 * robot->angle / M_PI, robot->dir);

  t = 0.0;
  angle_acc = 0.0;
  sample_time = 0.0;
  x = 0.0; // robot.x;
  y = 0.0; // robot.y;
  angle = robot->angle;
  angle0 = angle;
  v = robot->velocity;
  omega = robot->omega;
  // omega = 0.0;
  dir = robot->dir;
  acc = -v * v / 2 / 90.0;

  while (mode != 999 && logcnt < 999)
  {
    // sampling data
    if (t >= sample_time)
    {
      printf("%d %13.9f %12.8f %12.8f %f %f %f %d\n", logcnt, t, x, y, angle * 180 / M_PI, v, omega, dir);
      data[logcnt].t = sample_time;
      data[logcnt].x = robot->x + x;
      data[logcnt].y = robot->y + y;
      data[logcnt].angle = angle;
      data[logcnt].velocity = v;
      data[logcnt].omega = omega;
      data[logcnt].red = 0.5;
      data[logcnt].green = 0.5;
      data[logcnt].blue = 0.5;
      sample_time = sample_time + 1 / framerate;
      logcnt++;
    }

    switch (mode)
    {
    case 0: //減速
    {
      if (v <= 0)
      {
        acc = 0.0;
        angle_acc = (TurnFlag)*angle_acc0;
        mode = 1;
      }
      break;
    }
    case 1: //旋回加速
    {
      if (fabs(angle - angle0) >= angle1 / 2)
      {
        angle_acc = -(TurnFlag)*angle_acc0;
        mode = 2;
      }
      break;
    }
    case 2: //旋回減速
    {
      if (TurnFlag == CW)
      {
        if (omega <= 0)
        {
          acc = 0.0;
          angle_acc = 0.0;
          mode = 999;
          printf("Hit!\n");
        }
      }
      else
      {
        if (omega >= 0)
        {
          acc = 0.0;
          angle_acc = 0.0;
          mode = 999;
          printf("Hit!\n");
        }
      }
      break;
    }
    }

    // Fhysical Simulation
    state[0] = x;
    state[1] = y;
    state[2] = angle;
    state[3] = v;
    state[4] = omega;
    u[0] = acc;
    u[1] = angle_acc;
    rk4(eom, state, t, u, h, 5);
    x = state[0];
    y = state[1];
    angle = state[2];
    v = state[3];
    omega = state[4];
    t = t + h;
  }

  printf("%d %13.9f %12.8f %12.8f %f %f %f %d\n", logcnt, t, x, y, angle * 180 / M_PI, v, omega, dir);
  // printf("-------------------------\n");
  fflush(stdout);

  data[logcnt].t = sample_time;
  data[logcnt].x = x;
  data[logcnt].y = y;
  data[logcnt].angle = angle;
  data[logcnt].velocity = v;
  data[logcnt].omega = omega;
  data[logcnt].red = 0.5;
  data[logcnt].green = 0.5;
  data[logcnt].blue = 0.5;

  output_to_img(logcnt, data);

  // Robot State Update
  robot->t = robot->t + t;
  robot->x = robot->x + x;
  robot->y = robot->y + y;
  robot->angle = angle;
  robot->velocity = v;
  robot->omega = omega;
  robot->dir = dir;
  // printf("*** %f,%f,%f,%d",robot.x,robot.y,robot.angle*180/M_PI, robot.dir);
  printf("180End:%d %13.9f %12.8f %12.8f %f %f %f %d\n", logcnt, t, robot->x, robot->y, angle * 180 / M_PI, v, omega, dir);

  TurnFlag = -TurnFlag;
}

#if 0
void stop_phy(robot_t *robot)
{
  double state[5];
  double u[2];
  double x,y,angle,v,omega;
  double acc;
  double angle_acc;
  double acc0=ACC0;
  double angle_acc0 = ANGLE_ACC0;
  double t;
  double framerate=30.0;
  double h=STEP;
  double sample_time = 0.0;
  double vs=VELOCITY;
  double angle1=M_PI;
  double angle0;
  data_t data[1000];
  int logcnt=0;
  int mode=0;
  int dir;

  printf("StopStart:%f %f %f %d\n",robot->x, robot->y, 180*robot->angle/M_PI, robot->dir);

  t=0.0;
  angle_acc=0.0;
  sample_time = 0.0;
  x = 0.0;//robot.x;
  y = 0.0;//robot.y;
  angle = robot->angle;
  angle0 = angle;
  v = robot->velocity;
  omega = robot->omega;
  //omega = 0.0;
  dir = robot->dir;
  acc=-v*v/2/90.0;

  while(mode!=999 && logcnt<999)
  {
    //sampling data
    if(t>=sample_time){
      printf("%d %f %f %f %f %f %d\n", logcnt, x, y, angle*180/M_PI, v, omega, dir);
      data[logcnt].t = sample_time;
      data[logcnt].x = robot->x+x;
      data[logcnt].y = robot->y+y;
      data[logcnt].angle=angle;
      data[logcnt].velocity=v;
      data[logcnt].omega=omega;
      data[logcnt].red=0.5;
      data[logcnt].green=0.5;
      data[logcnt].blue=0.5;
      sample_time = sample_time + 1/framerate;
      logcnt++;
    }

    switch(mode)
    {
      case 0://減速
      {
        if(v<=0){acc=0.0;angle_acc=angle_acc0;mode=1;}
        break;
      }
      case 1://旋回加速
      {
        if(fabs(angle-angle0)>=angle1/2){angle_acc=-angle_acc0;mode=2;}
        break;
      }
      case 2://旋回減速
      {
        if(fabs(angle-angle0)>=angle1){acc=0.0;angle_acc=0.0;v=0.0;mode=999;}
        break;
      }
    }

    //Fhysical Simulation
    state[0]=x;
    state[1]=y;
    state[2]=angle;
    state[3]=v;
    state[4]=omega;
    u[0] = acc;
    u[1] = angle_acc;
    rk4(eom, state, t, u, h, 5);
    x=state[0];
    y=state[1];
    angle=state[2];
    v=state[3];
    omega=state[4];
    t=t+h;
  }

  double errx=x;
  double erry=y;
  printf("%d %13.9f %12.8f %12.8f %f %f %f %f\n", logcnt, t, x, y, sqrt(errx*errx+erry*erry), angle*180/M_PI, v, omega);
  //printf("-------------------------\n");
  fflush(stdout);

  data[logcnt].t = sample_time;
  data[logcnt].x = x;
  data[logcnt].y = y;
  data[logcnt].angle=angle;
  data[logcnt].velocity=v;
  data[logcnt].omega=omega;
  data[logcnt].red=0.5;
  data[logcnt].green=0.5;
  data[logcnt].blue=0.5;
  
  output_to_img(logcnt, data);

  //Robot State Update
  robot->t = robot->t + t;
  robot->x = robot->x + x;
  robot->y = robot->y + y;
  robot->angle = angle;
  robot->velocity=v;
  robot->omega=omega;
  robot->dir = dir;

  //printf("*** %f,%f,%f,%d",robot.x,robot.y,robot.angle*180/M_PI, robot.dir);
  printf("StopEnd  :%f %f %f %d\n",robot->x, robot->y, 180*robot->angle/M_PI, robot->dir);
}
#endif

void start_phy(robot_t *robot)
{
  double state[5];
  double u[2];
  double x, y, angle, v, omega;
  double acc;
  double angle_acc;
  double acc0 = ACC0;
  double angle_acc0 = ANGLE_ACC0;
  double t;
  double framerate = 30.0;
  double h = STEP;
  double sample_time = 0.0;
  double vs = VELOCITY;
  double vt = TURN_VELOCITY;
  double angle1 = 180.0 * M_PI / 180;
  double angle2 = M_PI / 2 - angle1;
  double angle0;
  data_t data[1000];
  int logcnt = 0;
  int mode = 0;
  int dir;

  printf("Start:%f %f %f %d\n", robot->x, robot->y, 180 * robot->angle / M_PI, robot->dir);

  t = 0.0;
  acc = acc0;
  ;
  angle_acc = 0.0;
  sample_time = 0.0;
  x = 0.0; // robot.x;
  y = 0.0; // robot.y;
  angle = robot->angle;
  angle0 = angle;
  v = 0.0; // robot->velocity;
  omega = robot->omega;
  // omega = 0.0;
  dir = robot->dir;

  while (mode != 999 && logcnt < 999)
  {
    // sampling data
    if (t >= sample_time)
    {
      printf("%d %f %f %f %f %f\n", logcnt, x, y, angle * 180 / M_PI, v, omega);
      data[logcnt].t = sample_time;
      data[logcnt].x = robot->x + x;
      data[logcnt].y = robot->y + y;
      data[logcnt].angle = angle;
      data[logcnt].velocity = v;
      data[logcnt].omega = omega;
      data[logcnt].red = 0.5;
      data[logcnt].green = 0.5;
      data[logcnt].blue = 0.5;
      sample_time = sample_time + 1 / framerate;
      logcnt++;
    }

    switch (mode)
    {
    case 0: //加速
    {
      if (v >= vs)
      {
        acc = 0.0;
        mode = 1;
      }
      break;
    }
    case 1: //位置合わせ
    {
      switch (dir)
      {
      case 0:
      {
        if (y >= 90.0)
        {
          mode = 999;
        }
        break;
      }
      case 1:
      {
        if (x >= 90.0)
        {
          mode = 999;
        }
        break;
      }
      case 2:
      {
        if (y <= -90.0)
        {
          mode = 999;
        }
        break;
      }
      case 3:
      {
        if (x <= -90.0)
        {
          mode = 999;
        }
        break;
      }
      }
      break;
    }
    }

    // Fhysical Simulation
    state[0] = x;
    state[1] = y;
    state[2] = angle;
    state[3] = v;
    state[4] = omega;
    u[0] = acc;
    u[1] = angle_acc;
    rk4(eom, state, t, u, h, 5);
    x = state[0];
    y = state[1];
    angle = state[2];
    v = state[3];
    omega = state[4];
    t = t + h;
  }

  double errx = x;
  double erry = y;
  printf("%d %13.9f %12.8f %12.8f %f %f %f %f %f\n", logcnt, t, x, y, sqrt(errx * errx + erry * erry), angle * 180 / M_PI, v, omega, vt);
  // printf("-------------------------\n");
  fflush(stdout);

  data[logcnt].t = sample_time;
  data[logcnt].x = x;
  data[logcnt].y = y;
  data[logcnt].angle = angle;
  data[logcnt].velocity = v;
  data[logcnt].omega = omega;
  data[logcnt].red = 0.5;
  data[logcnt].green = 0.5;
  data[logcnt].blue = 0.5;

  output_to_img(logcnt, data);

  // Robot State Update
  robot->t = robot->t + t;
  robot->x = robot->x + x;
  robot->y = robot->y + y;
  robot->angle = angle;
  robot->velocity = v;
  robot->omega = omega;
  robot->dir = dir;
  // printf("*** %f,%f,%f,%d",robot.x,robot.y,robot.angle*180/M_PI, robot.dir);
  printf("End  :%f %f %f %d\n", robot->x, robot->y, 180 * robot->angle / M_PI, robot->dir);
}
#endif

void draw_v_wall(int x, int y)
{
  cairo_set_source_rgb(C_maze, 0.7, 0, 0);
  cairo_rectangle(C_maze, x, y, 12, 180 - 16);
  cairo_fill(C_maze);
}

//水平の壁を描画
void draw_h_wall(int x, int y)
{
  cairo_set_source_rgb(C_maze, 0.7, 0, 0);
  cairo_rectangle(C_maze, x, y, 180 - 16, 12);
  cairo_fill(C_maze);
}

void draw_traj(int n, data_t *data)
{
  cairo_set_source_rgb(C_robot, data[0].red, data[0].green, data[0].blue);
  cairo_set_line_width(C_robot, 1);
  cairo_move_to(C_robot, (data[0].x + 6), (HEIGHT - data[0].y - 6));
  for (int i = 1; i < n; i++)
  {
    cairo_line_to(C_robot, (data[i].x + 6), (HEIGHT - data[i].y - 6));
  }
  cairo_stroke(C_robot);
}

void erase_traj(int n, data_t *data)
{
  cairo_set_source_rgb(C_robot, 0, 0, 0);
  cairo_set_line_width(C_robot, 3);
  cairo_move_to(C_robot, (data[0].x + 6), (HEIGHT - data[0].y - 6));
  for (int i = 1; i < n; i++)
  {
    cairo_line_to(C_robot, (data[i].x + 6), (HEIGHT - data[i].y - 6));
  }
  cairo_stroke(C_robot);
}

void draw_mouse(double x, double y, double angle)
{
  double x1 = -35.0;
  double y1 = -35.0;
  double x2 = 35.0;
  double y2 = -35.0;
  double x3 = 35.0;
  double y3 = 35.0;
  double x4 = 0.0;
  double y4 = 55.0;
  double x5 = -35.0;
  double y5 = 35.0;

  angle = angle - M_PI / 2;
  double x1_ = x1 * cos(angle) - y1 * sin(angle) + x;
  double y1_ = x1 * sin(angle) + y1 * cos(angle) + y;
  double x2_ = x2 * cos(angle) - y2 * sin(angle) + x;
  double y2_ = x2 * sin(angle) + y2 * cos(angle) + y;
  double x3_ = x3 * cos(angle) - y3 * sin(angle) + x;
  double y3_ = x3 * sin(angle) + y3 * cos(angle) + y;
  double x4_ = x4 * cos(angle) - y4 * sin(angle) + x;
  double y4_ = x4 * sin(angle) + y4 * cos(angle) + y;
  double x5_ = x5 * cos(angle) - y5 * sin(angle) + x;
  double y5_ = x5 * sin(angle) + y5 * cos(angle) + y;

  // cairo_set_operator(C_robot, CAIRO_OPERATOR_OVER);
  cairo_set_source_rgb(C_robot, 0, 0.8, 0.1);
  cairo_set_line_width(C_robot, 1);
  cairo_move_to(C_robot, (x1_ + 6), (HEIGHT - y1_ - 6));
  cairo_line_to(C_robot, (x2_ + 6), (HEIGHT - y2_ - 6));
  cairo_line_to(C_robot, (x3_ + 6), (HEIGHT - y3_ - 6));
  cairo_line_to(C_robot, (x4_ + 6), (HEIGHT - y4_ - 6));
  cairo_line_to(C_robot, (x5_ + 6), (HEIGHT - y5_ - 6));
  cairo_close_path(C_robot);
  cairo_fill(C_robot);
  cairo_stroke(C_robot);
}

void erase_mouse(double x, double y, double angle)
{
  double x1 = -35.5;
  double y1 = -35.5;
  double x2 = 35.5;
  double y2 = -35.5;
  double x3 = 35.5;
  double y3 = 35.5;
  double x4 = 0.0;
  double y4 = 55.5;
  double x5 = -35.5;
  double y5 = 35.5;

  angle = angle - M_PI / 2;
  double x1_ = x1 * cos(angle) - y1 * sin(angle) + x;
  double y1_ = x1 * sin(angle) + y1 * cos(angle) + y;
  double x2_ = x2 * cos(angle) - y2 * sin(angle) + x;
  double y2_ = x2 * sin(angle) + y2 * cos(angle) + y;
  double x3_ = x3 * cos(angle) - y3 * sin(angle) + x;
  double y3_ = x3 * sin(angle) + y3 * cos(angle) + y;
  double x4_ = x4 * cos(angle) - y4 * sin(angle) + x;
  double y4_ = x4 * sin(angle) + y4 * cos(angle) + y;
  double x5_ = x5 * cos(angle) - y5 * sin(angle) + x;
  double y5_ = x5 * sin(angle) + y5 * cos(angle) + y;

  // cairo_set_operator(C_robot, CAIRO_OPERATOR_OVER);
  cairo_set_source_rgb(C_robot, 0, 0, 0);
  cairo_set_line_width(C_robot, 1);
  cairo_move_to(C_robot, (x1_ + 6), (HEIGHT - y1_ - 6));
  cairo_line_to(C_robot, (x2_ + 6), (HEIGHT - y2_ - 6));
  cairo_line_to(C_robot, (x3_ + 6), (HEIGHT - y3_ - 6));
  cairo_line_to(C_robot, (x4_ + 6), (HEIGHT - y4_ - 6));
  cairo_line_to(C_robot, (x5_ + 6), (HEIGHT - y5_ - 6));
  cairo_close_path(C_robot);
  cairo_fill(C_robot);
  cairo_stroke(C_robot);
}

void erase_mouse2(double x, double y, double angle)
{
  double x1 = -35.0;
  double y1 = -35.0;
  double x2 = 35.0;
  double y2 = -35.0;
  double x3 = 35.0;
  double y3 = 35.0;
  double x4 = 0.0;
  double y4 = 55.0;
  double x5 = -35.0;
  double y5 = 35.0;

  angle = angle - M_PI / 2;
  double x1_ = x1 * cos(angle) - y1 * sin(angle) + x;
  double y1_ = x1 * sin(angle) + y1 * cos(angle) + y;
  double x2_ = x2 * cos(angle) - y2 * sin(angle) + x;
  double y2_ = x2 * sin(angle) + y2 * cos(angle) + y;
  double x3_ = x3 * cos(angle) - y3 * sin(angle) + x;
  double y3_ = x3 * sin(angle) + y3 * cos(angle) + y;
  double x4_ = x4 * cos(angle) - y4 * sin(angle) + x;
  double y4_ = x4 * sin(angle) + y4 * cos(angle) + y;
  double x5_ = x5 * cos(angle) - y5 * sin(angle) + x;
  double y5_ = x5 * sin(angle) + y5 * cos(angle) + y;

  // cairo_set_operator(C_robot, CAIRO_OPERATOR_CLEAR);
  cairo_set_line_width(C_robot, 1);
  cairo_move_to(C_robot, (x1_ + 6), (HEIGHT - y1_ - 6));
  cairo_line_to(C_robot, (x2_ + 6), (HEIGHT - y2_ - 6));
  cairo_line_to(C_robot, (x3_ + 6), (HEIGHT - y3_ - 6));
  cairo_line_to(C_robot, (x4_ + 6), (HEIGHT - y4_ - 6));
  cairo_line_to(C_robot, (x5_ + 6), (HEIGHT - y5_ - 6));
  cairo_close_path(C_robot);
  cairo_set_source_rgb(C_robot, 0, 0, 0);
  cairo_fill_preserve(C_robot);
  cairo_set_source_rgb(C_robot, 0.2, 0.2, 0.2);
  cairo_stroke(C_robot);
}

void draw_maze(int width, int height)
{
  /* background */

  cairo_set_source_rgb(C_maze, 0, 0, 0);
  cairo_rectangle(C_maze, 0, 0, width, height);
  cairo_fill(C_maze);

  /* red rectangle */
  for (int y = 0; y < 16; y++)
  { //南北方向
    for (int x = 0; x < 16; x++)
    { //東西方向
      if (maze[2 * y][2 + 4 * x] == '-')
        draw_h_wall(x * 180 + 14, y * 180);
      if (maze[1 + 2 * y][4 * x] == '|')
        draw_v_wall(x * 180, y * 180 + 14);
      if (maze[2 + 2 * y][2 + 4 * x] == '-')
        draw_h_wall(x * 180 + 14, (y + 1) * 180);
      if (maze[1 + 2 * y][4 + 4 * x] == '|')
        draw_v_wall((x + 1) * 180, y * 180 + 14);
    }
  }

  for (int y = 0; y < (180 * 16 + 12); y += 180)
  {
    for (int x = 0; x < (180 * 16 + 12); x += 180)
    {
      cairo_set_source_rgb(C_maze, 0.7, 0, 0);
      cairo_rectangle(C_maze, x, y, 12, 12);
      cairo_fill(C_maze);
    }
  }
  // draw_mouse(c, 90+6, 180*15+90+6);
}

void make_mapdata(void)
{
  unsigned char x, y;
  unsigned char n_wall, e_wall, s_wall, w_wall;

  for (y = 0; y < 16; y++)
  { //南北方向
    for (x = 0; x < 16; x++)
    { //東西方向
      n_wall = 0x01 * (maze[30 - 2 * y][2 + 4 * x] == '-');
      e_wall = 0x02 * (maze[31 - 2 * y][4 + 4 * x] == '|');
      s_wall = 0x04 * (maze[32 - 2 * y][2 + 4 * x] == '-');
      w_wall = 0x08 * (maze[31 - 2 * y][4 * x] == '|');
      mapdata[x][y] = s_wall | w_wall | e_wall | n_wall;
    }
  }
  // printf("%02X\n",mapdata[1][0]);
}

// map配列をリセット
//迷路外周とスタート区画の壁情報は既知なので同時にセットする
void reset_map(void)
{
  mouse_x = 0;
  mouse_y = 0;
  mouse_dir = 0;

  for (short x = 0; x < MAZE_SIZE; x++)
  {
    for (short y = 0; y < MAZE_SIZE; y++)
    {
      smap[x][y] = 0;
      cmap[x][y] = 0xffff;
      map[x][y] = 0x00;
      if ((x == 0) && (y == 0))
        map[x][y] = 0xfe;
      if ((x == 1) && (y == 0))
        map[x][y] = 0xcc;
      if ((x == 0) && (y == 1))
        map[x][y] = 0xc8;

      if (y == (MAZE_SIZE - 1))
        map[x][y] |= 0x11;
      if (x == (MAZE_SIZE - 1))
        map[x][y] |= 0x22;
      if (y == 0)
        map[x][y] |= 0x44;
      if (x == 0)
        map[x][y] |= 0x88;
    }
  }
}

//向きを変更
//正:時計回り 負:反時計回り
void round_dir(char d)
{
  mouse_dir += d;
  mouse_dir &= 0x03;
}

//座標更新
// dir		現在の方角
// return値	-1:エラー 1:正常
char update_mouse_position(char dir)
{
  if (dir < 0 || dir > 3)
  {
    return -1;
  }

  const char dx[4] = {0, 1, 0, -1};
  const char dy[4] = {1, 0, -1, 0};

  char temp_x = mouse_x + dx[dir];
  char temp_y = mouse_y + dy[dir];

  if (temp_x < 0 || temp_y < 0 || temp_x >= MAZE_SIZE || temp_y >= MAZE_SIZE)
  {
    return -1;
  }
  else
  {
    mouse_x = temp_x;
    mouse_y = temp_y;
    return 1;
  }
}

//壁情報をmap配列から取得
// x x座標
// y y座標
// dir 向き 0:北 1:東 2:南 3:西
// return値 1:エラー 0:壁なし 1:壁あり 2:未探索
char get_map(char x, char y, char dir)
{
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE || dir < 0 || dir > 3)
  {
    return -1;
  }

  //未探索
  if (((map[x][y] >> dir) & 0x10) == 0)
  {
    return 2;
  }
  else
  {
    return ((map[x][y] >> dir) & 0x01);
  }
}

//壁情報をmap配列にセット
// x		x座標
// y		y座標
// dir	向き 0:北 1:東 2:南 3:西
// state	壁の有無 0:壁なし 1:壁あり
void set_map(char x, char y, char dir, char state)
{
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE || dir < 0 || dir > 3)
  {
    return;
  }

  if (state == 1)
  {
    switch (dir)
    {
    case 0:
      map[x][y] |= 0x11;
      if (y < MAZE_SIZE - 1)
      {
        map[x][y + 1] |= 0x44;
      }
      break;
    case 1:
      map[x][y] |= 0x22;
      if (x < MAZE_SIZE - 1)
      {
        map[x + 1][y] |= 0x88;
      }
      break;
    case 2:
      map[x][y] |= 0x44;
      if (y > 0)
      {
        map[x][y - 1] |= 0x11;
      }
      break;
    case 3:
      map[x][y] |= 0x88;
      if (x > 0)
      {
        map[x - 1][y] |= 0x22;
      }
      break;
    }
  }
  else if (state == 0)
  {
    switch (dir)
    {
    case 0:
      map[x][y] |= 0x10;
      map[x][y] &= 0xfe;
      if (y < MAZE_SIZE - 1)
      {
        map[x][y + 1] |= 0x40;
        map[x][y + 1] &= 0xfb;
      }
      break;
    case 1:
      map[x][y] |= 0x20;
      map[x][y] &= 0xfd;
      if (x < MAZE_SIZE - 1)
      {
        map[x + 1][y] |= 0x80;
        map[x + 1][y] &= 0xf7;
      }
      break;
    case 2:
      map[x][y] |= 0x40;
      map[x][y] &= 0xfb;
      if (y > 0)
      {
        map[x][y - 1] |= 0x10;
        map[x][y - 1] &= 0xfe;
      }
      break;
    case 3:
      map[x][y] |= 0x80;
      map[x][y] &= 0xf7;
      if (x > 0)
      {
        map[x - 1][y] |= 0x20;
        map[x - 1][y] &= 0xfd;
      }
      break;
    }
  }
}

// value1: velocity
// value2: angle

// Micromouse Equation of Motion
void eom(double *xdot, double *x, double t, double *u)
{
  double _x = x[0];
  double _y = x[1];
  double _angle = x[2];
  double _v = x[3];
  double _omega = x[4];

  double _xdot = _v * cos(_angle);
  double _ydot = _v * sin(_angle);
  double _angledot = _omega;
  double _vdot = u[0];
  double _omegadot = u[1];

  xdot[0] = _xdot;
  xdot[1] = _ydot;
  xdot[2] = _angledot;
  xdot[3] = _vdot;
  xdot[4] = _omegadot;

  return;
}

void rk4(void (*dxdt)(double *, double *, double, double *), double *x, double t, double *u, double h, int n)
{
  double *state;
  double *k1, *k2, *k3, *k4;

  state = (double *)malloc(sizeof(double) * n);
  k1 = (double *)malloc(sizeof(double) * n);
  k2 = (double *)malloc(sizeof(double) * n);
  k3 = (double *)malloc(sizeof(double) * n);
  k4 = (double *)malloc(sizeof(double) * n);

  dxdt(k1, x, t, u);
  for (int i = 0; i < n; i++)
    state[i] = x[i] + 0.5 * h * k1[i];
  dxdt(k2, state, t, u);
  for (int i = 0; i < n; i++)
    state[i] = x[i] + 0.5 * h * k2[i];
  dxdt(k3, state, t, u);
  for (int i = 0; i < n; i++)
    state[i] = x[i] + h * k3[i];
  dxdt(k4, state, t, u);

  for (int i = 0; i < n; i++)
    x[i] = x[i] + h * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6;

  free(state);
  free(k1);
  free(k2);
  free(k3);
  free(k4);
  return;
}

#if 0
//引数どう的に可変サンプル
double rk4(void (*dxdt)(double, double, double*), double x, double t, double h, int n, ...)
{
  va_list args;
  double *value;
  double k1,k2,k3,k4;

  value=(double*)malloc(sizeof(double) * n);
  va_start(args , n);
  for(int i=0;i<n;i++)
  {
    value[i]=va_arg(args, double);
  }
  va_end(args);
  
  k1 = h * dxdt(x, t, value);
  k2 = h * dxdt(x+0.5*k1, t+0.5*h, value);
  k3 = h * dxdt(x+0.5*k2, t+0.5*h, value);
  k4 = h * dxdt(x+k3, t+h, value);

  free(value);
  
  return x+(k1 + k2*2.0 + k3*2.0 + k4)/6.0;
}
#endif

void one_step(int dir, double x, double y, double angle, double velocity, double framerate)
{
  double fin_time = SECTION / velocity;
  int frame = (int)(fin_time * framerate);
  char str[100];
  double x0 = x;
  double y0 = y;

  if (dir == 0)
  {
    while (y < (y0 + SECTION))
    {
      sprintf(str, "%s/test%05d.png", folder, Index);
      draw_mouse(x, y, angle);
      cairo_surface_write_to_png(CS, str);
      erase_mouse(x, y, angle);
      y = y + velocity / framerate;
      Index++;
    }
    sprintf(str, "%s/test%05d.png", folder, Index);
    draw_mouse(x, y0 + SECTION, angle);
    cairo_surface_write_to_png(CS, str);
    erase_mouse(x, y0 + SECTION, angle);
  }
  else if (dir == 1)
  {
    while (x < (x0 + SECTION))
    {
      sprintf(str, "%s/test%05d.png", folder, Index);
      draw_mouse(x, y, angle);
      cairo_surface_write_to_png(CS, str);
      erase_mouse(x, y, angle);
      x = x + velocity / framerate;
      Index++;
    }
    sprintf(str, "%s/test%05d.png", folder, Index);
    draw_mouse(x0 + SECTION, y, angle);
    cairo_surface_write_to_png(CS, str);
    erase_mouse(x0 + SECTION, y, angle);
  }
  else if (dir == 2)
  {
    while (y > (y0 - SECTION))
    {
      sprintf(str, "%s/test%05d.png", folder, Index);
      draw_mouse(x, y, angle);
      cairo_surface_write_to_png(CS, str);
      erase_mouse(x, y, angle);
      y = y - velocity / framerate;
      Index++;
    }
    sprintf(str, "%s/test%05d.png", folder, Index);
    draw_mouse(x, y0 - SECTION, angle);
    cairo_surface_write_to_png(CS, str);
    erase_mouse(x, y0 - SECTION, angle);
  }
  else if (dir == 3)
  {
    while (x > (x0 - SECTION))
    {
      sprintf(str, "%s/test%05d.png", folder, Index);
      draw_mouse(x, y, angle);
      cairo_surface_write_to_png(CS, str);
      erase_mouse(x, y, angle);
      x = x - velocity / framerate;
      Index++;
    }
    sprintf(str, "%s/test%05d.png", folder, Index);
    draw_mouse(x0 - SECTION, y, angle);
    cairo_surface_write_to_png(CS, str);
    erase_mouse(x0 - SECTION, y, angle);
  }
}

void r_turn(double x, double y, double angle, double velocity, double framerate)
{
  double fin_time = (M_PI / 2) / velocity;
  int frame = (int)(fin_time * framerate);
  char str[100];
  double angle0 = angle;

  while (angle < (angle0 + M_PI / 2))
  {
    sprintf(str, "%s/test%05d.png", folder, Index);
    draw_mouse(x, y, angle);
    cairo_surface_write_to_png(CS, str);
    erase_mouse(x, y, angle);
    angle = angle + velocity / framerate;
    Index++;
  }
  sprintf(str, "%s/test%05d.png", folder, Index);
  draw_mouse(x, y, angle0 + (M_PI / 2));
  cairo_surface_write_to_png(CS, str);
  erase_mouse(x, y, angle0 + (M_PI / 2));
}

void l_turn(double x, double y, double angle, double velocity, double framerate)
{
  double fin_time = (M_PI / 2) / velocity;
  int frame = (int)(fin_time * framerate);
  char str[100];
  double angle0 = angle;

  while (angle > (angle0 - M_PI / 2))
  {
    sprintf(str, "%s/test%05d.png", folder, Index);
    draw_mouse(x, y, angle);
    cairo_surface_write_to_png(CS, str);
    sprintf(str, "%s/test%05d.png", folder, Index);
    erase_mouse(x, y, angle);
    angle = angle - velocity / framerate;
    Index++;
  }
  sprintf(str, "%s/test%05d.png", folder, Index);
  draw_mouse(x, y, angle0 - (M_PI / 2));
  cairo_surface_write_to_png(CS, str);
  erase_mouse(x, y, angle0 - (M_PI / 2));
}

void straight(void)
{
  one_step(mouse_dir, Mx, My, Mangle, Mv, Framerate);
  if (mouse_dir == 0)
  {
    My = My + (double)SECTION;
  }
  else if (mouse_dir == 1)
  {
    Mx = Mx + (double)SECTION;
  }
  else if (mouse_dir == 2)
  {
    My = My - (double)SECTION;
  }
  else if (mouse_dir == 3)
  {
    Mx = Mx - (double)SECTION;
  }
}

void right_turn(void)
{
  r_turn(Mx, My, Mangle, Momega, Framerate);
  Mangle = Mangle + M_PI / 2;
  Mangle = fmod(Mangle, 2 * M_PI);
}

void left_turn(void)
{
  l_turn(Mx, My, Mangle, Momega, Framerate);
  Mangle = Mangle - M_PI / 2;
  Mangle = fmod(Mangle, 2 * M_PI);
}

void turn180(void)
{
  right_turn();
  right_turn();
}

//拡張左手法
void mode0(void)
{
  make_mapdata();
  reset_map();

  // Start
  start_phy(&Micromouse);

  while (1)
  {
    smap[mouse_x][mouse_y] = 1;
    //座標更新（柱と柱のあいだで更新）
    Micromouse.dir = mouse_dir;
    update_mouse_position(mouse_dir);
    //マップ更新
    if (smap[mouse_x][mouse_y] == 0)
    {
      //正面
      if (get_map(mouse_x, mouse_y, mouse_dir) == 2)
      {
        set_map(mouse_x, mouse_y, mouse_dir, (mapdata[mouse_x][mouse_y] >> mouse_dir) & 0x01);
      }
      else
      { //仮想壁挿入
        set_map(mouse_x, mouse_y, mouse_dir, 1);
      }

      //右
      if (get_map(mouse_x, mouse_y, (mouse_dir + 1) & 3) == 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir + 1) & 3, (mapdata[mouse_x][mouse_y] >> ((mouse_dir + 1) & 3)) & 0x01);
      }
      else
      { //仮想壁挿入
        set_map(mouse_x, mouse_y, (mouse_dir + 1) & 3, 1);
      }

      //左
      if (get_map(mouse_x, mouse_y, (mouse_dir - 1) & 3) == 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir - 1) & 3, (mapdata[mouse_x][mouse_y] >> ((mouse_dir - 1) & 3)) & 0x01);
      }
      else
      { //仮想壁挿入
        set_map(mouse_x, mouse_y, (mouse_dir - 1) & 3, 1);
      }
    }
    else
    {
      //正面
      if (get_map(mouse_x, mouse_y, mouse_dir) == 2)
      {
        set_map(mouse_x, mouse_y, mouse_dir, (mapdata[mouse_x][mouse_y] >> mouse_dir) & 0x01);
      }
      //右
      if (get_map(mouse_x, mouse_y, (mouse_dir + 1) & 3) == 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir + 1) & 3, (mapdata[mouse_x][mouse_y] >> ((mouse_dir + 1) & 3)) & 0x01);
      }
      //左
      if (get_map(mouse_x, mouse_y, (mouse_dir - 1) & 3) == 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir - 1) & 3, (mapdata[mouse_x][mouse_y] >> ((mouse_dir - 1) & 3)) & 0x01);
      }
    }

    printf("(%7.3f, %2d, %2d, %02X, %02X)\n",
           (double)Index / Framerate, mouse_x, mouse_y, mapdata[mouse_x][mouse_y], map[mouse_x][mouse_y]);
    fflush(stdout);
    //スタートに戻ったら終了
    if (mouse_x == 0 && mouse_y == 0)
      break;

    //行動
    if (get_map(mouse_x, mouse_y, (mouse_dir - 1) & 3) == 0)
    { //左壁なし
      mouse_dir = (mouse_dir - 1) & 0x03;
      slalom_phy(&Micromouse, CCW);
    }
    else if (get_map(mouse_x, mouse_y, mouse_dir) == 0)
    { //前壁なし
      straight_phy(&Micromouse);
    }
    else if (get_map(mouse_x, mouse_y, (mouse_dir + 1) & 3) == 0)
    { //右壁なし
      mouse_dir = (mouse_dir + 1) & 0x03;
      slalom_phy(&Micromouse, CW);
    }
    else
    { //袋小路
      mouse_dir = (mouse_dir + 2) & 0x03;
      turn180_phy(&Micromouse);
      // straight_phy(&Micromouse);
    }
  }
  stop_phy(&Micromouse);
  mouse_dir = (mouse_dir + 2) & 0x3;
  Micromouse.dir = mouse_dir;
}

//最短
void mode1(void)
{
  u_int8_t current_step;
  u_int8_t north_step;
  u_int8_t east_step;
  u_int8_t south_step;
  u_int8_t west_step;
  int8_t com;

  mouse_x = 0;
  mouse_y = 0;
  mouse_dir = 0;

  make_contourmap(map, START_X, START_Y, GOAL_X, GOAL_Y);
  // copy_mapdata(mapdata);

  // Start
  start_phy(&Micromouse);

  while (1)
  {
    //座標更新
    Micromouse.dir = mouse_dir;
    update_mouse_position(mouse_dir);

    //スタートに戻ったら終了
    if (mouse_x == GOAL_X && mouse_y == GOAL_Y)
      break;

    //行動
    current_step = cmap[mouse_x][mouse_y];
    north_step = cmap[mouse_x][mouse_y + 1];
    east_step = cmap[mouse_x + 1][mouse_y];
    south_step = cmap[mouse_x][mouse_y - 1];
    west_step = cmap[mouse_x - 1][mouse_y];

    // North
    if (!(map[mouse_x][mouse_y] & 0x01) && (north_step - current_step) == -1)
    {
      if (mouse_dir == 0)
        com = 0;
      else if (mouse_dir == 3)
        com = 1;
      else if (mouse_dir == 1)
        com = 3;
    }
    // East
    else if (!((map[mouse_x][mouse_y] & 0x02) >> 1) && (east_step - current_step) == -1)
    {
      if (mouse_dir == 1)
        com = 0;
      else if (mouse_dir == 2)
        com = 3;
      else if (mouse_dir == 0)
        com = 1;
    }
    // South
    else if (!((map[mouse_x][mouse_y] & 0x04) >> 2) && (south_step - current_step) == -1)
    {
      if (mouse_dir == 2)
        com = 0;
      else if (mouse_dir == 1)
        com = 1;
      else if (mouse_dir == 3)
        com = 3;
    }
    // West
    else if (!((map[mouse_x][mouse_y] & 0x08) >> 3) && (west_step - current_step) == -1)
    {
      if (mouse_dir == 3)
        com = 0;
      else if (mouse_dir == 2)
        com = 1;
      else if (mouse_dir == 0)
        com = 3;
    }
    printf("(%d, %d) map=%02X dir=%d com=%d\n", mouse_x, mouse_y, map[mouse_x][mouse_y], mouse_dir, com);

    switch (com)
    {
    case 0:
    {
      straight_phy(&Micromouse);
      break;
    }
    case 1:
    {
      mouse_dir = (mouse_dir + 1) & 0x03;
      slalom_phy(&Micromouse, CW);
      break;
    }
    case 3:
    {
      mouse_dir = (mouse_dir - 1) & 0x03;
      slalom_phy(&Micromouse, CCW);
      break;
    }
    }
  }
  stop_phy(&Micromouse);
  mouse_dir = (mouse_dir + 2) & 0x3;
  Micromouse.dir = mouse_dir;
}

//足立法
void mode2(void)
{
  u_int8_t current_step;
  u_int8_t north_step;
  u_int8_t east_step;
  u_int8_t south_step;
  u_int8_t west_step;
  int8_t com;

  mouse_x = 0;
  mouse_y = 0;
  mouse_dir = 0;

  make_mapdata();
  reset_map();

  // Start
  start_phy(&Micromouse);

  while (1)
  {
    //既探索区画をマーク
    smap[mouse_x][mouse_y] = 1;

    //座標更新（柱と柱のあいだで更新）
    Micromouse.dir = mouse_dir;
    update_mouse_position(mouse_dir);
    //マップ更新
    if (smap[mouse_x][mouse_y] == 0) //はじめて訪れる区画の場合
    {
      //正面
      if (get_map(mouse_x, mouse_y, mouse_dir) == 2)
      {
        set_map(mouse_x, mouse_y, mouse_dir, (mapdata[mouse_x][mouse_y] >> mouse_dir) & 0x01);
      }

      //右
      if (get_map(mouse_x, mouse_y, (mouse_dir + 1) & 3) == 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir + 1) & 3, (mapdata[mouse_x][mouse_y] >> ((mouse_dir + 1) & 3)) & 0x01);
      }

      //左
      if (get_map(mouse_x, mouse_y, (mouse_dir - 1) & 3) == 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir - 1) & 3, (mapdata[mouse_x][mouse_y] >> ((mouse_dir - 1) & 3)) & 0x01);
      }
    }
    else // 2回目以降の場合（拡張左手法の名残り）
    {
      //正面
      if (get_map(mouse_x, mouse_y, mouse_dir) == 2)
      {
        set_map(mouse_x, mouse_y, mouse_dir, (mapdata[mouse_x][mouse_y] >> mouse_dir) & 0x01);
      }
      //右
      if (get_map(mouse_x, mouse_y, (mouse_dir + 1) & 3) == 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir + 1) & 3, (mapdata[mouse_x][mouse_y] >> ((mouse_dir + 1) & 3)) & 0x01);
      }
      //左
      if (get_map(mouse_x, mouse_y, (mouse_dir - 1) & 3) == 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir - 1) & 3, (mapdata[mouse_x][mouse_y] >> ((mouse_dir - 1) & 3)) & 0x01);
      }
    }

    //現時点の地図情報で等高線マップを作成
    reset_contourmap();
    make_contourmap2(map, mouse_x, mouse_y, GOAL_X, GOAL_Y);

    printf("(%7.3f, %2d, %2d, %02X, %02X)\n",
           (double)Index / Framerate, mouse_x, mouse_y, mapdata[mouse_x][mouse_y], map[mouse_x][mouse_y]);
    fflush(stdout);
    //ゴールしたら終了
    if (mouse_x == GOAL_X && mouse_y == GOAL_Y)
      break;

    //行動
    current_step = cmap[mouse_x][mouse_y];
    north_step = cmap[mouse_x][mouse_y + 1];
    east_step = cmap[mouse_x + 1][mouse_y];
    south_step = cmap[mouse_x][mouse_y - 1];
    west_step = cmap[mouse_x - 1][mouse_y];

    // North
    if (!(map[mouse_x][mouse_y] & 0x01) && (north_step - current_step) == -1)
    {
      if (mouse_dir == 0)
        com = 0;
      else if (mouse_dir == 3)
        com = 1;
      else if (mouse_dir == 2)
        com = 2;
      else if (mouse_dir == 1)
        com = 3;
    }
    // East
    else if (!((map[mouse_x][mouse_y] & 0x02) >> 1) && (east_step - current_step) == -1)
    {
      if (mouse_dir == 1)
        com = 0;
      else if (mouse_dir == 0)
        com = 1;
      else if (mouse_dir == 3)
        com = 2;
      else if (mouse_dir == 2)
        com = 3;
    }
    // South
    else if (!((map[mouse_x][mouse_y] & 0x04) >> 2) && (south_step - current_step) == -1)
    {
      if (mouse_dir == 2)
        com = 0;
      else if (mouse_dir == 1)
        com = 1;
      else if (mouse_dir == 0)
        com = 2;
      else if (mouse_dir == 3)
        com = 3;
    }
    // West
    else if (!((map[mouse_x][mouse_y] & 0x08) >> 3) && (west_step - current_step) == -1)
    {
      if (mouse_dir == 3)
        com = 0;
      else if (mouse_dir == 2)
        com = 1;
      else if (mouse_dir == 1)
        com = 2;
      else if (mouse_dir == 0)
        com = 3;
    }
    printf("(%d, %d) map=%02X dir=%d com=%d\n", mouse_x, mouse_y, map[mouse_x][mouse_y], mouse_dir, com);

    switch (com)
    {
    case 0: //直進
    {
      straight_phy(&Micromouse);
      break;
    }
    case 1: //右旋回
    {
      mouse_dir = (mouse_dir + 1) & 0x03;
      slalom_phy(&Micromouse, CW);
      break;
    }
    case 2: // 180度ターン
    {
      mouse_dir = (mouse_dir + 2) & 0x03;
      turn180_phy(&Micromouse);
      break;
    }
    case 3: //左旋回
    {
      mouse_dir = (mouse_dir - 1) & 0x03;
      slalom_phy(&Micromouse, CCW);
      break;
    }
    }
  }
  stop_phy(&Micromouse);
  mouse_dir = (mouse_dir + 2) & 0x3;
  Micromouse.dir = mouse_dir;
}

//直線の交点を求める
//a1:直線1の傾き
//b1:直線1の切片
//a2:直線1の傾き
//b2:直線1の切片

point2d_t find_cross_point(double a1, double b1, double a2, double b2)
{
  //
  //y=ax+b
  //a1x-y=-b1
  //a2x-y=-b2
  // [a1 -1][x]=[-b1]
  // [a2 -1][y]=[-b2]
  // [x]=    1   [  -1  1][-b1]
  // [y]= -a1+a2 [ -a2 a1][-b2]

  point2d_t pt;
  double det = -a1 + a2;
 
  pt.x=(b1-b2)/det;
  pt.y=(a2*b1 - a1*b2)/det;

  return pt;
}


int main(int argc, char **argv)
{
  /*
  reset_map();
  make_mapdata();
  // make_contourmap(mapdata);
  run();
  */
  double a1=tan(90*M_PI/180);
  double b1=a1*(-100.0);
  double a2=tan(45*M_PI/180);
  double b2=0.0;
  point2d_t pt;

  pt=find_cross_point(a1, b1, a2, b2);
  printf("%f %f　%f\n", pt.x, pt.y, a1);





}

void run(void)
{
  // int width = 392, height = 392;
  CS = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, WIDTH, HEIGHT);
  C_maze = cairo_create(CS);
  C_robot = cairo_create(CS);

  Micromouse.x = 90.0;
  Micromouse.y = 90.0;
  Micromouse.angle = 90.0 * M_PI / 180;
  Micromouse.velocity = VELOCITY;
  Micromouse.omega = 0.0;
  Micromouse.dir = 0;
  Micromouse.turn_velocity = TURN_VELOCITY;
  Micromouse.angle1 = ANGLE1;

  // slalom_opt();

  // Start
  make_mapdata();
  draw_maze(WIDTH, HEIGHT);

  // mode0();
  // mode1();

  mode2();

  // Finsih

  cairo_destroy(C_maze);
  cairo_destroy(C_robot);
  cairo_surface_destroy(CS);
}

void slalom_opt(void)
{
  double xerr, yerr;
  double err;
  double olderr;
  double oldt;
  double h = 0.01;
  double vt;
  double nextvt;
  int counter = 0;
  int flag = 0;

  Micromouse.x = 90.0;
  Micromouse.y = 180.0;
  Micromouse.angle = 90.0 * M_PI / 180;
  Micromouse.velocity = VELOCITY;
  Micromouse.omega = 0.0;
  Micromouse.dir = 0;
  // Micromouse.turn_velocity = 150.0;
  Micromouse.angle1 = 1.0 * M_PI / 180;
  err = 1000.0;
  olderr = err + 1.0;
  nextvt = 70.0;

  while (Micromouse.angle1 < 46.0 * M_PI / 180.0) //角度を大きくしていくループ
  {
    h = 10.0;
    vt = nextvt;
    while (h > 0.001)
    { //刻み幅を小さくするループ
      counter = 0;
      while (1) //速度を小さくしていくループ
      {
        flag = 0;
        oldt = Micromouse.t;
        Micromouse.t = 0.0;
        Micromouse.x = 90.0;
        Micromouse.y = 180.0;
        Micromouse.angle = 90.0 * M_PI / 180;
        Micromouse.velocity = VELOCITY;
        Micromouse.omega = 0.0;
        Micromouse.dir = 0;
        Micromouse.turn_velocity = vt;

        olderr = err;
        slalom_phy(&Micromouse, CW);

        xerr = Micromouse.x - 90.0 - 90.0;
        yerr = Micromouse.y - 180.0 - 90.0;
        err = sqrt(xerr * xerr + yerr * yerr);
        // printf("%d %f %f %13.8f %13.8f %f\n",
        //   counter, Micromouse.turn_velocity, Micromouse.angle1*180/M_PI, err, (err-olderr), Micromouse.t );

        if (counter > 0)
        {
          if (err - olderr > 0.0)
            break;
        }
        counter++;
        if (counter > (int)(VELOCITY / h) - 1)
        {
          flag = 1;
          break;
        }
        vt = vt - h;
      } //速度を小さくしていくループ
      if (flag == 1)
      {
        flag = 0;
        break;
      }
      vt = Micromouse.turn_velocity + 3 * h;
      if (h == 10.0)
        nextvt = vt;
      h = h / 10.0;
    } //刻み幅を小さくするループ

    printf("%f %f %f %f\n", Micromouse.angle1, Micromouse.turn_velocity + 2 * h, olderr, oldt);
    Micromouse.angle1 = Micromouse.angle1 + 1.0 * M_PI / 180.0;
  } //角度を大きくしていくループ

  // calc_trajectry();
  // start_phy(&Micromouse);
  // slalom_phy(&Micromouse,  CW);
  // straight_phy(&Micromouse);
  // turn180_phy(&Micromouse);
}

#if 0
//2017 Clasic mouse expart final maze
char maze[33][66]={
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",//    0
    "|   |                                                           |",//15  1
    "+   +   +   +---+---+---+---+---+---+---+---+---+---+   +   +---+",//    2
    "|       |   |               |               |       |   |       |",//14  3
    "+   +---+   +   +---+---+   +   +---+---+   +   +   +---+---+   +",//    4
    "|           |   |       |       |       |       |           |   |",//13  5
    "+   +---+   +   +   +   +---+---+   +   +---+---+---+---+   +   +",//    6
    "|           |       |               |               |       |   |",//12  7
    "+   +---+---+---+---+---+---+---+---+---+---+---+   +   +---+   +",//    8
    "|   |       |       |           |       |   |       |       |   |",//11  9
    "+   +   +   +   +   +   +   +   +   +   +   +   +---+---+   +   +",//   10
    "|   |   |       |       |   |       |           |           |   |",//10 11
    "+   +   +---+---+---+---+---+---+---+---+---+---+---+   +---+   +",//   12
    "|   |       |           |               |       |           |   |",// 9 13
    "+   +---+   +   +   +---+   +   +---+   +   +   +---+   +---+   +",//   14
    "|   |       |   |       |   |       |       |               |   |",// 8 15
    "+   +   +---+   +---+   +   +   +   +---+---+---+---+   +---+   +",//   16
    "|   |   |       |       |   |       |   |                   |   |",// 7 17
    "+   +   +   +---+   +---+   +---+---+   +   +   +---+---+---+   +",//   18
    "|   |       |           |                   |               |   |",// 6 19
    "+   +---+---+   +---+---+---+---+---+---+---+---+---+---+   +   +",//   20
    "|   |           |       |   |       |   |           |       |   |",// 5 21
    "+   +   +---+---+   +   +   +   +   +   +   +   +---+   +---+   +",//   22
    "|   |       |       |           |           |   |   |       |   |",// 4 23
    "+   +---+   +   +---+---+   +---+---+   +---+   +   +---+   +   +",//   24
    "|   |       |       |   |   |       |   |       |           |   |",// 3 25
    "+   +   +---+---+   +   +---+   +   +---+   +---+   +---+---+   +",//   26
    "|   |   |           |           |           |   |   |           |",// 2 27
    "+   +   +   +---+---+   +---+---+---+---+---+   +   +   +   +   +",//   28
    "|       |   |   |   |                               |   |   |   |",// 1 29
    "+   +---+   +   +   +---+---+---+---+---+---+---+---+---+---+   +",//   30
    "|   |                                                           |",// 0 31
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"//    32
    // 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
};

//2019+alpha Clasic mouse expart final maze
char maze[33][66]={
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",//    0
    "|                                                               |",//15  1
    "+   +---+---+---+---+---+---+---+---+---+---+---+---+---+   +---+",//    2
    "|                               |       |       |   |           |",//14  3
    "+---+---+---+---+---+---+---+   +   +   +   +   +   +   +   +   +",//    4
    "|               |       |       |   |       |           |   |   |",//13  5
    "+   +---+---+   +   +   +   +   +   +---+   +---+   +---+---+   +",//    6
    "|   |       |       |       |   |       |               |       |",//12  7
    "+   +   +   +---+---+---+---+   +---+   +---+---+---+---+   +---+",//    8
    "|   |   |   |   |   |   |       |       |       |       |       |",//11  9
    "+   +   +   +   +   +   +   +---+   +---+   +   +   +   +---+   +",//   10
    "|   |   |                   |       |       |       |       |   |",//10 11
    "+   +   +---+   +   +   +---+   +---+   +---+---+---+---+   +   +",//   12
    "|   |       |   |   |   |       |           |       |           |",// 9 13
    "+   +---+   +---+---+---+   +---+---+   +---+   +   +   +---+---+",//   14
    "|                   |                   |       |   |           |",// 8 15
    "+---+   +---+   +---+---+   +   +   +---+   +   +   +---+---+   +",//   16
    "|                       |   |       |       |   |               |",// 7 17
    "+   +---+   +---+   +   +   +---+---+   +---+---+---+---+---+---+",//   18
    "|                   |   |   |   |   |                           |",// 6 19
    "+---+   +---+   +   +   +---+   +   +---+---+---+---+---+---+   +",//   20
    "|   |           |           |                   |       |       |",// 5 21
    "+   +---+   +   +   +---+   +   +---+   +   +   +   +   +   +---+",//   22
    "|   |       |       |       |   |       |   |       |           |",// 4 23
    "+   +   +   +   +---+   +---+   +   +---+   +---+---+---+---+---+",//   24
    "|       |       |       |       |   |           |       |       |",// 3 25
    "+   +   +   +---+   +---+   +---+   +---+   +   +   +   +   +   +",//   26
    "|   |       |       |       |       |       |       |       |   |",// 2 27
    "+   +   +---+   +---+   +---+   +   +   +   +   +   +   +   +   +",//   28
    "|           |                   |       |       |       |       |",// 1 29
    "+   +   +   +---+---+---+---+---+---+---+---+---+---+---+---+   +",//   30
    "|   |   |                                                       |",// 0 31
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"//    32
    // 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
};

//kantan maze for debug
char maze[33][66]={
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",//    0
    "|                                                               |",//15  1
    "+   +---+---+---+---+---+---+---+---+---+---+---+---+---+   +---+",//    2
    "|                               |       |       |   |           |",//14  3
    "+---+---+---+---+---+---+---+   +   +   +   +   +   +   +   +   +",//    4
    "|               |       |       |   |       |           |   |   |",//13  5
    "+   +---+---+   +   +   +   +   +   +---+   +---+   +---+---+   +",//    6
    "|   |       |       |       |   |       |               |       |",//12  7
    "+   +   +   +---+---+---+---+   +---+   +---+---+---+---+   +---+",//    8
    "|   |   |   |   |   |   |       |       |       |       |       |",//11  9
    "+   +   +   +   +   +   +   +---+   +---+   +   +   +   +---+   +",//   10
    "|   |   |                   |       |       |       |       |   |",//10 11
    "+   +   +---+   +   +   +---+   +---+   +---+---+---+---+   +   +",//   12
    "|   |       |   |   |   |       |           |       |           |",// 9 13
    "+   +---+   +---+---+---+   +---+---+   +---+   +   +   +---+---+",//   14
    "|                   |                   |       |   |           |",// 8 15
    "+---+   +---+   +---+---+   +   +   +---+   +   +   +---+---+   +",//   16
    "|                       |   |       |       |   |               |",// 7 17
    "+   +---+   +---+   +   +   +---+---+   +---+---+---+---+---+---+",//   18
    "|                   |   |   |   |   |                           |",// 6 19
    "+---+   +---+   +   +   +---+   +   +---+---+---+---+---+---+   +",//   20
    "|   |           |           |                   |       |       |",// 5 21
    "+---+---+---+---+---+---+   +   +---+   +   +   +   +   +   +---+",//   22
    "|   |   |   |   |   |       |   |       |   |       |           |",// 4 23
    "+---+---+---+---+---+   +---+   +   +---+   +---+---+---+---+---+",//   24
    "|               |   |   |       |   |           |       |       |",// 3 25
    "+   +   +   +   +---+---+   +---+   +---+   +   +   +   +   +   +",//   26
    "|   |   |   |   |   |       |       |       |       |       |   |",// 2 27
    "+---+   +---+   +---+   +---+   +   +   +   +   +   +   +   +   +",//   28
    "|       |       |   |           |       |       |       |       |",// 1 29
    "+   +   +   +---+---+---+---+---+---+---+---+---+---+---+---+   +",//   30
    "|   |           |   |                                           |",// 0 31
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"//    32
    // 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
};



//ffmpeg -framerate 30 -i imgx/test%05d.png -vf scale=1200:-1 -vcodec libx264 -pix_fmt yuv420p -r 30 ./left_long_step_state_save1200.mp4
#endif
