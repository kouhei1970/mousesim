#include "ray.h"

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
