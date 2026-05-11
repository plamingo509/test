#ifndef _PROJECTS_PROJECT1_AW_MANAGER_H__
#define _PROJECTS_PROJECT1_AW_MANAGER_H__

#include <stdio.h>

#include "threads/thread.h"
#include "projects/automated_warehouse/robot.h"
#include "projects/automated_warehouse/aw_thread.h"

#define ROW_A 1
#define COL_A 0
#define ROW_B 3
#define COL_B 0
#define ROW_C 5
#define COL_C 0
#define ROW_S 5
#define COL_S 5
#define ROW_W 6
#define COL_W 5

#define MAP_WIDTH 7
#define MAP_HEIGHT 7

extern unsigned int step;
extern const char thread_status[4][10];
extern const char map_draw_default[MAP_HEIGHT][MAP_WIDTH];

void init_automated_warehouse(char** argv);

void handle_parse_completion(tid_t _cnt_thread, tid_t* _robot_threads, int _number_of_robots);

void print_map(struct robot* __robots, int __number_of_robots);

void increase_step(void);

#endif