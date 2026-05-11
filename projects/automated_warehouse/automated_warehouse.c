#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"

struct robot* robots;
char* destinations;
int robot_count;
int current_robot = 0;
int simulation_done = 0;

int* completed;

int payload_rows[8] = {-1, 2, 3, 4, 2, 3, 4, 1};
int payload_cols[8] = {-1, 2, 2, 2, 4, 4, 4, 2};

//하역장 위치
void get_destination_position(char dest, int* row, int* col) {
        if (dest == 'A') {
                *row = 1;
                *col = 0;
        } else if (dest == 'B') {
                *row = 3;
                *col = 0;
        } else if (dest == 'C') {
                *row = 5;
                *col = 0;
        }
}

//이동 명령 계산 함수
int get_move_command(int current_row, int current_col, int target_row, int target_col) {
        if (current_row > target_row) {
                return 1; // up
        } else if (current_row < target_row) {
                return 2; // down
        } else if (current_col > target_col) {
                return 3; // left
        } else if (current_col < target_col) {
                return 4; // right
        }

        return 0; // wait
}

// test code for central control node thread
void central_control_thread(void* aux){
        while (1) {
                int i;
                int all_received = 0;

                while (all_received == 0) {
                        all_received = 1;

                        for (i = 0; i < robot_count; i++) {
                                if (boxes_from_robots[i].dirtyBit == 0) {
                                        all_received = 0;
                                }
                        }
                }

                print_map(robots, robot_count);

                for (i = 0; i < robot_count; i++) {
                        boxes_from_robots[i].dirtyBit = 0;

                        if (i == current_robot && completed[i] == 0) {
                                int target_row;
                                int target_col;

                                if (robots[i].current_payload == 0) {
                                        int payload = robots[i].required_payload;

                                        target_row = payload_rows[payload];
                                        target_col = payload_cols[payload];
                                } else {
                                        get_destination_position(destinations[i], &target_row, &target_col);
                                }

                                boxes_from_central_control_node[i].msg.cmd =
                                        get_move_command(robots[i].row, robots[i].col, target_row, target_col);
                        } else {
                                boxes_from_central_control_node[i].msg.cmd = 0;
                        }
                        boxes_from_central_control_node[i].dirtyBit = 1;
                }

                increase_step();

                if (current_robot < robot_count && completed[current_robot] == 1) {
                        current_robot++;
                }

                if (current_robot >= robot_count) {
                        simulation_done = 1;

                        for (i = 0; i < robot_count; i++) {
                                boxes_from_central_control_node[i].msg.cmd = -1;
                                boxes_from_central_control_node[i].dirtyBit = 1;
                        }

                        unblock_threads();
                        thread_exit();
                }

                   unblock_threads();
        }
}

// test code for robot thread
void robot_thread(void* aux){
        int idx = *((int *)aux);

        while (1) {
                enum intr_level old_level;

                old_level = intr_disable();

                boxes_from_robots[idx].msg.row = robots[idx].row;
                boxes_from_robots[idx].msg.col = robots[idx].col;
                boxes_from_robots[idx].msg.current_payload = robots[idx].current_payload;
                boxes_from_robots[idx].msg.required_payload = robots[idx].required_payload;
                boxes_from_robots[idx].dirtyBit = 1;

                list_push_back(&blocked_threads, &thread_current()->elem);
                thread_block();

                intr_set_level(old_level);

                if (boxes_from_central_control_node[idx].dirtyBit == 1) {
                        int cmd = boxes_from_central_control_node[idx].msg.cmd;

                        if (cmd == -1 || simulation_done == 1) {
                                boxes_from_central_control_node[idx].dirtyBit = 0;
                                thread_exit();
                        }

                        if (cmd == 1) {
                                robots[idx].row--;
                        } else if (cmd == 2) {
                                robots[idx].row++;
                        } else if (cmd == 3) {
                                robots[idx].col--;
                        } else if (cmd == 4) {
                                robots[idx].col++;
                        }

                        if (robots[idx].current_payload == 0 &&
                        robots[idx].row == payload_rows[robots[idx].required_payload] &&
                        robots[idx].col == payload_cols[robots[idx].required_payload]) {
                                robots[idx].current_payload = robots[idx].required_payload;
                        }

                        if (robots[idx].current_payload == robots[idx].required_payload) {
                                int dest_row;
                                int dest_col;

                                get_destination_position(destinations[idx], &dest_row, &dest_col);

                                if (robots[idx].row == dest_row && robots[idx].col == dest_col) {
                                        completed[idx] = 1;
                                }
                        }

                        boxes_from_central_control_node[idx].dirtyBit = 0;
                }
        }
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this
        list_init(&blocked_threads);

        robot_count = atoi(argv[1]);
        char *robot_infos = argv[2];

        robots = malloc(sizeof(struct robot) * robot_count);
        destinations = malloc(sizeof(char) * robot_count);

        completed = malloc(sizeof(int) * robot_count);

        boxes_from_robots = malloc(sizeof(struct message_box) * robot_count);
        boxes_from_central_control_node = malloc(sizeof(struct message_box) * robot_count);

        for (int i = 0; i < robot_count; i++) {
                boxes_from_robots[i].dirtyBit = 0;
                boxes_from_central_control_node[i].dirtyBit = 0;
                completed[i] = 0;
        }

        char *save_ptr;
        char *token = strtok_r(robot_infos, ":", &save_ptr);

        int i = 0;

        while (token != NULL && i < robot_count) {
                int required_payload = token[0] - '0';
                char destination = token[1];

                char *name = malloc(sizeof(char) * 4);
                snprintf(name, 4, "R%d", i + 1);

                // 초기 위치는 W(6,5)
                setRobot(&robots[i], name, 6, 5, required_payload, 0);
                destinations[i] = destination;

                token = strtok_r(NULL, ":", &save_ptr);
                i++;
        }

        tid_t cnt_thread = 0;
        cnt_thread = thread_create("CNT", 0, &central_control_thread, NULL);

        tid_t* threads = malloc(sizeof(tid_t) * robot_count);

        int* idxs = malloc(sizeof(int) * robot_count);

        for (i = 0; i < robot_count; i++) {
                char *name = malloc(sizeof(char) * 4);
                snprintf(name, 4, "R%d", i + 1);

                idxs[i] = i;
                threads[i] = thread_create(name, 0, &robot_thread, &idxs[i]);
        }

        handle_parse_completion(cnt_thread, threads, robot_count);
}