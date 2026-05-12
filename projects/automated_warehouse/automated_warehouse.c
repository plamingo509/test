#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "threads/interrupt.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/aw_thread.h"

struct robot* robots;
char* destinations;
int robot_count;
int* completed;
int simulation_done = 0;

int payload_rows[8] = {-1, 2, 3, 4, 2, 3, 4, 1};
int payload_cols[8] = {-1, 2, 2, 2, 4, 4, 4, 2};

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

void get_final_target(int idx, int* target_row, int* target_col) {
        if (robots[idx].current_payload == 0) {
                int payload = robots[idx].required_payload;

                *target_row = payload_rows[payload];
                *target_col = payload_cols[payload];
        } else {
                get_destination_position(destinations[idx], target_row, target_col);
        }
}

int is_shared_place(int row, int col) {
        if ((row == 6 && col == 5) ||
            (row == 1 && col == 0) ||
            (row == 3 && col == 0) ||
            (row == 5 && col == 0)) {
                return 1;
        }

        return 0;
}

int is_x_position(int row, int col) {
        if (row < 0 || row >= 7 || col < 0 || col >= 7) {
                return 1;
        }

        if (row == 0) {
                return 1;
        }

        if (row == 6 && col != 5) {
                return 1;
        }

        if (col == 6) {
                return 1;
        }

        if ((row == 2 && col == 0) ||
            (row == 4 && col == 0) ||
            (row == 2 && col == 3) ||
            (row == 3 && col == 3) ||
            (row == 4 && col == 3)) {
                return 1;
        }

        return 0;
}

int get_payload_number_at(int row, int col) {
        int i;

        for (i = 1; i <= 7; i++) {
                if (payload_rows[i] == row && payload_cols[i] == col) {
                        return i;
                }
        }

        return 0;
}

void get_next_position(int row, int col, int cmd, int* next_row, int* next_col) {
        *next_row = row;
        *next_col = col;

        if (cmd == 1) {
                *next_row = row - 1;
        } else if (cmd == 2) {
                *next_row = row + 1;
        } else if (cmd == 3) {
                *next_col = col - 1;
        } else if (cmd == 4) {
                *next_col = col + 1;
        }
}

int is_blocked_cell_for_robot(int idx, int row, int col) {
        int payload;

        if (is_x_position(row, col)) {
                return 1;
        }

        payload = get_payload_number_at(row, col);

        if (payload != 0 && payload != robots[idx].required_payload) {
                return 1;
        }

        return 0;
}

int get_bfs_command(int idx, int target_row, int target_col) {
        int visited[7][7];
        int prev_row[7][7];
        int prev_col[7][7];

        int queue_row[49];
        int queue_col[49];

        int front = 0;
        int rear = 0;

        int dr[4] = {-1, 1, 0, 0};
        int dc[4] = {0, 0, -1, 1};

        int r;
        int c;
        int i;

        for (r = 0; r < 7; r++) {
                for (c = 0; c < 7; c++) {
                        visited[r][c] = 0;
                        prev_row[r][c] = -1;
                        prev_col[r][c] = -1;
                }
        }

        queue_row[rear] = robots[idx].row;
        queue_col[rear] = robots[idx].col;
        rear++;

        visited[robots[idx].row][robots[idx].col] = 1;

        while (front < rear) {
                int cur_row = queue_row[front];
                int cur_col = queue_col[front];

                front++;

                if (cur_row == target_row && cur_col == target_col) {
                        break;
                }

                for (i = 0; i < 4; i++) {
                        int next_row = cur_row + dr[i];
                        int next_col = cur_col + dc[i];

                        if (next_row < 0 || next_row >= 7 ||
                            next_col < 0 || next_col >= 7) {
                                continue;
                        }

                        if (visited[next_row][next_col] == 1) {
                                continue;
                        }

                        if (is_blocked_cell_for_robot(idx, next_row, next_col)) {
                                continue;
                        }

                        visited[next_row][next_col] = 1;
                        prev_row[next_row][next_col] = cur_row;
                        prev_col[next_row][next_col] = cur_col;

                        queue_row[rear] = next_row;
                        queue_col[rear] = next_col;
                        rear++;
                }
        }

        if (visited[target_row][target_col] == 0) {
                return 0;
        }

        r = target_row;
        c = target_col;

        while (!(prev_row[r][c] == robots[idx].row &&
                 prev_col[r][c] == robots[idx].col)) {
                int pr = prev_row[r][c];
                int pc = prev_col[r][c];

                if (pr == -1 || pc == -1) {
                        return 0;
                }

                r = pr;
                c = pc;
        }

        if (r == robots[idx].row - 1 && c == robots[idx].col) {
                return 1;
        } else if (r == robots[idx].row + 1 && c == robots[idx].col) {
                return 2;
        } else if (r == robots[idx].row && c == robots[idx].col - 1) {
                return 3;
        } else if (r == robots[idx].row && c == robots[idx].col + 1) {
                return 4;
        }

        return 0;
}

int get_alternative_command(int idx, int final_cmd[], int next_rows[], int next_cols[], int robot_order) {
        int cmds[4] = {1, 2, 3, 4};
        int k;
        int j;

        for (k = 0; k < 4; k++) {
                int cmd = cmds[k];
                int nr;
                int nc;
                int payload_on_next;
                int conflict = 0;

                get_next_position(robots[idx].row, robots[idx].col, cmd, &nr, &nc);

                if (is_x_position(nr, nc)) {
                        continue;
                }

                if (is_shared_place(nr, nc)) {
                        continue;
                }

                payload_on_next = get_payload_number_at(nr, nc);

                if (payload_on_next != 0) {
                        continue;
                }

                /*
                 * A/B/C/W는 중복 가능하므로 로봇 점유 충돌 검사 제외
                 */
                if (!is_shared_place(nr, nc)) {
                        /*
                         * 다른 로봇이 현재 있는 칸이면 피함.
                         * 단, 그 로봇이 앞 순서이고 이번 step에 이동해서 비우는 경우는 허용.
                         */
                        for (j = 0; j < robot_count; j++) {
                                if (j != idx &&
                                    completed[j] == 0 &&
                                    robots[j].row == nr &&
                                    robots[j].col == nc) {

                                        if (j < robot_order && final_cmd[j] != 0) {
                                                // 앞 순서 로봇이 이동해서 비울 예정이므로 허용
                                        } else {
                                                conflict = 1;
                                        }
                                }
                        }

                        /*
                         * 이번 step에 이미 다른 로봇이 예약한 칸이면 피함.
                         */
                        for (j = 0; j < robot_order; j++) {
                                if (final_cmd[j] != 0 &&
                                    next_rows[j] == nr &&
                                    next_cols[j] == nc) {
                                        conflict = 1;
                                }
                        }

                        /*
                         * 서로 자리 바꾸기 방지.
                         */
                        for (j = 0; j < robot_order; j++) {
                                if (final_cmd[j] != 0 &&
                                    nr == robots[j].row &&
                                    nc == robots[j].col &&
                                    next_rows[j] == robots[idx].row &&
                                    next_cols[j] == robots[idx].col) {
                                        conflict = 1;
                                }
                        }
                }

                if (conflict == 0) {
                        next_rows[idx] = nr;
                        next_cols[idx] = nc;
                        return cmd;
                }
        }

        return 0;
}

void central_control_thread(void* aux) {
        int *desired_cmd = malloc(sizeof(int) * robot_count);
        int *final_cmd = malloc(sizeof(int) * robot_count);
        int *next_rows = malloc(sizeof(int) * robot_count);
        int *next_cols = malloc(sizeof(int) * robot_count);

        while (1) {
                int i;
                int j;
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

                int all_completed = 1;

                for (i = 0; i < robot_count; i++) {
                        if (completed[i] == 0) {
                                all_completed = 0;
                        }
                }

                if (all_completed == 1) {
                        simulation_done = 1;

                        for (i = 0; i < robot_count; i++) {
                                boxes_from_central_control_node[i].msg.cmd = -1;
                                boxes_from_central_control_node[i].dirtyBit = 1;
                        }

                        unblock_threads();

                        free(desired_cmd);
                        free(final_cmd);
                        free(next_rows);
                        free(next_cols);

                        thread_exit();
                }

                for (i = 0; i < robot_count; i++) {
                        int target_row;
                        int target_col;

                        desired_cmd[i] = 0;
                        final_cmd[i] = 0;
                        next_rows[i] = robots[i].row;
                        next_cols[i] = robots[i].col;

                        if (completed[i] == 0) {
                                get_final_target(i, &target_row, &target_col);
                                desired_cmd[i] = get_bfs_command(i, target_row, target_col);

                                get_next_position(
                                        robots[i].row,
                                        robots[i].col,
                                        desired_cmd[i],
                                        &next_rows[i],
                                        &next_cols[i]
                                );
                        }
                }

                for (i = 0; i < robot_count; i++) {
                        int conflict = 0;
                        int payload_on_next;

                        if (completed[i] == 1 || desired_cmd[i] == 0) {
                                final_cmd[i] = 0;
                                continue;
                        }

                        if (is_x_position(next_rows[i], next_cols[i])) {
                                final_cmd[i] = 0;
                                continue;
                        }

                        payload_on_next = get_payload_number_at(next_rows[i], next_cols[i]);

                        if (payload_on_next != 0 &&
                        payload_on_next != robots[i].required_payload) {
                                final_cmd[i] = 0;
                                continue;
                        }

                        /*
                        * W, A, B, C는 중복 가능.
                        * 따라서 shared place로 들어가는 이동은
                        * 다른 로봇이 있어도 허용한다.
                        */
                        if (is_shared_place(next_rows[i], next_cols[i])) {
                                final_cmd[i] = desired_cmd[i];
                                continue;
                        }

                        for (j = 0; j < i; j++) {
                                if (final_cmd[j] != 0 &&
                                next_rows[j] == next_rows[i] &&
                                next_cols[j] == next_cols[i]) {
                                        conflict = 1;
                                }
                        }

                        for (j = 0; j < robot_count; j++) {
                                if (i != j &&
                                completed[j] == 0 &&
                                robots[j].row == next_rows[i] &&
                                robots[j].col == next_cols[i]) {

                                        if (j < i) {
                                                if (final_cmd[j] == 0) {
                                                        conflict = 1;
                                                }
                                        } else {
                                                conflict = 1;
                                        }
                                }
                        }

                        for (j = 0; j < i; j++) {
                                if (final_cmd[j] != 0 &&
                                next_rows[i] == robots[j].row &&
                                next_cols[i] == robots[j].col &&
                                next_rows[j] == robots[i].row &&
                                next_cols[j] == robots[i].col) {
                                        conflict = 1;
                                }
                        }

                        if (conflict == 1) {
                                int alt_cmd;

                                alt_cmd = get_alternative_command(i, final_cmd, next_rows, next_cols, i);

                                final_cmd[i] = alt_cmd;

                                if (alt_cmd == 0) {
                                        next_rows[i] = robots[i].row;
                                        next_cols[i] = robots[i].col;
                                }
                        } else {
                                final_cmd[i] = desired_cmd[i];
                        }
                }

                for (i = 0; i < robot_count; i++) {
                        boxes_from_robots[i].dirtyBit = 0;
                        boxes_from_central_control_node[i].msg.cmd = final_cmd[i];
                        boxes_from_central_control_node[i].dirtyBit = 1;
                }

                increase_step();

                unblock_threads();

        }
}

void robot_thread(void* aux) {
        int idx = *((int *)aux);

        while (1) {
                enum intr_level old_level;

                old_level = intr_disable();

                boxes_from_robots[idx].msg.row = robots[idx].row;
                boxes_from_robots[idx].msg.col = robots[idx].col;
                boxes_from_robots[idx].msg.current_payload = robots[idx].current_payload;
                boxes_from_robots[idx].msg.required_payload = robots[idx].required_payload;
                boxes_from_robots[idx].dirtyBit = 1;

                block_thread();

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

                                if (robots[idx].row == dest_row &&
                                    robots[idx].col == dest_col) {
                                        completed[idx] = 1;
                                }
                        }

                        boxes_from_central_control_node[idx].dirtyBit = 0;
                }
        }
}

void run_automated_warehouse(char **argv) {
        init_automated_warehouse(argv);

        list_init(&blocked_threads);

        robot_count = atoi(argv[1]);
        char *robot_infos = argv[2];

        robots = malloc(sizeof(struct robot) * robot_count);
        destinations = malloc(sizeof(char) * robot_count);
        completed = malloc(sizeof(int) * robot_count);

        boxes_from_robots = malloc(sizeof(struct message_box) * robot_count);
        boxes_from_central_control_node = malloc(sizeof(struct message_box) * robot_count);

        int i;

        for (i = 0; i < robot_count; i++) {
                boxes_from_robots[i].dirtyBit = 0;
                boxes_from_central_control_node[i].dirtyBit = 0;
                completed[i] = 0;
        }

        char *save_ptr;
        char *token = strtok_r(robot_infos, ":", &save_ptr);

        i = 0;

        while (token != NULL && i < robot_count) {
                int required_payload = token[0] - '0';
                char destination = token[1];

                char *name = malloc(sizeof(char) * 16);
                snprintf(name, 16, "R%d", i + 1);

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
                char *name = malloc(sizeof(char) * 16);
                snprintf(name, 16, "R%d", i + 1);

                idxs[i] = i;
                threads[i] = thread_create(name, 0, &robot_thread, &idxs[i]);
        }

        handle_parse_completion(cnt_thread, threads, robot_count);
}