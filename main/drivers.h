#include <stdio.h>
#include <inttypes.h>

void move_forward(uint32_t speed);
void move_backward(uint32_t speed);
void move_left(uint32_t speed);
void move_right(uint32_t speed);
void rotate_clockwise(uint32_t speed);
void rotate_counterclockwise(uint32_t speed);
void stop_all_motors(void);
static float get_distance_cm();
