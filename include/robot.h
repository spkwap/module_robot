#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#define MAX_RULES 10

typedef enum { IN_NONE=0, IN_DISTANCE=1, IN_TCRT=2, IN_FLAG_1=3, IN_FLAG_2=4, IN_FLAG_3=5, IN_FLAG_4=6, IN_FLAG_5=7 } input_type_t;
typedef enum { OP_LESS, OP_GREATER, OP_EQUAL } operator_t;
typedef enum { OUT_NONE=0, OUT_MOTOR_LEFT=1, OUT_MOTOR_RIGHT=2, OUT_MOTORS_BOTH=3, OUT_FLAG_1=4, OUT_FLAG_2=5, OUT_FLAG_3=6, OUT_FLAG_4=7, OUT_FLAG_5=8 } output_type_t;

typedef struct {
    bool active;
    input_type_t in_type;  operator_t op;   int threshold;
    input_type_t in_type_2; operator_t op_2; int threshold_2;
    int logic_link; 
    output_type_t act_out; int act_val; 
    output_type_t els_out; int els_val; 
    int duration_ms; 
} rule_t;

extern char g_dist[16];
extern bool g_obs;
extern bool g_mot;
extern int virtual_flags[5];
extern rule_t system_rules[MAX_RULES];
extern i2c_master_bus_handle_t i2c_bus;

void hardware_init(void);
bool read_tcrt_sensor(void);
void motors_set_speed(int target_ml, int target_mr);
void motors_stop(void);

void network_init_ap(void);
void webserver_start(void);

#endif // ROBOT_H