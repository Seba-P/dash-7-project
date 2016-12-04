
/******************
 * COMPONENT LIBS *
 ******************/
// #include "debug.h"
// #include "fifo.h"
#include "log.h"
#include "scheduler.h"
#include "timer.h"

/*****************
 * PLATFORM LIBS *
 *****************/
#include "platform.h"
#include "platform_lcd.h"
#include "platform_sensors.h"
#include "userbutton.h"

#include "hwadc.h"
#include "hwlcd.h"
#include "hwleds.h"
// #include "hwsystem.h"
// #include "hwuart.h"


/****************
 * MODULES LIBS *
 ****************/
#include "d7ap_stack.h"
// #include "dll.h"
// #include "alp_cmd_handler.h"
// #include "fs.h"

/*****************
 * STANDARD LIBS *
 *****************/
#include <stdio.h>
#include <stdlib.h>

#include "version.h"
#include "app_files_cfg.h"

#ifdef PLATFORM_EFM32WG_STK3800
 #define GATEWAY
#endif

/********************
 * APP FILES CONFIG *
 ********************/

void init_gateway_files(void)
{
    fs_init_file(SENSOR_DATA_FILE_ID, &sensor_data_file, NULL);
    fs_init_file(SENSOR_DELTA_FILE_ID, &sensor_delta_file, NULL);
    fs_init_file(SENSOR_HEARTBEAT_FILE_ID, &sensor_heartbeat_file, NULL);

    uint32_t sensor_data_file_init = SENSOR_DATA_FILE_INIT;
    uint32_t sensor_delta_file_init = SENSOR_DELTA_FILE_INIT;
    uint32_t sensor_heartbeat_file_init = SENSOR_HEARTBEAT_FILE_INIT;

    fs_write_file(SENSOR_DATA_FILE_ID, 0, (uint8_t*)&sensor_data_file_init, SENSOR_DATA_FILE_SIZE);
    fs_write_file(SENSOR_DELTA_FILE_ID, 0, (uint8_t*)&sensor_delta_file_init, SENSOR_DELTA_FILE_SIZE);
    fs_write_file(SENSOR_HEARTBEAT_FILE_ID, 0, (uint8_t*)&sensor_heartbeat_file_init, SENSOR_HEARTBEAT_FILE_SIZE);

    log_print_string("\nFile system initialized...");
}

void init_sensor_files(void)
{
    fs_init_file(SENSOR_DATA_FILE_ID, &sensor_data_file, NULL);
    fs_init_file(SENSOR_DELTA_FILE_ID, &sensor_delta_file, NULL);
    fs_init_file(SENSOR_HEARTBEAT_FILE_ID, &sensor_heartbeat_file, NULL);

    uint32_t sensor_data_file_init = SENSOR_DATA_FILE_INIT;
    uint32_t sensor_delta_file_init = SENSOR_DELTA_FILE_INIT;
    uint32_t sensor_heartbeat_file_init = SENSOR_HEARTBEAT_FILE_INIT;

    fs_write_file(SENSOR_DATA_FILE_ID, 0, (uint8_t*)&sensor_data_file_init, SENSOR_DATA_FILE_SIZE);
    fs_write_file(SENSOR_DELTA_FILE_ID, 0, (uint8_t*)&sensor_delta_file_init, SENSOR_DELTA_FILE_SIZE);
    fs_write_file(SENSOR_HEARTBEAT_FILE_ID, 0, (uint8_t*)&sensor_heartbeat_file_init, SENSOR_HEARTBEAT_FILE_SIZE);

    log_print_string("\nFile system initialized...");
}

d7asp_master_session_config_t d7asp_session_config = {
    .qos = {
        .qos_resp_mode = SESSION_RESP_MODE_ANY,
        .qos_nls                 = false,
        .qos_stop_on_error       = false,
        .qos_record              = false
    },
    .dormant_timeout = 0,
    .addressee = {
        .ctrl = {
          .id_type = ID_TYPE_BCAST,
          .access_class = 0
        },
        .id = 0
    }
};

extern bool alp_send_command(uint8_t* alp_cmd, uint8_t alp_cmd_len, uint8_t* alp_resp, uint8_t alp_resp_len, d7asp_master_session_config_t* d7asp_session_config);

/*************
 * APP TASKS *
 *************/
#ifdef GATEWAY
 typedef struct
 {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
 } date_struct_t;

 date_struct_t current_date =
 {
    .year = 2016,
    .month = 12,
    .day = 31,
    .hour = 23,
    .minute = 58,
    .second = 58,
 };

 void current_date_calculation(void)
 {  
    uint8_t month_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    if(++(current_date.second) >= 60)
    {
        current_date.second = 0;
        if(++(current_date.minute) >= 60)
        {
            current_date.minute = 0;
            if(++(current_date.hour) >= 24)
            {
                current_date.hour = 0;
                if(++(current_date.day) >= month_days[current_date.month-1])
                {
                    current_date.day = 0;
                    if(++(current_date.month) >= 13)
                    {
                        current_date.month = 1;
                        current_date.year++;
                    }
                }
            }
        } 
    }

    timer_post_task_delay(&current_date_calculation, TIMER_TICKS_PER_SEC);
 }

 void print_current_date(void)
 {
    log_print_string("\n[%4d-%02d-%02d, %02d:%02d:%02d] ", current_date.year, current_date.month, current_date.day,
                                                    current_date.hour, current_date.minute, current_date.second);
 }
#endif

uint32_t sensor_data = 50;
uint32_t last_recvd_time = 0;

void sensor_connection_check(void)
{
    #define SENSOR_UPDATE_PERIOD (TIMER_TICKS_PER_SEC / 4)

    uint8_t sensor_alp_resp[ALP_PAYLOAD_MAX_SIZE];

    volatile uint32_t sensor_heartbeat;

    last_recvd_time += SENSOR_UPDATE_PERIOD;
    fs_read_file(SENSOR_HEARTBEAT_FILE_ID, 0, (uint8_t*)&sensor_heartbeat, SENSOR_HEARTBEAT_FILE_SIZE);

    if(last_recvd_time >= (sensor_heartbeat + SENSOR_UPDATE_PERIOD))
    {
        print_current_date();
        log_print_string("Sensor does not respond...");
        last_recvd_time = SENSOR_UPDATE_PERIOD;
    }

    timer_post_task_delay(&sensor_connection_check, SENSOR_UPDATE_PERIOD);

    #undef SENSOR_UPDATE_PERIOD
}

void execute_sensor_measurement(void)
{
    #define SENSOR_UPDATE_PERIOD (TIMER_TICKS_PER_SEC / 4)

    uint8_t sensor_alp_resp[ALP_PAYLOAD_MAX_SIZE];

    // static uint32_t sensor_data;
    volatile uint32_t sensor_delta;
    volatile uint32_t sensor_heartbeat;
    static uint32_t last_transm_time;
    static uint32_t last_transm_data;

    last_transm_time += SENSOR_UPDATE_PERIOD;
    // log_print_string("\nsensor_data = %d", sensor_data);
    // lcd_write_line(3, "val = %d\n", val & 0xFF);

    //fs_read_file(SENSOR_DATA_FILE_ID, 0, (uint8_t*)&sensor_data, SENSOR_DATA_FILE_SIZE);
    fs_read_file(SENSOR_DELTA_FILE_ID, 0, (uint8_t*)&sensor_delta, SENSOR_DELTA_FILE_SIZE);
    fs_read_file(SENSOR_HEARTBEAT_FILE_ID, 0, (uint8_t*)&sensor_heartbeat, SENSOR_HEARTBEAT_FILE_SIZE);

    fs_write_file(SENSOR_DATA_FILE_ID, 0, (uint8_t*)&sensor_data, SENSOR_DATA_FILE_SIZE);
    if(sensor_data < (last_transm_data - sensor_delta) || sensor_data > (last_transm_data + sensor_delta) || last_transm_time >= sensor_heartbeat)
    {
    last_transm_data = sensor_data;
    last_transm_time = 0;
    led_toggle(1);

    sensor_data_write_cmd.val = sensor_data;
    alp_send_command((uint8_t*)&sensor_data_write_cmd, sizeof(sensor_alp_cmd_write_t), sensor_alp_resp, sizeof(sensor_alp_cmd_write_t), &d7asp_session_config);
    }

    timer_post_task_delay(&execute_sensor_measurement, SENSOR_UPDATE_PERIOD);

    #undef SENSOR_UPDATE_PERIOD
}

/*****************
 * APP CALLBACKS *
 *****************/
// Toggle different operational modes
void userbutton_callback(button_id_t button_id)
{
    #ifdef GATEWAY
     uint32_t sensor_heartbeat;
     uint8_t sensor_alp_resp[ALP_PAYLOAD_MAX_SIZE];

     fs_read_file(SENSOR_HEARTBEAT_FILE_ID, 0, (uint8_t*)&sensor_heartbeat, SENSOR_HEARTBEAT_FILE_SIZE);
     if(button_id)
     {
        sensor_heartbeat += TIMER_TICKS_PER_SEC;
     }else
     {
        sensor_heartbeat -= TIMER_TICKS_PER_SEC;
     }
     fs_write_file(SENSOR_HEARTBEAT_FILE_ID, 0, (uint8_t*)&sensor_heartbeat, SENSOR_HEARTBEAT_FILE_SIZE);

     sensor_heartbeat_write_cmd.val = sensor_heartbeat;
     alp_send_command((uint8_t*)&sensor_heartbeat_write_cmd, sizeof(sensor_alp_cmd_write_t), sensor_alp_resp, sizeof(sensor_alp_cmd_write_t), &d7asp_session_config);

     log_print_string("\nButton: %d, sensor_heartbeat = %d", button_id, sensor_heartbeat);
    #else
     if(!button_id)
     {
        sensor_data++;
     }else
     {
        sensor_data--;
     }

     log_print_string("\nButton: %d, sensor_data = %d", button_id, sensor_data);
    #endif
    // lcd_write_line(4, "Button: %d\n", button_id);
}

bool notify_reception(uint8_t* alp_cmd, uint8_t alp_cmd_len, alp_command_origin_t origin)
{
    alp_control_t alp_ctrl;
    uint8_t alp_operand[ALP_PAYLOAD_MAX_SIZE];

    alp_ctrl.raw = alp_cmd[0];
    memcpy(alp_operand, alp_cmd + 1, alp_cmd_len - 1);

    if(alp_ctrl.operation == ALP_OP_WRITE_FILE_DATA && alp_operand[0] == SENSOR_DATA_FILE_ID)
    {
        print_current_date();
        log_print_string("Sensor data = %d", (uint32_t)*(alp_operand + sizeof(alp_operand_file_data_t)));
        last_recvd_time = 0;      
    }

    return true;
}

static d7asp_init_args_t d7asp_init_args;

static void on_unsollicited_response_received(d7asp_result_t d7asp_result, uint8_t *alp_command, uint8_t alp_command_size)
{
    alp_cmd_handler_output_d7asp_response(d7asp_result, alp_command, alp_command_size);
}

void bootstrap(void)
{
    log_print_string("\nDEVICE BOOTED\n");
    dae_access_profile_t access_classes[1] = {
        {
            #ifdef GATEWAY
             .control_scan_type_is_foreground = true,
            #else
             .control_scan_type_is_foreground = true,
            #endif
            .control_csma_ca_mode = CSMA_CA_MODE_UNC,
            .control_number_of_subbands = 1,
            .subnet = 0,
            .scan_automation_period = 0,
            .transmission_timeout_period = 50,
            .subbands[0] = (subband_t){
                .channel_header = {
                    .ch_coding = PHY_CODING_PN9,
                    .ch_class = PHY_CLASS_NORMAL_RATE,
                    .ch_freq_band = PHY_BAND_868
                },
                .channel_index_start = 16,
                .channel_index_end = 16,
                .eirp = 10,
                .ccao = 0
            }
        }
    };

    fs_init_args_t fs_init_args = (fs_init_args_t){
        #ifdef GATEWAY
         .fs_user_files_init_cb = init_gateway_files,
        #else
         .fs_user_files_init_cb = init_sensor_files,
        #endif
        .access_profiles_count = 1,
        .access_profiles = access_classes
    };

    ubutton_register_callback(0, &userbutton_callback);
    ubutton_register_callback(1, &userbutton_callback);

    #ifdef GATEWAY
     d7asp_init_args.d7asp_received_unsollicited_data_cb = &on_unsollicited_response_received;
     d7ap_stack_init(&fs_init_args, &d7asp_init_args, true, NULL);
     fs_write_dll_conf_active_access_class(0); // use access class 0 for scan automation
     lcd_write_string("GATEWAY\n");

     sched_register_task(&sensor_connection_check);
     sched_register_task(&current_date_calculation);
     timer_post_task_delay(&sensor_connection_check, TIMER_TICKS_PER_SEC * 2);
     timer_post_task_delay(&current_date_calculation, TIMER_TICKS_PER_SEC);
    #else
     d7ap_stack_init(&fs_init_args, NULL, false, NULL);
     lcd_write_string("SENSOR\n");

     sched_register_task(&execute_sensor_measurement);
     timer_post_task_delay(&execute_sensor_measurement, TIMER_TICKS_PER_SEC * 3);
    #endif
}
