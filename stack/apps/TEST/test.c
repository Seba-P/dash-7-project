
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

#define SENSOR_FILE_ID           0x40
#define SENSOR_FILE_SIZE         4
#define ACTION_FILE_ID           0x41

#define SENSOR_UPDATE   TIMER_TICKS_PER_SEC * 3

#ifdef PLATFORM_EFM32WG_STK3800
 #define GATEWAY
#endif

/********************
 * APP FILES CONFIG *
 ********************/
void init_gateway_files()
{
    // file 0x40: contains our sensor data + configure an action file to be executed upon write
    fs_file_header_t file_header = (fs_file_header_t){
        .file_properties.action_protocol_enabled = 0,
        .file_properties.action_file_id = ACTION_FILE_ID,
        .file_properties.action_condition = ALP_ACT_COND_WRITE,
        .file_properties.storage_class = FS_STORAGE_VOLATILE,
        .file_properties.permissions = 0, // TODO
        .length = SENSOR_FILE_SIZE
    };

    fs_init_file(SENSOR_FILE_ID, &file_header, NULL);
}

void init_sensor_files()
{
    // file 0x40: contains our sensor data + configure an action file to be executed upon write
    fs_file_header_t file_header = (fs_file_header_t){
        .file_properties.action_protocol_enabled = 0,
        .file_properties.action_file_id = ACTION_FILE_ID,
        .file_properties.action_condition = ALP_ACT_COND_WRITE,
        .file_properties.storage_class = FS_STORAGE_VOLATILE,
        .file_properties.permissions = 0, // TODO
        .length = SENSOR_FILE_SIZE
    };

    fs_init_file(SENSOR_FILE_ID, &file_header, NULL);

    // configure file notification using D7AActP: write ALP command to broadcast changes made to file 0x40 in file 0x41
    // first generate ALP command consisting of ALP Control header, ALP File Data Request operand and D7ASP interface configuration
    alp_control_regular_t alp_ctrl = {
        .group = false,
        .response_requested = false,
        .operation = ALP_OP_READ_FILE_DATA
        // .operation = ALP_OP_WRITE_FILE_DATA
    };

    alp_operand_file_data_request_t file_data_request_operand = {
    // alp_operand_file_data_t file_data_operand = {
        .file_offset = {
            .file_id = SENSOR_FILE_ID,
            .offset = 0
        },
        .requested_data_length = SENSOR_FILE_SIZE,
        // .provided_data_length = SENSOR_FILE_SIZE,
    };

    d7asp_master_session_config_t session_config = {
        .qos = {
            .qos_resp_mode = SESSION_RESP_MODE_NO,
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

    // finally, register D7AActP file
    fs_init_file_with_D7AActP(ACTION_FILE_ID, &session_config, (alp_control_t*)&alp_ctrl, (uint8_t*)&file_data_request_operand);
    // fs_init_file_with_D7AActP(ACTION_FILE_ID, &session_config, (alp_control_t*)&alp_ctrl, (uint8_t*)&file_data_operand);
}

/*************
 * APP TASKS *
 *************/
void log_current_val()
{
  uint32_t val;

  fs_read_file(SENSOR_FILE_ID, 0, (uint8_t*)&val, SENSOR_FILE_SIZE);

  led_toggle(1);
  log_print_string("\nval = %d", val & 0xFF);
  // lcd_write_line(3, "val = %d\n", val & 0xFF);

  timer_post_task_delay(&log_current_val, TIMER_TICKS_PER_SEC);
}

typedef struct {
    alp_control_regular_t alp_ctrl;
    // alp_operand_file_data_request_t file_data_request_operand = {
    alp_operand_file_data_t file_data_operand;
    uint32_t val;
} sensor_alp_cmd_t;

sensor_alp_cmd_t sensor_alp_cmd = {
    .alp_ctrl = {
        .group = false,
        .response_requested = false,
        // .operation = ALP_OP_READ_FILE_DATA
        .operation = ALP_OP_WRITE_FILE_DATA
    },

    // alp_operand_file_data_request_t file_data_request_operand = {
    .file_data_operand = {
        .file_offset = {
            .file_id = SENSOR_FILE_ID,
            .offset = 0
        },
        // .requested_data_length = SENSOR_FILE_SIZE,
        .provided_data_length = SENSOR_FILE_SIZE,
    },

    .val = 0,
};

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

void execute_sensor_measurement()
{
  // static uint32_t val;
  static uint8_t sensor_alp_resp[ALP_PAYLOAD_MAX_SIZE];
  static uint8_t sensor_alp_resp_len = SENSOR_FILE_SIZE;
  // val++;
  sensor_alp_cmd.val++;

  led_toggle(1);
  log_print_string("val = %d, tab[0] = %d", sensor_alp_cmd.val & 0xFF, *((uint8_t*)&sensor_alp_cmd));
  // lcd_write_line(3, "val = %d\n", val & 0xFF);

  fs_write_file(SENSOR_FILE_ID, 0, (uint8_t*)&sensor_alp_cmd.val, SENSOR_FILE_SIZE);

  // alp_process_command(&sensor_alp_cmd, sizeof(sensor_alp_cmd_t), sensor_alp_resp, &sensor_alp_resp_len, ALP_CMD_ORIGIN_APP);
  alp_send_command((uint8_t*)&sensor_alp_cmd, sizeof(sensor_alp_cmd_t), sensor_alp_resp, sizeof(sensor_alp_cmd_t), &d7asp_session_config);

  timer_post_task_delay(&execute_sensor_measurement, SENSOR_UPDATE);
}

/*****************
 * APP CALLBACKS *
 *****************/
// Toggle different operational modes
void userbutton_callback(button_id_t button_id)
{
  log_print_string("\nButton: %d", button_id);
    // lcd_write_line(4, "Button: %d\n", button_id);
}

static d7asp_init_args_t d7asp_init_args;

static void on_unsollicited_response_received(d7asp_result_t d7asp_result, uint8_t *alp_command, uint8_t alp_command_size)
{
    alp_cmd_handler_output_d7asp_response(d7asp_result, alp_command, alp_command_size);
}

void bootstrap()
{
    log_print_string("\nDEVICE BOOTED\n");
    dae_access_profile_t access_classes[1] = {
        {
            #ifdef GATEWAY
             .control_scan_type_is_foreground = true,
            #else
             .control_scan_type_is_foreground = false,
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

     sched_register_task(&log_current_val);
     timer_post_task_delay(&log_current_val, TIMER_TICKS_PER_SEC);
    #else
     d7ap_stack_init(&fs_init_args, NULL, false, NULL);
     lcd_write_string("SENSOR\n");

     sched_register_task(&execute_sensor_measurement);
     timer_post_task_delay(&execute_sensor_measurement, TIMER_TICKS_PER_SEC * 2);
    #endif
}
