#ifndef _APP_FILES_CFG_H_
 #define _APP_FILES_CFG_H_

 #include "d7ap_stack.h"
 #include "alp.h"
 #include "fs.h"

 /***********************
  * SENSOR FILES CONFIG *
  ***********************/
 #define SENSOR_DATA_FILE_ID			0x40
 #define SENSOR_DATA_FILE_SIZE			4
 #define SENSOR_DATA_FILE_ACTION		0x41
 #define SENSOR_DATA_FILE_INIT			0

 fs_file_header_t sensor_data_file =
 {
    .file_properties.action_protocol_enabled = 0,
    .file_properties.action_file_id = SENSOR_DATA_FILE_ACTION,
    .file_properties.action_condition = ALP_ACT_COND_WRITE,
    .file_properties.storage_class = FS_STORAGE_VOLATILE,
    .file_properties.permissions = 0, // TODO
    .length = SENSOR_DATA_FILE_SIZE
 };

 #define SENSOR_DELTA_FILE_ID			0x42
 #define SENSOR_DELTA_FILE_SIZE			4
 #define SENSOR_DELTA_FILE_ACTION		0x43
 #define SENSOR_DELTA_FILE_INIT			5

 fs_file_header_t sensor_delta_file =
 {
    .file_properties.action_protocol_enabled = 0,
    .file_properties.action_file_id = SENSOR_DELTA_FILE_ACTION,
    .file_properties.action_condition = ALP_ACT_COND_WRITE,
    .file_properties.storage_class = FS_STORAGE_VOLATILE,
    .file_properties.permissions = 0, // TODO
    .length = SENSOR_DELTA_FILE_SIZE
 };

 #define SENSOR_HEARTBEAT_FILE_ID			0x44
 #define SENSOR_HEARTBEAT_FILE_SIZE			4
 #define SENSOR_HEARTBEAT_FILE_ACTION		0x45
 #define SENSOR_HEARTBEAT_FILE_INIT			TIMER_TICKS_PER_SEC * 10

 fs_file_header_t sensor_heartbeat_file =
 {
    .file_properties.action_protocol_enabled = 0,
    .file_properties.action_file_id = SENSOR_HEARTBEAT_FILE_ACTION,
    .file_properties.action_condition = ALP_ACT_COND_WRITE,
    .file_properties.storage_class = FS_STORAGE_VOLATILE,
    .file_properties.permissions = 0, // TODO
    .length = SENSOR_HEARTBEAT_FILE_SIZE
 };

 /***********************
  * ALP COMMANDS CONFIG *
  ***********************/
 typedef struct {
    alp_control_regular_t alp_ctrl;
    alp_operand_file_data_t file_data_operand;
    int32_t val;
 } sensor_alp_cmd_write_t;

 typedef struct {
    alp_control_regular_t alp_ctrl;
    alp_operand_file_data_request_t file_data_request_operand;
    int32_t val;
 } sensor_alp_cmd_read_t;

 sensor_alp_cmd_write_t sensor_data_write_cmd = {
    .alp_ctrl = {
        .group = false,
        .response_requested = false,
        .operation = ALP_OP_WRITE_FILE_DATA
    },

    .file_data_operand = {
        .file_offset = {
            .file_id = SENSOR_DATA_FILE_ID,
            .offset = 0
        },
        .provided_data_length = SENSOR_DATA_FILE_SIZE,
    },

    .val = 0,
 };

 sensor_alp_cmd_write_t sensor_delta_write_cmd = {
    .alp_ctrl = {
        .group = false,
        .response_requested = false,
        .operation = ALP_OP_WRITE_FILE_DATA
    },

    .file_data_operand = {
        .file_offset = {
            .file_id = SENSOR_DELTA_FILE_ID,
            .offset = 0
        },
        .provided_data_length = SENSOR_DELTA_FILE_SIZE,
    },

    .val = 0,
 };

 sensor_alp_cmd_write_t sensor_heartbeat_write_cmd = {
    .alp_ctrl = {
        .group = false,
        .response_requested = false,
        .operation = ALP_OP_WRITE_FILE_DATA
    },

    .file_data_operand = {
        .file_offset = {
            .file_id = SENSOR_HEARTBEAT_FILE_ID,
            .offset = 0
        },
        .provided_data_length = SENSOR_HEARTBEAT_FILE_SIZE,
    },

    .val = 0,
 };

#endif /*_APP_FILES_CFG_H_*/
