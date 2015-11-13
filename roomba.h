/****************************************************************************
 ** File
 ** · roomba.h
 **
 ** Author
 ** · RooWifi Development Team (@roowifi)
 **
 ** Date
 ** · 13-06-2013
 **
 ** Description and comments:
 **
 ** ·This file contains constant definition of iRobot SCI Commands and values
 **  for RooWifi class usage for Qt Creator
 **
 ** ·This file will be useful to develop applications using c programming
 **
 ** ·Read the SCI Specification document from iRobot before use this
 **  source code.
 **
 ** Copyright (C) 2013 RooWifi.com
 ** Contact: http://www.roowifi.com/contact
 **
 ** This file is part of the RooWifi class of Qt API for developers.
 **
 ** Information summary of versions :
 **
 ** Version     Author          Date
 ** 1.0b        @roowifi        13 / 06 / 2013
 ****************************************************************************/
#ifndef ROOMBA_H
#define ROOMBA_H

//Number of sensors in SCI specification
#define SCI_NUMBER_OF_SENSORS       26

//Byte postion for each sensor in response frame
#define BUMPWHEELDROPS_SENSOR       0
#define WALL_SENSOR                 1
#define CLIFFT_LEFT_SENSOR          2
#define CLIFFT_FRONT_LEFT_SENSOR    3
#define CLIFFT_FRONT_RIGHT_SENSOR   4
#define CLIFFT_RIGHT_SENSOR         5
#define VIRTUAL_WALL_SENSOR         6
#define MOTOR_OVERCURRENTS_SENSOR   7
#define DIRT_DETECTOR_LEFT_SENSOR   8
#define DIRT_DETECTOR_RIGHT_SENSOR  9
#define REMOTE_OPCODE_SENSOR        10
#define BUTTONS_SENSOR              11
#define DISTANCE_MSB_SENSOR         12
#define DISTANCE_LSB_SENSOR         13
#define ANGLE_MSB_SENSOR            14
#define ANGLE_LSB_SENSOR            15
#define CHARGING_STATE_SENSOR       16
#define VOLTAGE_MSB_SENSOR          17
#define VOLTAGE_LSB_SENSOR          18
#define CURRENT_MSB_SENSOR          19
#define CURRENT_LSB_SENSOR          20
#define TEMPERATURE_SENSOR          21
#define CHARGE_MSB_SENSOR           22
#define CHARGE_LSB_SENSOR           23
#define CAPACITY_MSB_SENSOR         24
#define CAPACITY_LSB_SENSOR         25

//Battery Charging States
#define CHARGING_STATE_NO_CHARGING          0
#define CHARGING_STATE_CHARGING_RECOVERY    1
#define CHARGING_STATE_CHARGING             2
#define CHARGING_STATE_TRICKLE_CHAGING      3
#define CHARGING_STATE_WAITING              4
#define CHARGING_STATE_CHARGING_ERROR       5

//Commands
#define COMMAND_SAFE    131
#define COMMAND_FULL    132
#define COMMAND_POWER   133
#define COMMAND_SPOT    134
#define COMMAND_CLEAN   135
#define COMMAND_MAX     136
#define COMMAND_DRIVE   137
#define COMMAND_MOTORS  138
#define COMMAND_LEDS    139
#define COMMAND_SONG    140
#define COMMAND_PLAY    141
#define COMMAND_SENSORS 142
#define COMMAND_DOCK    143

//Number of parameters of Led commands
#define LEDS_NUM_PARAMETERS         3

//Song Notes

//Note duration
///British names
#define NOTE_DURATION_SEMIQUAVER        16      //semicorchea
#define NOTE_DURATION_QUAVER            32      //corchea
#define NOTE_DURATION_CROTCHET          64      //negra
///US names
#define NOTE_DURATION_SIXTEENTH_NOTE    NOTE_DURATION_SEMIQUAVER
#define NOTE_DURATION_EIGHTH_NOTE       NOTE_DURATION_QUAVER
#define NOTE_DURATION_QUARTER_NOTE      NOTE_DURATION_CROTCHET

//Led Control MASKS
#define LED_CLEAN_ON                0x04
#define LED_CLEAN_OFF               0xFB
#define LED_SPOT_ON                 0x08
#define LED_SPOT_OFF                0xF7
#define LED_DIRT_ON                 0x01
#define LED_DIRT_OFF                0xFE
#define LED_MAX_ON                  0x02
#define LED_MAX_OFF                 0xFD
#define LED_STATUS_OFF              0x0F
#define LED_STATUS_AMBAR            0x30
#define LED_STATUS_RED              0x10
#define LED_STATUS_GREEN            0x20

//Cleaning Motors Control MASKS
#define SIDE_BRUSH_ON               0x01
#define SIDE_BRUSH_OFF              0xFE
#define VACUUM_ON                   0x02
#define VACUUM_OFF                  0xFD
#define MAIN_BRUSH_ON               0x04
#define MAIN_BRUSH_OFF              0xFB
#define ALL_CLEANING_MOTORS_ON      0xFF
#define ALL_CLEANING_MOTORS_OFF     0x00

#endif // ROOMBA_H
