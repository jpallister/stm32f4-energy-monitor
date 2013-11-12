#ifndef __HOST_RECEIVER_H__
#define __HOST_RECEIVER_H__
#include "libusbinterface.h"

void cmd_connect();
void cmd_connect_to(std::string connect_to);

void cmd_getserial();
void cmd_setserial(std::string new_serial);

void cmd_setresistor(std::string new_resist);
void cmd_setrefvoltage(std::string new_voltage);
void cmd_setgain(std::string new_gain);

void cmd_trigger(std::string trigger);

void cmd_leds();

void cmd_start();
void cmd_start_with_file(std::string output_file);
void cmd_stop();

void cmd_power();
void cmd_power_set(std::string power);

void cmd_mode(std::string new_mode);

LibusbInterface::accumulated_data cmd_getenergy();

void cmd_help();

void cmd_exit();
bool cmd_is_running();

void processCommand(std::string input);

#endif /* __HOST_RECEIVER_H__ */
