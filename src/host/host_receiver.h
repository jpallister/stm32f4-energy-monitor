#ifndef __HOST_RECEIVER_H__
#define __HOST_RECEIVER_H__
using namespace std;

void cmd_connect();
void cmd_connect_to(string connect_to);

void cmd_getserial();
void cmd_setserial(string new_serial);

void cmd_setresistor(string new_resist);
void cmd_setrefvoltage(string new_voltage);
void cmd_setgain(string new_gain);

void cmd_trigger(string trigger);

void cmd_leds();

void cmd_start();
void cmd_start_with_file(string output_file);
void cmd_stop();

void cmd_power();
void cmd_power_set(string power);

void cmd_mode(string new_mode);

unsigned long long cmd_getenergy();

void cmd_help();

void cmd_exit();
bool cmd_is_running();

void processCommand(string input);

#endif /* __HOST_RECEIVER_H__ */
