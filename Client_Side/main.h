#include "Serial_Connection/serial_port.h"
//#include "TCP_Client/tcp_client.h"
//#include "Autopilot_Interface/autopilot_interface.h"
#include "Translater/translater.h"
#include "TCP_Client/tcp_client.h"

#include "/home/oliver/MySoftware/AHSMAV/AhsMav/c_library_v2-master/ardupilotmega/mavlink.h"

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main();

// quit handler
Serial_Port *serial_port_quit;
Translater *translater_quit;
TCP_Client *tcp_client_quit;
//Autopilot_Interface *autopilot_interface_quit;
void quit_handler( int sig );
