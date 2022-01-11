#ifndef _MODEM4G_H_
#define _MODEM4G_H_

//#define G4_Modem_STATE_RFON 1
//#define G4_Modem_STATE_RFOFF 0
#define G4_TCP_STATE_CONNECT 1
#define G4_TCP_STATE_DISCONNECT 0
#define G4_TCP_STATE_IDLE 89
#define  G4_Modem_STATE_FIRSTINIT 98

 
 int G4_init();
 void G4_tcp_task();
 
#endif