#include "sys.h"
#include "modbus.h"

PublicPara Device;


void System_Init(void)
{
    Device.addr_rs485 = 0x00;
    Modbus_Init_Reg();
}
