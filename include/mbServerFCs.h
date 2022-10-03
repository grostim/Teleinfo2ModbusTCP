//#include "ModbusServerEthernet.h"

#define MBTCP_ID 1 // modbus TCP server ID
#define MBPV_MAX 0xA102

uint16_t getValue(int ModbusOffset);
ModbusMessage FC03(ModbusMessage request);
/*
ModbusMessage FC06(ModbusMessage request);
ModbusMessage FC16(ModbusMessage request);
ModbusMessage FC23(ModbusMessage request);
*/