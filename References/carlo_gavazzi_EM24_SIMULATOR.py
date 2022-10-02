#!/bin/python
#https://www.enika.eu/data/files/produkty/energy%20m/CP/em24%20ethernet%20cp.pdf
#https://github.com/victronenergy/dbus-modbus-client/blob/master/carlo_gavazzi.py

from pyModbusTCP.server import ModbusServer, DataBank
from time import sleep
import logging

# Create an instance of ModbusServer
server = ModbusServer("0.0.0.0", 5020, no_block=True)

logging.getLogger('pyModbusTCP.server').setLevel(logging.DEBUG)

def getValues():
    ret = {
        12: 1653,  #//Carlo Gavazzi identification code UIN
        771: 1, #// Version and revision code of measurement module
        773: 1, #// Version and revision code of communication module
        4099: 0,  #// Measuring system  (3"1Ph", 0=“3P.n”)
        51: 0,#// Phase sequence 
        52: 53, #// Hz
        79: 0, #// KwH - TOT
        1: 2500, #// V L1-N
        13: 1350, #// A L1 
        3: 2500, #// V L2-N
        15: 0, #// A L2 
        5: 2500, #// V L3-N
        17: 0, #// A L3 
        41217: 1 #//Front selector status 
    }
    ret[19] = (ret[1]) * (ret[13]/1000) #// W L1
    ret[21] = (ret[3]) * (ret[15]/1000) #// W L2
    ret[23] = (ret[5]) * (ret[17]/1000) #// W L3
    ret[65] = ret[19] #// KWh(+) L1
    ret[67] = ret[21] #// KWh(+) L2
    ret[69] = ret[23] #// KWh(+) L3
    ret[53] = ret[65] + ret[67] + ret[69] #// KwH + TOT
    ret[41] = ret[19] + ret[21] + ret[23] #// Watt


    return(ret)

try:
    print("Start server...")
    server.start()
    print("Server is online")
    state = [0]
    while True:
        kv_set = getValues()
        for k in kv_set:
            server.data_bank.set_holding_registers(k-1, [kv_set[k]])
        sleep(1.0)

except Exception as e: 
    print(e)
    print("Shutdown server ...")
    server.stop()
    print("Server is offline")
