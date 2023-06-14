# -*- coding: utf-8 -*-
"""
Created on Wed Dec  1 14:03:23 2021
@author: stephen coshatt
"""
from datetime import datetime

import paho.mqtt.client as mqtt
import sys, os
import asyncio

import socket as s
import time
import subprocess
import datetime

import DSSClient

from pathlib import Path

channel = sys.argv[1]

var = "C:/TestBed/Control_ACIM_F28335_v1/"


client = DSSClient.DSSClient("127.0.0.1", 4444)


# Connect to the CCS json server.
client.open()

# execute command
def execute_command(cmd):
    result = client.execute(cmd)
    
    if (result):

        print(str(cmd['name'])+": "+str(result['status']))
        # If there is a message, print it
        if ('message' in result.keys()):           
        	print ("  message: "+str(result['message'])+"\n")
        # If a value was returned, print it
        if ('value' in result.keys()):           
        	print("  value: "+str(result['value'])+"\n")
    else:
        print (str(cmd['name'])+" execution failed\n")

async def main():
    p = Path('.')

    print("Channel is ",channel)

        # create command file
    file_name = "motor_command"+channel+".txt"
    command_file = p / file_name
    if command_file.is_file() == False:
        f = open(file_name, "x")
        f.close()
    last_modification = command_file.stat().st_mtime

    m_file = "file"+channel

    # Connect to the target.
    cmd = {
        "name": "connect",
    }
    execute_command(cmd)
    
    # Load program to memory
    cmd = {
        "name" : "load",
        "program" : "C:/TestBed/Control_ACIM_F28335_v1/Debug/Control_ACIM_F28335_v1.out",
    }
    execute_command(cmd)
    
    # Execute program.
    #cmd = {
    #    "name": "runAsynch",
    #}
    #execute_command(cmd)
        
    motor_command = ''
    # Loop to check for motor board commands from subscriber.py
    while motor_command != '11':
        time.sleep(2)
        file_modification = command_file.stat().st_mtime
        if  file_modification > last_modification:
            last_modification = file_modification
            # get command from file            
            with open(file_name,"r") as f1:
                content = f1.read()
                motor_command = str(content.split('\n', 1)[0])
                print("motor command is:",motor_command)                
            #"""

            # Command Logic
            if motor_command == '0': # Stop attack, set attack init flag to 0
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C00C,
                    "value": 0,
                    "typeSize": 32,
                }
                execute_command(cmd)
            elif motor_command == '1': # Set attack init flag to 1
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C00C,
                    "value": 1,
                    "typeSize": 32,
                }
                execute_command(cmd)
            elif motor_command == '2': # Set attack init flag to 2
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C00C,
                    "value": 2,
                    "typeSize": 32,
                }
                execute_command(cmd)   
            elif motor_command == '3': # Set attack init flag to 3
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C00C,
                    "value": 3,
                    "typeSize": 32,
                }
                execute_command(cmd) 
            elif motor_command == '4': # Set attack init flag to 4
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C00C,
                    "value": 4,
                    "typeSize": 32,
                }
                execute_command(cmd) 
            elif motor_command == '5': # Set attack init flag to 5
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C00C,
                    "value": 5,
                    "typeSize": 32,
                }
                execute_command(cmd) 
            elif motor_command == '6': # Set attack init flag to 6
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C00C,
                    "value": 6,
                    "typeSize": 32,
                }
                execute_command(cmd) 
            elif motor_command == '7': # Set attack init flag to 7
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C00C,
                    "value": 7,
                    "typeSize": 32,
                }
                execute_command(cmd) 
            elif motor_command == '8': # Set Enable flag to 0
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C006,
                    "value": 0,
                    "typeSize": 32,
                }
                execute_command(cmd)
            elif motor_command == '9': # Set Enable flag to 1
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C006,
                    "value": 1,
                    "typeSize": 32,
                }
                execute_command(cmd)
            elif motor_command == '10': # HARD STOP: set to 1 for Emergency Stop
                # Write a 32 bit value to memory
                cmd = {
                    "name": "writeData",
                    "page": 0,
                    "address": 0x0000C008,
                    "value": 1,
                    "typeSize": 32,
                }
                execute_command(cmd)                  
            else:
                print("An invalid command was entered:",motor_command)
    #"""

 
    
#"""    
if __name__ == "__main__":
    asyncio.run(main()) 
#"""