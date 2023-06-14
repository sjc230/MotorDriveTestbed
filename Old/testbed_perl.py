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

from pathlib import Path

channel = '8' #sys.argv[1]

var = "C:/TestBed/Control_ACIM_F28335_v1/"

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

    # Load the program to memory
    pipe = subprocess.Popen(["perl", "perl_load_program.pl", var])
        
    motor_command = ''
    # Loop to check for motor board commands from subscriber.py
    while motor_command != '6':
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
                pipe = subprocess.Popen(["perl", "perl_write_0.pl", var])
            elif motor_command == '1': # Set attack init flag to 1
                pipe = subprocess.Popen(["perl", "perl_write_1.pl", var])  
            elif motor_command == '2': # Set Enable flag to 0
                pipe = subprocess.Popen(["perl", "perl_Enable_0.pl", var])
            elif motor_command == '3': # Set Enable flag to 1
                pipe = subprocess.Popen(["perl", "perl_Enable_1.pl", var])                 
            else:
                print("An invalid command was entered:",motor_command)
    #"""

 
    
#"""    
if __name__ == "__main__":
    asyncio.run(main()) 
#"""