@ECHO OFF
set "str1=Setting up testbed for motor"
set "str2=%~1"
set "str3=%str1% %str2%"
set "str4=node"
set "node=%str4% %str2%"

echo.%str3%

cd ..\..
cd

cd C:\ti\ccs1210\ccs\ccs_base\scripting\bin
cd 

start cmd.exe /k CALL dss C:\TestBed\Control_ACIM_F28335_v1\test_server.js

cd ..\..
cd

cd C:\TestBed\Control_ACIM_F28335_v1
cd

start cmd.exe /k CALL python subscriber.py %str2%

echo "Once the DSS Test Server outputs Test Server Ready to the command line screen, press any key with this command line open to coninue setup."

PAUSE

start cmd.exe /k CALL python testbed.py %str2%