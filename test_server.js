// Original code found here: https://e2e.ti.com/support/tools/code-composer-studio-group/ccs/f/code-composer-studio-forum/388665/testserver-javascript-doesn-t-work-when-launched-from-scripting-console
// Modified by S. Coshatt

// Import the TestServer code
load("C:/TestBed/Control_ACIM_F28335_v1/TestServer.js"); // Modified by S. Coshatt

/* Details of the board configuration.  */
board_configuration = {
  configFile:	"C:/TestBed/Control_ACIM_F28335_v1/TMS320F28335.ccxml", //C:\Users\by51849\workspace_v12\Control_ACIM_F28335_v1\TMS320F28335.ccxml
     timeout:	150000,
     logFile:	"C:/TestBed/Control_ACIM_F28335_v1/test_server.xml",  // paths changed by S. Coshatt
    sessions:	[
                    {
                        name:	".*",//"C:/Users/by51849/Desktop/Motor_Control/Control_ACIM_F28335_v1/Control_ACIM_F28335_v1", // Modified by S. Coshatt
                    },
                ],
};

/* Start the server, for the sessions start using TCP/IP ports from 4444.  */
var server = new TestServer(board_configuration, 4444);

print("************ Prior to Run ************")
/* Run and wait for completion of the TestServer.  */
server.run();
//server.runAsynch(); tested but doesn't work
server.shutdown();

java.lang.System.exit(0);
