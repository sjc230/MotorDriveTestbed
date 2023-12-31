// Original code found here: https://e2e.ti.com/support/tools/code-composer-studio-group/ccs/f/code-composer-studio-forum/388665/testserver-javascript-doesn-t-work-when-launched-from-scripting-console
// Modified by S. Coshatt

// Import the DSS packages into our namespace to save on typing
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
importClass(java.lang.Thread,java.lang.Runnable,java.lang.System);

// Import necessary packages for network interaction.
importPackage(java.net);
importPackage(java.io);

//load(java.lang.System.getenv("SCRPITING_ROOT") + "/examples/TestServer/json2.js"); // path changed by S. Coshatt
load("C:/ti/ccs1210/ccs/ccs_base/scripting/examples/TestServer/json2.js"); // C:\ti\ccs1210\ccs\ccs_base\scripting\examples\TestServer\json2.js

/* What this program needs to know in order to work _properly_
    (1) Which config file for the board it will be bound to
    (2) Which logfile?
    (3) What ports? == ports in node config file?
    (4) What sessions? *.* for simulator xor Emulator cpu path
    (5) What timeout?
*/

/* This Object Oriented Javascript code. */
/* We define an object type TestServer that will handle access to all the sessions of DebugServer.

   var server = new TestServer("config_file.ccxml", 4444, "logfile.xml", 50000);
   server.run();
   server.shutdown();

   Timeout is an integer expressing time in milliseconds.

*/
function TestServer(board_configuration, base_port) {

    /* Get the enviroment instance for scripting */
    print("Server startup...");
    this.script = ScriptingEnvironment.instance();
    this.script.traceBegin(board_configuration.logFile, "C:/TestBed/Control_ACIM_F28335_v1/DefaultStylesheet.xsl"); // Modified by S. Coshatt
    this.script.traceSetConsoleLevel(TraceLevel.SEVERE);
    this.script.traceSetFileLevel(TraceLevel.ALL);
    this.script.setScriptTimeout(board_configuration.timeout);

    // Open a CCS Session to bring up the CCS GUI which will share the same debug context // Added by S.Coshatt
    this.ccsServer = this.script.getServer("CCSServer.1");//.openSession(".*"); 
    this.ccsSession = this.ccsServer.openSession(".*");

    /* Create main DebugServer which will be used by ALL simulators/emulators */
    this.debugServer = this.script.getServer("DebugServer.1");
    print("  Server started.");
    this.debugServer.setConfig(board_configuration.configFile);//.openSession(".*");
    print("  Configuration Set.");
    this.debugSession = this.debugServer.openSession(".*");
    print("  Session Opened.");
    this.debugSession.target.connect();
    print("  Connected to Session.");

    // Load a program
    this.debugSession.memory.loadProgram("C:/TestBed/Control_ACIM_F28335_v1/Debug/Control_ACIM_F28335_v1.out"); // modified by S. Coshatt
    print("  Program Loaded.");

    /* Predefined handlers supported out-of-the-box on the TestServer sessions.
       More can be added very easily, using the addHandlers method of the TestServer. */
    this.handlers = {
        "stop":                 stopCommandHandler,
        "connect":              connectCommandHandler,
        "disconnect":           disconnectCommandHandler,
        "load":                 loadCommandHandler,
        "run":                  runCommandHandler,
        "runAsynch":            runAsynchCommandHandler,
        "halt":                 haltCommandHandler,
        "reset":                resetCommandHandler,
        "timeout":              timeoutCommandHandler,
        "redirectCIO":          redirectCIOCommandHandler,
        "setBreakpoint":        setBreakpointCommandHandler,
        "removeAllBreakpoints": removeAllBreakpointsCommandHandler,
        "loadRawFromFile":      loadRawFromFileCommandHandler,
        "saveRawToFile":        saveRawToFileCommandHandler,
        "loadDataFromFile":     loadDataFromFileCommandHandler,
        "saveDataToFile":       saveDataToFileCommandHandler,            
        "loadGel":              loadGelCommandHandler,
        "runGel":               runGelCommandHandler,
        "readData":             readDataCommandHandler,
        "writeData":            writeDataCommandHandler,
        "readDataArray":        readDataArrayCommandHandler,
        "writeDataArray":       writeDataArrayCommandHandler,                
    };

    this.file;
    this.redirectStatus = false;
    this.redirectErr = false;

    // Open the different sessions available
    /*
    this.sessions = [];
    for (var i = 0; i< board_configuration.sessions.length; i++) {
      this.sessions[i] = board_configuration.sessions[i].name;
    }
    this.ports = [];
    this.debugSessions = [];
    this.ccsSessions = []; // added by S. Coshatt
    this.serverSockets = [];
    this.threads = [];
    for(var i = 0; i < this.sessions.length; i++) {
      this.ports[i] = base_port + i;
      this.debugSessions[i] = this.debugServer.openSession(this.sessions[i]);
      this.ccsSessions[i] = this.ccsServer.openSession(this.sessions[i]); // added by S. Coshatt
      this.debugSession.memory.loadProgram("C:/Users/by51849/Desktop/Motor_Control/Control_ACIM_F28335_v1/Debug/Control_ACIM_F28335_v1.out"); // modified by S. Coshatt
      script.traceWrite("Program Loaded");  // modified by S. Coshatt
      // Setup one socket per session.
      this.serverSockets[i] = ServerSocket(this.ports[i]);
      // Create one thread to handle commands to each socket/session.
      this.threads[i] = this.createSessionThread(this.serverSockets[i], this.debugSessions[i]);
      print("  Session["+i+"] opened on port " + this.ports[i] + ".");
    }*/

    this.port = base_port;
    print("  Base Port.");
    this.serverSocket = ServerSocket(base_port);
    print("  Socket Created.");
    this.thread = this.createSessionThread(this.serverSocket, this.debugSession);
    print("  Thread Created.");

    print("  End of Test Server Function.");
}

TestServer.prototype.addHandlers = function(handlers) {
    /* Add more handlers to support more commands. "handlers" is an object. */
    for (var command in handlers) {
        /* Sanity check: the handler must be a function. */
        if (typeof handlers[command] === 'function') {
            this.handlers[command] = handlers[command];
        } else {
            print("Error: '" + command + "' is not a function and will not be added.");
        }
    }
}

TestServer.prototype.run = function () {
    this.thread.start();
    /*
    for(var i = 0; i < this.ports.length; i++) {
        this.threads[i].start();
    } */
    print("TestServer is ready.");
    // Wait for all the threads to end.
    this.thread.join();
    this.thread = null;
    /*
    for(var i = 0; i < this.ports.length; i++) {
        this.threads[i].join();
        this.threads[i] = null;
    } */
}

TestServer.prototype.shutdown = function () {
    /* Finished running the server */ // Modified by S. Coshatt
    this.debugSession.terminate();
    this.ccsSession.terminate();
    /*
    for(var i = 0; i < this.sessions.length; i++) {
        this.debugSessions[i].terminate();
    }
    */
    this.debugServer.stop();
    this.script.traceEnd();
    print("TestServer is stopped.");
}

TestServer.prototype.createSessionThread = function (socket, session) {
    var server = this;
    //print("************ Entered Test Server Loop ************")                  
    return new Thread(
        new Runnable() {
          run: function() {
              //   * Accept connection from Perl driver. * Read a line from the driver.
              //   * Do the job.                         * Pipe output to the Perl driver.    
              //print("************ in runnable function ************")        
              var keep_running = true;
              while(keep_running) {
                //print("************ in keep running while ************")
                var client = socket.accept();
                var input  = new BufferedReader(new InputStreamReader(client.getInputStream()));
                var output = new PrintWriter(client.getOutputStream(), true);
                print(">>>>> Accepted connection :["+client+"]");
            
                /* We keep reading commands from the socket until the socket is closed by the client. */
                var line = input.readLine();
                while (line != null) {
                    /* The line should contain a JSON-encoded string */
                    //print("************ in while not null loop ************")
                    try {
                       /* FIXME JSON code does not work here. */
                       //var command = JSON.parse(line);
                       print('COMMAND: ' + line);
                       var command = eval('(' + line + ')');
                    } catch (err) {
                        /* Here we catch any execution error that may occur during the handler execution. */
                        var response = JSON.stringify({ "status": "FAIL", "message": "failed while parsing JSON command: " + err.description, });
                        print("RESPONSE: " + response);
                        output.println(response);
                        command = undefined;
                    }
        
                    if (command) {
                      /* We do a basic sanity-check on the command before calling the appropriate handler. */
                      if (!command.name) {
                        /* No command name specified */
                        var response = JSON.stringify({ "status": "FAIL", "message": "No command name specified", });
                        print("RESPONSE: " + response);
                        output.println(response);
                      } else if (!(server.handlers[command.name])) {
                        /* Command name specified is not among the supported commands */
                        var response = JSON.stringify({ "status": "FAIL", "message": command.name + ": Unsupported command", });
                        print("RESPONSE: " + response);
                        output.println(response);
                      } else {
                        try {
                            /* We start by running the handler. */
                            var result = server.handlers[command.name](session, command);
                            /* Special case for the stop command, we set keep_running to false. */
                            if (command.name == "stop") {
                                keep_running = false;
                            }
                                                                                                                
                            /* Send back to the client a JSON-encode result, on one line. */
                            var response = JSON.stringify(result);
                            print("RESPONSE: " + response);                         
                            output.println(response);
                        } catch (err) {
                            /* Here we catch any execution error that may occur during the handler execution. */
                            var response = JSON.stringify({ "status": "FAIL", "message": command.name + ": exception - " + err.description, });
                            print("RESPONSE: " + response);
                            output.println(response);
                        }
                      }
                    }
                    line = input.readLine();
                }
                
                //print("************ Exiting while not null loop ************")
                /* We are done with this client, therefore we release all the resources allocated. */
                input.close();
                output.close();                     
                print(">>>>> Closed connection :["+client+"]");
                client.close();
                
                /* Detect if the target is still running, and halt it */
                if ( (session.target.isConnected()) && (!session.target.isHalted()) ){
                    session.target.halt();
                }
                                    
                /* This may be redundant since the stop command handler will also disconnect the target */              
                if (session.target.isConnected()) {
                    //session.target.disconnect();
                }
              }
            }
          }
     );
     //print("************ The first loop completed ************")
}


/* Predefined handlers supported out-of-the-box on the TestServer. */

function stopCommandHandler(session, command) {
    /* This command is used to tell the session to stop taking connection from new clients.
       Any additional actions that needs to be done when a stop command is issued can be done here. */ 
    if (session.target.isConnected()) {
        try {
            session.target.disconnect();
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }
    }
    return { "status": "OK" };
}

function connectCommandHandler(session, command) {
    /* Connect to the target. */
    if (!session.target.isConnected()) {
        try {
            session.target.connect();
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }
        return { "status": "OK" };
    } else {
        return { "status": "FAIL", "message": "Target is already connected", };
    }
}

function disconnectCommandHandler(session, command) {
    /* Disconnect to the target. */
    if (session.target.isConnected()) {
        try {
            session.target.disconnect();
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }
        return { "status": "OK" };
    } else {
        return { "status": "FAIL", "message": "Target is already disconnected", };
    }
}

function loadCommandHandler(session, command) {
    /* Load a program on the session. */
    if (session.target.isConnected()) {
        try {        
            session.memory.loadProgram(command.program);
        } catch (err) {
            return { "status": "FAIL", "message": "" + "" + err};                  
        }
        return { "status": "OK" };
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function runCommandHandler(session, command) {
    /* Run the session. */
    if (session.target.isConnected()) {
        try {
            session.target.run();
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }            
        return { "status": "OK" };
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function runAsynchCommandHandler(session, command) {
    /* Run the session. */
    if (session.target.isConnected()) {
        try {
            session.target.runAsynch();
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }            
        return { "status": "OK" };
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}


function haltCommandHandler(session, command) {
    /* Halt the session. */
    if (!session.target.isHalted()) {
        try {
            session.target.halt();
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }         
        return { "status": "OK" };
    } else {
        return { "status": "FAIL", "message": "Target is already halted", };
    }
}

function resetCommandHandler(session, command) {
    /* Reset the session. */
    if (session.target.isConnected()) {
        try {    
            session.target.reset();
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }
        return { "status": "OK" };
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function timeoutCommandHandler(session, command) {
    /* Adjust the session timeout. */   
    session.setScriptTimeout(command.timeout);
    return { "status": "OK" };
}

function redirectCIOCommandHandler(session, command) {
    /* Redirect the CIO output of the session to a text file. */
    if (session.target.isConnected()) {
        if (command.file) {
            try {
                session.beginCIOLogging(command.file); 
            } catch (err) {
                return { "status": "FAIL", "message": "" + err};                  
            }                
        } else {
            try {                
                session.endCIOLogging(); 
            } catch (err) {
                return { "status": "FAIL", "message": "" + err};                  
            } 
        } 
        return { "status": "OK" };
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function setBreakpointCommandHandler(session, command) {
    /* Set breakpoint on address or symbol */
    if (session.target.isConnected()) { 
        try {           
            var bpID = session.breakpoint.add(command.address); 
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                
        }           
        if (bpID != -1) {
            return { "status": "OK", "message": "breakpoint ID is " + bpID };
        } else {
            return { "status": "FAIL", "message": "breakpoint ID is " + bpID };
        }
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function removeAllBreakpointsCommandHandler(session, command) {
    /* Remove all breakpoints. */
    if (session.target.isConnected()) {     
        try {         
            var bpID = session.breakpoint.removeAll(); 
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }               
        return { "status": "OK" };
        
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function loadRawFromFileCommandHandler(session, command) {
    /* Load binary data from file to target memory. */
    if (session.target.isConnected()) {     
        try { 
            session.memory.loadRaw(command.page, command.address, command.file, command.wordSize, !!command.byteSwap);
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }           
        return { "status": "OK" };
        
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function loadDataFromFileCommandHandler(session, command) {
    /* Load *.dat data from file to target memory. */
    if (session.target.isConnected()) {  
        try { 
            session.memory.loadData(command.page, command.address, command.file, command.length);
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }           
        return { "status": "OK" };
        
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function saveRawToFileCommandHandler(session, command) {
    /* Save target memory contents to binary file. */
    if (session.target.isConnected()) {             
        try { 
            session.memory.saveRaw(command.page, command.address, command.file, command.length, command.wordSize, !!command.byteSwap);
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }           
        return { "status": "OK" };
        
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}

function saveDataToFileCommandHandler(session, command) {
    /* Save target memory contents to *.dat file. */
    if (session.target.isConnected()) { 
        try {        
            session.memory.saveData(command.page, command.address, command.file, command.length, command.ioFormat, !!command.append);
        } catch (err) {
            return { "status": "FAIL", "message": "" + err};                  
        }   
        return { "status": "OK" };
        
    } else {
        return { "status": "FAIL", "message": "Target is not connected", };
    }
}
    
function loadGelCommandHandler(session, command) {
    /* Load GEL file. */         
    try { 
        session.expression.evaluate('GEL_LoadGel("' + command.file + '")');
    } catch (err) {
        return { "status": "FAIL", "message": "" + err};                  
    }           
    return { "status": "OK" };
}

function runGelCommandHandler(session, command) {
    /* Run GEL expression. */         
    try { 
        session.expression.evaluate(command.expression);
    } catch (err) {
        return { "status": "FAIL", "message": "" + err};                  
    }           
    return { "status": "OK" };
}    

function readDataCommandHandler(session, command) {
    /* Read one integer value from memory. */
    try {     
        var value = session.memory.readData(command.page, command.address, command.typeSize, !!command.signed);
    } catch (err) {
        return { "status": "FAIL", "message": "" + err};                  
    }      
    return { "status": "OK", "value": value };
}

function writeDataCommandHandler(session, command) {
    /* Write one integer value to memory. */
    try {     
        session.memory.writeData(command.page, command.address, command.value, command.typeSize);
    } catch (err) {
        return { "status": "FAIL", "message": "" + err};                  
    }      
    return { "status": "OK" };
}

function readDataArrayCommandHandler(session, command) {
    /* Read multiple values from memory. */
    
    var values = new Array();
    
    try {     
        values = session.memory.readData(command.page, command.address, command.typeSize, command.numValues, !!command.signed);
    } catch (err) {
        return { "status": "FAIL", "message": "" + err};                  
    }      
    
    return { "status": "OK", "value": values.join(",") };
}

function writeDataArrayCommandHandler(session, command) {
    /* Write multiple values to memory. */
    var values = new Array();
    values = command.values.split(",");
    
    try {     
        session.memory.writeData(command.page, command.address, values, command.typeSize);
    } catch (err) {
        return { "status": "FAIL", "message": "" + err};                  
    }      
    return { "status": "OK" };
}


