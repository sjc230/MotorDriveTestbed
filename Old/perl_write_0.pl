BEGIN {
	# Add the @INC path the directory where the DSSClient module is found.
	#push ("/Users/by51849/workspace_v12/Control_ACIM_F28335_v1/", "dss");
}
use strict;
use warnings;
use DSSClient;


my $client = new DSSClient("127.0.0.1", 4444);
my $cmd = undef;
my $result = undef;

# Connect to the CCS json server.
$client->open();

#Send commands to DSS Test Server
#----------------


# Write a 32 bit value to memory
$cmd = {
	"name" => "writeData",
	"page" => 0,
	"address" => 0x0000C00C,
	"value" => 0,
	"typeSize" => 32,
};
execute_command($cmd);


# We are done now.
#$client->close();

# execute command
sub execute_command
{
    $result = $client->execute($_[0]);
    
    if (defined $result) {
        print "$_[0]{name}: ". $result->{"status"} . "\n";  
        # If there is a message, print it
        if (exists $result->{"message"} ) {           
        	print "  message: " . $result->{"message"} . "\n";
        }
        # If a value was returned, print it
        if (exists $result->{"value"} ) {           
        	print "  value: " . $result->{"value"} . "\n";
        }
    } else {
        print "$_[0]{name} execution failed\n";
    }
}