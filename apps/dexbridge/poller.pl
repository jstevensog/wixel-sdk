#! perl -w

use strict;
use Win32::SerialPort;

my $ob = Win32::SerialPort->new ('COM2') || die;
my $line=""

$ob->baudrate(9600) 	|| die "fail setting baud";
$ob->parity('none') 	|| die "fail setting parity";
$ob->databits(8) 	|| die "fail setting databits";
$ob->stopbits(1) 	|| die "fail setting stopbits";
$ob->write_settings 	|| die "no settings"; 
$ob->are_match("\r\n");

$obj->lookclear;
while true {
	until ("" ne $line) {
		$line=$pb->lookfor;
		die "Aborted.\n" unless(defined $line);
		last if($gotit);
		sleep1
	}
	printf("%s, %s\r\n", localtime, $string);
	$string="";
	$ob->lookclear;
}