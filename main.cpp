#include "serialman.h"

#include <iostream>
#include <exception>
#include <string>
#include <thread>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;
using serial::Serial;

#include <algorithm>

SerialMan* g_serial = nullptr;
bool keepgoing = true;

void my_handler(int s)
{
	std::cout << "In handler\n";

	if(g_serial) 
	{ 
		g_serial->close();
		g_serial->quitPeriodicTask();
	}

	keepgoing = false;
}

int main()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	SerialMan *serial = new SerialMan("/dev/ttyUSB0", 9600);
	g_serial = serial;

	auto commThread = new std::thread(&SerialMan::runTask, serial);

	while(keepgoing)
	{
		sleep(1);
		Msg m = {'a', 'b'};
		serial->sendMsg(m);
		std::cout << "sent msg" << std::endl;
	}


	// try {

	// 	cout << "Creating serial object\n";
	// 	Serial serial("/dev/ttyUSB0", 19200);

	// 	cout << "Sending...\n";
	// 	serial.write("Hello World!");

	// 	cout << "Receiving...\n";
	// 	int nb = 0;
	// 	uint8_t buffer[13];
	// 	buffer[12] = 0;

	// 	while(nb < 12)
	// 	{
	// 		int x = serial.read(buffer, 12 - nb);
	// 		if(x > 0)
	// 		{
	// 			cout << buffer << endl;
	// 			nb += x;
	// 		}
	// 	}

	// 	cout << "Quiting...\n";
	// 	serial.close();		
	// } catch(serial::IOException e) {
	// 	cout << "IO exception\n";
	// } catch(std::invalid_argument e) {
	// 	cout << "invalid arg exception\n";
	// } catch(serial::SerialException e) {
	// 	cout << "Serial exception\n" << e.what();
	// }


	return 0;

}