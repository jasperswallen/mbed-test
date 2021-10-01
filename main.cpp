#include <mbed.h>

#include "EthernetInterface.h"

#include "ConsoleRedirection.hpp"

static ConsoleOutput networkConsole;
static ConsoleRedirection console(networkConsole);

namespace mbed
{
/**
 * @brief Override the default console (for printf) to use a CustomFileHandle
 */
FileHandle *mbed_override_console(int fd)
{
    return &console;
}
}; // namespace mbed

/* Allow for manually printing over USBTX and RX for testing */
BufferedSerial serial(USBTX, USBRX, 115200);
FILE *pc;

EthernetInterface ethernet;

SocketAddress mbedIP("192.168.1.20");
SocketAddress pcIP("192.168.1.214", 6144);
SocketAddress subnetMask("255.255.255.0");
SocketAddress defGateway("192.168.1.254");
SocketAddress bindAddr("0.0.0.0", 6144);

void networkStatusCallback(nsapi_event_t event, intptr_t parameter)
{
	if(event == NSAPI_EVENT_CONNECTION_STATUS_CHANGE)
	{
		auto newStatus = static_cast<nsapi_connection_status_t>(parameter);
		char const * message;
		switch(newStatus)
		{
			case NSAPI_STATUS_LOCAL_UP:
				message = "Local IP address set";
				break;
			case NSAPI_STATUS_GLOBAL_UP:
				message = "Global IP address set";
				break;
			case NSAPI_STATUS_DISCONNECTED:
				message = "No connection to network";
				break;
			case NSAPI_STATUS_CONNECTING:
				message = "Connecting to network";
				break;
			default:
				message = "Unknown status";
		}

		fprintf(pc, "Connection status change: %s (%d)\n", message, newStatus);
	}
}

constexpr size_t MAX_MSG_LEN = 1024;
char receiveBuffer[MAX_MSG_LEN + 1];
void receiverThread(UDPSocket * socket)
{
	SocketAddress senderAddr;

	printf("Receiver loop started\n");
	while(true)
	{
		int recvRet = socket->recvfrom(&senderAddr, receiveBuffer, MAX_MSG_LEN);
		if(recvRet > 0)
		{
			receiveBuffer[recvRet] = '\0';
			printf("Packet from \"%s\": %s\n", senderAddr.get_ip_address(), receiveBuffer);
		}
		else{
			printf("Receive error %d\n", recvRet);
		}
	}
}

int main()
{
    pc = fdopen(&serial, "r+");

	ethernet.set_network(mbedIP, subnetMask, defGateway);
	ethernet.attach(networkStatusCallback);

	// prevent connect() from blocking for the timeout (60s) if there is no network
	ethernet.set_blocking(false);

	nsapi_error_t connectResult = ethernet.connect();
	if(connectResult != NSAPI_ERROR_OK)
	{
		printf("Failed to connect to Ethernet network! Err %d\n", connectResult);
	}

	// create socket
	UDPSocket hostSocket;
	nsapi_error_t socketResult = hostSocket.open(&ethernet);
	if(socketResult != NSAPI_ERROR_OK)
	{
		printf("Failed to create socket! Err %d\n", socketResult);
	}
	hostSocket.bind(bindAddr);

	// start receiver thread
	Thread receiver;
	receiver.start(callback(receiverThread, &hostSocket));

    fprintf(pc, "1st: %u\n", console.getNetworkConsole().length);

    printf("Hello\r\n");

    fprintf(pc, "2nd: %u\n", console.getNetworkConsole().length);

    printf("Hello Again!\r\n");

    fprintf(pc, "3rd: %u\n", console.getNetworkConsole().length);

    fprintf(pc, "%.*s", console.getNetworkConsole().length, reinterpret_cast<const char *>(console.getNetworkConsole().buf));

    ThisThread::sleep_for(2s);

	// start transmitter
	fprintf(pc, "Transmitter loop started\n");
	while(true)
	{
    	printf("Will transmit: %u bytes\n", console.getNetworkConsole().length);
		hostSocket.sendto(pcIP, console.getNetworkConsole().buf, console.getNetworkConsole().length);
    	fprintf(pc, "Transmitted %u bytes\n", console.getNetworkConsole().length);
        fprintf(pc, "Buffer was: %.*s", console.getNetworkConsole().length, reinterpret_cast<const char *>(console.getNetworkConsole().buf));
        console.flushNetworkConsole();

        serial.sync();

		ThisThread::sleep_for(1000ms);
	}



}
