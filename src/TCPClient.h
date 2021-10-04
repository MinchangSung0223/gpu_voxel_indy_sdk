#ifndef TCPCLIENT_H_
#define TCPCLIENT_H_
#include <Poco/Runnable.h>
#include <Poco/Thread.h>
#include <Poco/Net/StreamSocket.h>
#include "spline.h"

#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <array>
#define JOINTNUM 6
 using namespace std;
 
using Poco::Net::StreamSocket;
using Poco::Net::SocketAddress;
using Poco::Timespan;
 typedef std::vector<std::array<double,JOINTNUM>> JT;
 typedef std::array<double,JOINTNUM> J;
class TCPClient : public Poco::Runnable{
	enum {
		SIZE_HEADER = 52,
		SIZE_COMMAND = 4,
		SIZE_HEADER_COMMAND = 56,
		SIZE_DATA_MAX =52,
		SIZE_DATA_ASCII_MAX = 32,
		BUFF_SIZE= 52,
		DATA_SIZE = 13
	};
	union Data{
		unsigned char byte[SIZE_DATA_MAX];
		float float6dArr[DATA_SIZE];
	};
	 std::string _hostname = "127.0.0.1"; //localhost IP Address
	 Poco::UInt16 _PORT = 9911;

private:
    bool _isRunning;
    double send_q[JOINTNUM];
    double send_qdot[JOINTNUM];
    double now_q[JOINTNUM];
    StreamSocket _ss;    
    Timespan timeout;    
protected:
    Poco::Thread _thread;
public:
	TCPClient();
	
	virtual ~TCPClient();
	bool activate();
	bool activate(const std::string hostname,const Poco::UInt16 PORT);
	bool deactivate();
	void waitForEvent();
	void set(J q,J qdot);
	J get();
	virtual void run();



};
#endif /* TCPCLIENT_H_ */
