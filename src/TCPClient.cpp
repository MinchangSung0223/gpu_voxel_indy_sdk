#include "TCPClient.h"
#include <iostream>


TCPClient::TCPClient(): _isRunning(false){
}
TCPClient::~TCPClient(){
	deactivate();
}
bool TCPClient::deactivate()
{
    Poco::Thread::sleep(1000);
    _isRunning = false;

    if (_thread.isRunning())
    {
        _thread.yield();
        _thread.join();
    }
    return true;
}

bool TCPClient::activate()
{
 
    _isRunning = true;
    _thread.start(*this);
    try{
    		cout << "Trying to connect server..." << endl;
	    _ss.connect(SocketAddress(_hostname, _PORT));

		timeout = Timespan(1,0);
		while (_ss.poll(timeout, Poco::Net::Socket::SELECT_WRITE) == true)

		{

			cout << "Connecting to server..." << endl;

		}

		cout << "Complete to connect server" << endl;

    }catch (Poco::Exception& exc){
    		cout << "Fail to connect server..." << exc.displayText() << endl;
    		return false;
    }
    return true;
}
bool TCPClient::activate(const std::string hostname,const Poco::UInt16 PORT)
{
 
    _isRunning = true;
    _thread.start(*this);
    _hostname = hostname;
    _PORT = PORT;
    try{
    		cout << "Trying to connect server..." << endl;
	    _ss.connect(SocketAddress(_hostname, _PORT));

		timeout = Timespan(1,0);
		while (_ss.poll(timeout, Poco::Net::Socket::SELECT_WRITE) == true)

		{

			cout << "Connecting to server..." << endl;

		}

		cout << "Complete to connect server" << endl;

    }catch (Poco::Exception& exc){
    		cout << "Fail to connect server..." << exc.displayText() << endl;
    		return false;
    }
    return true;
}
void TCPClient::run()
{

	Data data_recv, data_send;
	unsigned char readBuff[BUFF_SIZE];
	unsigned char writeBuff[BUFF_SIZE];
	data_send.float6dArr[0] = 100;
    	memcpy(writeBuff, data_send.byte, SIZE_DATA_MAX);
	_ss.sendBytes(writeBuff, BUFF_SIZE);
	_ss.receiveBytes(readBuff, BUFF_SIZE);
	 memcpy(data_recv.byte, readBuff, SIZE_DATA_MAX);
	 now_q[0] = data_recv.float6dArr[1];
	 now_q[1] = data_recv.float6dArr[2];
	 now_q[2] = data_recv.float6dArr[3];
	 now_q[3] = data_recv.float6dArr[4];
	 now_q[4] = data_recv.float6dArr[5];
	 now_q[5] = data_recv.float6dArr[6];
	 
	 
	while(_isRunning){
		data_send.float6dArr[0] = 107;
		data_send.float6dArr[1] = send_q[0];
		data_send.float6dArr[2] = send_q[1];
		data_send.float6dArr[3] = send_q[2];
		data_send.float6dArr[4] = send_q[3];
		data_send.float6dArr[5] = send_q[4];
		data_send.float6dArr[6] = send_q[5];

		data_send.float6dArr[7] = send_qdot[0];
		data_send.float6dArr[8] = send_qdot[1];
		data_send.float6dArr[9] = send_qdot[2];
		data_send.float6dArr[10] = send_qdot[3];
		data_send.float6dArr[11] = send_qdot[4];
		data_send.float6dArr[12] = send_qdot[5];
	    	memcpy(writeBuff, data_send.byte, SIZE_DATA_MAX);
		_ss.sendBytes(writeBuff, BUFF_SIZE);
		_ss.receiveBytes(readBuff, BUFF_SIZE);
		 memcpy(data_recv.byte, readBuff, SIZE_DATA_MAX);
		 now_q[0] = data_recv.float6dArr[1];
		 now_q[1] = data_recv.float6dArr[2];
		 now_q[2] = data_recv.float6dArr[3];
		 now_q[3] = data_recv.float6dArr[4];
		 now_q[4] = data_recv.float6dArr[5];
		 now_q[5] = data_recv.float6dArr[6];
		Poco::Thread::sleep(0.001);
	}
}

void TCPClient::set(J q,J qdot){
	for(int i= 0;i<JOINTNUM;i++){
		send_q[i] = q.at(i);
		send_qdot[i] = qdot.at(i);
	}
}
J TCPClient::get(){
	J q;
	for(int i= 0;i<JOINTNUM;i++){
		q.at(i) = now_q[i];
	}
	return q;
}