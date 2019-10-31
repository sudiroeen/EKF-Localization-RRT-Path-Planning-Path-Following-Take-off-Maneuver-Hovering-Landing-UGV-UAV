#pragma once

extern "C" {
  #include "extApi.h"
}

string serverIP = "127.0.0.1";
int serverPort = 19999;
int clientID=simxStart((simxChar*)serverIP.c_str(),serverPort,true,true,2000,5);
