#!/bin/bash
g++ servo_server.cpp -o server -L /usr/local/lib -I /usr/local/include -I ./PCA9685 -I ../include -Wall -fPIC -lPCA9685  -Wconversion-null   -O3  -pthread -lpthread -std=c++11
