/*
This is ROBOTIQ_function 1.4
*/

#include <string>
#include <conio.h>
#include <iostream>
#include <stdio.h>
#include "ROBOTIQ_function.h"
#include <vector>

typedef unsigned char uchar;
//
//Class Function
//

// constructor, set the initial value
ROBOTIQ_function::ROBOTIQ_function( std::string com_port ) 
{
    //object construction
	myProtocol=new MyRobot_Protocol( com_port,115200,boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));	
}
//夾爪指令

void ROBOTIQ_function::ROBOTIQ_active_on( ){ 
	myData.clear();
	unsigned char order[] = {0x09,0x06,0x03,0xE8,0x01,0x00}; 
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
		
    printf("\n");
	//std::system("pause");	
} 

void ROBOTIQ_function::ROBOTIQ_active_off(){ 
	myData.clear();
	unsigned char order[] = {0x09,0x06,0x03,0xE8,0x00,0x00};  
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

    printf("\n");
	//std::system("pause");
} 

void ROBOTIQ_function::ROBOTIQ_action_status_read(double &pr,double & pn,double & current ){ 
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD0,0x00,0x03};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	int pos_request = myData[6];
	pr = ((255-pos_request)*87/255);
	int pos_now = myData[7];
	pn = ((255- pos_now)*87/255); // change : 2023/09/17
	current = 0.1*myData[8];
	printf("目標位置= %2.2f mm , 現在位置= %2.2f mm , 電流= %d mA \n",pr,pn,current);
    printf("\n");
	//std::system("pause");
} 
void ROBOTIQ_function::ROBOTIQ_gripper_status_read(int& Active,int& GoTo,int& GIMC,int& GOBJ){

	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD0,0x00,0x03};
	int size = sizeof(order);
    for(int i = 0; i < size; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	int a = myData[3];
	int p = a/16;
	int q = a%16;
	std::string x = this->Hex_to_Binary(p);
	std::string y = this->Hex_to_Binary(q);
	//printf("十進位 = %d %d \n", p,q);
	//printf("二進位 = %s %s\n",x.c_str(),y.c_str());
	printf("ACTIVE: %c  GOTO: %c  Gripper Status: %c%c Objective detection status: %c%c \n",y[3],y[0],x[3],x[2],x[1],x[0]);
	int gOBJ = p/4;
	int gIMC = p%4;
	printf("Gripper Status: ");
	this->ROBOTIQ_gIMC(gIMC);
	printf("Objective detection status: ");
	this->ROBOTIQ_gOBJ(gOBJ);
	Active = y[3];
	GoTo = y[0];
	GIMC = x[2]*2+x[3] ;
	GOBJ = x[0]*2+x[1];
    printf("\n");
	//std::system("pause");

}
void ROBOTIQ_function::ROBOTIQ_fault_read(int& fault){ 

	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD0,0x00,0x03};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	int no = myData[5];
	printf("Fault no.= 0%X \n",no);
	this->ROBOTIQ_fault(no);
	fault = no;
    printf("\n");
	//std::system("pause");

} 

void ROBOTIQ_function::ROBOTIQ_set_pos(double pos){ 

	myData.clear();
	unsigned char order[] ={0x09,0x06,0x03,0xE9,0x00,0x00}; 
	int a = sizeof(order);

	//位置
	int p =(int)((255-(pos*255.0/87.0))+0.5);
	order[5]=p;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
	//std::system("pause");
	
} 
void ROBOTIQ_function::ROBOTIQ_set_all(double pos ,double vel, double force){ 

	myData.clear();
	unsigned char order[] ={0x09,0x10,0x03,0xE9,0x00,0x02,0x04,0x00,0x00,0x00,0x00}; 
	int a = sizeof(order);

	//位置
	int p = (int)((255-(pos*255.0/87.0))+0.5);
	order[8]=p;
	//速度
	int v = (int)((vel-19)*255/81+0.5);
	order[9]=v;
	//施力
	int f = (int)((force-30)*255/70+0.5);
	order[10]=f;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
	//std::system("pause");

}

void ROBOTIQ_function::ROBOTIQ_goto(){ 

	myData.clear();
	unsigned char order[] = {0x09,0x06,0x03,0xE8,0x09,0x00}; 
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
	//std::system("pause");
	
}
void ROBOTIQ_function::ROBOTIQ_stop(){ 

	myData.clear();
	unsigned char order[] = {0x09,0x06,0x03,0xE8,0x01,0x00};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
	std::system("pause");
	
}

void ROBOTIQ_function::ROBOTIQ_auto_release(){ 

	myData.clear();
	unsigned char order[] = {0x09,0x06,0x03,0xE8,0x11,0x00}; 
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
	std::system("pause");
	
}

//----------------------------------------------------

void ROBOTIQ_function::ROBOTIQ_fault(int &a){
	a = a%16;
	switch (a){
	case 0: 
		printf("No Fault\n"); 
		break;
	case 5: 
        printf("Action delayed, initialization must be completed prior to action\n");
		break;
	case 7:
		printf("The activation bit must be set prior to action\n");
		break;
	case 9:
	    printf("NetIC is not ready(may be booting\n");
		break;
	case 11:
		printf("Automatic release in progress\n");
		break;
	case 13:
		printf("Initialization fault\n");
		break;
	case 14:
		printf("Over current protection triggered\n");
		break;
	case 15:
		printf("Automatic release completed\n");
		break;
	default:
		printf("system error\n");
		break;
	}
}

void ROBOTIQ_function::ROBOTIQ_gIMC(int &a){
	switch (a){
	case 0: 
		printf("Gripper is in reset (or automatic release) state.\n"); 
		break;
	case 1: 
        printf("Activation in progress.\n");
		break;
	case 2:
		printf("No used.\n");
		break;
	case 3:
	    printf("Activation is completed.\n");
		break;
	default:
		printf("system error\n");
		break;
	}
}

void ROBOTIQ_function::ROBOTIQ_gOBJ(int &a){
		switch (a){
	case 0: 
		printf("Fingers are in motion.\n"); 
		break;
	case 1: 
        printf("Fingers have stopped due to a contact while opening.\n");
		break;
	case 2:
		printf("Fingers have stopped due to a contact while closing.\n");
		break;
	case 3:
	    printf("Fingers are at requested position.\n");
		break;
	default:
		printf("system error\n");
		break;
		}
}

std::string ROBOTIQ_function::Hex_to_Binary(int ch){
	char bin[5]={0};
	std::string tmp;
	switch (ch){
	case 0: 
		bin[0] ='0'; bin[1] ='0'; bin[2] ='0'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 1: 
		bin[0] ='0'; bin[1] ='0'; bin[2] ='0'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 2:
		bin[0] ='0'; bin[1] ='0'; bin[2] ='1'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 3:
	    bin[0] ='0'; bin[1] ='0'; bin[2] ='1'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 4:
		bin[0] ='0'; bin[1] ='1'; bin[2] ='0'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 5:
		bin[0] ='0'; bin[1] ='1'; bin[2] ='0'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 6:
		bin[0] ='0'; bin[1] ='1'; bin[2] ='1'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 7:
		bin[0] ='0'; bin[1] ='1'; bin[2] ='1'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 8:
		bin[0] ='1'; bin[1] ='0'; bin[2] ='0'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 9:
		bin[0] ='1'; bin[1] ='0'; bin[2] ='0'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 10:
		bin[0] ='1'; bin[1] ='0'; bin[2] ='1'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 11:
		bin[0] ='1'; bin[1] ='0'; bin[2] ='1'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 12:
		bin[0] ='1'; bin[1] ='1'; bin[2] ='0'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 13:
		bin[0] ='1'; bin[1] ='1'; bin[2] ='0'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 14:
		bin[0] ='1'; bin[1] ='1'; bin[2] ='1'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 15:
		bin[0] ='1'; bin[1] ='1'; bin[2] ='1'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	default:	
		bin[0] ='x'; bin[1] ='x'; bin[2] ='x'; bin[3] ='x';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	}
	
}


// deconstructor 
ROBOTIQ_function::~ROBOTIQ_function() 
{
	//serial port close
	myProtocol->close();

	delete myProtocol;
}
