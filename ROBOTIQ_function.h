/*
*   File:   ROBOTIQ_function.h
*	This Class contains ROBOTIQ gripper library.
*   Using library function to drive gripper. Input type will be int or double.
*	The variable is the parameter you want to set to SMC gripper,like position, velocity, force and so on.
*	Before use this class, please include all Protocol libraries and set the boost_1_54 library.
*   Boost library: http://blog.monkeypotion.net/gameprog/note/first-touch-of-boost-cpp-libraries
*
*	Author: Chieh-Chun Lin
*	Version: v1.4
*	History:
*			2014/02/11 set up all the functions      
*			2014/03/28 write the description about library										    
*           2014/05/05 revise using namespace std
                       revise protective function name
*           2014/06/22 revise the gripper order
*           2014/07/28 revise function description and add variables in functions
                       delete reference(&) in ROBOTIQ_set function
*			2014/07/30 revise the constructor function 
			           (add "parity::none" into myProtocol) 
*/

#include <string>
#include <iostream>
#include <vector>
#include <conio.h>
#include <boost/thread.hpp>
#include <boost/crc.hpp>      // for boost::crc_basic, boost::crc_optimal
#include <boost/cstdint.hpp>  // for boost::uint16_t
#include "MyRobot_AsyncSerial.h"
#include "MyRobot_Protocol.h"
#include "MyRobot_ProtocolDef.h"

typedef unsigned char uchar;

#ifndef ROBOTIQ_FUNCTION   
#define	ROBOTIQ_FUNCTION




class ROBOTIQ_function
{ 
public:	
    /*
	* ROBOTIQ_function constructor
	* >number of comport as srting type, like string "COM0"
	*/

    ROBOTIQ_function( std::string com_port );
	~ROBOTIQ_function();

public:
	MyRobot_Protocol* myProtocol;
	

public:
	/*
	* send data buffer
	*/
	std::vector<uchar> myData;
    /*
	* 驅動夾爪，並初始化夾爪(servo on)
	* Note:在使用其他函式前，必須先呼叫此函式將夾爪啟動
	       初始化的夾爪狀態為: pos=52.88 mm , vel=50.76 mm/s , force=57.45 N		   
	*/
    void ROBOTIQ_active_on();  
	/*
	* 關閉夾爪(servo off)
	* Note:在結束程式前，必須呼叫此函式結束夾爪
	*/
	void ROBOTIQ_active_off(); //
	/*
	* 讀取夾爪發生錯誤的敘述資訊
	* 將數值存放在FAULT的變數，轉成十六進位，可參照"2 finger gripper Manual" p.28的說明，得知夾爪目前的系統發生錯誤的相關敘述
	*/
	void ROBOTIQ_fault_read(int& fault);       
	/*
	* 讀取夾爪運動的狀態資訊
	* 得知position_request、position_now、current的三個數值
	*/
	void ROBOTIQ_action_status_read(double &pr,double &pn,double &current); 
	/*
	* 讀取夾爪系統狀態(active,goto,Gripper Status,Objective detection status)
	* Note: 可讀取夾爪系統的狀態資訊
	        得知Initialization status (gACT)、Action status (gGTO)、Gripper status (gIMC)、Object detection status(gOBJ)
			其四個變數，轉成二進位，可參照"2 finger gripper Manual" p.27的說明，得知夾爪目前的系統狀態
	*/
	void ROBOTIQ_gripper_status_read(int& Active,int& GoTo,int& GIMC,int& GOBJ);       
	/*
	* 僅設定夾爪的目標位置(pos)
	* Note:僅設定數值進暫存器，要使用goto function 夾爪才會移動
	       其速度、施力沿用之前設定，
	       若無設定則沿用初始化數值 vel=50.76 mm/s , force=57.45 N	
	*/
	void ROBOTIQ_set_pos(double pos); 
	/*
	* 設定夾爪的目標位置(pos)、夾取速度(vel)、夾取施力(force)
	* Note:僅設定數值進暫存器，要使用goto function 夾爪才會移動
	       夾爪限制 pos限制範圍:0(閉)-87(開) mm
	                vel限制範圍:19-100 mm/s
					force限制範圍:30-100 N
	*/
	void ROBOTIQ_set_all(double pos ,double vel, double force); 
	/*
	* goto function
	* Note:在使用ROBOTIQ_set的相關function，必須要呼叫goto，夾爪才會移動
	*/
	void ROBOTIQ_goto();            
	/*
	* 停止夾爪運動
	*/
	void ROBOTIQ_stop();            
	/*
	* 夾爪全開
	* Note: 其全開的寬度大於87mm
	        使用此函式，必須要重新servo on才能使用其他函式
	*/
	void ROBOTIQ_auto_release();     

private:
    void ROBOTIQ_fault(int&);   //fault description
	void ROBOTIQ_gIMC(int&);    //Gripper Status description
	void ROBOTIQ_gOBJ(int&);    //Objective detection status description
	std::string Hex_to_Binary(int);  //Hex to Binary

private:
     unsigned char* order;
};

#endif //ROBOTIQ_FUNCTION