#pragma once



#include <iostream>
#pragma comment(lib, "dinput8.lib") 
#pragma comment(lib, "dxguid.lib") 
#include <dinput.h>

using namespace std;

enum
{
	ERROR_INITIALIZE = 100,
	ERROR_NO_DEVICE = 101,
	ERROR_SET_JOYSTICK = 102,
	ERROR_CANT_GET_CAPABILITIES = 103,
	ERROR_ENUM_OBJECTS = 104
};

class UJoysticks
{
public:
	//UJoysticks(void);
	~UJoysticks(void);
public:
	void			displayDeviceName();
	int			Init(int joyNumIn = 0, LONG nMin = -1000, LONG nMax = 1000); //初始化函数
	HRESULT			getState(DIJOYSTATE* js);
	//枚举设备
	static BOOL CALLBACK	enumCallback(const DIDEVICEINSTANCE* instance, VOID* context);
	static BOOL CALLBACK	enumAxesCallback(const DIDEVICEOBJECTINSTANCE* instance, VOID* context);

private:
	LPDIRECTINPUT8 DirectInput;
	LPDIRECTINPUTDEVICE8 GamePad;

	DIDEVCAPS device_capabilities;

	LONG Min;
	LONG Max;
	int JoyNum;

};