#include "JoySticks.h"

UJoysticks::~UJoysticks(void) {
	if (GamePad) {
		GamePad->Unacquire();
	}
}

struct DI_ENUM_CONTEXT
{
	LPDIRECTINPUTDEVICE8* gamepad;
	LPDIRECTINPUT8* directinput;
	int joynum;
};

struct MJoystick
{
	LPDIRECTINPUTDEVICE8* gamepad;
	LPDIRECTINPUT8* directinput;
	int joynum;
};

int  UJoysticks::Init(int joyNumIn, LONG nMin, LONG nMax) {

	Min = nMin;
	Max = nMax;
	JoyNum = joyNumIn;

	DI_ENUM_CONTEXT  enumContext;
	enumContext.gamepad = &GamePad;
	enumContext.directinput = &DirectInput;
	enumContext.joynum = JoyNum;

	HRESULT temp_hrResult;
	// DirectInput initialize
	if (FAILED(temp_hrResult = DirectInput8Create(GetModuleHandle(NULL),
		DIRECTINPUT_VERSION, IID_IDirectInput8, (VOID * *)& DirectInput, NULL))) {
		return ERROR_INITIALIZE;
	}

	// search for joystick
	if (FAILED(temp_hrResult = DirectInput->EnumDevices(DI8DEVCLASS_GAMECTRL, enumCallback,
		&enumContext, DIEDFL_ATTACHEDONLY))) {
		return ERROR_NO_DEVICE;
	}

	// check for 
	if (GamePad == NULL) {
		cout << "Device not found!" << endl;
		system("pause");
		return E_FAIL;
	}
	else {
		cout << "Device found!" << endl;
		displayDeviceName();
	}

	if (FAILED(temp_hrResult = GamePad->SetDataFormat(&c_dfDIJoystick))) {
		return ERROR_SET_JOYSTICK;
	}


	// count joystick axes
	device_capabilities.dwSize = sizeof(DIDEVCAPS);
	if (FAILED(temp_hrResult = GamePad->GetCapabilities(&device_capabilities))) {
		return ERROR_CANT_GET_CAPABILITIES;
	}

	if (FAILED(temp_hrResult = GamePad->EnumObjects(enumAxesCallback, (VOID*)this, DIDFT_AXIS))) {
		return ERROR_ENUM_OBJECTS;
	}

	return 0;
}

void UJoysticks::displayDeviceName() {
	DIDEVICEINSTANCE device_info;
	device_info.dwSize = sizeof(DIDEVICEINSTANCE);
	GamePad->GetDeviceInfo(&device_info);
	//wchar_t* device_name = device_info.tszProductName;
	//wprintf(device_name);
	printf("%s", device_info.tszProductName);
	cout << endl;
}

// collback from joystick
BOOL CALLBACK  UJoysticks::enumCallback(const DIDEVICEINSTANCE* instance, VOID* context)
{
	DI_ENUM_CONTEXT* pEnumContext = (DI_ENUM_CONTEXT*)context;
	if (pEnumContext->joynum-- != 0)
	{
		return DIENUM_CONTINUE;
	}
	HRESULT device;

	// initialize joystick

	device = (*(pEnumContext->directinput))->CreateDevice(instance->guidInstance, pEnumContext->gamepad, NULL);

	// No joystick on the system
	if (FAILED(device)) {
		return DIENUM_CONTINUE;
	}
	return DIENUM_STOP;
}

// get information about x,y axis state
BOOL CALLBACK  UJoysticks::enumAxesCallback(const DIDEVICEOBJECTINSTANCE* instance, VOID* context)
{
	UJoysticks* js = (UJoysticks*)context;


	DIPROPRANGE propRange;
	propRange.diph.dwSize = sizeof(DIPROPRANGE);
	propRange.diph.dwHeaderSize = sizeof(DIPROPHEADER);
	propRange.diph.dwHow = DIPH_BYID;
	propRange.diph.dwObj = instance->dwType;
	propRange.lMin = js->Min;
	propRange.lMax = js->Max;


	if (FAILED(js->GamePad->SetProperty(DIPROP_RANGE, &propRange.diph))) {
		return DIENUM_STOP;
	}

	return DIENUM_CONTINUE;
}

// get information about button and x,y axis state
HRESULT  UJoysticks::getState(DIJOYSTATE* js)
{
	HRESULT  device_result;

	if (GamePad == NULL) {
		return S_OK;
	}

	// check joystick isn't bussy
	device_result = GamePad->Poll();
	if (FAILED(device_result)) {

		// fix input stream
		device_result = GamePad->Acquire();
		while (device_result == DIERR_INPUTLOST) {
			device_result = GamePad->Acquire();
		}

		// connection lost
		if ((device_result == DIERR_INVALIDPARAM) || (device_result == DIERR_NOTINITIALIZED)) {
			return E_FAIL;
		}

		// wait for joystick
		if (device_result == DIERR_OTHERAPPHASPRIO) {
			return S_OK;
		}
	}

	// get device state 
	if (FAILED(device_result = GamePad->GetDeviceState(sizeof(DIJOYSTATE), js))) {
		return device_result;
	}

	return S_OK;
}