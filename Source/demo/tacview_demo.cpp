
#include <iostream>

#include "../TacView/TacViewOutput.h"


using namespace std;

using namespace TacView;

int main() {
	TacViewOutput tacview;

	//状态初始化
	tacview.InitServer();
	tacview.InitOneObject(0, 10000001, "F-16", "Air+FixedWing", "ai_gaoyang", "Red",
		15.5, 9.7, 4.8, 127.0, 30.0, 20000.0,0.0,0.0,0);
	tacview.InitOneObject(1, 10000002, "F-16", "Air+FixedWing", "gaoyang", "Blue",
		15.5, 9.7, 4.8, 127.01, 30.001, 20000.0, 0.0, 0.0, 0);

	tacview.SendOneFrame(0.0);

	double index = 0;
	double dt = 0.1;
	int object_count = 2;
	while (index < 200) {
		// 每一帧

		tacview.OneFrameFlightState(0, 10000001, 1, tacview.state->object[0].coordinate_longitude,
			tacview.state->object[0].coordinate_latitude + 0.001, tacview.state->object[0].coordinate_altitude,
			tacview.state->object[0].coordinate_roll, tacview.state->object[0].coordinate_pitch,
			tacview.state->object[0].coordinate_yaw, 0, -1, 0, 0);

		tacview.OneFrameFlightState(1, 10000002, 1, tacview.state->object[1].coordinate_longitude,
			tacview.state->object[1].coordinate_latitude + 0.001, tacview.state->object[1].coordinate_altitude,
			tacview.state->object[1].coordinate_roll, tacview.state->object[1].coordinate_pitch,
			tacview.state->object[1].coordinate_yaw, 1, -1, 0, 0);


		if (index > 10) {
			tacview.OneFrameMissileState(2, 20000001, "Red", 1,
				tacview.state->object[2].coordinate_longitude, tacview.state->object[2].coordinate_latitude + 0.002,
				tacview.state->object[2].coordinate_altitude-30, 0, 0, 0, -1);
			object_count = 3;
		}
		else {
			tacview.state->object[2].coordinate_longitude = tacview.state->object[0].coordinate_longitude;
			tacview.state->object[2].coordinate_latitude = tacview.state->object[0].coordinate_latitude;
			tacview.state->object[2].coordinate_altitude = tacview.state->object[0].coordinate_altitude;
		}

		tacview.SendOneFrame(index);
		index += dt;
		Sleep(100);
		cout << index << endl;
	}

	return 0;
}
