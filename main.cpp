#pragma warning(disable:4996)
#include "KinectCamera.h"
#include "transform.h"
#include "pointCloudProcess.h"
#include <iostream>

using namespace std;

int main()
{
	kinectCamera kinect;
	if (!kinect.initKinect())
	{
		cout << "init kinect failed" << endl;
		return 1;
	}
	kinect.getData();
	transformer trans;
	for (vector<vector<float>>::const_iterator iter = nine_points_xyz.begin(); iter != nine_points_xyz.end(); iter++)
	{
		trans.addSourcePoints((*iter)[0], (*iter)[1], (*iter)[2]);
	}
	for (size_t i = 0; i < nine_points_xyz.size(); i++)
	{
		float temp_x, temp_y, temp_z;
		cout << "input point " << i << " x" << endl;
		cin >> temp_x;
		cout << "input point " << i << " y" << endl;
		cin >> temp_y;
		cout << "input point " << i << " z" << endl;
		cin >> temp_z;
		trans.addTargetPoints(temp_x, temp_y, temp_z);
	}
	/*trans.addSourcePoints(0.126901f, -0.054710f, 0.938f);
	trans.addSourcePoints(0.076113f, -0.057638f, 0.942f);
	trans.addSourcePoints(0.074728f, -0.081546f, 0.895f);
	trans.addTargetPoints(0.46022323f, 0.50710499f, 0.28645349f);
	trans.addTargetPoints(0.42473236f, 0.47370705f, 0.28595987f);
	trans.addTargetPoints(0.38551146f, 0.51143277f, 0.28599533f);*/
	trans.computerTranform();
	trans.convert_coordinate_to_robot(1.0f, 2.0f, 3.0f);
	return 0;
}