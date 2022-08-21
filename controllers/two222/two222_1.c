/*
 * File:          two222.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

 /*
  * You may need to add include files like <webots/distance_sensor.h> or
  * <webots/motor.h>, etc.
  */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <math.h>
#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/gps.h>

#define TIME_STEP 4
#define PI 3.1415926
#define RAD_TO_DEG(X) ( X / PI * 180.0)

double aimAngle = 0.2;//设定角度
double aimVelocity = 2.0;//设定速度
double stopPosition = 0;
double aimZ = 1;
double angle = 0;
double v = 0;
double errorAngle = 0;
double angleV = 0;
double accV = 0;
double z = 0;
double lastZ = 0;
double force = 0;
double angle_PID = 0;
double velocity_PID = 0; 
double position_PID = 0;
double innerAngle = 0;
double p_pid = 0;
double a_pid = 0;


double PositionSet(void);//停在一点
double AngleControl(void);//保持一个角度
double VelocitySet(void); //以平衡车的位置控制速度（速度PID）
//double AngelVelocitySet(void);//以平衡车的速度控制加速度(加速度PID)
double AngleSet(void);//以平衡车的加速度控制角度(角度PID)

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char** argv) {
	/*
	 *函数声明
	 */
	double BalanceControl(double angle, double gyro);
	double VelocityUpright(double velocity);
	double VelocityControl(double velocity, double angle_PID);
	double UprightControl(double angle);
	double innerUprightControl(double angle, double innerAngle);
	double PositionSet(void);
	double AngleControl(void);
	double VelocitySet(void);
	double AngleSet(void);

	/*
	 *necessary to initialize webots stuff
	 */
	wb_robot_init();

	/*
	 * You should declare here WbDeviceTag variables for storing
	 * robot devices like this:
	 *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
	 *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
	 */

	WbDeviceTag wheel1
		= wb_robot_get_device("wheel1");
	wb_motor_set_position(wheel1, INFINITY);
	wb_motor_set_velocity(wheel1, 0);
	wb_motor_enable_force_feedback(wheel1, 1);

	WbDeviceTag wheel2
		= wb_robot_get_device("wheel2");
	wb_motor_set_position(wheel2, INFINITY);
	wb_motor_set_velocity(wheel2, 0);
	wb_motor_enable_force_feedback(wheel2, 1);

	WbDeviceTag inertial_unit
		= wb_robot_get_device("inertial_unit");
	wb_inertial_unit_enable(inertial_unit, 1);

	WbDeviceTag gps
		= wb_robot_get_device("gps");
	wb_gps_enable(gps, 1);

	WbDeviceTag gyro
		= wb_robot_get_device("gyro");
	wb_gyro_enable(gyro, 1);

	WbDeviceTag acc
		= wb_robot_get_device("acc");
	wb_accelerometer_enable(acc, 1);

	/* main loop
	 * Perform simulation steps of TIME_STEP milliseconds
	 * and leave the loop when the simulation is over
	 */
	while (wb_robot_step(TIME_STEP) != -1) {
		/*
		 * Read the sensors :
		 * Enter here functions to read sensor data, like:
		 *  double val = wb_distance_sensor_get_value(my_sensor);
		 */

		 /*
		  *Process sensor data here
		  */
		accV = -wb_accelerometer_get_values(acc)[2];//平衡车加速度
		angleV = -wb_gyro_get_values(gyro)[0];//倾角速度
		angle = (wb_inertial_unit_get_roll_pitch_yaw(inertial_unit)[0]) * 57.3;//倾角
		z = wb_gps_get_values(gps)[2];
		static double ac = 0;
		ac = z;
		v = (z - lastZ) / 0.005;

		innerAngle = UprightControl(angle);
		a_pid = innerUprightControl(angle, innerAngle);
		p_pid = PositionSet();
		if (fabs(angle) > 0.6)
		{
			angle_PID = 0.1 * a_pid + 0.9 * p_pid;
		}
		if (fabs(angle) <= 0.6)
		{
		    angle_PID = 0.0001 * a_pid + 0.9999 * p_pid;
		}
		//angle_PID = 0.05 * a_pid + 0.95 * p_pid;能停
		printf("%f  %f  %f  %f  %f  %f\r\n", angle,angle_PID,a_pid,v,p_pid,z);
		lastZ = z;
		/*static double k[4] = { -10.9545, - 18.9388, 270.4765  , 40.3900 };
		force = -(k[0] * ac + k[1] * v + k[2] * error_angle + k[3] * angle_v);
		force = force / 2;*/
		//wb_motor_set_force(wheel1, angle_PID);
		//wb_motor_set_force(wheel2, angle_PID);
		//wb_motor_set_acceleration(wheel1, angle_PID);
		//wb_motor_set_acceleration(wheel2, angle_PID);
		wb_motor_set_velocity(wheel1, angle_PID);
		wb_motor_set_velocity(wheel2, angle_PID);
		fflush(stdout);

		/*
		 * Enter here functions to send actuator commands, like:
		 * wb_motor_set_position(my_actuator, 10.0);
		 */
	};

	/* Enter your cleanup code here */

	/* This is necessary to cleanup webots resources */
	wb_robot_cleanup();
	return 0;
}





/*
 *控制平衡车在设定角度周围(成功)直立环
 */
double UprightControl(double angle)
{
	//double kp = 15, ki = 0, kd = 0.5;
	//double kp = 15, ki = 0, kd = 0.5;
	//if (fabs(angle) < 0.4)
	//{s
	//	kp = 20, ki = 0, kd = 1;
	//}
	double kp = 15, ki = 0, kd = 0.5;
	if (fabs(angle) < 0.4)
	{
		kp = 25, ki = 0, kd = 1;
	}
	static double errorAngle, errorAngleSum, lastAngle, derrorAngle;
	static double errorAnglev, errorAnglevSum;
	double angleIntergral = 0;
	errorAngle = angle - (0);
	derrorAngle = angle - lastAngle;
	errorAngleSum += errorAngle;
	angleIntergral = kp * errorAngle + ki * errorAngleSum + kd * derrorAngle;
	lastAngle = angle;
	//printf("%f  %f  %f  %f  %f  %f\r\n", angle, angle_PID, accV, v, angleV, angleIntergral);
	return angleIntergral;
}
/*
 *控制平衡车在设定角度周围(成功)速度环
 */
double innerUprightControl(double angle, double innerAngle)
{
	//double kp = 50, ki = 0.8, kd = 10;
	//double kp = 50, ki = 0.8, kd  = 0;
	//if (fabs(angle) < 0.5)
	//{
	//	kp = 10, ki = 0.8, kd = 0;
	//}
	double kp = 50, ki = 0.5, kd = 0;
	//if (fabs(angle) < 0.4)
	//{
	//	kp = 10, ki = 0, kd = 0;
	//}
	static double inerrorAngle, inerrorAngleSum, inlastAngle, inderrorAngle;
	static double inerrorAnglev, inerrorAnglevSum;
	double inangleIntergral = 0;
	inerrorAngle = innerAngle - angle;
	inderrorAngle = angle - inlastAngle;
	inerrorAngleSum += inerrorAngle;
	inangleIntergral = kp * inerrorAngle + ki * inerrorAngleSum + kd * inderrorAngle;
	inlastAngle = angle;
	//printf("%f  %f  %f  %f  %f  %f\r\n", angle, angle_PID, accV, v, angleV, inangleIntergral);
	return inangleIntergral;
}

/*
 *控制平衡车以一个0.2角度直行
 */
double AngleControl(void)
{
	double kp = 3, ki = 0, kd = 20;
	static double errorAngle, errorAngleSum, lastAngle, derrorAngle;
	static double errorAnglev, errorAnglevSum;
	double angleIntergral = 0;
	errorAngle = angle - 10;
	derrorAngle = angle - lastAngle;
	errorAngleSum += errorAngle;
	angleIntergral = kp * errorAngle + ki * errorAngleSum + kd * derrorAngle;
	lastAngle = angle;
	return angleIntergral;
}

//

/*
 *平衡车的位置控制（位置PID）
 */
double PositionSet(void)
{
	//double kp = 3.825, kd = 0.01;
	double kp = 50, ki = 0, kd = 0.1;
	if (fabs(aimZ - z) < 0.1)
	{
		kp = 300, ki = 0, kd = 0.1;
	}	
	static double errorZ, errorzSum, lastZ, derrorZ;
	double positionOutput = 0;
	errorZ = aimZ - z;
	errorzSum += errorZ;
	derrorZ = z - lastZ;
	positionOutput = kp * errorZ + kd * derrorZ;
	return positionOutput;
	}

/*
 *平衡车的角度控制(角度PID)
 */
double AngleSet(void)
{
	double kp = 3, ki = 0, kd = 19;
	static double errorAngle, errorAngleSum, lastAngle, derrorAngle;
	double angleOutput;
	errorAngle = angle - 0;
	errorAngleSum += errorAngle;
	derrorAngle = angle - lastAngle;
	angleOutput = kp * errorAngle + ki * errorAngleSum + kd * derrorAngle;
	lastAngle = angle;
	return angleOutput;
}

/*
 * 以平衡车的位置和角度控制速度（速度PID）
 */
double VelocitySet(void)
{
	double PositionSet(void);
	double AngleSet(void);
	double AngleControl(void);
	//double kp = 8, ki = 0.43658, kd = 20;
	double kp = 8, ki = 0.43658, kd = 20;
	static double errorVelocity, errorvSum, lastVelocity, derrorVelocity;
	double velocityOutput = 0, positionPid, anglePid, inputPid;
	positionPid = PositionSet();//位置PID作为输入
	anglePid = AngleSet();//角度PID作为输入
	if (fabs(z - stopPosition) >= 0.2 * stopPosition)//距离设定的10%以上处以固定角度直行
	{
		velocityOutput = AngleControl();//匀速直行
	}
	else //距离设定的10%以内减速到0
	{
		inputPid = positionPid * 0.2 + anglePid * 0.8;//设置位置和角度PID的比重
		//if (abs(angle) >= 0.25)
		//{
		//	inputPid = positionPid * 0.4 + anglePid * 1.2;
		//}
		//else
		//{
		//	inputPid = positionPid * 0.4 + anglePid * 0.8;
		//}
		errorVelocity = inputPid - 0;//将速度调到0
		if (angle <= 0)//反向时重新开始积分
		{
			errorvSum = 0;
		}
		errorvSum += errorVelocity;
		derrorVelocity = inputPid - lastVelocity;
		velocityOutput = kp * errorVelocity + ki * errorvSum + kd * derrorVelocity;
		lastVelocity = inputPid;
	}
	return velocityOutput;
}








