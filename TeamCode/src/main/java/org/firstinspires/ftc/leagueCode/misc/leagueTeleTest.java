package org.firstinspires.ftc.leagueCode.misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="leagueTeleTest", group = "A")
//@Disabled
public class leagueTeleTest extends LinearOpMode
{
    leagueMap robot = new leagueMap();
    Orientation angles;
    BNO055IMU imu;
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------
private void turnIMU()
{
	BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
	parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
	parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
	parameters.calibrationDataFile = "BNO055IMUCalibration.json";
	parameters.loggingEnabled = true;
	parameters.loggingTag = "IMU";
	parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

	robot.init(hardwareMap);
	imu = hardwareMap.get(BNO055IMU.class,"imu1");
	imu.initialize(parameters);
}
//--------------------------------------------------------------------------------------------------
private void driveIMU()
{
	BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
	parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
	parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
	parameters.calibrationDataFile = "BNO055IMUCalibration.json";
	parameters.loggingEnabled = true;
	parameters.loggingTag = "IMU";
	parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
	
	robot.init(hardwareMap);
	imu = hardwareMap.get(BNO055IMU.class,"imu2");
	imu.initialize(parameters);
}
//--------------------------------------------------------------------------------------------------
//this method stores most of our standard (not automated) teleop code like driving and toggling
//the plate grabber.
//making this a method allowed us to add this functionality to other methods easier
public void normalTeleopStuff()
{
	if(gamepad1.a)
	{
		robot.front_right.setPower(.5);
	}
	else
	{
		robot.front_right.setPower(0);
	}
	
	
	if(gamepad1.b)
	{
		robot.front_left.setPower(.5);
	}
	else
	{
		robot.front_left.setPower(0);
	}
	
	
	if(gamepad1.x)
	{
		robot.back_right.setPower(.5);
	}
	else
	{
		robot.back_right.setPower(0);
	}
	
	
	if(gamepad1.y)
	{
		robot.back_left.setPower(.5);
	}
	else
	{
		robot.back_left.setPower(0);
	}
//---------------------------------------------------------
	if(gamepad1.right_bumper)
	{
		robot.intake1.setPower(1);
	}
	else
	{
		robot.intake1.setPower(0);
	}
	
	
	
	if(gamepad1.left_bumper)
	{
		robot.intake2.setPower(1);
	}
	else
	{
		robot.intake2.setPower(0);
	}
//---------------------------------------------------------
	if(gamepad1.left_stick_y !=0)
	{
		robot.liftPrimary.setPower(gamepad1.left_stick_y);
		robot.liftSecondary.setPower(-gamepad1.left_stick_y);
	}
	else
	{
		robot.liftPrimary.setPower(0);
		robot.liftSecondary.setPower(0);
	}
}
//--------------------------------------------------------------------------------------------------
//------------------------------------------------------//
//------------------------------------------------------//
//---Here is my Actual Run Op where I call my methods---//
//------------------------------------------------------//
//------------------------------------------------------//
//--------------------------------------------------------------------------------------------------
    public void runOpMode()
    {
        turnIMU();
        waitForStart();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
            normalTeleopStuff();
        }
    }
}
//--------------------------------------------------------------------------------------------------
//-------------------------------------------//
//-------------------------------------------//
//---There is No More Code Past This Point---//
//-------------------------------------------//
//-------------------------------------------//
//--------------------------------------------------------------------------------------------------