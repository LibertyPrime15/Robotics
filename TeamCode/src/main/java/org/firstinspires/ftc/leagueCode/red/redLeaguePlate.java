package org.firstinspires.ftc.leagueCode.red;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.leagueCode.misc.leagueMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="redLeaguePlate", group = "redLeague")
//@Disabled
public class redLeaguePlate extends LinearOpMode
{
	leagueMap robot = new leagueMap();
	Orientation angles;
	BNO055IMU imuTurn;
	
	
	double diameter = 4;//4
	double radius   = (diameter/2);//2
	double circ     = (22/18 * (Math.PI * diameter));//12.5
	
	float currHeading;
	float currAngle;
	double Compensation = 1.5;
	double Steps        = 537.6;
	
	double drive = 0;
	double turn  = 0;
	
	private double flippedInit = 0.7;
	private double flippedGrab = 0.8;
	//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//gyro.getHeading()
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------
//This method is for reseting the imu - must be called after every time a turn is used
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
	imuTurn = hardwareMap.get(BNO055IMU.class, "imu1");
	imuTurn.initialize(parameters);
}
//--------------------------------------------------------------------------------------------------
//This method turns the robot a certain angle: 0-180 to the left && 0 to -180 in the right
private void turnAngle(double angle, double time)
{
	double start = System.currentTimeMillis();
	double end   = start + time;
	
	double angleDifference = angle - currAngle;//-60, -40 = -20
	double power;//.02*179+179
	
	while(System.currentTimeMillis() < end && !isStopRequested())
	{
		angleDifference = currAngle - angle;
		telemetry.update();
		if(Math.abs(angleDifference) > 180)
		{
			if(angleDifference > 0)
			{
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				
				angleDifference = currAngle - angle;
				power = .02 * (360 - Math.abs(angleDifference));
				robot.turnLeft(power);
			}
			else if(angleDifference < 0)
			{
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				
				angleDifference = currAngle - angle;
				power = .02 * (360 - Math.abs(angleDifference));
				robot.turnRight(power);
			}
		}
		else if(Math.abs(angleDifference) < 180)
		{
			telemetry.addData("----Heading", currAngle);
			telemetry.addData("----angleDifference", angleDifference);
			angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			this.imuTurn.getPosition();
			currAngle = angles.firstAngle;
			
			angleDifference = currAngle - angle;
			power = .02 * angleDifference;
			robot.turnRight(power);
		}
	}
	robot.Halt();
	robot.resetEncoder();
}
//--------------------------------------------------------------------------------------------------
public void setFlipPosition(double position)
{
	robot.flip2.setPosition(position);
	robot.flip1.setPosition(position);
}
//--------------------------------------------------------------------------------------------------
//This is a method that will move a certain distance at a certain angle and a certain power
//Distance is the distance that we want to travel, in inches
//Angle is the absolute field angle that we want to move along; 90 degrees is pointing from the quarry side to the foundation side on blue
//on red, 90 degrees points from foundation to quarry. 0 degrees is directly off the starting wall, and 180 or -180 is towards the
//starting wall
//Power is the power that we want to move at; it should be between 0 and 1. Making this value negative will put the bot into a loop that goes on forever
public void moveDistanceAtAngle(double distance, double angle, double power)
{
	//this resets the encoders, to make sure that all the values start at 0
	robot.resetEncoder();
	robot.setDriveToBrake();
	
	//This an int that represents the number of steps that the bot has to travel to have travelled distance
	double totalDistInSteps = 35 * distance;
	telemetry.addData("Total number of steps to travel", totalDistInSteps);
	telemetry.update();
	//This is essentially a value that says how far off the bot is from angle
	double angleDifference = 0;
	
	//If we want to go forward, go forward until the encoders say we've gone far enough
	if(distance > 0)
	{
		telemetry.addData("We are in the if statement", "");
		telemetry.update();
		double realAngleDifference = angleDifference;
		
		//while we aren't supposed to be stopped, and we haven't yet reached the distance we are supposed to travel,
		while(!isStopRequested() && (robot.front_right.getCurrentPosition() + robot.front_left.getCurrentPosition() + robot.back_right.getCurrentPosition() + robot.back_left.getCurrentPosition()) < (4 * totalDistInSteps))
		{
			
			//need to split this into 2 if statements - the same ones that are used in turnAngle
			telemetry.addData("Front right encoder value", robot.front_right.getCurrentPosition());
			telemetry.addData("Total number of steps to travel", totalDistInSteps);
			
			//Find the current angle using the rev hub I.M.U., and find the difference between that and the angle we are trying to reach
			angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			this.imuTurn.getPosition();
			currAngle = angles.firstAngle;
			angleDifference = angle - currAngle;
			
			if(angleDifference < 180)
			{
				realAngleDifference = angleDifference;
			}
			else if(angleDifference >= 180)
			{
				if(angleDifference > 0)
				{
					realAngleDifference = -(360 - Math.abs(angleDifference));
				}
				else if (angleDifference < 0)
				{
					realAngleDifference = (360 - Math.abs(angleDifference));
				}
			}
			
			telemetry.addData("Power", power + (0.02 * realAngleDifference));
			telemetry.addData("While loop value",(robot.front_right.getCurrentPosition() + robot.front_left.getCurrentPosition() + robot.back_right.getCurrentPosition() + robot.back_left.getCurrentPosition()));
			telemetry.update();
			
			//Drive at the the power that was input as a parameter, and correct for the angle difference
			robot.front_right.setPower(power - (0.02 * realAngleDifference));
			robot.front_left.setPower(power + (0.02 * realAngleDifference));
			robot.back_right.setPower(power - (0.02 * realAngleDifference));
			robot.back_left.setPower(power + (0.02 * realAngleDifference));
		}
	}
	//If we want to go backwards, go backwards until the encoders say we've gone far enough
	else if(distance < 0)
	{
		double realAngleDifference = angleDifference;
		//while we aren't supposed to be stopped, and we haven't yet reached the distance we are supposed to travel,
		while(!isStopRequested() && (robot.front_right.getCurrentPosition() + robot.front_left.getCurrentPosition() + robot.back_right.getCurrentPosition() + robot.back_left.getCurrentPosition()) > (4 * totalDistInSteps))
		{
			//Find the current angle using the rev hub I.M.U., and find the difference between that and the angle we are trying to reach
			angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			this.imuTurn.getPosition();
			currAngle = angles.firstAngle;
			angleDifference = angle - currAngle;
			
			if(Math.abs(angleDifference) < 180)
			{
				realAngleDifference = angleDifference;
			}
			else if(Math.abs(angleDifference) >= 180)
			{
				if(angleDifference > 0)
				{
					realAngleDifference = -(360 - Math.abs(angleDifference));
				}
				else if (angleDifference < 0)
				{
					realAngleDifference = (360 - Math.abs(angleDifference));
				}
			}
			
			//Drive at the the power that was input as a parameter, and correct for the angle difference
			robot.front_right.setPower(-power - (0.02 * realAngleDifference));
			robot.front_left.setPower(-power + (0.02 * realAngleDifference));
			robot.back_right.setPower(-power - (0.02 * realAngleDifference));
			robot.back_left.setPower(-power + (0.02 * realAngleDifference));
		}
	}
	
	telemetry.addLine("We are out of the while loop");
	telemetry.update();
	// turn off the motors and reset the encoders for the next moveDistance method
	robot.Halt();
	robot.setDriveToBrake();
	robot.resetEncoder();
	robot.setDriveToBrake();
}
private void moonDrive(double angle, double power, double moonPower, double time)
{
	double start = System.currentTimeMillis();
	double end = start + time;
	
	double angleDifference = angle - currAngle;
	
	int cycleTracker = 0;
	
	robot.setDriveToBrake();
	
	if(angleDifference > 0)
	{
		while(System.currentTimeMillis() < end && !isStopRequested())
		{
			robot.back_left.setPower(power + moonPower);
			robot.front_left.setPower(power + moonPower);
			robot.back_right.setPower(-power + moonPower);
			robot.front_right.setPower(-power + moonPower);
			
			telemetry.addData("----Heading", currAngle);
			telemetry.addData("----angleDifference", angleDifference);
			telemetry.update();
			
			angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			this.imuTurn.getPosition();
			currAngle = angles.firstAngle;
			
			angleDifference = angle - currAngle;
			
			if(angleDifference < 0)
			{
				break;
			}
		}
	}
	else
	{
		while(System.currentTimeMillis() < end && !isStopRequested())
		{
			robot.back_left.setPower(-power + moonPower);
			robot.front_left.setPower(-power + moonPower);
			robot.back_right.setPower(power + moonPower);
			robot.front_right.setPower(power + moonPower);
			
			angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			this.imuTurn.getPosition();
			currAngle = angles.firstAngle;
			
			angleDifference = angle - currAngle;
			
			if(angleDifference > 0)
			{
				break;
			}
		}
	}
	robot.Halt();
}
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---No More Methods Are Made Past This---//
//----------------------------------------//
////--------------------------------------//
//--------------------------------------------------------------------------------------------------
	public void runOpMode()
	{
		turnIMU();
		telemetry.addData("Status", "Hit it Bois");
		telemetry.update();
		waitForStart();
//--------------------------------------------------------------------------------------------------
		while (opModeIsActive() && (!(isStopRequested())))
		{
//----------------------------------
			double start = System.currentTimeMillis();
			double end   = start + 1000000;
			
			moveDistanceAtAngle(-4,0,.6);
			setFlipPosition(flippedGrab);
			turnAngle(-45, 1500);
			moveDistanceAtAngle(-12, -45, .6);
			turnAngle(0, 1500);
			moveDistanceAtAngle(-13,0,.1);
			sleep(500);
			robot.grabPlate();
			sleep(800);
			moonDrive(-90, .7, .8,20000);
			sleep(500);
			moveDistanceAtAngle(-20,-90,.4);
			sleep(700);
			robot.ungrabPlate();
			sleep(700);
			turnAngle(-45, 1500);
			moveDistanceAtAngle(2,-45,.5);
			sleep(500);
			turnAngle(-89,1500);
			moveDistanceAtAngle(32,-89,.4);
			break;
//----------------------------------
		}
	}
}