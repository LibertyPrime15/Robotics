package org.firstinspires.ftc.leagueCode.blue;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.leagueCode.misc.leagueMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "blueLeagueBlock", group = "blueLeague")
//@Disabled
public class blueLeagueBlock extends LinearOpMode
{
	leagueMap robot = new leagueMap();
	Orientation angles;
	BNO055IMU imuTurn;
	
	List<Recognition> updatedRecognitions;
	
	public static float tensorLeft;
	public static float tensorRight;
	public float tensorAvgDist;
	
	double yPos = 0;
	double xPos = 0;
	
	double testCounter = 0;
	
	double startTime = System.currentTimeMillis();
	double endTime;
	double actualTime;
	
	double diameter = 4;//4
	double radius   = (diameter/2);//2
	double circ     = (22/18 * (Math.PI * diameter));//12.5
	
	float currHeading;
	float currAngle;
	double Compensation = 1.5;
	double Steps        = 537.6;
	
	boolean canTogglePlateGrabber = true;
	boolean canAddToLiftPos = true;
	boolean canSubtractFromLiftPos = true;
	boolean canInitiateSpitCycle = true;
	
	double flippedIn = 0.91;
	double flippedGrab = 0.80;
	double flippedOut = 0.2;
	double flipStartPos = 0.7;
	double wristWhenIn = 0.81;
	double wristWhenOut = 0.06;
	double rotateGrab = 0.98;
	double rotateFar = 0.58;
	double rotateLeft = 0.23;
	double rotateClose = 0;
	double grabbed = 0.8;
	double ungrabbed = 0.25;
	double capStore = 0;
	double capSlap = 0.38;
	
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
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
	imuTurn = hardwareMap.get(BNO055IMU.class,"imu1");
	imuTurn.initialize(parameters);
}
//--------------------------------------------------------------------------------------------------
private void initVuforia()
{
	VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
	parameters.vuforiaLicenseKey = VUFORIA_KEY;
	parameters.cameraDirection = CameraDirection.BACK;
	vuforia = ClassFactory.getInstance().createVuforia(parameters);
}
//--------------------------------------------------------------------------------------------------
private void initTfod()
{
	int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
	TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
	tfodParameters.minimumConfidence = 0.77;
	tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
	tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
}
//--------------------------------------------------------------------------------------------------
//If you don't know how to use this method than please don't call it and expect it to work
private void cycleTimeStart()
{
	double startTime = 0;
	double previousStartTime = 0;
//-------------------------------------------------------------
//This bit \/ goes inside of an if statement or in a while loop
	previousStartTime = startTime;
	startTime = System.currentTimeMillis();
	double cycleTime = startTime - previousStartTime;
	telemetry.addData("Cycle Time Average", cycleTime);
	telemetry.update();
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
public void passPosition(double yPos, double xPos)
{
	this.yPos = yPos;
	this.xPos = xPos;
}
//--------------------------------------------------------------------------------------------------
public ArrayList<Integer> returnEndocerValues()
{
	int sum         = 0;
	Integer average = 0;
	int count       = 0;
	ArrayList<Integer> returnList = new ArrayList<Integer>();
	returnList.add(robot.front_left.getCurrentPosition());
	returnList.add(robot.front_right.getCurrentPosition());
	returnList.add(robot.back_left.getCurrentPosition());
	returnList.add(robot.back_right.getCurrentPosition());
	for(int value : returnList)
	{
		sum =+ value;
		count++;
	}
	average = sum/count;
	telemetry.addData("Return list Average", average);
	telemetry.addData("return List", returnList);
	telemetry.update();
}
//--------------------------------------------------------------------------------------------------
public double returnAngle()
{
	angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
	this.imuTurn.getPosition();
	currAngle = angles.firstAngle;
	return currAngle;
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
//--------------------------------------------------------------------------------------------------
public void setFlipPosition(double position)
{
	robot.flip2.setPosition(position);
	robot.flip1.setPosition(position);
}
//--------------------------------------------------------------------------------------------------
public void turnOffTensorFlow()
{
	tfod.deactivate();
	tfod.shutdown();
}
//--------------------------------------------------------------------------------------------------
private void blockPositionOne()
{
	turnOffTensorFlow();
	moveDistanceAtAngle(-17, 0, 0.3);
	setFlipPosition(grabbed);
	turnAngle(20, 1500);
	robot.intake(0.05);
	moveDistanceAtAngle(-18, 20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(11, 20, 0.3);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-54, 90, 0.5);
	robot.disengageIntake();
	moveDistanceAtAngle(8,90,.5);
	robot.ungrabPlate();
	moveDistanceAtAngle(42, 90, 0.5);
	turnAngle(-20, 2000);
	robot.intake(0.05);
	moveDistanceAtAngle(-19, -20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(13.8, -20, 0.3);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-54, 90, 0.5);
	robot.disengageIntake();
	moveDistanceAtAngle(8,90,.3);
	robot.ungrabPlate();
	moveDistanceAtAngle(4,90,.3);
	stop();
}
//--------------------------------------------------------------------------------------------------
private void blockPositionTwo()
{
	turnOffTensorFlow();
	moveDistanceAtAngle(-14, 0, 0.3);
	setFlipPosition(grabbed);
	turnAngle(-45, 1000);
	moveDistanceAtAngle(-8, -45, 0.2);
	turnAngle(20, 1000);
	robot.intake(0.05);
	moveDistanceAtAngle(-20, 20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(19, 20, 0.2);
	turnAngle(90, 1750);
	moveDistanceAtAngle(-58, 90, 0.5);
	robot.disengageIntake();
	moveDistanceAtAngle(8,90, 0.3);
	robot.ungrabPlate();
	moveDistanceAtAngle(50, 90, 0.5);
	turnAngle(-20, 2000);
	robot.intake(0.05);
	moveDistanceAtAngle(-20, -20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(12, -20, 0.5);
	turnAngle(90, 1000);
	moveDistanceAtAngle(-64, 90, .6);
	robot.disengageIntake();
	moveDistanceAtAngle(4,90, 0.6);
	robot.ungrabPlate();
	moveDistanceAtAngle(6,90,0.6);
	stop();
}
//--------------------------------------------------------------------------------------------------
private void blockPositionThree()
{
	turnOffTensorFlow();
	moveDistanceAtAngle(-18, 0, 0.3);
	setFlipPosition(grabbed);
	turnAngle(-20, 1000);
	robot.intake(0.05);
	moveDistanceAtAngle(-19, -20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(11, -20, 0.3);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-58, 90, 0.5);
	robot.disengageIntake();
	moveDistanceAtAngle(8,90, 0.5);
	robot.ungrabPlate();
	moveDistanceAtAngle(46, 90, 0.5);
	turnAngle(-55, 2000);
	moveDistanceAtAngle(-26, -55, 0.2);
	robot.intake(0.05);
	moveDistanceAtAngle(-8, -55, 0.1);
	robot.intake(1);
	moveDistanceAtAngle(20, -55, 0.3);
	robot.stopIntake();
	turnAngle(90, 2000);
	moveDistanceAtAngle(-64, 90, 0.5);
	robot.disengageIntake();
	moveDistanceAtAngle(8,90, 0.5);
	robot.ungrabPlate();
	moveDistanceAtAngle(10, 90, 0.5);
	stop();
}
//--------------------------------------------------------------------------------------------------
public void waitForStart()
{
	boolean alreadyRecorded;
	//This starts our multiThreading Program
	multiThreading object = new multiThreading();
	object.start();
	object.passInOpMode(this);
	while (!isStarted())
	{
		if(tfod != null)
		{
			tfod.activate();
			updatedRecognitions = tfod.getUpdatedRecognitions();
		}
		if(updatedRecognitions != null)
		{
			telemetry.addData("# Object Detected", updatedRecognitions.size());
			telemetry.addLine("We're above the for loop");
			telemetry.update();
			alreadyRecorded = false;
			for(Recognition recognition : updatedRecognitions)
			{
				telemetry.addLine("We're in the for loop");
				telemetry.update();
				if(recognition.getLabel() == LABEL_SECOND_ELEMENT)
				{
					telemetry.addLine("We're in the if statement");
					tensorLeft    = (int) recognition.getTop();
					tensorRight   = (int) recognition.getBottom();
					if(((tensorLeft + tensorRight)/2 > tensorAvgDist) && alreadyRecorded)
					{
						//do nothing
					}
					else
					{
						tensorAvgDist = (tensorLeft + tensorRight)/2;
					}
					alreadyRecorded = true;
					telemetry.addLine("We're in the bottom of the if statement");
					telemetry.addData("TensorDistAverage", tensorAvgDist);
					telemetry.update();
				}
			}
		}
		if(tensorAvgDist > 725)
		{
			telemetry.addLine("Tensorflow sees block 1");
		}
		else if(tensorAvgDist < 725 && tensorAvgDist > 550)
		{
			telemetry.addLine("Tensorflow sees block position 2");
		}
		else if(tensorAvgDist < 550)
		{
			telemetry.addLine("Tensorflow sees block position 3");
		}
		synchronized(this)
		{
			try
			{
				this.wait();
			}
			catch(InterruptedException e)
			{
				Thread.currentThread().interrupt();
				return;
			}
		}
	}
}
//--------------------------------------------------------------------------------------------------
    public void runOpMode()
	{
		robot.init(hardwareMap);
		turnIMU();
		initVuforia();
        if(ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        else
		{
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        waitForStart();
//--------------------------------------------------------------------------------------------------
		if(opModeIsActive())
		{
			while(opModeIsActive() && (!(isStopRequested())))
			{
				telemetry.addData("yPos", yPos);
				telemetry.addData("xPos", xPos);
				testCounter++;
				telemetry.addData("testCounter", testCounter);
				telemetry.update();
//				if(tensorAvgDist > 725)
//				{
//					blockPositionThree();
//				}
//				if((tensorAvgDist < 725) && (tensorAvgDist > 550))
//				{
//					blockPositionTwo();
//				}
//				if(tensorAvgDist < 550)
//				{
//					blockPositionOne();
//				}
			}
			this.stop();
		}
	}
//--------------------------------------------------------------------------------------------------
}