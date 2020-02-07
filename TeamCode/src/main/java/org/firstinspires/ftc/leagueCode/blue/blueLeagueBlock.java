package org.firstinspires.ftc.leagueCode.blue;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.leagueCode.misc.leagueMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

@Autonomous(name="blueLeagueBlock", group = "blueLeague")
//@Disabled
public class blueLeagueBlock extends LinearOpMode
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
	
	private VuforiaLocalizer vuforiaLocalizer;
	private VuforiaLocalizer.Parameters parameters;
	private VuforiaTrackables visionTargets;
	private VuforiaTrackable target;
	private VuforiaTrackableDefaultListener listener;
	private OpenGLMatrix lastKnownLocation;
	private OpenGLMatrix phoneLocation;
	
	private static final String VUFORIA_KEY = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";
	
	boolean inView  = false;
	
	boolean blockPositionOne   = false;
	boolean blockPositionTwo   = false;
	boolean blockPositionThree = false;
	
	private float robotX = 0;
	private float robotY = 0;
	private float robotAngle = 0;
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//gyro.getHeading()
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------
//This method is used for setting up Vuforia and runs everytime the program initializes
private void setupVuforia()
{
	parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
	parameters.vuforiaLicenseKey = VUFORIA_KEY;
	parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
	parameters.useExtendedTracking = false;
	vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);
	
	visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");
	Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
	
	target = visionTargets.get(0);
	target.setName("Wheels Target");
	target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));
	phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);
	
	
	listener = (VuforiaTrackableDefaultListener) target.getListener();
	listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
	
}
private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
{
	return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
}
private String formatMatrix(OpenGLMatrix matrix)
{
	//This method formats weird
	return matrix.formatAsTransform();
}
public boolean vuforia()
{
	OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
	
	if(latestLocation != null)
	{
		lastKnownLocation = latestLocation;
	}
	float[] coordinates = lastKnownLocation.getTranslation().getData();
	
	robotX = coordinates[0];
	robotY = coordinates[1];
	robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
	
	telemetry.addData("Tracking " + target.getName(), listener.isVisible());
	telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
	telemetry.update();
	return listener.isVisible();
}
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
//This method moves a certain distance in inches SIDEWAYS at a certain speed - when moving it will move perfectly straight
public void moveSide(double distance, double power, double time, boolean goingRight)
{
	double length = distance;
	double totDistInSteps = (Math.abs(circ / Steps) * (length * Compensation));
	
	double start = System.currentTimeMillis();
	double end   = start + time;
	
	int currentEncSumRight = (robot.front_right.getCurrentPosition() + robot.back_right.getCurrentPosition());
	int currentEncSumLeft  = (robot.front_left.getCurrentPosition()  + robot.back_left.getCurrentPosition());
	
	int currentEncPosAvgRight = (Math.abs(currentEncSumRight / 2));
	int currentEncPosAvgLeft  = (Math.abs(currentEncSumLeft  / 2));
	
	double topRight;
	double topLeft;
	double bottomRight;
	double bottomLeft;
	telemetry.addLine("Line Check 1");
	telemetry.update();
	if(goingRight)//DRIVE TO THE RIGHT
	{
//------------------------------
		telemetry.addLine("Line Check 2");
		telemetry.update();
		while(totDistInSteps > currentEncPosAvgRight && opModeIsActive() && (!(isStopRequested())) && System.currentTimeMillis() < end)
		{
			currentEncSumRight    = (robot.front_right.getCurrentPosition() + robot.back_right.getCurrentPosition());
			currentEncPosAvgRight = (Math.abs(currentEncSumRight / 2));
			
			telemetry.addData("Average Encoder RIGHT",currentEncPosAvgRight);
			telemetry.addData("TotDistInSteps",totDistInSteps);
			telemetry.update();
			
			turnIMU();
			drive = -power;
			turn  = .05 * currHeading;
			
			topRight    = Range.clip(drive + turn, -1.0, 1.0);
			topLeft     = Range.clip(drive - turn, -1.0, 1.0);
			bottomRight = Range.clip(drive + turn, -1.0, 1.0);
			bottomLeft  = Range.clip(drive - turn, -1.0, 1.0);
			
			robot.front_right.setPower(topRight);
			robot.front_left.setPower(topLeft);
			robot.back_right.setPower(bottomRight);
			robot.back_left.setPower(bottomLeft);
		}
	}
	
	else if(!goingRight)//DRIVE TO THE LEFT
	{
//------------------------------
		while(totDistInSteps < currentEncPosAvgLeft && opModeIsActive() && (!(isStopRequested())) && System.currentTimeMillis() < end)
		{
			currentEncSumLeft    = (robot.front_left.getCurrentPosition()  + robot.back_left.getCurrentPosition());
			currentEncPosAvgLeft = (Math.abs(currentEncSumLeft / 2));
			
			telemetry.addData("----Average Encoder LEFT",currentEncPosAvgLeft);
			telemetry.addData("----TotDistInSteps",totDistInSteps);
			telemetry.update();
			
			turnIMU();
			drive = -power;
			turn  = .05 * currHeading;
			
			topRight    = Range.clip(drive + turn, -1.0, 1.0);
			topLeft     = Range.clip(drive - turn, -1.0, 1.0);
			bottomRight = Range.clip(drive + turn, -1.0, 1.0);
			bottomLeft  = Range.clip(drive - turn, -1.0, 1.0);
			
			robot.front_right.setPower(topRight);
			robot.front_left.setPower(topLeft);
			robot.back_right.setPower(bottomRight);
			robot.back_left.setPower(bottomLeft);
		}
	}
	robot.resetEncoder();
	robot.Halt();
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
//--------------------------------------------------------------------------------------------------
public void setFlipPosition(double position)
{
	robot.flip2.setPosition(position);
	robot.flip1.setPosition(position);
}
//--------------------------------------------------------------------------------------------------
//This checks to see if the skystone is in view
public void checkSight()
{
	turnAngle(7,2500);//Positions to see the block
	if(listener.isVisible() && System.currentTimeMillis() > 2000)
	{
		inView = true;
	}
	if(inView)
	{
		blockPositionOne = true;
		turnAngle(1,400);
		runCycle();
	}
	else
	{
		turnAngle(-10,400);
		moveDistanceAtAngle(-1.2,-10,.2);
		turnAngle(-10,2500);//These are all positioning to see the block
		if(listener.isVisible())
		{
			inView = true;
		}
		if(inView)
		{
			blockPositionTwo = true;
			turnAngle(-8,500);
			runCycle();
		}
		else
		{
			blockPositionThree = true;
			runCycle();
		}
	}
}
//--------------------------------------------------------------------------------------------------
//This method runs actual atonomous code and calls the methods that run actual atonomous code
public void beginScanning()
{
	//This moves the robot out of its start position and prepares for scanning
	moveDistanceAtAngle(-16.4,0,.2);
	robot.wrist.setPosition(.03);
	setFlipPosition(.8);
	checkSight();
}
//--------------------------------------------------------------------------------------------------
public void runCycle()
{
	if(blockPositionOne)//THIS IS A POSITION 1 TEST
	{
		moveDistanceAtAngle(5, 0, 0.3);
		turnAngle(-40, 1000);
		moveDistanceAtAngle(-8, -40, 0.3);
		turnAngle(18, 1500);
		robot.intake(.05);
		moveDistanceAtAngle(-20, 20, .15);
		robot.stopIntake();
		moveDistanceAtAngle(10, 20, .3);
		turnAngle(90, 1200);
		moveDistanceAtAngle(-40, 90, .5);
		robot.disengageIntake();
		sleep(400);
		moveDistanceAtAngle(47, 90, .5);
		robot.ungrabPlate();
		turnAngle(-20, 1200);
		robot.intake(.05);
		moveDistanceAtAngle(-10, -20, .2);
		robot.stopIntake();
		moveDistanceAtAngle(10, -20, .2);
		turnAngle(90, 1000);
		moveDistanceAtAngle(-58, 90, .5);
		robot.disengageIntake();
		sleep(400);
		moveDistanceAtAngle(12,90,.5);
		robot.ungrabPlate();
		stop();
	}
	else if(blockPositionTwo)//THIS IS A POSITION 2 TEST
	{
		turnAngle(0, 1500);
		moveDistanceAtAngle(-2, 0, 0.3);
		turnAngle(-19, 1500);
		robot.intake(.05);
		moveDistanceAtAngle(-15, -19, .15);
		robot.stopIntake();
		moveDistanceAtAngle(12.5, -19, .3);
		turnAngle(90, 1200);
		moveDistanceAtAngle(-37, 90, .6);
		robot.disengageIntake();
		sleep(400);
		moveDistanceAtAngle(52, 90, .6);
		robot.ungrabPlate();
		turnAngle(-20, 1300);
		robot.intake(.05);
		moveDistanceAtAngle(-18, -20, .3);
		robot.stopIntake();
		moveDistanceAtAngle(14.2, -20, .3);
		turnAngle(90, 1000);
		moveDistanceAtAngle(-60, 90, .6);
		robot.disengageIntake();
		sleep(400);
		moveDistanceAtAngle(15, 90, .4);
		robot.ungrabPlate();
		stop();
	}
	else if(blockPositionThree)//THIS IS A POSITION 3 TEST - Unfinished
	{
		moveDistanceAtAngle(10, -10, 0.3);
		turnAngle(-25, 1000);
		moveDistanceAtAngle(-12, -28, 0.3);
		robot.intake(.05);
		moveDistanceAtAngle(-20, -28, .1);
		robot.stopIntake();
		moveDistanceAtAngle(10.5, -27, .3);
		turnAngle(90, 1000);
		moveDistanceAtAngle(-50, 90, .6);
		robot.disengageIntake();
		sleep(400);
		moveDistanceAtAngle(52.5, 90, .6);
		robot.ungrabPlate();
		turnAngle(-20, 1000);
		robot.intake(.05);
		moveDistanceAtAngle(-16, -20, .1);
		robot.stopIntake();
		moveDistanceAtAngle(10, -20, .3);
		turnAngle(90, 1000);
		moveDistanceAtAngle(-60, 90, .8);
		robot.disengageIntake();
		sleep(700);
		moveDistanceAtAngle(15, 90, .8);
		robot.ungrabPlate();
		stop();
	}
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
		setupVuforia();
		lastKnownLocation = createMatrix(0, 500, 0, 90, 0, 90);
		telemetry.addData("Status", "Hit it Bois");
		telemetry.update();
		
		waitForStart();
		visionTargets.activate();
//--------------------------------------------------------------------------------------------------
		while (opModeIsActive() && (!(isStopRequested())))
		{
//----------------------------------
			double start = System.currentTimeMillis();
			double end   = start + 1000000;
//--------------------------------------------------------------------------------------------------
			vuforia();
			beginScanning();
//----------------------------------
		}
	}
}