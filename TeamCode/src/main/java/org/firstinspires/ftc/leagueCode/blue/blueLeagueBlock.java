package org.firstinspires.ftc.leagueCode.blue;

import android.app.Activity;
import android.graphics.Color;
import android.hardware.camera2.CameraDevice;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.leagueCode.leagueMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
	
	private static final String VUFORIA_KEY = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";
	
	boolean inSight = false;
	
	double tX;
	double tY;
	double tZ;
	
	double rX;
	double rY;
	double rZ;
	
	public static final String TAG = "Vuforia VuMark Sample";
	OpenGLMatrix lastLocation = null;
	VuforiaLocalizer vuforia;
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
//----------------------------------------//
//----------------------------------------//
//---No More Methods Are Made Past This---//
//----------------------------------------//
////--------------------------------------//
//------------------------------------------------------------------------------------------------
	public void runOpMode()
	{
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		parameters.vuforiaLicenseKey = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";
		
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		vuforia = ClassFactory.getInstance().createVuforia(parameters);
		
		VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("Skystone");
		VuforiaTrackable relicTemplate = relicTrackables.get(0);
		relicTemplate.setName("relicVuMarkTemplate");
		
		// hsvValues is an array that will hold the hue, saturation, and value information.
		float hsvValues[] = {0F, 0F, 0F};
		
		// values is a reference to the hsvValues array.
		final float values[] = hsvValues;
		
		// sometimes it helps to multiply the raw RGB values with a scale factor
		// to amplify/attentuate the measured values.
		final double SCALE_FACTOR = 255;
		
		// get a reference to the RelativeLayout so we can change the background
		// color of the Robot Controller app to match the hue detected by the RGB sensor.
		int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
		final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
		
		turnIMU();
		telemetry.addData("Status", "Hit it Bois");
		telemetry.update();
		waitForStart();
		relicTrackables.activate();
		
		
		moveDistanceAtAngle(-13, 0, 0.2);
//--------------------------------------------------------------------------------------------------
		while (opModeIsActive() && (!(isStopRequested())))
		{
//----------------------------------
			double start = System.currentTimeMillis();
			double end   = start + 1000000;
//--------------------------------------------------------------------------------------------------
			while((inSight == false) && (end > System.currentTimeMillis()) && (!(isStopRequested())))
			{
				RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
				telemetry.addData("VuMark", "%s visible", vuMark);
				
				OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
				telemetry.addData("Pose", format(pose));
				
				if(pose != null)
				{
					VectorF trans = pose.getTranslation();
					Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
					
					// Extract the X, Y, and Z components of the offset of the target relative to the robot
					tX = trans.get(0);
					tY = trans.get(1);
					tZ = trans.get(2);
					
					// Extract the rotational components of the target relative to the robot
					rX = rot.firstAngle;
					rY = rot.secondAngle;
					rZ = rot.thirdAngle;
					telemetry.addLine().addData("First tX", tX);
					telemetry.addLine().addData("First tY", tY);
					telemetry.addLine().addData("First tZ", tZ);
					telemetry.update();
					inSight = true;
				}
				
				else if(pose == null)
				{
					while ((!(isStopRequested())) && rX > 0)
					{
						telemetry.addLine().addData("rX", rX);
						telemetry.addLine().addData("rY", rY);
						telemetry.addLine().addData("rZ", rZ);
						telemetry.update();
					}
				}
				// convert the RGB values to HSV values.
				// multiply by the SCALE_FACTOR.
				// then cast it back to int (SCALE_FACTOR is a double)
				Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR), (int) (robot.sensorColor.green() * SCALE_FACTOR), (int) (robot.sensorColor.blue() * SCALE_FACTOR), hsvValues);
				
				// send the info back to driver station using telemetry function.
//				telemetry.addData("Alpha", robot.sensorColor.alpha());
//				telemetry.addData("Red  ", robot.sensorColor.red());
//				telemetry.addData("Green", robot.sensorColor.green());
//				telemetry.addData("Blue ", robot.sensorColor.blue());
//				telemetry.addData("Hue", hsvValues[0]);
				
				// change the background color to match the color detected by the RGB sensor.
				// pass a reference to the hue, saturation, and value array as an argument
				// to the HSVToColor method.
				relativeLayout.post(new Runnable()
				{
					//I hate how this line reformats
					public void run()
					{
						//Formatting Comment
						relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
					}
				});
				telemetry.addLine("We still haven't seen the brick");
				telemetry.update();
			}
//--------------------------------------------------------------------------------------------------
			if(inSight == false)
			{
				telemetry.addLine().addData("First tX", tX);
				telemetry.addLine().addData("First tY", tY);
				telemetry.addLine().addData("First tZ", tZ);
				telemetry.update();
				telemetry.addLine("Grab two Blocks");
				//code to get the first 2 blocks
			}
			else if(inSight == true)//THIS IS A POSITION 1 TEST
			{
				turnAngle(-20,1000);
				moveDistanceAtAngle(15,-20,.02);
				turnAngle(-5,1000);
				robot.intake(.05);
				moveDistanceAtAngle(-25,-5,.02);
				robot.stopIntake();
				moveDistanceAtAngle(25,-5,.02);
				turnAngle(90,1000);
				moveDistanceAtAngle(50,90,.02);
				robot.outtake(.05);
				robot.stopIntake();
				moveDistanceAtAngle(-50,-90,.02);
				turnAngle(-45,1000);
				robot.intake(.05);
				moveDistanceAtAngle(-25,-45,.02);
				robot.stopIntake();
				moveDistanceAtAngle(-10,-45,.02);
				turnAngle(-90,1000);
				moveDistanceAtAngle(45,-45,.02);
			}
			else if(inSight == true)//THIS IS A POSITION 2 TEST - Unfinished
			{
				turnAngle(-20,1000);
				moveDistanceAtAngle(15,-20,.02);
				turnAngle(-5,1000);
				robot.intake(.05);
				moveDistanceAtAngle(-25,-5,.02);
				robot.stopIntake();
				moveDistanceAtAngle(25,-5,.02);
				turnAngle(90,1000);
				moveDistanceAtAngle(50,90,.02);
				robot.outtake(.05);
				robot.stopIntake();
				moveDistanceAtAngle(-50,-90,.02);
				turnAngle(-45,1000);
				robot.intake(.05);
				moveDistanceAtAngle(-25,-45,.02);
				robot.stopIntake();
				moveDistanceAtAngle(-10,-45,.02);
				turnAngle(-90,1000);
				moveDistanceAtAngle(45,-45,.02);
			}
			
			
			
			
			
			
			
			
			
			
			
			
			else if(tX < 10)//Position One
			{
				telemetry.addLine().addData("First tX", tX);
				telemetry.addLine().addData("First tY", tY);
				telemetry.addLine().addData("First tZ", tZ);
				telemetry.addLine("Position 1");
				telemetry.update();
//				moveDistanceAtAngle(-17, 0, 0.3);
//				turnAngle(20, 1500);
//				robot.intake(0.05);
//				moveDistanceAtAngle(-18, 20, 0.1);
//				robot.stopIntake();
//				moveDistanceAtAngle(13, 20, 0.3);
//				turnAngle(90, 2000);
//				moveDistanceAtAngle(-58, 90, 0.5);
//				robot.outtake(0.5);
//				sleep(1000);
//				robot.stopIntake();
//				moveDistanceAtAngle(63, 90, 0.5);
//				turnAngle(-20, 2000);
//				robot.intake(0.05);
//				moveDistanceAtAngle(-19, -20, 0.1);
//				robot.stopIntake();
//				moveDistanceAtAngle(12.5, -20, 0.3);
//				turnAngle(90, 2000);
//				moveDistanceAtAngle(-58, 90, 0.5);
//				robot.outtake(0.5);
//				sleep(1000);
//				robot.stopIntake();
//				moveDistanceAtAngle(12, 90, 0.5);
			}
			else if(tX > -10)//Position 2
			{
				telemetry.addLine().addData("First tX", tX);
				telemetry.addLine().addData("First tY", tY);
				telemetry.addLine().addData("First tZ", tZ);
				telemetry.addLine("Position 2");
				telemetry.update();
//				moveDistanceAtAngle(-12, 0, 0.3);
//				turnAngle(-45, 2000);
//				moveDistanceAtAngle(-6, -45, 0.3);
//				turnAngle(20, 2000);
//				robot.intake(0.05);
//				moveDistanceAtAngle(-20, 20, 0.1);
//				robot.stopIntake();
//				moveDistanceAtAngle(18, 20, 0.3);
//				turnAngle(90, 2000);
//				moveDistanceAtAngle(-58, 90, 0.5);
//				robot.outtake(0.05);
//				sleep(1000);
//				robot.stopIntake();
//				moveDistanceAtAngle(56, 90, 0.5);
//				turnAngle(-25, 3000);
//				robot.intake(0.05);
//				moveDistanceAtAngle(-20, -25, 0.1);
//				robot.stopIntake();
//				moveDistanceAtAngle(12, -25, 0.5);
//				turnAngle(90, 2000);
//				moveDistanceAtAngle(-53, 90, 0.6);
//				robot.outtake(0.05);
//				sleep(1000);
//				robot.stopIntake();
//				moveDistanceAtAngle(30, 90, 0.6);
			}
			else//Position 3
			{
//				moveDistanceAtAngle(-16, 0, 0.3);
//				turnAngle(-20, 1000);
//				robot.intake(0.05);
//				moveDistanceAtAngle(-19, -20, 0.1);
//				robot.stopIntake();
//				moveDistanceAtAngle(13, -20, 0.3);
//				turnAngle(90, 2000);
//				moveDistanceAtAngle(-58, 90, 0.5);
//				robot.outtake(0.5);
//				sleep(500);
//				robot.stopIntake();
//				moveDistanceAtAngle(50, 90, 0.5);
//				turnAngle(-60, 2000);
//				robot.outtake(1);
//				moveDistanceAtAngle(-26, -60, 0.3);
//				robot.intake(0.05);
//				moveDistanceAtAngle(-8, -60, 0.1);
//				robot.stopIntake();
//				moveDistanceAtAngle(20, -60, 0.3);
//				turnAngle(90, 2000);
//				moveDistanceAtAngle(-68, 90, 0.5);
//				robot.outtake(0.5);
//				sleep(500);
//				robot.stopIntake();
//				moveDistanceAtAngle(14, 90, 0.5);
			}
//			stop();
			telemetry.addLine().addData("First tX", tX);
			telemetry.addLine().addData("First tY", tY);
			telemetry.addLine().addData("First tZ", tZ);
			telemetry.update();
//----------------------------------
		}
		relativeLayout.post(new Runnable()//I hate how this line reformats
		{
			//Formatting Comment
			public void run()
			{
				//Formatting Comment
				relativeLayout.setBackgroundColor(Color.WHITE);
			}
		});
	}
	String format(OpenGLMatrix transformationMatrix)
	{
		return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
	}
}