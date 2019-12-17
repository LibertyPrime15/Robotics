package org.firstinspires.ftc.leagueCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

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
import org.firstinspires.ftc.teamcode.R;
import java.util.Timer;
import java.util.TimerTask;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.lang.Thread;
import java.util.concurrent.TimeoutException;

import org.firstinspires.ftc.teamcode.RevMap;

@Autonomous(name="LeagueTestAuto", group = "A")
//@Disabled
public class leagueTestAuto extends LinearOpMode
{
    leagueMap robot = new leagueMap();
    Orientation angles;
    BNO055IMU imuTurn;
	BNO055IMU imuDrive;

    double diameter = 4;
    double radius   = (diameter/2);
    double circ     = Math.PI * (radius * radius);

    double Steps        = 560;
    float currHeading;
    float currAngle;
    double Compensation = 1.5;

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

    private float robotX     = 0;
    private float robotY     = 0;
    private float robotAngle = 0;

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
//This method is for reseting the imu - must be called after every time a turn is used
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
	imuDrive = hardwareMap.get(BNO055IMU.class, "imu2");
	imuDrive.initialize(parameters);
}
//--------------------------------------------------------------------------------------------------
//This method is used for setting up Vuforia and runs everytime the program initializes
private void setupVuforia()
{
    parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
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
    //I don't like how java reformats this line so I'm putting in a comment line
    return matrix.formatAsTransform();
}
public boolean vufoCrap()
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
//This program returns our current angular heading
private double headingAngle()
{
    angles = this.imuDrive.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
    this.imuDrive.getPosition();
    currHeading = angles.firstAngle;
    return currHeading;
}
//--------------------------------------------------------------------------------------------------
//This method moves a certain distance in inches at a certain speed - when moving it will move perfectly straight
public void moveDistance(double distance, double power, double time)
{
    double length = distance;

    currHeading = 0;
    
    double start = System.currentTimeMillis();
    double end   = start + time;

    double leftPower;
    double rightPower;

    if(distance > 0)//Going Backward
    {
//------------------------------
		double totDistInSteps = (Math.abs((Steps / circ) * length));
//------------------------------
        while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps < (Math.abs(robot.front_right.getCurrentPosition() + robot.front_left.getCurrentPosition() + robot.back_right.getCurrentPosition() + robot.back_left.getCurrentPosition()/4))  && System.currentTimeMillis() < end)
        {

            telemetry.addData("----currStepAVG",(Math.abs(robot.front_right.getCurrentPosition() + robot.front_left.getCurrentPosition() + robot.back_right.getCurrentPosition() + robot.back_left.getCurrentPosition()/4)));
			telemetry.addData("----TotDistInSteps",totDistInSteps);
			telemetry.addData("front_right",robot.front_right.getCurrentPosition());
			telemetry.addData("front_left",robot.front_left.getCurrentPosition());
			telemetry.addData("back_right",robot.back_right.getCurrentPosition());
			telemetry.addData("back_left",robot.back_left.getCurrentPosition());
			telemetry.addData("Heading",currHeading);
			telemetry.update();

            headingAngle();
            drive = power;
            turn  = .01 * currHeading;
            leftPower    = Range.clip(drive - turn, -1.0, 1.0);
            rightPower   = Range.clip(drive + turn, -1.0, 1.0);

            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
        }
    }

    else if(distance < 0)//Going Forward
    {
//------------------------------
		double totDistInSteps = (-1 * ((Steps / circ) * length));
//------------------------------
        while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps > (-1 * (robot.front_right.getCurrentPosition() + robot.front_left.getCurrentPosition() + robot.back_right.getCurrentPosition() + robot.back_left.getCurrentPosition()/4)) && System.currentTimeMillis() < end)
        {
            telemetry.addData("currStepsAVG",(-1 * (robot.front_right.getCurrentPosition() + robot.front_left.getCurrentPosition() + robot.back_right.getCurrentPosition() + robot.back_left.getCurrentPosition()/4)));
			telemetry.addData("TotDistInSteps",totDistInSteps);
			telemetry.addData("Heading",currHeading);
			telemetry.addData("front_right",robot.front_right.getCurrentPosition());
			telemetry.addData("front_left",robot.front_left.getCurrentPosition());
			telemetry.addData("back_right",robot.back_right.getCurrentPosition());
			telemetry.addData("back_left",robot.back_left.getCurrentPosition());
			telemetry.update();

            headingAngle();
            drive = -power;
            turn  = .01 * currHeading;
            leftPower    = Range.clip(drive - turn, -1.0, 1.0);
            rightPower   = Range.clip(drive + turn, -1.0, 1.0);

            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
        }
    }
    robot.Halt();
    robot.resetEncoder();
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

            headingAngle();
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

            headingAngle();
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
	
	double posCurrAngle;
	
	double angleDifference = angle - currAngle;
	double power = .02 * angleDifference;
	
	while(angle != currAngle)
	{
		posCurrAngle = Math.abs(currAngle);
		if(angle == 180 || angle == -180)
		{
			if(posCurrAngle > 0 && currAngle < 0)
			{
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				telemetry.update();
				
				angleDifference = angle - currAngle;
				power = .025 * angleDifference;
				robot.turnRight(power);
			}
			else if(posCurrAngle > 0 && currAngle > 0)
			{
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				telemetry.update();
				
				angleDifference = angle - currAngle;
				power = .025 * angleDifference;
				robot.turnLeft(power);
			}
		}
		else if(Math.abs(angleDifference) > 180)
		{
			telemetry.addData("Heading", currAngle);
			telemetry.addData("angleDifference", angleDifference);
			angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			this.imuTurn.getPosition();
			currAngle = angles.firstAngle;
			telemetry.update();
			
			angleDifference = angle - currAngle;
			power = .025 * angleDifference;
			robot.turnRight(power);
		}
		else if(currAngle > 0)
		{
			if(angle > 177 && currAngle > 150)
			{
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				telemetry.update();
				
				angleDifference = angle - currAngle;
				power = .016 * angleDifference;
				robot.turnLeft(power);
			}
			else
			{
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				telemetry.update();
				
				angleDifference = angle - currAngle;
				power = .025 * angleDifference;
				robot.turnLeft(power);
			}
		}
		else if(currAngle < 0)
		{
			if (angle < -177 && currAngle < -150)
			{
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				telemetry.update();
				
				angleDifference = angle - currAngle;
				power = .016 * angleDifference;
				robot.turnLeft(power);
			}
			else
			{
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				telemetry.update();
				
				angleDifference = angle - currAngle;
				power = .025 * angleDifference;
				robot.turnLeft(power);
			}
		}
	}
	robot.Halt();
	robot.resetEncoder();
	driveIMU();
	currHeading = 0;
}
//--------------------------------------------------------------------------------------------------
//This is a method that moves diagonally such that we can align to the block to grab it in autonomous
//public void diagonalMove(double distance, double power, double time, boolean goingRight, boolean goingBack)
//{
//	double totDistInSteps = (Math.abs((circ / Steps) * (distance * Compensation)));
//
//	double averageEncoderRightCorner = (Math.abs((.front_right.getCurrentPosition() + robot.back_left.getCurrentPosition())/2));
//	double averageEncoderLeftCorner  = (Math.abs((robot.front_left.getCurrentPosition()  + robot.back_right.getCurrentPosition())/2));
//
//	double start = System.currentTimeMillis();
//	double end   = start + time;
//
//	double topRight;
//	double topLeft;
//	double bottomRight;
//	double bottomLeft;
//
//	if(goingRight && goingBack)//DRIVE TO THE BACK RIGHT
//	{
//		while(opModeIsActive() && (!(isStopRequested())) && System.currentTimeMillis() < end)
//		{
//			averageEncoderLeftCorner  = (-1 *((robot.front_left.getCurrentPosition()  + robot.back_right.getCurrentPosition())/2));
//			headingAngle();
//			drive = -power;
//			turn  = .05 * currHeading;
//
//			topRight    = Range.clip(drive + turn, -1.0, 1.0);
//			topLeft     = Range.clip(drive - turn, -1.0, 1.0);
//			bottomRight = Range.clip(drive + turn, -1.0, 1.0);
//			bottomLeft  = Range.clip(drive - turn, -1.0, 1.0);
//
//			robot.front_right.setPower(topRight);
//			robot.front_left.setPower(topLeft);
//			robot.back_right.setPower(bottomRight);
//			robot.back_left.setPower(bottomLeft);
//		}
//	}
//	else if(goingRight && !goingBack)//DRIVE TO THE TOP RIGHT
//	{
//		while(opModeIsActive() && (!(isStopRequested())) && System.currentTimeMillis() < end)
//		{
//			averageEncoderRightCorner = (Math.abs((robot.front_right.getCurrentPosition() + robot.back_left.getCurrentPosition())/2));
//			headingAngle();
//			drive = -power;
//			turn  = .05 * currHeading;
//
//			topRight    = Range.clip(drive + turn, -1.0, 1.0);
//			topLeft     = Range.clip(drive - turn, -1.0, 1.0);
//			bottomRight = Range.clip(drive + turn, -1.0, 1.0);
//			bottomLeft  = Range.clip(drive - turn, -1.0, 1.0);
//
//			robot.front_right.setPower(topRight);
//			robot.front_left.setPower(topLeft);
//			robot.back_right.setPower(bottomRight);
//			robot.back_left.setPower(bottomLeft);
//		}
//	}
//	else if(!goingRight && goingBack)//DRIVE TO THE BACK LEFT
//	{
//		while(opModeIsActive() && (!(isStopRequested())) && System.currentTimeMillis() < end)
//		{
//			averageEncoderRightCorner = (-1 *((robot.front_right.getCurrentPosition() + robot.back_left.getCurrentPosition())/2));
//			headingAngle();
//			drive = -power;
//			turn  = .05 * currHeading;
//
//			topRight    = Range.clip(drive + turn, -1.0, 1.0);
//			topLeft     = Range.clip(drive - turn, -1.0, 1.0);
//			bottomRight = Range.clip(drive + turn, -1.0, 1.0);
//			bottomLeft  = Range.clip(drive - turn, -1.0, 1.0);
//
//			robot.front_right.setPower(topRight);
//			robot.front_left.setPower(topLeft);
//			robot.back_right.setPower(bottomRight);
//			robot.back_left.setPower(bottomLeft);
//		}
//	}
//	else if(!goingRight && !goingBack)//DRIVE TO THE TOP LEFT
//	{
//		while(opModeIsActive() && (!(isStopRequested())) && System.currentTimeMillis() < end)
//		{
//			averageEncoderLeftCorner  = (Math.abs((robot.front_left.getCurrentPosition()  + robot.back_right.getCurrentPosition())/2));
//			headingAngle();
//			drive = -power;
//			turn  = .05 * currHeading;
//
//			topRight    = Range.clip(drive + turn, -1.0, 1.0);
//			topLeft     = Range.clip(drive - turn, -1.0, 1.0);
//			bottomRight = Range.clip(drive + turn, -1.0, 1.0);
//			bottomLeft  = Range.clip(drive - turn, -1.0, 1.0);
//
//			robot.front_right.setPower(topRight);
//			robot.front_left.setPower(topLeft);
//			robot.back_right.setPower(bottomRight);
//			robot.back_left.setPower(bottomLeft);
//		}
//	}
//	robot.resetEncoder();
//	robot.Halt();
//}
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---No More Methods Are Made Past This---//
//----------------------------------------//
////--------------------------------------//
////--------------------------------------------------------------------------------------------------





























































































//--------------------------------------------------------------------------------------------------
    public void runOpMode()
    {
        turnIMU();
        driveIMU();
//        setupVuforia();
//        lastKnownLocation = createMatrix(0, 500, 0, 90, 0, 90);








        telemetry.addData("Status","Initialized");
        telemetry.update();
        waitForStart();
//        visionTargets.activate();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
//----------------------------------
//            vufoCrap();
	
			sleep(2000);
			turnAngle(45,1000);
			sleep(2000);
			turnAngle(20,1000);
			sleep(2000);
			turnAngle(-50,1000);
			sleep(2000);
			turnAngle(10,1000);
			sleep(2000);
			turnAngle(180,1000);
			telemetry.addLine("Beginning Side Move Forward");
			telemetry.update();
			
            sleep(2000);
            moveDistance(20,.1,2000);
            sleep(2000);
            moveDistance(-20,.1,2000);
            telemetry.addLine("Beginning Side Move Testing");
            telemetry.update();

//            sleep(4000);
//			telemetry.addLine("Line Check 0");
//			telemetry.update();
//            moveSide(20,.1,4000,true);
//            sleep(2000);
//            moveSide(25,.1,4000,false);
//            sleep(4000);
//			telemetry.addLine("Beginning Turn Angle Testing");
//			telemetry.update();
            stop();
//----------------------------------
        }
    }
}
//--------------------------------------------------------------------------------------------------