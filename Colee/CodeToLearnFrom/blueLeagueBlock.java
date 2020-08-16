//The following lines are imports and pull objects from other classes in different directories in order for the objects we have implemented in this class to function properly
package org.firstinspires.ftc.Colee.CodeToLearnFrom;
//^^^^That is the directory that the current class is in
//btw this class probably doesn't work as a whole so likeeeee

import com.qualcomm.hardware.bosch.BNO055IMU;
//For example this is our rev IMU which can be used for navigation and has several different sensors
//Everything below has a purpose as well but most of it is TensorFlow:)

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.Colee.leagueCode.misc.leagueMap;
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
//This is all for the phone^^^^ This determines which side of the UI this specific class will be on, the name of the class on the phone, and the grouping it will be in

//@Disabled
//"@Disabled" will, once uploaded to a phone not show the class as on the phone therefore cannot be run. You can disable classes for cleaning up a cluttered UI on the phone
public class blueLeagueBlock extends LinearOpMode
{
	//Public means the class and it's public attributes can be accessed by other programs, Class is really the prefix for the name of this class which is
	//BlueLeagueBlock and extends LinearOpMode means were are utilizing several different imports and objects within the super class "LinearOpMode"
	leagueMap robot = new leagueMap();
	//^^^ That is our hardware map where we set up all of our hardware, you can set up certain methods, and can also pull public objects
	//However, a hardware map is strictly a reference sheet and NEVER runs, therefore cannot handle any kind of real time data like used with encoders or turning
	Orientation angles;
	//This starts an iteration of Object Orientation known as Angles which is an imported object from the class imported on line 5
	BNO055IMU imuTurn;
	//Objects are completely useless if they are not instantiated, instantiation taking a concept, and creating it with a name.
	//Objects are like an unborn baby, once it's born it can do things and has a name and can have all kinds of unique characteristics, even if it has a twin...
	
	List<Recognition> updatedRecognitions;
	
	public static float tensorLeft;
	public static float tensorRight;
	public float tensorAvgDist;
	//So we have static floats which mean the value of these instantiate// objects will not change even though multiple instantiations of a class could be running
	//Floats are a designation of the type of memory that is going to be stored in the computer itself
	//Floats can old both real number values, letters, and can be objects
	double yPos = 0;
	double xPos = 0;
	//Double is just like float in the sense that they are for memory allocation, doubles can have a larger allocation of memory than char, int, and some others. They can use real numbers

	double testCounter = 0;
	//When a variable is grey and not highlighted like the variable "testCounter" that means it is not being called into use anywhere in this class or another class

	double startTime = System.currentTimeMillis();
	double endTime;
	double actualTime;

	double diameter = 4;//4
	double radius   = (diameter/2);//2
	//You can do calculations when instantiating a variable

	double circ     = (22/18 * (Math.PI * diameter));//12.5
	
	float currHeading;
	float currAngle;
	double Compensation = 1.5;
	//Besides setting a variable to a number like above, you can set them to strings like on lines 95
	double Steps        = 537.6;
	
	boolean canTogglePlateGrabber = true;
	//Variables can also be set to booleans and use true and false conditions
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
    //These variables are string variables and are set to strings
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
//--------------------------------------------------------------------------------------------------
//This method is private meaning it cannot be used outside of the class
//This method is also void meaning it does not have a return variable like some other do below
//Turn angle uses an imported class for our imu and has instances of the degree system created below
private void turnIMU()
{
	BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
	parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
	parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
	parameters.calibrationDataFile = "BNO055IMUCalibration.json";
	parameters.loggingEnabled = true;
	parameters.loggingTag = "IMU";
	parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//^^^That is all mumbo jumbo so you really don't need to mess with it:)

	robot.init(hardwareMap);
//^^^This refers the class to our hardware map so it knows where to look for all of our hardware - in reality this should not be it, it should be in our main run statement``
	imuTurn = hardwareMap.get(BNO055IMU.class,"imu1");
//^^^This establishes the imu as actual hardware for the robot and names it on the phone``
	imuTurn.initialize(parameters);
}
//--------------------------------------------------------------------------------------------------
private void initVuforia()
{
	//This method initializes vuforia and does cool stuff😎
	//You can set which phone camera you would like to use in this method as well
	VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
	parameters.vuforiaLicenseKey = VUFORIA_KEY;
	parameters.cameraDirection = CameraDirection.BACK;
	vuforia = ClassFactory.getInstance().createVuforia(parameters);
}
//--------------------------------------------------------------------------------------------------
private void initTfod()
{
	//This method creates arguements for what the tensorflow should be detecting and initilizes it
	int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
	TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
	tfodParameters.minimumConfidence = 0.77;
	//This adjusts the intensity of the confidence tensorflow has when it thinks it detects a certain image, if it thinks it's the object it will establish that vs taking longer to be sure
	tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
	tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
}
//--------------------------------------------------------------------------------------------------
//If you don't know how to use this method than please don't call it and expect it to work
private void cycleTimeStart()
{
	//Paul wrote this and it says not too mess with it:)
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
//turnAngle takes in the parameters angle and time - angle is the angle you want the robot to turn to and time is a kill switch because the robot will try to get as close as it can to the position and never actually reach
//It because it bounces back and forth above and below the angle slowly, so time just turns it off - it does need to be tuned for every turn
private void turnAngle(double angle, double time)
{
	double start = System.currentTimeMillis();
	double end   = start + time;
	//These variables are for the timer used to stop the turn
	
	double angleDifference = angle - currAngle;//-60, -40 = -20
	double power;//.02*179+179
	//Power is dependent on the distance from our target angle and slows down the closer it is to ut
	
	while(System.currentTimeMillis() < end && !isStopRequested())
	{
		//This while loop will not end unless the timer ends and the distance from the angle is constantly being recalculated on 177
		angleDifference = currAngle - angle;
		telemetry.update();
		//These following if statements are logic for our imu because it uses a 179 degree and -179 degree system
		//0 is straight forward and 180 which is unreachable is the direct backward, negative is clockwise and positive is counter clockwise
		//This code determines the shortest distance to the target angle even if you have to cross over the nonexistent 180 mark
		if(Math.abs(angleDifference) > 180)
		{
			if(angleDifference > 0)
			{
				//telemetry means it is displaying to the phone
				telemetry.addData("----Heading", currAngle);
				telemetry.addData("----angleDifference", angleDifference);
				angles = this.imuTurn.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
				this.imuTurn.getPosition();
				currAngle = angles.firstAngle;
				//^^^ Those lines pull up to date imu angle readings
				
				angleDifference = currAngle - angle;
				power = .02 * (360 - Math.abs(angleDifference));
				robot.turnLeft(power);
				//^^^Power is calculated and then begins turning
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
	//The robot calls 2 methods in the hardware map stopping the robot and resetting the motor's encoder
	robot.Halt();
	robot.resetEncoder();
}
//--------------------------------------------------------------------------------------------------
//This method can be called in other classes because it is public and it returns a variable of type double
//The variable it returns when called is currAngle - this method updates the imu's current angle and assigns a variable to that value
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
//This method calls 2 servos from the hardware map and moves them into a position called via parameters used as arguments when you call the class
//Parameters go where it says "double position" arguments are where it says "(position)"
public void setFlipPosition(double position)
{
	robot.flip2.setPosition(position);
	robot.flip1.setPosition(position);
}
//--------------------------------------------------------------------------------------------------
//This method turns on and off our tensorflow that way the phone camera isn't just on all round
public void turnOffTensorFlow()
{
	tfod.deactivate();
	tfod.shutdown();
}
//--------------------------------------------------------------------------------------------------
//These are our autonomous paths so it is a bunch of our method calls from above using tuned parameters such that depending on the position the block is in, our robot will accomplish certain goals lol
//It's really just not worth my time to explain where the robot is and what it's doing on each line soooooo please just use these as examples of how to call methods
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
//This is our modified wait for start method
//While the robot is initialized tensorflow is scanning for one skystone brick
public void waitForStart()
{
	boolean alreadyRecorded;
	while(!isStarted())
	{
		//If tensorflow is on, keep updating the data
		if(tfod != null)
		{
			tfod.activate();
			updatedRecognitions = tfod.getUpdatedRecognitions();
		}
		//If tfod has detected a stone it determines whether it is a skystone
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
				//Once a skystone is detected, the average distance between two corners is taken to determine where the center of the skystone is, also indicating which place the skystone is in
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
		//Depending on which of these ranges the average is, determines where the skystone is and displays that to the phone
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
		//Once we run the class, tfod turns on and runs the code above^^^
		robot.init(hardwareMap);
		turnIMU();
		initVuforia();
		//If an instance of tfod is not running, than start one
        if(ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        else
		{
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        //This calls our waitForStart method directly above
        waitForStart();
//--------------------------------------------------------------------------------------------------
		//One the play button has been pressed \/
		if(opModeIsActive())
		{
			while(opModeIsActive() && (!(isStopRequested())))
			{
				//These 3 if statements run three distinct paths which are above depending on the average value calculated by tensorFlow
				if(tensorAvgDist > 725)
				{
					blockPositionThree();
				}
				if((tensorAvgDist < 725) && (tensorAvgDist > 550))
				{
					blockPositionTwo();
				}
				if(tensorAvgDist < 550)
				{
					blockPositionOne();
				}
			}
		}
	}
//--------------------------------------------------------------------------------------------------
}