package org.firstinspires.ftc.leagueCode.blue;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.leagueCode.leagueMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "tensorFlowAUTOS", group = "Concept")
//@Disabled
public class tensorFlowAUTOS extends LinearOpMode
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
	
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
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
	tfodParameters.minimumConfidence = 0.9;
	tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
	tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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
private void blockPositionOne()
{
	moveDistanceAtAngle(-17, 0, 0.3);
	turnAngle(20, 1500);
	robot.intake(0.05);
	moveDistanceAtAngle(-18, 20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(13, 20, 0.3);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-58, 90, 0.5);
	robot.outtake(0.5);
	sleep(1000);
	robot.stopIntake();
	moveDistanceAtAngle(63, 90, 0.5);
	turnAngle(-20, 2000);
	robot.intake(0.05);
	moveDistanceAtAngle(-19, -20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(12.5, -20, 0.3);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-58, 90, 0.5);
	robot.outtake(0.5);
	sleep(1000);
	robot.stopIntake();
	moveDistanceAtAngle(12, 90, 0.5);
}
//--------------------------------------------------------------------------------------------------
private void blockPositionTwo()
{
	moveDistanceAtAngle(-12, 0, 0.3);
	turnAngle(-45, 2000);
	moveDistanceAtAngle(-6, -45, 0.3);
	turnAngle(20, 2000);
	robot.intake(0.05);
	moveDistanceAtAngle(-20, 20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(18, 20, 0.3);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-58, 90, 0.5);
	robot.outtake(0.05);
	sleep(1000);
	robot.stopIntake();
	moveDistanceAtAngle(56, 90, 0.5);
	turnAngle(-25, 3000);
	robot.intake(0.05);
	moveDistanceAtAngle(-20, -25, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(12, -25, 0.5);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-53, 90, 0.6);
	robot.outtake(0.05);
	sleep(1000);
	robot.stopIntake();
	moveDistanceAtAngle(30, 90, 0.6);
}
//--------------------------------------------------------------------------------------------------
private void blockPositionThree()
{
	moveDistanceAtAngle(-16, 0, 0.3);
	turnAngle(-20, 1000);
	robot.intake(0.05);
	moveDistanceAtAngle(-19, -20, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(13, -20, 0.3);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-58, 90, 0.5);
	robot.outtake(0.5);
	sleep(500);
	robot.stopIntake();
	moveDistanceAtAngle(50, 90, 0.5);
	turnAngle(-60, 2000);
	robot.outtake(1);
	moveDistanceAtAngle(-26, -60, 0.3);
	robot.intake(0.05);
	moveDistanceAtAngle(-8, -60, 0.1);
	robot.stopIntake();
	moveDistanceAtAngle(20, -60, 0.3);
	turnAngle(90, 2000);
	moveDistanceAtAngle(-68, 90, 0.5);
	robot.outtake(0.5);
	sleep(500);
	robot.stopIntake();
	moveDistanceAtAngle(14, 90, 0.5);
}
//--------------------------------------------------------------------------------------------------
    public void runOpMode()
	{
		initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        else
		{
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null)
        {
            tfod.activate();
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
//--------------------------------------------------------------------------------------------------
		if(opModeIsActive())
		{
			float tensorDist;
			while(opModeIsActive() && (!(isStopRequested())))
			{
				if(tfod != null)
				{
					List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
					
					if(updatedRecognitions != null)
					{
						telemetry.addData("# Object Detected", updatedRecognitions.size());
						telemetry.update();
						for(Recognition recognition : updatedRecognitions)
						{
							tensorDist = recognition.getTop();
							if(((recognition.getLabel() == LABEL_FIRST_ELEMENT) && (System.currentTimeMillis() > 6000)))
							{
								blockPositionOne();
							}
							if(tensorDist > 400)
							{
								blockPositionOne();
							}
							if((tensorDist < 400) && (tensorDist > 200))
							{
								blockPositionTwo();
							}
							if((tensorDist < 200) && (tensorDist > 0))
							{
								blockPositionThree();
							}
						}
						telemetry.update();
					}
				}
				if((System.currentTimeMillis() > 6000) && (tfod == null))
				{
					blockPositionOne();
				}
			}
		}
	}
//--------------------------------------------------------------------------------------------------
}