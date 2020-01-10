package org.firstinspires.ftc;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.R;

@TeleOp(name="leagueSolo", group = "A")
//@Disabled
public class newLeagueTele extends LinearOpMode
{
    newLeagueMap robot = new newLeagueMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    // these booleans are all used for toggles
    boolean canTogglePlateGrabber = true;
    boolean canAddToLiftPos = true;
    boolean canSubtractFromLiftPos = true;

    //This boolean tells us if we have a block in the intake
    boolean hasBlock = false;

    //this array allows us to store the encoder values that correspond to different positions we
    // might want the lift to go to
    int[] liftPositions = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //these are the servo positions for the end effector - they allow us to change these values
    // everywhere in the code at once
    double flippedIn = 0.9;
    double flippedOut = 0.15;
    double flipStartPos = 0.7;
    double wristWhenIn = 0.1;
    double wristWhenOut = 0.9;
    double rotateGrab = 0;
    double rotateFar = 0;
    double rotateLeft = 0;
    double rotateClose = 0;
    double grabbed = 0.8;
    double ungrabbed = 0.1;

    //This tells the code what position the lift should be in at the moment
    int currentLiftPos = 0;

    //This is a value that lowers the lift slightly when we want to place a brick
    int blockPlaceValue = 500;

    //these track what our next place position on, to make it so our driver doesn't have to deal
    // with as much
    int nextLiftPos = 0;
    int nextPlacePos = 0;

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
    private double angleBoi()
    {
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        this.imu.getPosition();
        currHeading = angles.firstAngle;
        return currHeading;
    }
//--------------------------------------------------------------------------------------------------
    //this method tells the lift motors to drive towards their target position
    public void driveLiftToPosition()
    {
        int currentPos = robot.liftPrimary.getCurrentPosition();
        int posDifference = currentPos - liftPositions[currentLiftPos];
        double power = posDifference * 0.003;
        robot.liftPrimary.setPower(-power);
        robot.liftSecondary.setPower(power);
    }
//--------------------------------------------------------------------------------------------------
    //this method sets the target position for the lift
    public void setLiftPosition(int position)
    {
        if(position >= 0 && position <= 9)
        {
            currentLiftPos = position;
        }
    }
//--------------------------------------------------------------------------------------------------
    //this method starts an intake cycle - it will start the intake, wait until we have a block, and
    //then grab onto it
    public void initiateIntakeCycle()
    {
        setLiftPosition(liftPositions[1]);
        setFlipPosition(flippedIn);
        robot.rotate.setPosition(rotateGrab);
        robot.grabber.setPosition(ungrabbed);
        robot.wrist.setPosition(wristWhenIn);
        robot.intake(0.1);
        while(!hasBlock && !isStopRequested())
        {
            telemetry.addLine("We are in the initiateIntakeCycle method");
            normalTeleopStuff();
        }
        robot.stopIntake();
        setLiftPosition(0);
        sleep(500);
        robot.grabber.setPosition(grabbed);
    }
//--------------------------------------------------------------------------------------------------
    //this method stores most of our standard (not automated) teleop code like driving and toggling
    //the plate grabber.
    //making this a method allowed us to add this functionality to other methods easier
    public void normalTeleopStuff()
    {
        //--------------------------------------------------------------------------------------------------
        double frontRight;
        double frontLeft;
        double backRight;
        double backLeft;
        //This is full Holonomic
        frontRight = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) + (0.25 * gamepad1.left_stick_x) + gamepad1.right_stick_x;
        frontLeft  = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) - (0.25 * gamepad1.left_stick_x) - gamepad1.right_stick_x;
        backRight  = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) - (0.25 * gamepad1.left_stick_x) + gamepad1.right_stick_x;
        backLeft   = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) + (0.25 * gamepad1.left_stick_x) - gamepad1.right_stick_x;
        //This is holonomic drive
        robot.front_right.setPower(frontRight);
        robot.front_left.setPower(frontLeft);
        robot.back_right.setPower(backRight);
        robot.back_left.setPower(backLeft);
        //--------------------------------------------------------------------------------------------------
        if(robot.sensorColor.red() > 2 * robot.sensorColor.blue())
        {
            hasBlock = true;
        }
        else if(robot.sensorColor.red() <= 2 * robot.sensorColor.blue())
        {
            hasBlock = false;
        }
        telemetry.addData("hasBlock is", hasBlock);
        //--------------------------------------------------------------------------------------------------
        //this spits out a block if for some reason we need to
        if(gamepad1.left_bumper)
        {
            robot.outtake(1);
            sleep(500);
        }
        //--------------------------------------------------------------------------------------------------
        if(canTogglePlateGrabber && gamepad1.left_bumper)
        {
            if(robot.plateGrabber1.getPosition() == 0.73 && canTogglePlateGrabber)
            {
                robot.grabPlate();
            }
            else
            {
                robot.ungrabPlate();
            }
            canTogglePlateGrabber = false;
        }
        else if(!canTogglePlateGrabber && !gamepad1.left_bumper)
        {
            canTogglePlateGrabber = true;
        }
        //--------------------------------------------------------------------------------------------------
        if(canAddToLiftPos && gamepad1.dpad_up && nextLiftPos < 9)
        {
            nextLiftPos++;
            canAddToLiftPos = false;
        }
        else if(!canAddToLiftPos && !gamepad1.dpad_up)
        {
            canAddToLiftPos = true;
        }
        if(canSubtractFromLiftPos && gamepad1.dpad_down && nextLiftPos > 0)
        {
            nextLiftPos--;
            canSubtractFromLiftPos = false;
        }
        else if(!canSubtractFromLiftPos && !gamepad1.dpad_down)
        {
            canSubtractFromLiftPos = true;
        }
        telemetry.addData("Next lift position", nextLiftPos);
        //--------------------------------------------------------------------------------------------------
        if(gamepad2.a)
        {
            nextPlacePos = 0;
            telemetry.addLine("the next brick will be placed close");
        }
        if(gamepad1.x)
        {
            nextPlacePos = 1;
            telemetry.addLine("the next brick will be placed on the left");
        }
        if(gamepad1.y)
        {
            nextPlacePos = 2;
            telemetry.addLine("the next brick will be placed far");
        }
        if(gamepad1.b)
        {
            nextPlacePos = 3;
            telemetry.addLine("the next brick will be placed on the right");
        }
        //--------------------------------------------------------------------------------------------------
        driveLiftToPosition();
        telemetry.update();

    }
//--------------------------------------------------------------------------------------------------
    //this sets both the flip servos to a position.
    public void setFlipPosition(double position)
    {
        robot.flip1.setPosition(position);
        robot.flip2.setPosition(position);
    }
//--------------------------------------------------------------------------------------------------
    //this method moves the lift to it's next place position, as tracked with nextLiftPos and
    // nextPlacePos
    //It allows our driver to press 1 button and have the robot do everything to get ready to place
    public void goToNextPosition()
    {
        double start = System.currentTimeMillis();
        setLiftPosition(nextLiftPos);
        while((System.currentTimeMillis() - start) < 1000 && !isStopRequested())
        {
            telemetry.addLine("We are in the goToNextPosition method");
            normalTeleopStuff();
        }
        setFlipPosition(flippedOut);
        robot.wrist.setPosition(wristWhenOut);
        if(nextPlacePos == 0)
        {
            robot.rotate.setPosition(rotateClose);
        }
        else if(nextPlacePos == 1)
        {
            robot.rotate.setPosition(rotateLeft);
        }
        else if(nextPlacePos == 2)
        {
            robot.rotate.setPosition(rotateFar);
        }
        else
        {
            robot.rotate.setPosition(rotateGrab);
        }
    }
//--------------------------------------------------------------------------------------------------
    //this method drops the block, flips the arm back in, and collapses the slides to get ready for
    // the next cycle
    public void place()
    {
        setLiftPosition(currentLiftPos - blockPlaceValue);
        double start = System.currentTimeMillis();
        setLiftPosition(nextLiftPos);
        while((System.currentTimeMillis() - start) < 1000 && !isStopRequested())
        {
            telemetry.addLine("we are in the place method");
            normalTeleopStuff();
        }
        robot.grabber.setPosition(ungrabbed);
        while((System.currentTimeMillis() - start) < 200 && !isStopRequested())
        {
            telemetry.addLine("we are in the place method");
            normalTeleopStuff();
        }
        setFlipPosition(flippedIn);
        robot.rotate.setPosition(rotateGrab);
        robot.wrist.setPosition(wristWhenIn);
        sleep(300);
        setLiftPosition(0);
        driveLiftToPosition();
    }
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---No More Methods Are Made Past This---//
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------

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
        driveIMU();
        waitForStart();
        robot.sensorColor.enableLed(true);
        setFlipPosition(flipStartPos);
        robot.rotate.setPosition(rotateGrab);
        robot.wrist.setPosition(wristWhenIn);
        robot.grabber.setPosition(ungrabbed);
        robot.ungrabPlate();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
            normalTeleopStuff();
            if(gamepad1.left_stick_button)
            {
                initiateIntakeCycle();
            }
            if(gamepad1.right_stick_button)
            {
                goToNextPosition();
            }
            if(gamepad1.right_bumper)
            {
                place();
            }
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