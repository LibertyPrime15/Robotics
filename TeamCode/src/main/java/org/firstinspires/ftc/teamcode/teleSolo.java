package org.firstinspires.ftc.teamcode;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.File;

@TeleOp(name="teleSolo", group = "B")
//@Disabled
//--------------------------------------------------------------------------------------------------
//----------------------------------------------------//
//----------------------------------------------------//
//--This is where I start the program declare stuffs--//
//----------------------------------------------------//
//----------------------------------------------------//
//--------------------------------------------------------------------------------------------------
public class teleSolo extends LinearOpMode
{
    RevMap robot = new RevMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    boolean liftStartesDown  = false;

//    private boolean niFile;
//    private boolean knightsFile;
//    private boolean deadFile;
//    private boolean shrubberyFile;

    double liftSteps = 770;
    double armSteps  = 1120 * .7;

    boolean armIsExtended = false;
    boolean liftDown      = false;
    boolean servosAreOpen = false;
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------
private void imuInit()
{
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json";
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    robot.init(hardwareMap);
    imu = hardwareMap.get(BNO055IMU.class,"imu");
    imu.initialize(parameters);
}
//--------------------------------------------------------------------------------------------------
private double angleBoi()
{
    telemetry.addLine().addData("Heading",currHeading);
    telemetry.update();
    angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
    this.imu.getPosition();
    currHeading = angles.firstAngle;
    return currHeading;
}
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
//        int niID        = hardwareMap.appContext.getResources().getIdentifier("ni", "raw", hardwareMap.appContext.getPackageName());
//        int knightsID  = hardwareMap.appContext.getResources().getIdentifier("knights",   "raw", hardwareMap.appContext.getPackageName());
//        int deadID      = hardwareMap.appContext.getResources().getIdentifier("dead", "raw", hardwareMap.appContext.getPackageName());
//        int shrubberyID = hardwareMap.appContext.getResources().getIdentifier("shrubbery",   "raw", hardwareMap.appContext.getPackageName());
//
//        if (niID != 0)
//            niFile   = SoundPlayer.getInstance().preload(hardwareMap.appContext, niID);
//
//        if (knightsID != 0)
//            knightsFile = SoundPlayer.getInstance().preload(hardwareMap.appContext, knightsID);
//
//        if (deadID != 0)
//            deadFile = SoundPlayer.getInstance().preload(hardwareMap.appContext, deadID);
//
//        if (shrubberyID != 0)
//            shrubberyFile = SoundPlayer.getInstance().preload(hardwareMap.appContext, shrubberyID);


        imuInit();

//        telemetry.addData("niFile",      niFile ?   "Found" : "NOT found\n Add ni.wav to /src/main/res/raw" );
//        telemetry.addData("knightsFile", knightsFile ? "Found" : "Not found\n Add knights.wav to /src/main/res/raw" );
//        telemetry.addData("deadID",      deadFile ?   "Found" : "NOT found\n Add dead.wav to /src/main/res/raw" );
//        telemetry.addData("shrubberyID", shrubberyFile ? "Found" : "Not found\n Add shrubbery.wav to /src/main/res/raw" );

        waitForStart();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
            angleBoi();
//--------------------------------------------------------------------------------------------------
//            This is the block of code for driving and turning the robot
            double leftPower;
            double rightPower;
            double drive = gamepad1.right_stick_y;
            double turn  = -gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//----------------------------------
            //This drives the robot forward
            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
//--------------------------------------------------------------------------------------------------
            //This keeps the lift always flipped up so that it won't fall over and break the program
            if(!liftStartesDown)
            {
                if(robot.lift.getCurrentPosition() < 100 && !liftDown)//Less then 100 = armUp
                {
                    telemetry.addLine("The arm is Up exactly as it should be muy guy");
                    telemetry.update();
                }
//----------------------------------
                else if(robot.lift.getCurrentPosition() > 5 && !liftDown)//Greater than 200 = liftDown
                {
                    while(robot.lift.getCurrentPosition() > 5 && (!isStopRequested()))
                    {
                        robot.lift.setPower(-1);
                        telemetry.addData("Realigning the Arm",robot.lift.getCurrentPosition());
                        telemetry.update();
                    }
                }
            }
//--------------------------------------------------------------------------------------------------
//-------------------------------------------------------//
//-------------------------------------------------------//
//-------------Below are my Gamepad controls for ! CONTROLLER-------------//
//-------------------------------------------------------//
//-------------------------------------------------------//
//--------------------------------------------------------------------------------------------------
            //This section extends the arm ---------------- BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB
            if(gamepad1.b && !armIsExtended)
            {
                robot.resetArm();
                while(armSteps > robot.arm.getCurrentPosition() && (!(isStopRequested())))
                {
                    robot.arm.setPower(.7);
                }
                robot.arm.setPower(0);
                robot.resetArm();
                armIsExtended = true;
            }
//-----------------------------------------------------------------------
            else if(gamepad1.b && armIsExtended)
            {
                robot.resetArm();
                while(-armSteps < robot.arm.getCurrentPosition() && (!(isStopRequested())))
                {
                    robot.arm.setPower(-.7);
                }
                robot.arm.setPower(0);
                robot.resetArm();
//--------------------
                armIsExtended = false;
            }
//--------------------------------------------------------------------------------------------------
            //This closes the claw --------------------- XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
            if(gamepad1.x && servosAreOpen)
            {
                if(robot.claw1.getPosition() <= 0)
                {
                    robot.claw1.setPosition(.5);
                    robot.claw2.setPosition(-.5);
                }
                else
                {
                    robot.claw1.setPosition(-.5);
                    robot.claw2.setPosition(.5);
                }
                servosAreOpen = false;
            }
            else if(((gamepad1.x && !servosAreOpen)) || ((!gamepad1.x && servosAreOpen)))
            {
                telemetry.addData("Can Toggle = ",servosAreOpen);
                telemetry.update();
            }
            else if(!gamepad1.x && !servosAreOpen)
            {
                servosAreOpen = true; // this resets it for the next cycle only if the button was pressed, then released
            }
            else
            {
                telemetry.addData("Servos Are Open = ",servosAreOpen);
                telemetry.update();
            }
//--------------------------------------------------------------------------------------------------
            //This tells the robot if the arm starts up or down
            if(gamepad1.dpad_down)
            {
                liftStartesDown = true;
                liftDown = true;
            }
//--------------------------------------------------------------------------------------------------
            //This moves the arm up --------------- RIGHTTTTTTTTTTTT STICKKKKKKKKKKKKKKKK YYYYYYYYY
            if(gamepad1.right_stick_y!=0)
            {
                robot.arm.setPower(-gamepad1.right_stick_y);
            }
//--------------------
            else
            {
                robot.arm.setPower(0);
            }
//--------------------------------------------------------------------------------------------------
            //if the lift didn't start in the down position, but rather the up position
            if(!liftStartesDown)
            {
                //Moves the lift up ---------------------------------- AAAAAAAAAAAAAAAAAAA
                if(gamepad1.a && liftDown)
                {
                    while(-liftSteps < robot.lift.getCurrentPosition() && (!(isStopRequested())))
                    {
                        robot.lift.setPower(-.8);
                        leftPower = Range.clip(drive + turn,-1.0,1.0);
                        rightPower = Range.clip(drive - turn,-1.0,1.0);
                        //This drives the robot forward
                        robot.front_right.setPower(rightPower);
                        robot.front_left.setPower(leftPower);
                        robot.back_right.setPower(rightPower);
                        robot.back_left.setPower(leftPower);
                        if (gamepad1.right_stick_y != 0)
                        {
                            robot.arm.setPower(-gamepad1.right_stick_y);
                        }
                        else
                        {
                            robot.arm.setPower(0);
                        }
                    }
                    liftDown = false;
                    robot.resetLift();
                    robot.lift.setPower(0);
                }
            }
            //moves the lift down
            else if(gamepad1.a && !liftDown && !liftStartesDown)
            {
                while(liftSteps > robot.lift.getCurrentPosition() && (!(isStopRequested())))
                {
                    robot.lift.setPower(.8);
                    if(gamepad1.left_stick_y !=0)
                    {
                        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
                        //This drives the robot forward
                        robot.front_right.setPower(rightPower);
                        robot.front_left.setPower(leftPower);
                        robot.back_right.setPower(rightPower);
                        robot.back_left.setPower(leftPower);
                    }
                    if(gamepad1.right_stick_y !=0)
                    {
                        robot.arm.setPower(-gamepad1.right_stick_y);
                    }
                    else
                    {
                        robot.arm.setPower(0);
                    }
                }
                liftDown = true;
                robot.resetLift();
                robot.lift.setPower(0);
            }
            else
            {
                robot.lift.setPower(0);
            }
//-------------------------------------------------------
            //if the lift didn't start in the down position, but rather the up position
            if(liftStartesDown)
            {
                //Moves the lift up ---------------------------------- AAAAAAAAAAAAAAAAAAA
                if(gamepad1.a && liftDown)
                {
                    while(-liftSteps < robot.lift.getCurrentPosition() && (!(isStopRequested())))
                    {
                        robot.lift.setPower(-.8);
                        if(gamepad1.left_stick_y !=0)
                        {
                            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//----------------------------------
                            //This drives the robot forward
                            robot.front_right.setPower(rightPower);
                            robot.front_left.setPower(leftPower);
                            robot.back_right.setPower(rightPower);
                            robot.back_left.setPower(leftPower);
                        }
                        if(gamepad1.right_stick_y !=0)
                        {
                            robot.arm.setPower(-gamepad1.right_stick_y);
                        }
                        else
                        {
                            robot.arm.setPower(0);
                        }
                    }
                    liftDown = false;
                    liftStartesDown = false;
                    robot.resetLift();
                    robot.lift.setPower(0);
                }
                else
                {
                    robot.lift.setPower(0);
                }
            }
//--------------------------------------------------------------------------------------------------
//-------------------------------------------//
//-------------------------------------------//
//---There is No More Code FOR CONTROLLER 1 Past This Point---//
//-------------------------------------------//
//-------------------------------------------//
//--------------------------------------------------------------------------------------------------
//            if(gamepad2.a)
//            {
//                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, niID);
//                telemetry.addData("Playing", "Ni File");
//                telemetry.update();
//            }
//
//            else if(gamepad2.b)
//            {
//                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, knightsID);
//                telemetry.addData("Playing", "Knights File");
//                telemetry.update();
//            }
//            else if(gamepad2.x)
//            {
//                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, deadID);
//                telemetry.addData("Playing", "Dead File");
//                telemetry.update();
//            }
//            else if(gamepad2.y)
//            {
//                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, shrubberyID);
//                telemetry.addData("Playing", "Shrubbery File");
//                telemetry.update();
//            }
//--------------------------------------------------------------------------------------------------
//-------------------------------------------//
//-------------------------------------------//
//---There is No More Code Past This Point---//
//-------------------------------------------//
//-------------------------------------------//
//--------------------------------------------------------------------------------------------------
        }
    }
}