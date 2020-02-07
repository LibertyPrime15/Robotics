package org.firstinspires.ftc.robotcontroller.internal;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="leagueTele", group = "Concept")
//@Disabled
public class leagueTele extends LinearOpMode
{
    leagueMap robot = new leagueMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    boolean intaking = false;
    int targetPosition = 0;
    boolean canToggle1 = true;
    boolean canToggle2 = true;
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
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        this.imu.getPosition();
        currHeading = angles.firstAngle;
        return currHeading;
    }

    private void intake()
    {
        robot.intake1.setPower(-0.2);
        robot.intake2.setPower(-0.2);
        intaking = true;
    }

    public void outtake()
    {
        robot.intake1.setPower(1);
        robot.intake2.setPower(1);
        intaking = false;
    }

    public void stopIntake()
    {
        robot.intake1.setPower(0);
        robot.intake2.setPower(0);

        intaking = false;
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
        imuInit();
        waitForStart();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
//--------------------------------------------------------------------
//            angleBoi();
            //This is what allows for holonomic drive -- the y values for each stick, plus the side to side or turn values
            //This will do parallel holonomic on the left stick and normal turning on the right stick
            robot.front_left.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
            robot.front_right.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);
            robot.back_left.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
            robot.back_right.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
//------------------------------------------------
            //This is the function that allows for the intake toggle to work, on the left bumper
            if(canToggle1 && gamepad1.left_bumper)
            {
                if(intaking)
                {
                    outtake();
                }
                else
                {
                    intake();
                }
                canToggle1 = false;
            }
            else if(!canToggle1 && !gamepad1.left_bumper)
            {
                canToggle1 = true;
            }

//--------------------------------------------------------------------------------------------------
            //This allows for the grabber toggle to work -- this should only be used for the placing of blocks, as the intake
            //method will allow for the grabber to grab
            if(canToggle2 && gamepad1.right_bumper)
            {
                if(robot.grabber.getPosition() == 0.1 && canToggle2)
                {
                    robot.grabber.setPosition(0.80);
                }
                else
                {
                    robot.grabber.setPosition(0.1);
                }
                canToggle2 = false;
            }
            else if(!canToggle2 && !gamepad1.right_bumper)
            {
                canToggle2 = true;
            }

//------------------------------------------------------------------
            //This is the method that alows for the lift to move to a target position
            //the range in +- 50 encoder steps, which isn't great, and the power will mess with the accuracy a lot.
            //This code should be replaced ASAP with code using the dcmotor.setTargetPosition() method, which will
            //do a better job with accuracy and overcoming torque issues without overshooting the target position
//            if(robot.liftPrimary.getCurrentPosition() < targetPosition - 50)
//            {
//                robot.liftPrimary.setPower(1);
//                robot.liftSecondary.setPower(1);
//            }
//            else if(robot.liftPrimary.getCurrentPosition() > targetPosition + 50);
//            {
//                robot.liftPrimary.setPower(-1);
//                robot.liftSecondary.setPower(-1);
//            }
//            else
//            {
//                robot.liftPrimary.setPower(0);
//                robot.liftSecondary.setPower(0);
//            }

            robot.liftPrimary.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            robot.liftSecondary.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

//--------------------------------------------------------------------------------------------------
            //This is what sets the target position, based on the buttons on gamepad 1. TThey are each assigned a
            //different level, and we may have issues with there not being enough buttons for the different levels.
//            if(gamepad1.a)
//            {
//                targetPosition = 0;
//            }
//            else if(gamepad1.b)
//            {
//                targetPosition = [[just above brick]];
//            }
//            else if(gamepad1.x)
//            {
//                targetPosition = [[block height 1]];
//            }
//            else if(gamepad1.y)
//            {
//                targetPosition = [[block height 2]];
//            }
//--------------------------------------------------------------------------------------------------
            //This is what moves the end effector to it's place positions. The D-pad buttons will all correspond to a different
            //position, which will be the space of the block on the tower as if the robot was facing the same direction as the driver
            if(gamepad1.dpad_up)
            {
                robot.flip1.setPosition(0.22);
                robot.flip2.setPosition(0.22);
                robot.wrist.setPosition(0.99);
                robot.rotate.setPosition(0.666);
            }
            if(gamepad1.dpad_down)
            {
                robot.flip1.setPosition(0.22);
                robot.flip2.setPosition(0.22);
                robot.wrist.setPosition(0.99);
                robot.rotate.setPosition(0);
            }
            if(gamepad1.dpad_left)
            {
                robot.flip1.setPosition(0.22);
                robot.flip2.setPosition(0.22);
                robot.wrist.setPosition(0.99);
                robot.rotate.setPosition(1);
            }
            if(gamepad1.dpad_right)
            {
                robot.flip1.setPosition(0.22);
                robot.flip2.setPosition(0.22);
                robot.wrist.setPosition(0.99);
                robot.rotate.setPosition(0.3333);
            }
//--------------------------------------------------------------------------------------------------
//            This is what initiates and completes an intake cycle -- it will bring the lift down, turn on the intake motors, and \
//            set the position of the grabber to be ready to pick up a block
            if(gamepad1.left_stick_button)
            {
                robot.flip1.setPosition(0.99);
                robot.flip2.setPosition(0.99);
                robot.wrist.setPosition(0.03);
                robot.rotate.setPosition(0.03);

//                targetPosition = [[just above brick]];

                robot.grabber.setPosition(0.1);

                intake();

            }
            //This is what completes an intake cycle -- it will drop the lift down and turn off the intake, and grab the block.
            if(gamepad1.right_stick_button)
            {
                robot.flip1.setPosition(0.99);
                robot.flip2.setPosition(0.99);
                robot.wrist.setPosition(0.03);
                robot.rotate.setPosition(0.03);

//                targetPosition = 0;

                stopIntake();

                sleep(500);

                robot.grabber.setPosition(0.8);
            }
// -------------------------------------------------------------------------------------------------

        }
    }
}
//--------------------------------------------------------------------------------------------------
//-------------------------------------------//
//-------------------------------------------//
//---There is No More Code Past This Point---//
//-------------------------------------------//
//-------------------------------------------//