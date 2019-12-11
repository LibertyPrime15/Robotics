package org.firstinspires.ftc.leagueCode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="leagueSolo", group = "A")
//@Disabled
public class leagueTele extends LinearOpMode
{
    leagueMap robot = new leagueMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    boolean isSpinningInward = false;
//    boolean canToggleIntake  = true;
    boolean canToggleGrabber = true;

    int armHeight = 0;
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
            telemetry.addData("The intake is intaking a block = ",robot.canToggleIntake);

            double frontRight;
            double frontLeft;
            double backRight;
            double backLeft;

            double leftPower;
            double rightPower;
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.left_stick_x;
//----------------------------------
            //This is full Holonomic
            frontRight = gamepad1.right_stick_y + gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            frontLeft  = gamepad1.right_stick_y + gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            backRight  = gamepad1.right_stick_y + gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            backLeft   = gamepad1.right_stick_y + gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            //This is holonomic drive
            robot.front_right.setPower(frontRight);
            robot.front_left.setPower(frontLeft);
            robot.back_right.setPower(backRight);
            robot.back_left.setPower(backLeft);

            //This is standard driving on the right toggle
            leftPower = Range.clip(drive + turn,-1.0,1.0);
            rightPower = Range.clip(drive - turn,-1.0,1.0);
            //This is standard drive on one stick
            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
//--------------------------------------------------------------------------------------------------
            //These are the intake motor controls using buttons AAAAAAAAAAAAAAAAA && BBBBBBBBBBBBBBB
            if(gamepad1.y)
            {
                robot.stopIntake();
            }
            if(gamepad1.x && robot.canToggleIntake)//If it is not moving or is spinning outward
            {
                if(robot.intake1.getPower() <= 0)//If it is not moving or spinning inward - spin inward
                {
                    robot.intake(.5);
                }
                else
                {
                    robot.outtake(.5);
                }
                robot.canToggleIntake = false;
            }
            else if((!gamepad1.x && !robot.canToggleIntake) || (gamepad1.x && robot.canToggleIntake))
            {
                telemetry.addData("Can Toggle = ",robot.canToggleIntake);
                telemetry.update();
            }
            else if(!gamepad1.x && !robot.canToggleIntake)
            {
                robot.canToggleIntake = true;
            }
//--------------------------------------------------------------------------------------------------
            robot.liftPrimary.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            robot.liftSecondary.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//--------------------------------------------------------------------------------------------------
            if(gamepad1.a)
            {
                armHeight = armHeight + 1;//This might sprout a problem
            }
            else if(gamepad1.b)
            {
                armHeight = armHeight - 1;
            }
//-------------------

//--------------------------------------------------------------------------------------------------
            if(canToggleGrabber && gamepad1.right_bumper)
            {
                if(robot.grabber.getPosition() == 0.1 && canToggleGrabber)
                {
                    robot.grabber.setPosition(0.80);
                }
                else
                {
                    robot.grabber.setPosition(0.1);
                }
                canToggleGrabber = false;
            }
            else if(!canToggleGrabber && !gamepad1.right_bumper)
            {
                canToggleGrabber = true;
            }
//--------------------------------------------------------------------------------------------------
            if(gamepad1.left_stick_button)
            {
                robot.flip1.setPosition(0.9);
                robot.flip2.setPosition(0.9);
                robot.wrist.setPosition(0.1);
                robot.rotate.setPosition(0);

//                targetPosition = [[just above brick]];

                robot.grabber.setPosition(0.1);

                robot.intake(.5);
//-------------------
            if(gamepad1.right_stick_button)
            {
                robot.flip1.setPosition(0.9);
                robot.flip2.setPosition(0.9);
                robot.wrist.setPosition(0.1);
                robot.rotate.setPosition(0);

//                targetPosition = 0;

                robot.stopIntake();

                sleep(500);

                robot.grabber.setPosition(0.8);
            }
//--------------------------------------------------------------------------------------------------
                if(gamepad1.dpad_up)
                {
                    robot.flip1.setPosition(0.15);
                    robot.flip2.setPosition(0.15);
                    robot.wrist.setPosition(0.9);
                    robot.rotate.setPosition(0.666);
                }
                if(gamepad1.dpad_down)
                {
                    robot.flip1.setPosition(0.15);
                    robot.flip2.setPosition(0.15);
                    robot.wrist.setPosition(0.9);
                    robot.rotate.setPosition(0);
                }
                if(gamepad1.dpad_left)
                {
                    robot.flip1.setPosition(0.15);
                    robot.flip2.setPosition(0.15);
                    robot.wrist.setPosition(0.9);
                    robot.rotate.setPosition(1);
                }
                if(gamepad1.dpad_right)
                {
                    robot.flip1.setPosition(0.15);
                    robot.flip2.setPosition(0.15);
                    robot.wrist.setPosition(0.9);
                    robot.rotate.setPosition(0.3333);
                }
            }
//--------------------------------------------------------------------------------------------------
            telemetry.update();
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