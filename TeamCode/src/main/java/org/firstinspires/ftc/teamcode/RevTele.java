package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@TeleOp(name="Rev Tele", group = "Main")
//@Disabled
public class RevTele extends LinearOpMode
{
    RevMap robot = new RevMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    boolean Position = true;
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
//private void moveLift()
//{
//    int totDistInSteps = 787;
//
//    if(gamepad1.a && Position == false)
//    {
//        while(totDistInSteps > robot.lift.getCurrentPosition() && (!(isStopRequested())))
//        {
//            telemetry.update();
//            robot.lift.setPower(.5);
//        }
//        Position = true;
//    }
//
//    else if(gamepad1.b && Position == true)
//    {
//        while(-totDistInSteps < robot.lift.getCurrentPosition() && (!(isStopRequested())))
//        {
//            telemetry.update();
//            robot.lift.setPower(-.5);
//        }
//        Position = false;
//    }
//    else
//    {
//        robot.lift.setPower(0);
//        robot.resetLift();
//    }
//}
//--------------------------------------------------------------------------------------------------
private void drive()
{
    double leftPower;
    double rightPower;

    double drive = gamepad1.left_stick_y;
    double turn  = -gamepad1.left_stick_x;

    leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
    rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//----------------------------------
    //This drives the robot forward
    robot.front_right.setPower(rightPower);
    robot.front_left.setPower(leftPower);
    robot.back_right.setPower(rightPower);
    robot.back_left.setPower(leftPower);
    telemetry.update();
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
            angleBoi();
            drive();
//            moveLift();
//----------------------------------
            //This closes the claw
            if(gamepad1.y)
            {
                robot.claw1.setPosition(0);
                robot.claw2.setPosition(0);
            }
//----------------------------------
            //This opens the claw
            if(gamepad1.x)
            {
                robot.claw1.setPosition(.5);
                robot.claw2.setPosition(.5);
            }
//----------------------------------
            //This moves the arm up
            if(gamepad1.left_trigger !=0)
            {
                robot.arm.setPower(-.5);
            }

            //This moves the arm down
            else if(gamepad1.right_trigger !=0)
            {
                robot.arm.setPower(.5);
            }

            //Otherwise, the arm won't move
            else
            {
                robot.arm.setPower(0);
            }
//----------------------------------
            if(gamepad1.right_stick_y !=0)
            {
                robot.lift.setPower(-.5);
            }
            else if(gamepad1.left_stick_y !=0)
            {
                robot.lift.setPower(.5);
            }
            else
            {
                robot.lift.setPower(0);
            }
//--------------------------------------------------------------------
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