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

@TeleOp(name="leagueSolo", group = "Concept")
//@Disabled
public class leagueTele extends LinearOpMode
{
    leagueMap robot = new leagueMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    boolean intakeSpinningInward = false;
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
private void holonomicDrive()
{
    if(gamepad1.dpad_right && gamepad1.dpad_up)//Drives diagonally at +45
    {
        robot.front_right.setPower(1);
        robot.front_left.setPower(-1);
        robot.back_right.setPower(-1);
        robot.back_left.setPower(1);
    }
    else if(gamepad1.dpad_right && gamepad1.dpad_down)//Drives diagonally at -45
    {
        robot.front_right.setPower(1);
        robot.front_left.setPower(-1);
        robot.back_right.setPower(-1);
        robot.back_left.setPower(1);
    }
    else if(gamepad1.dpad_left && gamepad1.dpad_up)//Drives diagonally at 135
    {
        robot.front_right.setPower(1);
        robot.front_left.setPower(-1);
        robot.back_right.setPower(-1);
        robot.back_left.setPower(1);
    }
    else if(gamepad1.dpad_left && gamepad1.dpad_down)//Drives diagonally at -135
    {
        robot.front_right.setPower(1);
        robot.front_left.setPower(-1);
        robot.back_right.setPower(-1);
        robot.back_left.setPower(1);
    }
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
            angleBoi();

            double leftPower;
            double rightPower;

            double drive = (-gamepad1.left_stick_y * 60)/100;
            double turn  = gamepad1.right_stick_x;
//----------------------------------
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//----------------------------------
            //This drives the robot forward
            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
//--------------------------------------------------------------------------------------------------
            //This is driving the bot sideways - this uses the LEFT TOGGLE XXXXXXXX AXISSSSSSSSSSSSS
            if(gamepad1.left_stick_x < 0)//Moves the bot to the left
            {
                robot.front_right.setPower(-1);
                robot.front_left.setPower(1);
                robot.back_right.setPower(1);
                robot.back_left.setPower(-1);
            }
            //This moves the robot to the right
            else if(gamepad1.left_stick_x > 0)//Moves the bot to the right
            {
                robot.front_right.setPower(1);
                robot.front_left.setPower(-1);
                robot.back_right.setPower(-1);
                robot.back_left.setPower(1);
            }
//--------------------------------------------------------------------------------------------------
            //These are the intake motor controls using buttons AAAAAAAAAAAAAAAAA && BBBBBBBBBBBBBBB
            if(gamepad1.a && !intakeSpinningInward)
            {
                robot.intake1.setPower(-.5);
                robot.intake2.setPower(-.5);
                intakeSpinningInward = true;
            }
            else if(gamepad1.a && intakeSpinningInward)
            {
                robot.intake1.setPower(.5);
                robot.intake2.setPower(.5);
                intakeSpinningInward = false;
            }
            else if(gamepad1.b)
            {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
                intakeSpinningInward = false;
            }
//--------------------------------------------------------------------------------------------------
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