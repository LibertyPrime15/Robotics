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
    boolean canToggle = true;

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
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }
            if(gamepad1.x && canToggle)//If it is not moving or is spinning outward
            {
                if(robot.intake1.getPower() <= 0)//If it is not moving or spinning inward - spin inward
                {
                    robot.intake1.setPower(-.5);
                    robot.intake2.setPower(-.5);
                }
                else
                {
                    robot.intake1.setPower(.5);
                    robot.intake2.setPower(.5);
                }
                canToggle = false;
            }
            else if((!gamepad1.x && !canToggle) || (gamepad1.x && canToggle))
            {
                telemetry.addData("Can Toggle = ",canToggle);
                telemetry.update();
            }
            else if(!gamepad1.x && !canToggle)
            {
                canToggle = true;
            }
//--------------------------------------------------------------------------------------------------
            if(gamepad1.right_trigger !=0)
            {
                robot.liftPrimary.setPower(Math.abs(gamepad1.right_trigger));
                robot.liftPrimary.setPower(Math.abs(gamepad1.right_trigger));
            }
            else if(gamepad1.left_trigger !=0)
            {
                robot.liftPrimary.setPower(-1 *(gamepad1.left_trigger));
                robot.liftSecondary.setPower(-1 *(gamepad1.left_trigger));
            }
            else
            {
                robot.liftPrimary.setPower(0);
                robot.liftSecondary.setPower(0);
            }
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