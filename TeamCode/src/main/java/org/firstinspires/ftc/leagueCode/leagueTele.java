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
            angleBoi();

            double frontRight;
            double frontLeft;
            double backRight;
            double backLeft;

            frontRight = gamepad1.right_stick_y + gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            frontLeft  = gamepad1.right_stick_y + gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            backRight  = gamepad1.right_stick_y + gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            backLeft   = gamepad1.right_stick_y + gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
////----------------------------------
//            //This drives the robot forward
            robot.front_right.setPower(frontRight);
            robot.front_left.setPower(frontLeft);
            robot.back_right.setPower(backRight);
            robot.back_left.setPower(backLeft);
//--------------------------------------------------------------------------------Forward & Backward
            //This drives the robot forward & backward - using LEFT TOGGLE YYYYYYYY AXISSSSSSSSSSSSS
//            if(gamepad1.left_stick_y !=0)
//            {
//                robot.front_right.setPower(gamepad1.left_stick_y);
//                robot.front_left.setPower(gamepad1.left_stick_y);
//                robot.back_right.setPower(gamepad1.left_stick_y);
//                robot.back_left.setPower(gamepad1.left_stick_y);
//            }
////------------------------------------------------------------------------------------------Sideways
//            //This is driving the bot sideways - this uses the LEFT TOGGLE XXXXXXXX AXISSSSSSSSSSSSS
//            else if(gamepad1.left_stick_x > 0)//Moves the bot to the left
//            {
//                robot.front_right.setPower(-1);
//                robot.front_left.setPower(1);
//                robot.back_right.setPower(1);
//                robot.back_left.setPower(-1);
//            }
//            //This is driving the bot sideways - this uses the LEFT TOGGLE XXXXXXXX AXISSSSSSSSSSSSS
//            else if(gamepad1.left_stick_x < 0)//Moves the bot to the right
//            {
//                robot.front_right.setPower(1);
//                robot.front_left.setPower(-1);
//                robot.back_right.setPower(-1);
//                robot.back_left.setPower(1);
//            }
////-------------------------------------------------------------------------------------------Turning
//            //This code is for turning the robot - this uses the RIGHT TOGGLE XXXXXXXXX AXISSSSSSSSS
//            else if(gamepad1.right_stick_x < 0)
//            {
//                robot.front_right.setPower(gamepad1.right_stick_x);
//                robot.front_left.setPower(-gamepad1.right_stick_x);
//                robot.back_right.setPower(gamepad1.right_stick_x);
//                robot.back_left.setPower(-gamepad1.right_stick_x);
//            }
////            //This code is for turning the robot - this uses the RIGHT TOGGLE XXXXXXXXX AXISSSSSSSSS
//            else if(gamepad1.right_stick_x > 0)//This is for turning the robot to the left
//            {
//                robot.front_right.setPower(gamepad1.right_stick_x);
//                robot.front_left.setPower(-gamepad1.right_stick_x);
//                robot.back_right.setPower(gamepad1.right_stick_x);
//                robot.back_left.setPower(-gamepad1.right_stick_x);
//            }
//----------------------------------------------------------------------------------------------Stop
            //This stops the robot from moving if none of the other things are happeningggggggggggg
//            else
//            {
//                robot.front_right.setPower(0);
//                robot.front_left.setPower(0);
//                robot.back_right.setPower(0);
//                robot.back_left.setPower(0);
//            }
//--------------------------------------------------------------------------------------------------
            //These are the intake motor controls using buttons AAAAAAAAAAAAAAAAA && BBBBBBBBBBBBBBB
            if(gamepad1.b)
            {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }
            if(gamepad1.x && canToggle)//If it is not moving or is spinning outward
            {
                if(isSpinningInward)//If it is not moving or spinning inward - spin inward
                {
                    robot.intake1.setPower(-.5);
                    robot.intake2.setPower(-.5);
                    isSpinningInward = true;
                }
                else
                {
                    robot.intake1.setPower(.5);
                    robot.intake2.setPower(.5);
                    isSpinningInward = false;
                }
                canToggle = false;
            }
            else if(!gamepad1.x && !canToggle)
            {
                canToggle = true; // this resets it for the next cycle only if the button was pressed, then released
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