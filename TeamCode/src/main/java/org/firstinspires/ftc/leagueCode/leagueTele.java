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
//--------------------------------------------------------------------------------------------------
            //This drives the robot forward & backward - using LEFT TOGGLE YYYYYYYY AXISSSSSSSSSSSSS
            if(gamepad1.left_stick_y !=0)
            {
                robot.front_right.setPower(gamepad1.left_stick_y);
                robot.front_left.setPower(gamepad1.left_stick_y);
                robot.back_right.setPower(gamepad1.left_stick_y);
                robot.back_left.setPower(gamepad1.left_stick_y);
            }
            //This is driving the bot sideways - this uses the LEFT TOGGLE XXXXXXXX AXISSSSSSSSSSSSS
            else if(gamepad1.left_stick_x > 0)//Moves the bot to the left
            {
                robot.front_right.setPower(-1);
                robot.front_left.setPower(1);
                robot.back_right.setPower(1);
                robot.back_left.setPower(-1);
            }
            //This is driving the bot sideways - this uses the LEFT TOGGLE XXXXXXXX AXISSSSSSSSSSSSS
            else if(gamepad1.left_stick_x < 0)//Moves the bot to the right
            {
                robot.front_right.setPower(1);
                robot.front_left.setPower(-1);
                robot.back_right.setPower(-1);
                robot.back_left.setPower(1);
            }
            //This code is for turning the robot - this uses the RIGHT TOGGLE XXXXXXXXX AXISSSSSSSSS
            else if(gamepad1.right_stick_x < 0)
            {
                robot.front_right.setPower(gamepad1.right_stick_x);
                robot.front_left.setPower(-gamepad1.right_stick_x);
                robot.back_right.setPower(gamepad1.right_stick_x);
                robot.back_left.setPower(-gamepad1.right_stick_x);
            }
            //This code is for turning the robot - this uses the RIGHT TOGGLE XXXXXXXXX AXISSSSSSSSS
            else if(gamepad1.right_stick_x > 0)//This is for turning the robot to the left
            {
                robot.front_right.setPower(gamepad1.right_stick_x);
                robot.front_left.setPower(-gamepad1.right_stick_x);
                robot.back_right.setPower(gamepad1.right_stick_x);
                robot.back_left.setPower(-gamepad1.right_stick_x);
            }
            //This stops the robot from moving if none of the other things are happeningggggggggggg
            else
            {
                robot.front_right.setPower(0);
                robot.front_left.setPower(0);
                robot.back_right.setPower(0);
                robot.back_left.setPower(0);
            }
//--------------------------------------------------------------------------------------------------
            //This is holonomic Drive ---- We can drive diagonally ---- USING THE DDDDD PAAADDDDDDDD
            if(gamepad1.dpad_right && gamepad1.dpad_up)//Drives diagonally at +45 ---- Done
            {
                robot.front_right.setPower(-1);
                robot.back_left.setPower(-1);
            }
            else if(gamepad1.dpad_right && gamepad1.dpad_down)//Drives diagonally at -45 ---- Done
            {
                robot.front_left.setPower(-1);
                robot.back_right.setPower(-1);
            }
            else if(gamepad1.dpad_left && gamepad1.dpad_up)//Drives diagonally at +135 ---- Done
            {
                robot.front_left.setPower(1);
                robot.back_right.setPower(1);
            }
            else if(gamepad1.dpad_left && gamepad1.dpad_down)//Drives diagonally at -135 ---- Done
            {
                robot.front_right.setPower(-1);
                robot.back_left.setPower(-1);
            }
//--------------------------------------------------------------------------------------------------
            //These are the intake motor controls using buttons AAAAAAAAAAAAAAAAA && BBBBBBBBBBBBBBB
//            if(gamepad1.a && !intakeSpinningInward)
//            {
//                robot.intake1.setPower(-.5);
//                robot.intake2.setPower(-.5);
//                intakeSpinningInward = true;
//            }
//            else if(gamepad1.a && intakeSpinningInward)
//            {
//                robot.intake1.setPower(.5);
//                robot.intake2.setPower(.5);
//                intakeSpinningInward = false;
//            }
            if(gamepad1.x && canToggle)
            {
                if(robot.intake1.getPower() <= 0)
                {
                    robot.intake1.setPower(.5);
                    robot.intake2.setPower(.5);
                }
                else
                {
                    robot.intake1.setPower(-.5);
                    robot.intake2.setPower(-.5);
                }
                canToggle = false;
            }
            else if(((gamepad1.x && !canToggle)) || ((!gamepad1.x && canToggle)))
            {
                telemetry.addData("Can Toggle = ",canToggle);
                telemetry.update();
            }
            else if(!gamepad1.x && !canToggle)
            {
                canToggle = true; // this resets it for the next cycle only if the button was pressed, then released
            }
            else if(gamepad1.b)
            {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
                robot.intake1.getPower();
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