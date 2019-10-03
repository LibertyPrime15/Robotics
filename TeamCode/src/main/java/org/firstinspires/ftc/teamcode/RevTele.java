package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="Rev Tele", group = "Main")
//@Disabled
public class RevTele extends LinearOpMode
{
    RevMap robot = new RevMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
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
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);
}
//--------------------------------------------------------------------------------------------------
private double angleBoi()
{
    telemetry.addLine().addData("Heading", currHeading);
    telemetry.update();
    angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    this.imu.getPosition();
    currHeading = angles.firstAngle;
    return currHeading;
}
//--------------------------------------------------------------------------------------------------



























































//--------------------------------------------------------------------------------------------------
    public void runOpMode()
    {
        imuInit();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
            angleBoi();
//----------------------------------
            //This drives the robot forward
            if(gamepad1.left_stick_y !=0)
            {
                robot.front_right.setPower(gamepad1.left_stick_y);
                robot.front_left.setPower(gamepad1.left_stick_y);
                robot.back_right.setPower(gamepad1.left_stick_y);
                robot.back_left.setPower(gamepad1.left_stick_y);
                telemetry.update();
            }
//----------------------------------
            //This turns the robot to the left
            else if(gamepad1.left_stick_x > 0)
            {
                robot.front_right.setPower(gamepad1.left_stick_x);
                robot.front_left.setPower(-gamepad1.left_stick_x);
                robot.back_right.setPower(gamepad1.left_stick_x);
                robot.back_left.setPower(-gamepad1.left_stick_x);
                telemetry.update();
            }
//----------------------------------
            //This turns the robot to the right
            else if(gamepad1.left_stick_x < 0)
            {
                robot.front_right.setPower(gamepad1.left_stick_x);
                robot.front_left.setPower(-gamepad1.left_stick_x);
                robot.back_right.setPower(gamepad1.left_stick_x);
                robot.back_left.setPower(-gamepad1.left_stick_x);
                telemetry.update();
            }

            else
            {
                robot.front_right.setPower(0);
                robot.front_left.setPower(0);
                robot.back_right.setPower(0);
                robot.back_left.setPower(0);
            }
//----------------------------------
            //This opens the claw
            if(gamepad1.y)
            {
                robot.claw1.setPosition(0);
                robot.claw2.setPosition(0);
            }
//----------------------------------
            //This closes the claw
            if(gamepad1.x)
            {
                robot.claw1.setPosition(.5);
                robot.claw2.setPosition(.5);
            }
//----------------------------------
            //This moves the arm up
            if(gamepad1.left_trigger !=0)
            {
                robot.arm.setPower(-.3);
            }

            //This moves the arm down
            else if(gamepad1.right_trigger !=0)
            {
                robot.arm.setPower(.3);
            }

            //Otherwise, the arm shouldn't move
            else
            {
                robot.arm.setPower(0);
            }
//----------------------------------
            //This move the arm around
            if(gamepad1.right_stick_y !=0)
            {
                robot.lift.setPower(gamepad1.right_stick_y);
            }

            //Otherwise, the list shouldn't move
            else
            {
                robot.lift.setPower(0);
            }
//----------------------------------
            telemetry.update();
        }
    }
}
//--------------------------------------------------------------------------------------------------