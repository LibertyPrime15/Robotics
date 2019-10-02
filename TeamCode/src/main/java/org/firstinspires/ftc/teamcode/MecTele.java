package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Mec Tele", group = "Main")
//@Disabled
public class MecTele extends LinearOpMode
{
    MecMap robot = new MecMap();
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
    public void imuInit()
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
    public double angleBoi()
    {
        telemetry.addLine().addData("Heading", currHeading);
        telemetry.update();
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        currHeading = angles.firstAngle;
        return currHeading;
    }
//--------------------------------------------------------------------------------------------------










































    public void runOpMode()
    {
        imuInit();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
            angleBoi();
            //This is for basic movement forward/backward
            if(gamepad1.left_stick_y !=0)
            {
                robot.front_right.setPower(gamepad1.left_stick_y);
                robot.front_left.setPower(gamepad1.left_stick_y);
                robot.back_right.setPower(gamepad1.left_stick_y);
                robot.back_left.setPower(gamepad1.left_stick_y);
            }
//--------------------------------------------------------------------------------------------------
            //This moves the robot to the  left
            else if(gamepad1.left_stick_x < 0)
            {
                robot.front_right.setPower(-gamepad1.left_stick_x);
                robot.front_left.setPower(gamepad1.left_stick_x);
                robot.back_right.setPower(gamepad1.left_stick_x);
                robot.back_left.setPower(-gamepad1.left_stick_x);
            }
            //This moves the robot to the right
            else if(gamepad1.left_stick_x > 0)
            {
                robot.front_right.setPower(-gamepad1.left_stick_x);
                robot.front_left.setPower(gamepad1.left_stick_x);
                robot.back_right.setPower(gamepad1.left_stick_x);
                robot.back_left.setPower(-gamepad1.left_stick_x);
            }
//--------------------------------------------------------------------------------------------------
//            This is for turning the robot to the left
            else if(gamepad1.right_stick_x < 0)
            {
                robot.front_right.setPower(gamepad1.right_stick_x);
                robot.front_left.setPower(-gamepad1.right_stick_x);
                robot.back_right.setPower(gamepad1.right_stick_x);
                robot.back_left.setPower(-gamepad1.right_stick_x);
            }
//            //This is for turning the robot to the left
            else if(gamepad1.right_stick_x > 0)
            {
                robot.front_right.setPower(gamepad1.right_stick_x);
                robot.front_left.setPower(-gamepad1.right_stick_x);
                robot.back_right.setPower(gamepad1.right_stick_x);
                robot.back_left.setPower(-gamepad1.right_stick_x);
            }
//--------------------------------------------------------------------------------------------------
            else
            {
                robot.front_right.setPower(0);
                robot.front_left.setPower(0);
                robot.back_right.setPower(0);
                robot.back_left.setPower(0);
            }
        telemetry.update();
        }
    }
}
