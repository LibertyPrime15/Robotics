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
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
//----------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Let's Go Get this Lego Boi");
        telemetry.update();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

//------------------------------------------------------------
        while(opModeIsActive())
        {
            telemetry.update();
            //This drives the robot forward
            if(gamepad1.left_stick_y !=0)
            {
                robot.front_right.setPower(gamepad1.left_stick_y);
                robot.front_left.setPower(gamepad1.left_stick_y);
                robot.back_right.setPower(gamepad1.left_stick_y);
                robot.back_left.setPower(gamepad1.left_stick_y);
                telemetry.update();
            }

            //This turns the robot to the left
            else if(gamepad1.left_stick_x > 0)
            {
                robot.front_right.setPower(-gamepad1.left_stick_x);
                robot.front_left.setPower(gamepad1.left_stick_x);
                robot.back_right.setPower(-gamepad1.left_stick_x);
                robot.back_left.setPower(gamepad1.left_stick_x);
                telemetry.update();
            }

            //This turns the robot to the right
            else if(gamepad1.left_stick_x < 0)
            {
                robot.front_right.setPower(-gamepad1.left_stick_x);
                robot.front_left.setPower(gamepad1.left_stick_x);
                robot.back_right.setPower(-gamepad1.left_stick_x);
                robot.back_left.setPower(gamepad1.left_stick_x);
                telemetry.update();
            }

            //This opens the claw
            else if(gamepad1.y)
            {
                robot.claw1.setPosition(0);
                robot.claw2.setPosition(.5);
            }

            //This closes the claw
            else if(gamepad1.x)
            {
                robot.claw1.setPosition(.5);
                robot.claw2.setPosition(0);
            }

            //This moves the arm up
            else if(gamepad1.left_trigger !=0)
            {
                robot.arm.setPower(-.3);
            }

            //This moves the arm down
            else if(gamepad1.right_trigger !=0)
            {
                robot.arm.setPower(.3);
            }

            //This move the arm around
            else if(gamepad1.right_stick_y !=0)
            {
                robot.lift.setPower(gamepad1.right_stick_y);
            }

            //This sets all of the motors to 0
            else
            {
                robot.front_right.setPower(0);
                robot.front_left.setPower(0);
                robot.back_right.setPower(0);
                robot.back_left.setPower(0);

                robot.lift.setPower(0);
                robot.arm.setPower(0);
            }
            telemetry.update();
        }
    }

    void composeTelemetry()
    {
        telemetry.addAction(new Runnable()
        {
            @Override public void run()
            {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
            }
        });

        telemetry.addLine()
            .addData("status", new Func<String>()
            {
                @Override public String value()
                {
                    return imu.getSystemStatus().toShortString();
                }
            })
            .addData("calib", new Func<String>()
            {
                @Override public String value()
                {
                    return imu.getCalibrationStatus().toString();
                }
            });

        telemetry.addLine()
            .addData("heading", new Func<String>()
            {
                @Override public String value()
                {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            })
            .addData("roll", new Func<String>()
            {
                @Override public String value()
                {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                }
            })
            .addData("pitch", new Func<String>()
            {
                @Override public String value()
                {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                }
            });
    }

//----------------------------------------------------------------------------------------------
// Formatting
//----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
//----------------------------------------------------------------------------------------------