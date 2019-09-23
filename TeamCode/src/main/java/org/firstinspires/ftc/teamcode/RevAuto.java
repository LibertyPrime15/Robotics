package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name ="Rev Auto", group = "Concept")
//@Disabled
public class RevAuto extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    RevMap robot = new RevMap();

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---// This autonomous is broken because the
//----------------------------------------// Gyros do not work - Do not run
//----------------------------------------//
    //Reset all encoder values
    public void resetEncoder()
    {
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Stop
    public void Halt()
    {
        robot.front_right.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.back_left.setPower(0);
    }

    //Forward
    public void Forward(double power)
    {
        robot.front_right.setPower(power);
        robot.front_left.setPower(power);
        robot.back_right.setPower(power);
        robot.back_left.setPower(power);
    }

    //Backward
    public void Backward(double power)
    {
        robot.front_right.setPower(-power);
        robot.front_left.setPower(-power);
        robot.back_right.setPower(-power);
        robot.back_left.setPower(-power);
    }

    //LTurn
    public void LTurn(double power)
    {
        robot.front_right.setPower(power);
        robot.front_left.setPower(-power);
        robot.back_right.setPower(power);
        robot.back_left.setPower(-power);
    }

    //RTurn
    public void RTurn(double power)
    {
        robot.front_right.setPower(-power);
        robot.front_left.setPower(power);
        robot.back_right.setPower(-power);
        robot.back_left.setPower(power);
    }

    //MoveDistance
    public void moveDistance(double length)
    {
        double distPerRot = Math.PI * 3.8125;
        double stepsPerRot = 1120;
        double totDistInSteps = ((length / distPerRot) * stepsPerRot);

        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO FORWARD!
        if (totDistInSteps > 0)
        {
            //Move forward until we over shoot
            while (totDistInSteps >= robot.front_right.getCurrentPosition())
            {
                Forward(.5);
                telemetry.addData("power", robot.front_right.getCurrentPosition());
                telemetry.update();
            }
        }

        //IF THE NUMBER IS A POSITIVE NUMBER WE GO BACKWARD!
        else if (totDistInSteps < 0)
        {
            //Move backward until we over shoot
            while (totDistInSteps <= robot.front_right.getCurrentPosition())
            {
                Backward(.5);
                telemetry.addData("power", robot.front_right.getCurrentPosition());
                telemetry.update();
            }
        }

        //reset our encoder values because we may need to do more encoder driving
        Halt();
        resetEncoder();
        sleep(1000);
    }
//--------------------------------------------------------------------------------------------------
    public void runOpMode() throws InterruptedException
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robot.init(hardwareMap);

        if (opModeIsActive())
        {
            telemetry.update();

            waitForStart();
            runtime.reset();
//--------------------------------------------------------------------------------------------------
            while(opModeIsActive())
            {
                runtime.reset();
//                while (!(robot.gyro.getHeading() >= 30 && robot.gyro.getHeading() <= 35))
                {
                    LTurn(.1);
                }

                moveDistance(-4);
                sleep(1000);
                resetEncoder();

//                while (!(robot.gyro.getHeading() >= 335 && robot.gyro.getHeading() <= 340))
                {
                    RTurn(.1);
                }

                moveDistance(-10);
            }
//--------------------------------------------------------------------------------------------------
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

//        telemetry.addLine()
//                .addData("status", new Func<String>()
//                {
//                    @Override public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>()
//                {
//                    @Override public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });

        telemetry.addLine()
                .addData("heading", new Func<String>()
                {
                    @Override public String value()
                    {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
//                .addData("roll", new Func<String>()
//                {
//                    @Override public String value()
//                    {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>()
//                {
//                    @Override public String value()
//                    {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
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
