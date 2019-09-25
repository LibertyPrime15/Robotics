package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name ="Rev Auto", group = "Concept")
//@Disabled
public class RevAuto extends LinearOpMode
{
    RevMap robot = new RevMap();
    double curHeading;
    Orientation angles;
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---// This autonomous is broken because the
//----------------------------------------// Gyros do not work - Do not run
//----------------------------------------//
    //Check imu angle
    public double angleCheck()
    {
        angles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.robot.imu.getPosition();
        curHeading = angles.firstAngle;
        return curHeading;
    }
//----------------------------------------//
    //MoveDistance
    public void moveDistance(double length)
    {
        double distPerRot = Math.PI * 3.8125;
        double stepsPerRot = 1120;
        double totDistInSteps = ((length / distPerRot) * stepsPerRot);

        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO FORWARD!
        if (totDistInSteps > 0)
        {
            while (totDistInSteps >= robot.front_right.getCurrentPosition())
            {
                robot.Forward(.5);
                telemetry.addData("power", robot.front_right.getCurrentPosition());
                telemetry.update();
            }
        }
        //IF THE NUMBER IS A POSITIVE NUMBER WE GO BACKWARD!
        else if (totDistInSteps < 0)
        {
            while (totDistInSteps <= robot.front_right.getCurrentPosition())
            {
                robot.Backward(.5);
                telemetry.addData("power", robot.front_right.getCurrentPosition());
                telemetry.update();
            }
        }
        robot.Halt();
        robot.resetEncoder();
        sleep(1000);
    }
//----------------------------------------//
    public void composeTelemetry()
    {
        telemetry.addAction(new Runnable()
        {
            @Override
            public void run()
            {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        telemetry.addLine()
                .addData("heading", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
    }
//--------------------------------------------------------------------------------------------------
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
//--------------------------------------------------------------------------------------------------



























































































//--------------------------------------------------------------------------------------------------
    public void runOpMode() throws InterruptedException
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        robot.imu.initialize(parameters);
        composeTelemetry();

        waitForStart();
        robot.init(hardwareMap);

        if (opModeIsActive())
        {
            telemetry.addData("Heading", this.robot.imu.getPosition());
            telemetry.update();

            waitForStart();
            robot.runtime.reset();
//--------------------------------------------------------------------------------------------------
            while (opModeIsActive())
            {
                telemetry.update();
                robot.runtime.reset();
                while (!(angleCheck() <= 30))
                {
                    robot.LTurn(.1);
                }

                telemetry.update();
                moveDistance(-4);
                sleep(1000);
                robot.resetEncoder();
                moveDistance(-10);
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------