package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name ="Rev Auto", group = "Concept")
//@Disabled
public class RevAuto extends LinearOpMode
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
    private void turnAngle(double angle, double thing)
    {
        if(angle > 0)
        {
            while(angle >= currHeading && (!(isStopRequested())))
            {
                telemetry.addLine().addData("Heading", currHeading);
                telemetry.update();
                angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                this.imu.getPosition();
                currHeading = angles.firstAngle;
                robot.turnRight(thing);
            }
            imuInit();
        }

        else if(angle < 0)
        {
            while(angle <= currHeading && (!(isStopRequested())))
            {
                telemetry.addLine().addData("---Heading", currHeading);
                telemetry.update();
                angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                this.imu.getPosition();
                currHeading = angles.firstAngle;
                robot.turnLeft(thing);
            }
            imuInit();
        }
    }
//--------------------------------------------------------------------------------------------------
    public void moveDistance(double length)
    {
        double totDistInSteps = (((length / 11.97) * 1120) * -1);

        //IF THE NUMBER IS A POSITIVE NUMBER WE GO FORWARD!
        if (totDistInSteps < robot.front_right.getCurrentPosition())
        {
            while(totDistInSteps < robot.front_right.getCurrentPosition() && (!(isStopRequested())))
            {
                telemetry.addData("Current Value",robot.front_right.getCurrentPosition());
                telemetry.addData("totDistInSteps",totDistInSteps);
                telemetry.update();
                robot.Forward(.1);
            }
        }
        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO BACKWARD!
        else if (totDistInSteps > robot.front_right.getCurrentPosition())
        {
            while (totDistInSteps > robot.front_right.getCurrentPosition() && (!(isStopRequested())))
            {
                telemetry.addData("---Current Value",robot.front_right.getCurrentPosition());
                telemetry.addData("---totDistInSteps",totDistInSteps);
                telemetry.update();
                robot.Backward(.1);
            }
        }
        robot.Halt();
    }
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




































//--------------------------------------------------------------------------------------------------
    public void runOpMode()
    {
        imuInit();
        waitForStart();

        if (opModeIsActive() && (!(isStopRequested())))
        {
            waitForStart();
            robot.runtime.reset();
//--------------------------------------------------------------------------------------------------
            while (opModeIsActive() && (!(isStopRequested())))
            {
                robot.runtime.reset();
//---------------------------------------\/ \/ \/ \/ Code Goes there
                turnAngle(35, .5);

                moveDistance(10);

                turnAngle(-45 ,.5);

                moveDistance(-25);
//---------------------------------------/\ /\ /\ /\ Code Goes There
                stop();
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------