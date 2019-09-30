package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name ="Rev Auto", group = "Concept")
//@Disabled
public class VuforiaAuto extends LinearOpMode
{
    RevMap robot = new RevMap();
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
    //This method turns the robot
    public double angleCheck()
    {
        telemetry.addLine().addData("After", robot.curHeading);
        telemetry.update();
        robot.angles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.robot.imu.getPosition();
        robot.curHeading = robot.angles.firstAngle;
        return robot.curHeading;
    }
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
    //This method moves the robot a certain distance in inches
    public void moveDistance(double length)
    {
        double totDistInSteps = (((length / 11.97) * 1120) * -1);

        //IF THE NUMBER IS A POSITIVE NUMBER WE GO FORWARD!
        if (totDistInSteps < robot.front_right.getCurrentPosition())
        {
            while(totDistInSteps <= robot.front_right.getCurrentPosition() && (!(isStopRequested())))
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
            while (totDistInSteps >= robot.front_right.getCurrentPosition() && (!(isStopRequested())))
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

















































//--------------------------------------------------------------------------------------------------
    public void runOpMode()
    {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        robot.init(hardwareMap);
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

        waitForStart();

        if (opModeIsActive() && (!(isStopRequested())))
        {
            waitForStart();
            robot.runtime.reset();
//--------------------------------------------------------------------------------------------------
            while (opModeIsActive() && (!(isStopRequested())))
            {
                robot.runtime.reset();
                moveDistance(10);
                while(opModeIsActive() && (robot.curHeading < 60))
                {
                    angleCheck();
                    robot.LTurn(.1);
                }
                stop();
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------