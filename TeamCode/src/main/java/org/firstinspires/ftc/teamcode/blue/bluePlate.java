package org.firstinspires.ftc.teamcode.blue;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RevMap;

@Autonomous(name="Blue Plate", group = "Y")
//@Disabled
public class bluePlate extends LinearOpMode
{
    RevMap robot = new RevMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    double Circ = 11.97;
    double Steps = 1160;

    //Inches -> Steps  = #Inches * (1120/11.97)
    //Steps  -> Inches = #Steps  * (11.97/1120)

    double drive = 0;
    double turn  = 0;

    double distGone   = 0;
    double distRemain = 0;
    double totField   = -6000;//length * ((1/11.97) * 1120); = steps per inch ------ 144in = 13473steps

    boolean isExtended = false;
    boolean isVertical = false;
    boolean inView = false;

    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//gyro.getHeading()
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
    telemetry.addLine().addData("Heading",currHeading);
    telemetry.update();
    angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
    this.imu.getPosition();
    currHeading = angles.firstAngle;
    return currHeading;
}
//--------------------------------------------------------------------------------------------------
public void moveDistance(double length, double power)
{
    double totDistInSteps = (((length / 11.97) * 1120) * -1);

    double leftPower;
    double rightPower;

    if(totDistInSteps < robot.front_right.getCurrentPosition())
    {
        while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps < robot.front_right.getCurrentPosition())
        {
            telemetry.addData("distRemain",distRemain);
            telemetry.addData("currSteps",robot.front_right.getCurrentPosition());
            angleBoi();
            drive = -power;
            turn  = .05 * currHeading;
            leftPower    = Range.clip(drive - turn, -1.0, 1.0);
            rightPower   = Range.clip(drive + turn, -1.0, 1.0);

            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
        }
        robot.Halt();
        robot.resetEncoder();
    }
    else if(totDistInSteps > robot.front_right.getCurrentPosition())
    {
        while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps > robot.front_right.getCurrentPosition())
        {
            telemetry.addData("----distRemain",distRemain);
            telemetry.addData("----currSteps",robot.front_right.getCurrentPosition());
            angleBoi();
            drive = power;
            turn  = .05 * currHeading;
            leftPower    = Range.clip(drive - turn, -1.0, 1.0);
            rightPower   = Range.clip(drive + turn, -1.0, 1.0);

            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
        }
        robot.Halt();
        robot.resetEncoder();
    }
}
//--------------------------------------------------------------------------------------------------
public void sadMove(double length, double power)
{
    double totDistInSteps = (((length / 11.97) * 1120) * -1);

    double leftPower;
    double rightPower;

    if(totDistInSteps < robot.front_left.getCurrentPosition())
    {
        while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps < robot.front_left.getCurrentPosition())
        {
            telemetry.addData("distRemain",distRemain);
            telemetry.addData("currSteps",robot.front_left.getCurrentPosition());
            angleBoi();
            drive = -power;
            turn  = .05 * currHeading;
            leftPower    = Range.clip(drive - turn, -1.0, 1.0);
            rightPower   = Range.clip(drive + turn, -1.0, 1.0);

            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
        }
        robot.Halt();
        robot.resetEncoder();
    }
    else if(totDistInSteps > robot.front_left.getCurrentPosition())
    {
        while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps > robot.front_left.getCurrentPosition())
        {
            telemetry.addData("----distRemain",distRemain);
            telemetry.addData("----currSteps",robot.front_left.getCurrentPosition());
            angleBoi();
            drive = power;
            turn  = .05 * currHeading;
            leftPower    = Range.clip(drive - turn, -1.0, 1.0);
            rightPower   = Range.clip(drive + turn, -1.0, 1.0);

            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
        }
        robot.Halt();
        robot.resetEncoder();
    }
}
//--------------------------------------------------------------------------------------------------
public void armUp(double length)
{
    double totDistInSteps = 1120 * length;
//3 inches --93.567
    while (totDistInSteps > robot.arm.getCurrentPosition() && (!(isStopRequested())))
    {
        telemetry.update();
        robot.arm.setPower(.7);
    }
    isExtended = true;
    robot.arm.setPower(0);
    robot.resetArm();
}
//--------------------------------------------
public void armDown(double length)
{
    double totDistInSteps = -1120 * length;

    while(totDistInSteps < robot.arm.getCurrentPosition() && (!(isStopRequested())))
    {
        telemetry.update();
        robot.arm.setPower(-.7);
    }
    isExtended = false;
    robot.arm.setPower(0);
    robot.resetArm();
}
//--------------------------------------------------------------------------------------------------
private void liftUp()
{
    double totDistInSteps = -787;

    while(totDistInSteps < robot.lift.getCurrentPosition() && (!(isStopRequested())))
    {
        telemetry.update();
        robot.lift.setPower(-.5);
    }
    isVertical = true;
    robot.lift.setPower(0);
    robot.resetLift();
}
//--------------------------------------------------------------------------------------------------
private void turnAngle(double angle)
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
            robot.turnLeft(.4);
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
            robot.turnRight(.4);
        }
        imuInit();
    }
    currHeading = 0;
}
//--------------------------------------------------------------------------------------------------
private void getBlock()//Needs to go 6000 steps remaining distance
{
    turnAngle(82);
    moveDistance(30,1);
    turnAngle(-75);
    liftUp();
    armUp(1.5);
    moveDistance(29,.6);
    armDown(1.2);
    sleep(200);
    sadMove(-40,.6);
    armUp(1.5);
    sadMove(3.5,.6);
    sadMove(-3.5,.6);
    turnAngle(78);
    moveDistance(-20,1);
    robot.Halt();
    stop();
}
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---No More Methods Are Made Past This---//
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------

































































//--------------------------------------------------------------------------------------------------
    public void runOpMode()
    {
        imuInit();
        telemetry.addData("Status","Initialized");
        telemetry.update();
        waitForStart();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
//----------------------------------
            getBlock();
//----------------------------------
        }
    }
}
//--------------------------------------------------------------------------------------------------