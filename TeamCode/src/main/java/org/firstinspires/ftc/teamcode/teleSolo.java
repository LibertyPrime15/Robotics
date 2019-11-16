package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="teleSolo", group = "Main")
//@Disabled
//--------------------------------------------------------------------------------------------------
//----------------------------------------------------//
//----------------------------------------------------//
//--This is where I start the program declare stuffs--//
//----------------------------------------------------//
//----------------------------------------------------//
//--------------------------------------------------------------------------------------------------
public class teleSolo extends LinearOpMode
{
    RevMap robot = new RevMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    boolean Position = true;

    double liftSteps = 770;
    double armSteps  = 1120 * .7;

    boolean armIsExtended = false;
    boolean armDown = false;
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
    telemetry.addLine().addData("Heading",currHeading);
    telemetry.update();
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
            //This is the block of code for driving and turning the robot
            double leftPower;
            double rightPower;

            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.left_stick_x;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//----------------------------------
            //This drives the robot forward
            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
//--------------------------------------------------------------------------------------------------
            //This section extends the arm and opens the servos on X or Y
            if(gamepad1.left_trigger !=0 && !armIsExtended)
            {
                robot.resetArm();
//-------------------------------------------
                robot.claw1.setPosition(0);
                robot.claw2.setPosition(0);
//-------------------------------------------
                while(armSteps > robot.arm.getCurrentPosition() && (!(isStopRequested())))
                {
                    robot.arm.setPower(.7);
                }
                robot.arm.setPower(0);
                robot.resetArm();
                armIsExtended = true;
            }
//-----------------------------------------------------------------------
            else if(gamepad1.right_trigger !=0 && armIsExtended)
            {
                robot.resetLift();
                robot.resetArm();
                while(-armSteps < robot.arm.getCurrentPosition() && (!(isStopRequested())))
                {
                    robot.arm.setPower(-.7);
                }
                robot.arm.setPower(0);
                robot.resetArm();
//-------------------------------------------
                robot.claw1.setPosition(.5);
                robot.claw2.setPosition(.5);
//-------------------------------------------
                armIsExtended = false;
            }
//--------------------------------------------------------------------------------------------------
            //This keeps the lift always flipped up so that it won't fall over and break the program
            if(robot.lift.getCurrentPosition() < 100 && !armDown)//Less then 100 = armUp
            {
                telemetry.addLine("The arm is Up exactly as it should be muy guy");
                telemetry.update();
            }
//----------------------------------
            else if(robot.lift.getCurrentPosition() > 30 && !armDown)//Greater than 200 = armDown
            {
                while(robot.lift.getCurrentPosition() > 5 && (!isStopRequested()))
                {
                    robot.lift.setPower(-1);
                    telemetry.addData("Realigning the Arm",robot.lift.getCurrentPosition());
                    telemetry.update();
                }
            }
//--------------------------------------------------------------------------------------------------





































































//--------------------------------------------------------------------------------------------------
            //This closes the claw
            if(gamepad1.left_bumper)
            {
                robot.claw1.setPosition(0);
                robot.claw2.setPosition(0);
            }
//----------------------------------
            //This opens the claw
            else if(gamepad1.right_bumper)
            {
                robot.claw1.setPosition(.5);
                robot.claw2.setPosition(.5);
            }
//--------------------------------------------------------------------------------------------------
            //This moves the arm up
            if(gamepad1.right_stick_y!=0)
            {
                robot.arm.setPower(-gamepad1.right_stick_y);
            }
//----------------------------------
            else
            {
                robot.arm.setPower(0);
            }
//--------------------------------------------------------------------------------------------------
            if(gamepad1.x)
            {
                robot.lift.setPower(.8);
            }
//----------------------------------
            else if(gamepad1.y)
            {
                robot.lift.setPower(-.8);
            }
//----------------------------------
            else
            {
                robot.resetLift();
                robot.lift.setPower(0);
            }
//--------------------------------------------------------------------------------------------------
            if(gamepad1.a && armDown)
            {
                while(-liftSteps < robot.lift.getCurrentPosition() && (!(isStopRequested())))
                {
                    robot.lift.setPower(-.8);
                }
                armDown = false;
                robot.resetLift();
                robot.lift.setPower(0);
            }
//----------------------------------
            else if(gamepad1.b && !armDown)
            {
                while(liftSteps > robot.lift.getCurrentPosition() && (!(isStopRequested())))
                {
                    robot.lift.setPower(.8);
                }
                armDown = true;
                robot.resetLift();
                robot.lift.setPower(0);
            }
//----------------------------------
            else
            {
                robot.lift.setPower(0);
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