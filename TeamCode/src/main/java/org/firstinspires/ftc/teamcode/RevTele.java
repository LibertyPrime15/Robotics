package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@TeleOp(name="Rev Tele", group = "Main")
//@Disabled
public class RevTele extends LinearOpMode
{
    RevMap robot = new RevMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    boolean Position = false;

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
private void moveLift()
{
    int totDistInSteps = 787;

    if(gamepad1.a && !Position)
    {
        while(totDistInSteps > robot.lift.getCurrentPosition() && (!(isStopRequested())))
        {
            telemetry.update();
            robot.lift.setPower(.5);
        }
        Position = true;
    }

    else if(gamepad1.b && Position)
    {
        while(-totDistInSteps < robot.lift.getCurrentPosition() && (!(isStopRequested())))
        {
            telemetry.update();
            robot.lift.setPower(-.5);
        }
        Position = false;
    }
    else
    {
        robot.lift.setPower(0);
        robot.resetLift();
    }
}
//--------------------------------------------------------------------------------------------------
private void drive()
{
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
    telemetry.update();
}
//--------------------------------------------------------------------------------------------------
private void setupVuforia()
{
// Setup parameters to create localizer
parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
parameters.vuforiaLicenseKey = VUFORIA_KEY;
parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
parameters.useExtendedTracking = false;
vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

// These are the vision targets that we want to use
// The string needs to be the name of the appropriate .xml file in the assets folder
visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");
Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

// Setup the target to be tracked
target = visionTargets.get(0); // 0 corresponds to the wheels target
target.setName("Wheels Target");
target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

// Set phone location on robot
phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

// Setup listener and inform it of phone information
listener = (VuforiaTrackableDefaultListener) target.getListener();
listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
}
//--------------------------------------------------------------------------------------------------
private OpenGLMatrix createMatrix(float x,float y,float z,float u,float v,float w)
{
    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    return OpenGLMatrix.translation(x, y, z).
            multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
}
//--------------------------------------------------------------------------------------------------
private String formatMatrix(OpenGLMatrix matrix)
{
    // Formats a matrix into a readable string
    return matrix.formatAsTransform();
}
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---No More Methods Are Made Past This---//
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------












































//--------------------------------------------------------------------------------------------------
//---------------------------------------------------//
//---------------------------------------------------//
//---These are my preprogrammed autonoumous routes---//
//---------------------------------------------------//
//---------------------------------------------------//
//--------------------------------------------------------------------------------------------------
private void blockA()
{

}
//--------------------------------------------------------------------------------------------------
private void blockB()
{

}
//--------------------------------------------------------------------------------------------------
private void blockC()
{

}
//--------------------------------------------------------------------------------------------------
private void blockD()
{

}
//--------------------------------------------------------------------------------------------------
private void blockE()
{

}
//--------------------------------------------------------------------------------------------------
private void blockF()
{

}
//--------------------------------------------------------------------------------------------------
//-------------------------------------------------------//
//-------------------------------------------------------//
//---No more preprogrammed Autonoumous Paths Past Here---//
//-------------------------------------------------------//
//-------------------------------------------------------//
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
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 500, 0, 90, 0, 90);

        waitForStart();
        visionTargets.activate();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
            angleBoi();
            drive();
            moveLift();
//----------------------------------
            //This closes the claw
            if(gamepad1.y)
            {
                robot.claw1.setPosition(0);
                robot.claw2.setPosition(0);
            }
//----------------------------------
            //This opens the claw
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

            //Otherwise, the arm won't move
            else
            {
                robot.arm.setPower(0);
            }
//--------------------------------------------------------------------
// Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
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