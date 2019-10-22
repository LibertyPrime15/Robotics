package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@Autonomous(name="blue Auto", group = "Main")
//@Disabled
public class blueAuto extends LinearOpMode
{
    RevMap robot = new RevMap();
    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;
    double Circ = 11.97;
    double Steps = 1160;

    boolean armPos = false;
    boolean Position = false;
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
private void setupVuforia()
{
    // Setup parameters to create localizer
    parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
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
private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
{
    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    return OpenGLMatrix.translation(x, y, z).
            multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
}
private String formatMatrix(OpenGLMatrix matrix)
{
    // Formats a matrix into a readable string
    return matrix.formatAsTransform();
}
public boolean vufoCrap()
{
    OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

    if(latestLocation != null)
    {
        lastKnownLocation = latestLocation;
    }
    float[] coordinates = lastKnownLocation.getTranslation().getData();

    robotX = coordinates[0];
    robotY = coordinates[1];
    robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

    telemetry.addData("Tracking " + target.getName(), listener.isVisible());
    telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
    telemetry.update();
    return listener.isVisible();
}
//--------------------------------------------------------------------------------------------------
public void moveDistance(double length)
{
    double totDistInSteps = (((length / 11.97) * 1120) * -1);

    double leftPower;
    double rightPower;

    double drive = 0;
    double turn  = 0;

    if(totDistInSteps < robot.front_right.getCurrentPosition())
    {
        while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps < robot.front_right.getCurrentPosition())
        {
            angleBoi();
            drive = -.3;
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
            angleBoi();
            drive = .3;
            turn  = .05 * currHeading;
            leftPower    = Range.clip(drive - turn, -1.0, 1.0);
            rightPower   = Range.clip(drive + turn, -1.0, 1.0);

            telemetry.addData("I'm right here3",robot.front_right.getCurrentPosition());
            telemetry.update();
            robot.front_right.setPower(rightPower);
            robot.front_left.setPower(leftPower);
            robot.back_right.setPower(rightPower);
            robot.back_left.setPower(leftPower);
        }
        robot.Halt();
        robot.resetEncoder();
    }
    else
    {
        robot.Halt();
        robot.resetEncoder();
    }
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
public void armUp()
{
    double totDistInSteps = 655;

    while (totDistInSteps > robot.arm.getCurrentPosition() && (!(isStopRequested())))
    {
        telemetry.addData("Other thing", totDistInSteps);
        telemetry.addData("thing", robot.arm.getCurrentPosition());
        telemetry.update();
        robot.arm.setPower(.5);
    }
    armPos = true;
    robot.arm.setPower(0);
    robot.resetArm();
}
//--------------------------------------------
public void armDown()
{
    double totDistInSteps = -655;

    while(totDistInSteps < robot.arm.getCurrentPosition() && (!(isStopRequested())))
    {
        telemetry.update();
        robot.arm.setPower(-.5);
    }
    armPos = false;
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
    Position = true;
    robot.lift.setPower(0);
    robot.resetLift();
}
//--------------------------------------------
private void liftDown()
{
    double totDistInSteps = 787;

    while(totDistInSteps > robot.lift.getCurrentPosition() && (!(isStopRequested())))
        {
            telemetry.update();
            robot.lift.setPower(.5);
        }
        Position = false;
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
            robot.turnRight(.5);
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
            robot.turnLeft(.5);
        }
        imuInit();
    }
}
//--------------------------------------------------------------------------------------------------
public boolean checkSight()
{
    while(listener.isVisible() && (!(isStopRequested())))
    {
        inView = true;
    }
    return inView;
}
//--------------------------------------------------------------------------------------------------
public void checkEncoder()
{
    while(opModeIsActive() && (!(isStopRequested())) && inView == false)
    {
//        moveDistance(10);
        liftUp();
        armUp();
//        robot.openClaw();
        armDown();
        robot.closeClaw();
        liftDown();
//        checkSight();
//        moveDistance(10);
//        checkSight();
//        moveDistance(10);
//        checkSight();
    }
    stop();
    //getBlock();
}
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//THIS IS FOR TESTING CODE//
//--------------------------------------------------------------------------------------------------
private void getBlock()
{
    robot.Halt();
    turnAngle(90);
    moveDistance(5);
    liftUp();
    robot.openClaw();
    moveDistance(3);
    armUp();
    robot.openClaw();
    armDown();
    robot.closeClaw();
    liftDown();
    moveDistance(-8);
    turnAngle(-90);
    checkDistance();
    robot.Halt();
}
//--------------------------------------------------------------------------------------------------
public void checkDistance()
{
    double distDriven = 0;     //In steps
    double distGone   = 0;     //This is the distDriven in inches
    double totField   = 13954; //This is the full field in inches
    double distRemain = 0;     //This is the distance I have to go to get to the end of the field

    while(opModeIsActive() && (!(isStopRequested())))
    {
        robot.Forward(.7);
        robot.front_right.getCurrentPosition();
        distDriven = robot.front_right.getCurrentPosition() * -1;
    }
    distGone = (distDriven * ((1/Circ)*Steps));
    distRemain = totField - distGone;
    moveDistance(distRemain);
}
//--------------------------------------------------------------------------------------------------








































































//--------------------------------------------------------------------------------------------------
    public void runOpMode()
    {
        imuInit();
        setupVuforia();
        lastKnownLocation = createMatrix(0, 500, 0, 90, 0, 90);

        waitForStart();
        visionTargets.activate();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
//----------------------------------
            checkEncoder();
//            vufoCrap();
//----------------------------------
        }
    }
}
//--------------------------------------------------------------------------------------------------