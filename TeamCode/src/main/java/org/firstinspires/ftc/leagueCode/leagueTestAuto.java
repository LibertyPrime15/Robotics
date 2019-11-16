package org.firstinspires.ftc.leagueCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RevMap;

@Autonomous(name="League Blue", group = "Concept")
//@Disabled
public class leagueTestAuto extends LinearOpMode
{
    //Call my hardware map
    leagueMap robot = new leagueMap();
    //Declare the variables needed for the imu/gyro
    Orientation angles;
    BNO055IMU imu;

    //Declare the variable I'm gonna use the gyro
    float currHeading = 0;
    double Circ = 11.97;////RUBBER WHEELS
//    double Circ = 00.00;///MECANUM WHEELS



    double Steps = 1120;///40:1 Gear Ratio
//    double Steps = 560;////20:1 Gear Ratio
//    double Steps = 1680;///60:1 Gear Ratio

    //Inches -> Steps  = #Inches * (1120/11.97)
    //Steps  -> Inches = #Steps  * (11.97/1120)

    //These variables are for driving in a straight line
    double drive = 0;
    double turn  = 0;

    //These variables are for moving the remaining distance across the field since our position changes every time
//    double distGone   = 0;
//    double distRemain = 0;
//    double totField   = -6200;//length * ((1/11.97) * 1120); = steps per inch ------ 144in = 13473steps ---- 2526 = 24
//
//    double distMultipler = -6;

    //These are booleans for determining whether the arm and lift are in certain positions
//    boolean isExtended = false;
//    boolean isVertical = false;
//    //This is a boolean that lets me know if vuforia is detecting the object
//    boolean inView     = false;

    //These are all variables needed for vuforia to work - they come with
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;
    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    //This is our vuforia key needed for the program to run
    private static final String VUFORIA_KEY = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";

    //These are variables I'm in the process of using for
    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;

    VuforiaLocalizer vuforia;
    //--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//gyro.getHeading()
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------
//This method is for reseting the imu - must be called after every time a turn is used
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
//This method is used for setting up Vuforia and runs everytime the program initializes
private void setupVuforia()
{
    parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
    parameters.useExtendedTracking = false;
    vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

    visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");
    Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

    target = visionTargets.get(0);
    target.setName("Wheels Target");
    target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));
    phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

    listener = (VuforiaTrackableDefaultListener) target.getListener();
    listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
}
private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
{
    return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
}
private String formatMatrix(OpenGLMatrix matrix)
{
    //I don't like how java reformats this line so I'm putting in a comment line
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
//This program returns our current angular heading
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
//This method moves a certain distance in inches at a certain speed - when moving it will move perfectly straight
public void moveDistance(double length, double power)
{
    double totDistInSteps = (((length / Circ) * Steps) * -1);

    double leftPower;
    double rightPower;

    if(totDistInSteps < robot.front_right.getCurrentPosition())
    {
//    addMultiplier();
    while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps < robot.front_right.getCurrentPosition())
        {
//            telemetry.addData("distRemain",distRemain);
            telemetry.addData("currSteps",robot.front_right.getCurrentPosition());
//            telemetry.addData("distMultiplier", distMultipler);
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
//        addMultiplier();
        while(opModeIsActive() && (!(isStopRequested())) && totDistInSteps > robot.front_right.getCurrentPosition())
        {
//            telemetry.addData("----distRemain",distRemain);
            telemetry.addData("----currSteps",robot.front_right.getCurrentPosition());
//            telemetry.addData("----distMultiplier", distMultipler);
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
//This method moves a certain distance in inches SIDEWAYS at a certain speed - when moving it will move perfectly straight
public void moveSide(int time, boolean direction, double power)
{
    double totDistInSteps = (((time / Circ) * Steps) * -1);

    double leftPower;
    double rightPower;

    if(direction)
    {
        angleBoi();
        drive = -power;
        turn  = .05 * currHeading;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);

        robot.front_right.setPower(rightPower);
        robot.front_left.setPower(leftPower);
        robot.back_right.setPower(rightPower);
        robot.back_left.setPower(leftPower);
        sleep(time);
        robot.Halt();
    }

    else if(!direction)
    {
        angleBoi();
        drive = -power;
        turn  = .05 * currHeading;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightPower   = Range.clip(drive - turn, -1.0, 1.0);

        robot.front_right.setPower(rightPower);
        robot.front_left.setPower(leftPower);
        robot.back_right.setPower(rightPower);
        robot.back_left.setPower(leftPower);
        sleep(time);
        robot.Halt();
    }
}
//--------------------------------------------------------------------------------------------------
//This method turns the robot a certain angle: 0-180 to the left && 0 to -180 in the right
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
            robot.turnLeft(.6);
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
            robot.turnRight(.6);
        }
        imuInit();
    }
    currHeading = 0;
}
//--------------------------------------------------------------------------------------------------
//This checks to see if the skystone is in view
//public boolean checkSight()
//{
//    if(listener.isVisible()  && (!(isStopRequested())))
//    {
//        inView = true;
//        getBlock();
//    }
//    else
//    {
//        inView = false;
//        moveDistance(6.7,.6);
//        sleep(1000);
//    }
//    return inView;
//}
//--------------------------------------------------------------------------------------------------
//This method adds a multiplier of 1 every time a move distance, this is for moving the remaining distance of the field depending on which skystone is detected
//public double addMultiplier()
//{
//    distMultipler = distMultipler + 1;
//    return distMultipler;
//}
//--------------------------------------------------------------------------------------------------
//This method runs actual atonomous code and calls the methods that run actual atonomous code
//public void checkEncoder()
//{
//    while(opModeIsActive() && (!(isStopRequested())))
//    {
//        if(inView == false)
//        {
//            while(inView == false && (!(isStopRequested())))
//            {
//                checkSight();
//            }
//        }
//    }
//}
//--------------------------------------------------------------------------------------------------
//This moves the remaining distance of the field
//public void checkDistance()
//{
//    distGone   = (467.83 * distMultipler) * (-1);//5 is the length in inches I travel per run = 467.83
//    distRemain = ((totField + distGone) * (11.97/1120)) * (-1);
//    moveDistance(distRemain,1);
//}
//--------------------------------------------------------------------------------------------------
//This method is the actual autonomous in its entirety running right here and all of its method calls
//private void getBlock()
//{
//    OpenGLMatrix createMatrix();
//    telemetry.addData("Relative Coordinates", OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w)));
//    telemetry.update();
//}
//--------------------------------------------------------------------------------------------------
//private void autoAdjust()//This is for auto adjusting the robot to align with the skystone
//{
//    float x = 0;
//    if(x > 0)
//    {
//        while(x > 0 && (!(isStopRequested())))
//        {
//
//        }
//    }
//
//    else if(x < 0)
//    {
//        while(x < 0 && (!(isStopRequested())))
//        {
//
//        }
//    }
//}
//--------------------------------------------------------------------------------------------------
//private void sideMove()//This is for moving to the side constantly
//{
//    float x = 0;
//    if(x > 0)
//    {
//        while(x > 0 && (!(isStopRequested())))
//        {
//
//        }
//    }
//
//    else if(x < 0)
//    {
//        while(x < 0 && (!(isStopRequested())))
//        {
//
//        }
//    }
//}
//--------------------------------------------------------------------------------------------------
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
        setupVuforia();
        lastKnownLocation = createMatrix(0, 500, 0, 90, 0, 90);






        telemetry.addData("Status","Initialized");
        telemetry.update();
        waitForStart();
        visionTargets.activate();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
//----------------------------------
            vufoCrap();
//            checkEncoder();
//----------------------------------
        }
    }
}
//--------------------------------------------------------------------------------------------------