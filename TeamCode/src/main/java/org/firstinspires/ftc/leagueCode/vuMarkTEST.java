package org.firstinspires.ftc.leagueCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.RevMap;

@TeleOp(name="VuMarkTEST", group ="Concept")
//@Disabled
public class vuMarkTEST extends LinearOpMode
{
    RevMap robot = new RevMap();

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    Orientation angles;
    BNO055IMU imu;

    float currHeading = 0;

    double tX;
    double tY;
    double tZ;

    double rX;
    double rY;
    double rZ;

    double drive = 0;
    double turn  = 0;
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
////--------------------------------------------------------------------------------------------------
//private void autoAlign(double power)
//{
//    double leftPower;
//    double rightPower;
//    if(rX > 0)
//    {
//        while(rX > 0)
//        {
//            angleBoi();
//            drive = -power;
//            turn  = .05 * currHeading;
//            leftPower    = Range.clip(drive + turn, -1.0, 1.0);
//            rightPower   = Range.clip(drive - turn, -1.0, 1.0);
//
//            robot.front_right.setPower(rightPower);
//            robot.front_left.setPower(leftPower);
//            robot.back_right.setPower(rightPower);
//            robot.back_left.setPower(leftPower);
//        }
//    }
//    else if(rX < 0)
//    {
//        while(rX < 0)
//        {
//            angleBoi();
//            drive = -power;
//            turn  = .05 * currHeading;
//            leftPower    = Range.clip(drive + turn, -1.0, 1.0);
//            rightPower   = Range.clip(drive - turn, -1.0, 1.0);
//
//            robot.front_right.setPower(rightPower);
//            robot.front_left.setPower(leftPower);
//            robot.back_right.setPower(rightPower);
//            robot.back_left.setPower(leftPower);
//        }
//    }
//}
//--------------------------------------------------------------------------------------------------

































































    public void runOpMode()
    {
        imuInit();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData("Status", "Hit it Bois");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while(opModeIsActive() && (!(isStopRequested())))
        {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

            if(pose != null)
            {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                tX = trans.get(0);
                tY = trans.get(1);
                tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                rX = rot.firstAngle;
                rY = rot.secondAngle;
                rZ = rot.thirdAngle;
                telemetry.addLine().addData("First tX",tX);
                telemetry.addLine().addData("First tY",tY);
                telemetry.addLine().addData("First tZ",tZ);
                telemetry.update();
            }

            else if(pose == null)
            {
                while((!(isStopRequested())) && rX > 0)
                {
                    telemetry.addLine().addData("rX",rX);
                    telemetry.addLine().addData("rY",rY);
                    telemetry.addLine().addData("rZ",rZ);
                    telemetry.update();
                }
            }
            telemetry.update();
        }
    }



    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
