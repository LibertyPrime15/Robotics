package org.firstinspires.ftc.leagueCode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.RevMap;

import java.util.Locale;

@TeleOp(name="VuMarkTEST", group ="A")
//@Disabled
public class vuMarkTEST extends LinearOpMode
{
	leagueMap robot = new leagueMap();
	Orientation angles;
	BNO055IMU imu;
	
	double diameter = 4;
	double radius   = (diameter/2);
	double circ     = Math.PI * (radius * radius);
	
	double Steps        = 560;
	float currHeading   = 0;
	double Compensation = 1.5;
	
	double drive = 0;
	double turn  = 0;

    double tX;
    double tY;
    double tZ;

    double rX;
    double rY;
    double rZ;
    
	public static final String TAG = "Vuforia VuMark Sample";
	OpenGLMatrix lastLocation = null;
	VuforiaLocalizer vuforia;
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
private void pickBlockPath()
{
	if(tY < 200 && tY > 100)
	{
		//Run this method
		block1();
	}
	else if(tY < 300 && tY > 200)
	{
		//Run this mehod
		block2();
	}
	else if(tY < 400 && tY > 300)
	{
		//Run this method
		block3();
	}
}
//--------------------------------------------------------------------------------------------------
private void blockPlacement()
{
	//Servo Positions
}
//--------------------------------------------------------------------------------------------------
private void startIntakeCycle()
{
	robot.intake(.2);
}
//--------------------------------------------------------------------------------------------------
private void endIntakeCycle()
{
	//Stop all of the intake stuff
}
//--------------------------------------------------------------------------------------------------
private void block1()
{
	//Block Spot 1
}
//--------------------------------------------------------------------------------------------------
private void block2()
{
	//Block Spot 2
}
//--------------------------------------------------------------------------------------------------
private void block3()
{
	//Block Spot 3
}
//--------------------------------------------------------------------------------------------------































































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
	
		// hsvValues is an array that will hold the hue, saturation, and value information.
		float hsvValues[] = {0F, 0F, 0F};
	
		// values is a reference to the hsvValues array.
		final float values[] = hsvValues;
	
		// sometimes it helps to multiply the raw RGB values with a scale factor
		// to amplify/attentuate the measured values.
		final double SCALE_FACTOR = 255;
	
		// get a reference to the RelativeLayout so we can change the background
		// color of the Robot Controller app to match the hue detected by the RGB sensor.
		int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
		final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
		
        telemetry.addData("Status", "Hit it Bois");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
//--------------------------------------------------------------------------------------------------
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
            
            if(tY != 0)
			{
				pickBlockPath();
			}
			// convert the RGB values to HSV values.
			// multiply by the SCALE_FACTOR.
			// then cast it back to int (SCALE_FACTOR is a double)
			Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR), (int) (robot.sensorColor.green() * SCALE_FACTOR), (int) (robot.sensorColor.blue() * SCALE_FACTOR), hsvValues);
	
			// send the info back to driver station using telemetry function.
			telemetry.addData("Alpha", robot.sensorColor.alpha());
			telemetry.addData("Red  ", robot.sensorColor.red());
			telemetry.addData("Green", robot.sensorColor.green());
			telemetry.addData("Blue ", robot.sensorColor.blue());
			telemetry.addData("Hue", hsvValues[0]);
	
			// change the background color to match the color detected by the RGB sensor.
			// pass a reference to the hue, saturation, and value array as an argument
			// to the HSVToColor method.
			relativeLayout.post(new Runnable()
			{
				public void run()
				{
					relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
				}
			});
        }
		relativeLayout.post(new Runnable()
		{
			public void run()
			{
				relativeLayout.setBackgroundColor(Color.WHITE);
			}
		});
//--------------------------------------------------------------------------------------------------
    }
//--------------------------------------------------------------------------------------------------
    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
//--------------------------------------------------------------------------------------------------
}
