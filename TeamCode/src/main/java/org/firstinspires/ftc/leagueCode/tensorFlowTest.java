package org.firstinspires.ftc.leagueCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@TeleOp(name = "tensorFlowTest", group = "Concept")
//@Disabled
public class tensorFlowTest extends LinearOpMode
{
	private boolean blockPosition1 = false;
	private boolean blockPosition2 = false;
	private boolean blockPosition3 = false;
	
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AZ6Zar7/////AAABmb9BpTFpR0aao8WchstmN7g6gEQUqWGKJOgwV0UnhrDJwzv1nw8KkSFm4bLbbd/e63bMkh4k2W5raskv2je6UOaSviD58AJtw7RiTt/T1hmt/Row6McUnaoB4KLMoADScEMRa6EnJuW2fMeSgFFy8554WHyYai9AjCfoF3MY4BXSYhZmAx/Y/8fSPBqsbfBxSs5sBZityMz6XsraptRFNQVuRuQlo19wDUc4eU3Eq9D0R1QxiFPxv8yxS6x1jN4rwfkkQBl9eQzNI0/FxSr7Caig9WOwrc65x1+3Op7UmUapHboIn+oRKlOktmT98sGtTBpxY/nz6IV9B6UTjquUNwS3Yu5eRJiu5IZoNWtuxjFA";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
//--------------------------------------------------------------------------------------------------
private void initVuforia()
{
	VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
	parameters.vuforiaLicenseKey = VUFORIA_KEY;
	parameters.cameraDirection = CameraDirection.BACK;
	vuforia = ClassFactory.getInstance().createVuforia(parameters);
}
private void initTfod()
{
	int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
	TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
	tfodParameters.minimumConfidence = 0.8;
	tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
	tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
}
//--------------------------------------------------------------------------------------------------
    public void runOpMode()
	{
		initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        else
		{
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null)
        {
            tfod.activate();
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
//--------------------------------------------------------------------------------------------------
        if(opModeIsActive())
        {
            while(opModeIsActive() && (!(isStopRequested())))
            {
                if(tfod != null)
                {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if(updatedRecognitions != null)
                    {
						telemetry.addData("# Object Detected", updatedRecognitions.size());
						telemetry.update();
						for(Recognition recognition : updatedRecognitions)
						{
//							if(((System.currentTimeMillis() > 6000) && (tfod == null)) || ((recognition.getLabel() == LABEL_FIRST_ELEMENT) && (System.currentTimeMillis() > 6000)))
//							{
//								telemetry.addLine("We can't see the brick");
//								telemetry.update();
////								tfod.shutdown();
//							}
							if(recognition.getLabel() == LABEL_SECOND_ELEMENT)
							{
								if(recognition.getTop() > 400)//Position 1
								{
									telemetry.addLine("Position 1");
									telemetry.update();
									tfod.shutdown();
								}
								else if(recognition.getTop() > 700)//Position 2
								{
									telemetry.addLine("Position 2");
									telemetry.update();
									tfod.shutdown();
								}
								else if(recognition.getTop() > 900)//Position 3
								{
									telemetry.addLine("Position 3");
									telemetry.update();
									tfod.shutdown();
								}
							}
							telemetry.addData(("Stone Type = "), recognition.getLabel());
//							telemetry.addData(("Distance From the Camera   = "), recognition.getLeft());
//							telemetry.addData(("Distance behind the Camera  = "), recognition.getRight());
							telemetry.addLine("---------------------------------------------------");
							telemetry.addData(("X Distance From Left    = "), recognition.getTop());
//							telemetry.addData(("X Distance From Right   = "), recognition.getBottom());
							telemetry.update();
                      }
                      telemetry.update();
                    }
                }
            }
        }
    }
//--------------------------------------------------------------------------------------------------
}
