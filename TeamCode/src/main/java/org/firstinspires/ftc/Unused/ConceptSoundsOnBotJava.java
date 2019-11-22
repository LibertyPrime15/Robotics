package org.firstinspires.ftc.Unused;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

/*
 *      https://github.com/ftctechnh/ftc_app/tree/master/FtcRobotController/src/main/res/raw/gold.wav
 *      https://github.com/ftctechnh/ftc_app/tree/master/FtcRobotController/src/main/res/raw/silver.wav
 */

@TeleOp(name="Concept: Sound Files", group="Concept")
@Disabled
public class ConceptSoundsOnBotJava extends LinearOpMode
{
    // Point to sound files on the phone's drive
    private String soundPath = "/FIRST/blocks/sounds";
    private File goldFile   = new File("/sdcard" + soundPath + "/gold.wav");
    private File silverFile = new File("/sdcard" + soundPath + "/silver.wav");

    public void runOpMode()
    {

        // Make sure that the sound files exist on the phone
        boolean goldFound   = goldFile.exists();
        boolean silverFound = silverFile.exists();

        // Display sound status
        telemetry.addData("gold sound",   goldFound ?   "Found" : "NOT Found \nCopy gold.wav to " + soundPath  );
        telemetry.addData("silver sound", silverFound ? "Found" : "NOT Found \nCopy silver.wav to " + soundPath );

        waitForStart();

        while (opModeIsActive())
        {
            if (silverFound && gamepad1.x)
            {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverFile);
                telemetry.addData("Playing", "Silver File");
                telemetry.update();
            }

            if (goldFound && gamepad1.b)
            {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldFile);
                telemetry.addData("Playing", "Gold File");
                telemetry.update();
            }
        }
    }
}
