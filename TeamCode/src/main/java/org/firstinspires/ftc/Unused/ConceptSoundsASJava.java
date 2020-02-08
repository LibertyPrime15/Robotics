package org.firstinspires.ftc.unused;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Concept: Sound Resources", group="Concept")
@Disabled
public class ConceptSoundsASJava extends LinearOpMode
{

    // Declare OpMode members.
    private boolean goldFound;      // Sound file present flags
    private boolean silverFound;

    private boolean isX = false;    // Gamepad button state variables
    private boolean isB = false;

    private boolean wasX = false;   // Gamepad button history variables
    private boolean WasB = false;

    @Override
    public void runOpMode()
    {

        int silverSoundID = hardwareMap.appContext.getResources().getIdentifier("silver", "raw", hardwareMap.appContext.getPackageName());
        int goldSoundID   = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());

        if (goldSoundID != 0)
            goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);

        if (silverSoundID != 0)
            silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);

        telemetry.addData("gold resource",   goldFound ?   "Found" : "NOT found\n Add gold.wav to /src/main/res/raw" );
        telemetry.addData("silver resource", silverFound ? "Found" : "Not found\n Add silver.wav to /src/main/res/raw" );

        waitForStart();
        while (opModeIsActive()) {

            if (silverFound && gamepad1.x)
            {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
                telemetry.addData("Playing", "Resource Silver");
                telemetry.update();
            }

            if (goldFound && gamepad1.b)
            {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                telemetry.addData("Playing", "Resource Gold");
                telemetry.update();
            }
        }
    }
}
