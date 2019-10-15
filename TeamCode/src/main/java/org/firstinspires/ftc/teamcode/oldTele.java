package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Old Tele", group = "Main")
//@Disabled
public class oldTele extends LinearOpMode
{
    public DcMotor  right_drive   = null;
    public DcMotor  left_drive    = null;
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------



















//--------------------------------------------------------------------------------------------------
    public void runOpMode()
    {



        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive  = hardwareMap.get(DcMotor.class, "left_drive");

        right_drive.setPower(0);
        left_drive.setPower(0);

        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive.setDirection(DcMotor.Direction.FORWARD);

        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);











        waitForStart();
//--------------------------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
//----------------------------------
            double leftPower;
            double rightPower;

            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.left_stick_x;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//----------------------------------
            //This drives the robot forward
            right_drive.setPower(rightPower);
            left_drive.setPower(leftPower);
            telemetry.update();
//----------------------------------
        }
    }
}
//--------------------------------------------------------------------------------------------------