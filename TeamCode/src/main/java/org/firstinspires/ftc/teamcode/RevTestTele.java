package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RevTest", group = "Main")
@Disabled
public class RevTestTele extends LinearOpMode
    {
        LibertyRevMap robot = new LibertyRevMap();
//----------------------------------------------------------------------------------------------
        @Override
        public void runOpMode()
        {
            robot.init(hardwareMap);
            telemetry.addData("Status", "Let's Go Boisssss");
            telemetry.update();

            waitForStart();
//------------------------------------------------------------
            while(opModeIsActive())
            {
                if(gamepad1.left_stick_y !=0)
                {
                    robot.front_right.setPower(gamepad1.left_stick_y);
                    robot.front_left.setPower(gamepad1.left_stick_y);
                    robot.back_right.setPower(gamepad1.left_stick_y);
                    robot.back_left.setPower(gamepad1.left_stick_y);
                }
            }
        }
    }
//----------------------------------------------------------------------------------------------