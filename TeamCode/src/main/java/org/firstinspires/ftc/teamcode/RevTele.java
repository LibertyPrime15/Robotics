package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Rev Tele", group = "Main")
@Disabled
public class RevTele extends LinearOpMode
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

                if(gamepad1.right_stick_x !=0)
                {
                    robot.front_right.setPower(gamepad1.right_stick_x);
                    robot.front_left.setPower(gamepad1.right_stick_x);
                    robot.back_right.setPower(gamepad1.right_stick_x);
                    robot.back_left.setPower(gamepad1.right_stick_x);
                }

                else
                {
                    robot.front_right.setPower(0);
                    robot.front_left.setPower(0);
                    robot.back_right.setPower(0);
                    robot.back_left.setPower(0);
                }
                telemetry.update();
            }
        }
    }
//----------------------------------------------------------------------------------------------