package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Colee's TeleOp", group = "Main")
public class LinearBoi extends LinearOpMode
{
    LibertysMap robot = new LibertysMap();
//----------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Let's Go Boisssss");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.left_stick_y!=0)
            {
                robot.front_right.setPower(gamepad1.left_stick_y);
                robot.front_left.setPower(gamepad1.left_stick_y);
                robot.back_right.setPower(gamepad1.left_stick_y);
                robot.back_left.setPower(gamepad1.left_stick_y);
            }
//--------------------
            if(gamepad1.right_stick_x!=0)
            {
                robot.front_right.setPower(gamepad1.right_stick_x);
                robot.front_left.setPower(-gamepad1.right_stick_x);
                robot.back_right.setPower(gamepad1.right_stick_x);
                robot.back_left.setPower(-gamepad1.right_stick_x);
            }
        }
    }
}
