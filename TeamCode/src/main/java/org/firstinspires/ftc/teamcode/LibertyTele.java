package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Colee's TeleOp", group = "Main")
public class LibertyTele extends LinearOpMode
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
//------------------------------------------------------------
        while(opModeIsActive())
        {
//--------------------
            //This is for basic movement forward - with the left joystick
            if(gamepad1.left_stick_y !=0)
            {
                robot.front_right.setPower(gamepad1.left_stick_y);
                robot.front_left.setPower(gamepad1.left_stick_y);
                robot.back_right.setPower(gamepad1.left_stick_y);
                robot.back_left.setPower(gamepad1.left_stick_y);
            }
//--------------------
//            //This is for mechanum/driving directly to the left - with the left joystick
//            if(gamepad1.left_stick_x !=0 && gamepad1.left_stick_x !=1)
//            {
//                robot.front_right.setPower(gamepad1.left_stick_x);
//                robot.front_left.setPower(-gamepad1.left_stick_x);
//                robot.back_right.setPower(-gamepad1.left_stick_x);
//                robot.back_left.setPower(gamepad1.left_stick_x);
//            }
////--------------------
//            //This is for mechanum/driving directly to the right - with the left joystick
//            if(gamepad1.left_stick_x !=0 && gamepad1.left_stick_x !=-1)
//            {
//                robot.front_right.setPower(-gamepad1.left_stick_x);
//                robot.front_left.setPower(gamepad1.left_stick_x);
//                robot.back_right.setPower(gamepad1.left_stick_x);
//                robot.back_left.setPower(-gamepad1.left_stick_x);
//            }
////--------------------These are the right joystick controls
//            //This is for turning the robot to the right - with the right joystick
//            if(gamepad1.right_stick_x !=0)
//            {
//                robot.front_right.setPower(gamepad1.right_stick_x);
//                robot.front_left.setPower(-gamepad1.right_stick_x);
//                robot.back_right.setPower(gamepad1.right_stick_x);
//                robot.back_left.setPower(-gamepad1.right_stick_x);
//            }
////--------------------
//            //This is for turning the robot to the left - with the right joystick
//            if(gamepad1.right_stick_x !=0)
//            {
//                robot.front_right.setPower(gamepad1.right_stick_x);
//                robot.front_left.setPower(-gamepad1.right_stick_x);
//                robot.back_right.setPower(gamepad1.right_stick_x);
//                robot.back_left.setPower(-gamepad1.right_stick_x);
//            }
//--------------------
        }
    }
}
