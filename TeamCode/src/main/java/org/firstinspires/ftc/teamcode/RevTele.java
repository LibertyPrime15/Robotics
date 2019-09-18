package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Rev Tele", group = "Main")
@Disabled
public class RevTele extends LinearOpMode
{
    RevMap robot = new RevMap();
//----------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Let's Go Get this Lego Boi");
        telemetry.update();

        waitForStart();
//------------------------------------------------------------
        while(opModeIsActive())
        {
            //This drives the robot forward
            if(gamepad1.left_stick_y !=0)
            {
                robot.front_right.setPower(gamepad1.left_stick_y);
                robot.front_left.setPower(gamepad1.left_stick_y);
                robot.back_right.setPower(gamepad1.left_stick_y);
                robot.back_left.setPower(gamepad1.left_stick_y);
            }

            //This turns the robot to the left
            else if(gamepad1.left_stick_x < 0)
            {
                robot.front_right.setPower(gamepad1.left_stick_x);
                robot.front_left.setPower(-gamepad1.left_stick_x);
                robot.back_right.setPower(gamepad1.left_stick_x);
                robot.back_left.setPower(-gamepad1.left_stick_x);
            }

            //This turns the robot to the right
            else if(gamepad1.left_stick_x > 0)
            {
                robot.front_right.setPower(gamepad1.left_stick_x);
                robot.front_left.setPower(-gamepad1.left_stick_x);
                robot.back_right.setPower(gamepad1.left_stick_x);
                robot.back_left.setPower(-gamepad1.left_stick_x);
            }

            //This opens the claw
            else if(gamepad1.y)
            {
                robot.claw1.setPosition(0);
                robot.claw2.setPosition(.5);
            }

            //This closes the claw
            else if(gamepad1.x)
            {
                robot.claw1.setPosition(.5);
                robot.claw2.setPosition(0);
            }

            //This moves the arm up
            else if(gamepad1.left_trigger !=0)
            {
                robot.arm.setPower(-.3);
            }

            //This moves the arm down
            else if(gamepad1.right_trigger !=0)
            {
                robot.arm.setPower(.3);
            }

            //This move the arm around
            else if(gamepad1.right_stick_y !=0)
            {
                robot.lift.setPower(gamepad1.right_stick_y);
            }

            //This sets all of the motors to 0
            else
            {
                robot.front_right.setPower(0);
                robot.front_left.setPower(0);
                robot.back_right.setPower(0);
                robot.back_left.setPower(0);

                robot.lift.setPower(0);
                robot.arm.setPower(0);
            }
            telemetry.update();
        }
    }
}
//----------------------------------------------------------------------------------------------