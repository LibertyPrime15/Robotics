package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class JacobMech extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_right = null;
    private DcMotor front_left = null;
    private DcMotor back_right = null;
    private DcMotor back_left = null;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left  = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            if(gamepad1.left_stick_y != 0)
            {
                front_right.setPower(gamepad1.left_stick_y);
                front_left.setPower(gamepad1.left_stick_y);
                back_right.setPower(gamepad1.left_stick_y);
                back_left.setPower(gamepad1.left_stick_y);
            }

            else if(gamepad1.left_stick_x < 0)
            {
                front_right.setPower(gamepad1.left_stick_x);
                front_left.setPower(-gamepad1.left_stick_x);
                back_right.setPower(gamepad1.left_stick_x);
                back_left.setPower(-gamepad1.left_stick_x);
            }

            else if(gamepad1.left_stick_x > 0)
            {
                front_right.setPower(-gamepad1.left_stick_x);
                front_left.setPower(gamepad1.left_stick_x);
                back_right.setPower(-gamepad1.left_stick_x);
                back_left.setPower(gamepad1.left_stick_x);
            }

            else
            {
                front_right.setPower(0);
                front_left.setPower(0);
                back_right.setPower(0);
                back_left.setPower(0);
            }
        }

    }
}