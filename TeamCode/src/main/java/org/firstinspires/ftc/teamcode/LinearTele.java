package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Linear Tele", group = "test")
@Disabled
public class LinearTele extends LinearOpMode
{
    LinearMap robot = new LinearMap();
//----------------------------------------------------------------------------------------------

    public void FaB(double pow)
    {
        robot.FL.setPower(pow);
        robot.FR.setPower(pow);
        robot.BL.setPower(pow);
        robot.BR.setPower(pow);
    }
    public void Turn(double pow)
    {
        robot.FL.setPower(pow);
        robot.FR.setPower(-pow);
        robot.BL.setPower(pow);
        robot.BR.setPower(-pow);
    }

    public void angle(int power)
    {
        robot.angle.setPower(power);
    }

    public void Halt()
    {
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
        robot.lift.setPower(0);
        robot.angle.setPower(0);
    }

    double dPosition = 100;
    double ePos = 100;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Groovy to Go Bois");
        telemetry.update();

        waitForStart();
//--------------------------------------------------------------------------------
        while(opModeIsActive())
        {
            if(gamepad1.right_stick_y!=0)
                FaB(-gamepad1.right_stick_y / 2);

            else if(gamepad1.left_stick_x != 0)
                Turn(gamepad1.left_stick_x / 2);

            else
            {
                robot.FL.setPower(0);
                robot.FR.setPower(0);
                robot.BL.setPower(0);
                robot.BR.setPower(0);
            }

            if(gamepad2.right_stick_y!=0)
                robot.lift.setPower(gamepad2.right_stick_y);

            else if(gamepad2.left_stick_y!=0)
                robot.angle.setPower(gamepad2.left_stick_y);

            else if(gamepad2.right_bumper)
                robot.D.setPower(.8);

            else if(gamepad2.left_bumper)
                robot.D.setPower(-.8);

            else if(gamepad2.right_trigger !=0)
                robot.EndDefector.setPower(gamepad2.right_trigger);

            else if(gamepad2.left_trigger !=0)
                robot.EndDefector.setPower(-gamepad2.left_trigger);

            else if(gamepad2.y)
                robot.HookMotor.setPower(1);

            else if(gamepad2.a)
                robot.HookMotor.setPower(-1);

            else
            {
                robot.EndDefector.setPower(0);
                robot.HookMotor.setPower(0);

                robot.D.setPower(0);
                robot.lift.setPower(0);
                robot.angle.setPower(0);
            }
            telemetry.update();
        }
    }
}
