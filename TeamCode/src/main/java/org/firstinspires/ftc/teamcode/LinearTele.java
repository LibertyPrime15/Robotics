package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Linear Tele", group = "test")
//Last Year's Robot
//@Disabled
public class LinearTele extends LinearOpMode
{
    LinearMap robot = new LinearMap();
//----------------------------------------------------------------------------------------------
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
        while(opModeIsActive() && (!(isStopRequested())))
        {
            double leftPower;
            double rightPower;

            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.left_stick_x;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//----------------------------------
            //This drives the robot forward
            robot.FR.setPower(rightPower);
            robot.FL.setPower(leftPower);
            robot.BR.setPower(rightPower);
            robot.BL.setPower(leftPower);
//----------------------------------
            if(gamepad2.right_stick_y!=0)
                robot.lift.setPower(gamepad2.right_stick_y);
            else
                robot.lift.setPower(0);
//----------------------------------
            if(gamepad2.left_stick_y!=0)
                robot.angle.setPower(gamepad2.left_stick_y);
            else
                robot.angle.setPower(0);
//----------------------------------
            if(gamepad2.right_bumper)
                robot.D.setPower(.8);
            else if(gamepad2.left_bumper)
                robot.D.setPower(-.8);
            else
                robot.D.setPower(0);
//----------------------------------
            if(gamepad2.right_trigger !=0)
                robot.EndDefector.setPower(gamepad2.right_trigger);
            else if(gamepad2.left_trigger !=0)
                robot.EndDefector.setPower(-gamepad2.left_trigger);
            else
                robot.EndDefector.setPower(0);
//----------------------------------
            if(gamepad2.y)
                robot.HookMotor.setPower(1);
            else if(gamepad2.a)
                robot.HookMotor.setPower(-1);
            else
                robot.HookMotor.setPower(0);
//----------------------------------
            telemetry.update();
        }
    }
}
