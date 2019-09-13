package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Colee's Auto", group = "Concept")
//@Disabled
public class LibertyAuto extends LinearOpMode
{
    LibertysMap robot = new LibertysMap();
    double distPerRot = (Math.PI * 3.8125);
    double stepsPerRot = 1120;
    double lengthOfField = 12 * 12;
//----------------------------------------
//----------------------------------------
//---These are all of my Called Methods---
//----------------------------------------
//----------------------------------------

    //Reset all encoder values
    public void resetEncoder()
    {
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Stop
    public void Halt()
    {
        robot.front_right.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.back_left.setPower(0);
    }

    //Forward
    public void Forward(double power)
    {
        robot.front_right.setPower(power);
        robot.front_left.setPower(power);
        robot.back_right.setPower(power);
        robot.back_left.setPower(power);
    }

    public void Backward(double power)
    {
        robot.front_right.setPower(-power);
        robot.front_left.setPower(-power);
        robot.back_right.setPower(-power);
        robot.back_left.setPower(-power);
    }

    public void LTurn(double power)
    {
        robot.front_right.setPower(-power);
        robot.front_left.setPower(power);
        robot.back_right.setPower(-power);
        robot.back_left.setPower(power);
    }

    public void RTurn(double power)
    {
        robot.front_right.setPower(power);
        robot.front_left.setPower(-power);
        robot.back_right.setPower(power);
        robot.back_left.setPower(-power);
    }

    public void moveDistance(double length)
    {
        double distPerRot = Math.PI * 3.8125;
        double stepsPerRot = 1120;
        double totDistInSteps = ((length / distPerRot) * stepsPerRot);

        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO FORWARD!
        if (totDistInSteps > 0)
        {
            //Move forward until we over shoot
            while (totDistInSteps >= robot.front_right.getCurrentPosition())
            {
                Forward(.5);
                telemetry.addData("power", robot.front_right.getCurrentPosition());
                telemetry.update();
            }
        }

        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO BACKWARD!
        else if (totDistInSteps < 0)
        {
            //Move backward until we over shoot
            while (totDistInSteps <= robot.front_right.getCurrentPosition())
            {
                Backward(.5);
                telemetry.addData("power", robot.front_right.getCurrentPosition());
                telemetry.update();
            }
        }

        //reset our encoder values because we may need to do more encoder driving
        Halt();
        resetEncoder();
        sleep(1000);
    }

//--------------------------------------------------------------------------------------------------------------
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        if (opModeIsActive())
        {
            gyro.resetZAxisIntegrator();
            gyro.calibrate();

            telemetry.addData("Heading", robot.gyro.getHeading());
            telemetry.update();
            waitForStart();
            runtime.reset();
//--------------------------------------------------------------------------------------------------------------
            while (opModeIsActive())
            {
                runtime.reset();
                while (!(gyro.getHeading() >= 30 && gyro.getHeading() <= 35))
                {
                    LTurn(.1);
                }

                moveDistance(-26);
                sleep(1000);
                resetEncoder();
                while (!(gyro.getHeading() >= 335 && gyro.getHeading() <= 340))
                {
                    RTurn(.1);
                }

                moveDistance(-10);
            }
//--------------------------------------------------------------------------------------------------------------
        }
    }
}