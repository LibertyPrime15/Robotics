package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "centerDepot", group = "centerDepot")
@Disabled
public class centerDepot extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    //Declaring Gyro sensor
    GyroSensor gyro;
    //Declaring Members
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;

    //Declaring Random Crap for positioning
    double distPerRot = (Math.PI * 3.8125);
    double stepsPerRot = 1120;
    double lengthOfField = 12 * 12;
//----------------------------------------

    //Reset all encoder values
    public void resetEncoder()
    {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Stop
    public void Stop()
    {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    //Forward
    public void Forward(double power)
    {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);
    }

    public void Backward(double power)
    {
        FrontLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackLeft.setPower(-power);
        BackRight.setPower(-power);
    }

    public void LTurn(double power)
    {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }

    public void RTurn(double power)
    {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }
//---------------------------------------------------------------------------------------------
    public void moveDistance(double length)
    {
        double distPerRot = Math.PI * 3.8125;
        double stepsPerRot = 1120;

        double totDistInSteps = ((length / distPerRot) * stepsPerRot);

        if (totDistInSteps > 0)
            while (totDistInSteps >= FrontLeft.getCurrentPosition())
            {
                telemetry.addData("power", FrontLeft.getCurrentPosition());
                Forward(.5);
            }

        else if (length < 0) {
            while (totDistInSteps < FrontLeft.getCurrentPosition())
            {
                telemetry.addData("power", FrontLeft.getCurrentPosition());
                Backward(.5);
            }
            //resetEncoder();
            Stop();
        }
    }
    //---------------------------------------------------------------------------------------------
    public void runOpMode() throws InterruptedException
    {

        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        BackLeft = hardwareMap.get(DcMotor.class, "BL");
        BackRight = hardwareMap.get(DcMotor.class, "BR");

        gyro = hardwareMap.get(GyroSensor.class, "gyro");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.resetZAxisIntegrator();
        gyro.calibrate();

        waitForStart();
        runtime.reset();
// ---------------------------------------------------------------------------------------------
        telemetry.addData("Heading", gyro.getHeading());
        while(opModeIsActive())
        {
//----------------------------------------
            //Move forward from origin into the sampling box
            moveDistance(62);
            sleep(200);

            //Back up so the block is not stuck to the bot
            moveDistance(-1);
            sleep(300);

            //Turn to face the red Crater after sampling
            while(gyro.getHeading() != 135)
            {
                telemetry.addData("heading", gyro.getHeading());
                LTurn(.2);

                if(gyro.getHeading() >= 136 && gyro.getHeading() < 300)
                {
                    telemetry.addData("heading", gyro.getHeading());
                    break;
                }
            }

            //Got crater on the red side
            moveDistance(144);
            Stop();
//----------------------------------------
            //Ends the Program and updates the encoders
            telemetry.addData("FrontLeftEncoderValue: ", FrontLeft.getCurrentPosition());
            telemetry.update();
            stop();
        }
    }
}