package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "rightDepot", group = "rightDepot")
@Disabled
public class rightDepot extends LinearOpMode
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

        else if (length < 0)
        {
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
//---------------------------------------------------------------------------------------------
        telemetry.addData("Heading", gyro.getHeading());
        while(opModeIsActive())
        {
//----------------------------------------
            //Turn to face the cube
            while(gyro.getHeading() != 146)
            {
                telemetry.addData("heading", gyro.getHeading());

                LTurn(.2);

                if(gyro.getHeading() >= 147)
                {
                    telemetry.addData("heading", gyro.getHeading());
                    break;
                }
            }

            //Sample the cube on the right side
            moveDistance(-40);

            //turn left to face the crater
            while(gyro.getHeading() != 36)
            {
                telemetry.addData("heading", gyro.getHeading());

                LTurn(.2);

                if(gyro.getHeading() >= 37)
                {
                    telemetry.addData("heading", gyro.getHeading());
                    break;
                }
            }

            //The robot will drive into the depot to drop off the team marker
            moveDistance(-20);

            //Drive into the red crater
            moveDistance(-144);
            Stop();
//----------------------------------------
            //Ends the Program and updates the encoders
            telemetry.addData("FrontLeftEncoderValue: ", FrontLeft.getCurrentPosition());
            telemetry.update();
            stop();

        }
    }
}