package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
public class leagueMap
{
    /* Public OpMode members */
    public DcMotor  front_right   = null;
    public DcMotor  front_left    = null;
    public DcMotor  back_left     = null;
    public DcMotor  back_right    = null;

    public DcMotor  lift          = null;
    public DcMotor  arm           = null;

    public Servo claw1 = null;
    public Servo claw2 = null;

    public ElapsedTime runtime = new ElapsedTime();
//--------------------------------------------------------------------------------------------------
    HardwareMap hwMap  =  null;
    public leagueMap(){}

    public void init(HardwareMap ahwMap)
    {
        hwMap  = ahwMap;

        front_right = hwMap.get(DcMotor.class, "front_right");
        front_left  = hwMap.get(DcMotor.class, "front_left");
        back_right  = hwMap.get(DcMotor.class, "back_right");
        back_left   = hwMap.get(DcMotor.class, "back_left");

        lift  = hwMap.get(DcMotor.class, "lift");
        arm   = hwMap.get(DcMotor.class, "arm");

        claw1  = hwMap.get(Servo.class, "claw1");
        claw2  = hwMap.get(Servo.class, "claw2");

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        lift.setPower(0);
        arm.setPower(0);

        front_right.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);

        lift.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
//----------------------------------------//
    //Stop
    public void Halt()
    {
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
    }
//----------------------------------------//
    //Forward
    public void Forward(double power)
    {
        front_right.setPower(-power);
        front_left.setPower(-power);
        back_right.setPower(-power);
        back_left.setPower(-power);
    }
//----------------------------------------//
    //Backward
    public void Backward(double power)
    {
        front_right.setPower(power);
        front_left.setPower(power);
        back_right.setPower(power);
        back_left.setPower(power);
    }
//----------------------------------------//
    //LTurn
    public void turnLeft(double power)
    {
        front_right.setPower(-power);
        front_left.setPower(power);
        back_right.setPower(-power);
        back_left.setPower(power);
    }
//----------------------------------------//
    //RTurn
    public void turnRight(double power)
    {
        front_right.setPower(power);
        front_left.setPower(-power);
        back_right.setPower(power);
        back_left.setPower(-power);
    }
//----------------------------------------//
    //Reset all of the encoder values
    public void resetEncoder()
    {
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
//----------------------------------------//
    public void resetLift()
    {
        lift.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        lift.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
    }
//----------------------------------------//
    public void resetArm()
    {
        arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        arm.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
    }
//----------------------------------------//
    public void openClaw()
    {
        claw1.setPosition(0);
        claw2.setPosition(0);
    }
//----------------------------------------//
    public void closeClaw()
    {
        claw1.setPosition(.5);
        claw2.setPosition(.5);
    }
//----------------------------------------//
    public void moveLeft(double power)
    {
        front_right.setPower(-power);
        front_left.setPower(power);
        back_right.setPower(power);
        back_left.setPower(-power);
    }
//----------------------------------------//
    public void moveRight(double power)
    {
        front_right.setPower(power);
        front_left.setPower(-power);
        back_right.setPower(-power);
        back_left.setPower(power);
    }
//----------------------------------------//
}
