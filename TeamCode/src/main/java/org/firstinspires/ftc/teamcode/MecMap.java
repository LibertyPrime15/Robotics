package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
public class MecMap
{
    /* Public OpMode members */
    public DcMotor  front_right   = null;
    public DcMotor  front_left    = null;
    public DcMotor  back_left     = null;
    public DcMotor  back_right    = null;

    public ElapsedTime runtime = new ElapsedTime();
//--------------------------------------------------------------------------------------------------
    HardwareMap hwMap  =  null;
    public MecMap(){}

    public void init(HardwareMap ahwMap)
    {
        hwMap  = ahwMap;

        front_right = hwMap.get(DcMotor.class, "front_right");
        front_left  = hwMap.get(DcMotor.class, "front_left");
        back_right  = hwMap.get(DcMotor.class, "back_right");
        back_left   = hwMap.get(DcMotor.class, "back_left");

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        front_right.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);

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
    public void LTurn(double power)
    {
        front_right.setPower(-power);
        front_left.setPower(power);
        back_right.setPower(-power);
        back_left.setPower(power);
    }
//----------------------------------------//
    //RTurn
    public void RTurn(double power)
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
}
