package org.firstinspires.ftc.leagueCode;
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

//    public DcMotor  motor1  = null;
//    public DcMotor  motor2  = null;
//    public DcMotor  motor3  = null;
//    public DcMotor  motor4  = null;
//
//    public Servo arm1 = null;
//    public Servo arm2 = null;
//    public Servo arm3 = null;
//    public Servo arm4 = null;
//    public Servo arm5 = null;
//
//    public Servo arm6  = null;
//    public Servo arm7  = null;
//    public Servo arm8  = null;
//    public Servo arm9  = null;
//    public Servo arm10 = null;
//
//    public Servo arm11 = null;
//    public Servo arm12 = null;
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

//        motor1   = hwMap.get(DcMotor.class, "motor1");
//        motor2   = hwMap.get(DcMotor.class, "motor2");
//        motor3   = hwMap.get(DcMotor.class, "motor3");
//        motor4   = hwMap.get(DcMotor.class, "motor4");
//
//        arm1  = hwMap.get(Servo.class, "arm1");
//        arm2  = hwMap.get(Servo.class, "arm2");
//        arm3  = hwMap.get(Servo.class, "arm3");
//        arm4  = hwMap.get(Servo.class, "arm4");
//        arm5  = hwMap.get(Servo.class, "arm5");
//
//        arm6  = hwMap.get(Servo.class, "arm6");
//        arm7  = hwMap.get(Servo.class, "arm7");
//        arm8  = hwMap.get(Servo.class, "arm8");
//        arm9  = hwMap.get(Servo.class, "arm9");
//        arm10 = hwMap.get(Servo.class, "arm10");
//
//        arm11 = hwMap.get(Servo.class, "arm11");
//        arm12 = hwMap.get(Servo.class, "arm12");
//------------------------------
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

//        motor1.setPower(0);
//        motor2.setPower(0);
//        motor3.setPower(0);
//        motor4.setPower(0);
//------------------------------
        front_right.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);

//        motor1.setDirection(DcMotor.Direction.REVERSE);
//        motor2.setDirection(DcMotor.Direction.FORWARD);
//        motor3.setDirection(DcMotor.Direction.REVERSE);
//        motor4.setDirection(DcMotor.Direction.FORWARD);
//------------------------------
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//------------------------------
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
}
