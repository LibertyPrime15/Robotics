package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
public class LinearMap
{
    /* Public OpMode members */
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BR = null;
    public DcMotor BL = null;

    //The motor for connecting to the lander
    public DcMotor Grapple = null;

    //The motors on the arm
    public DcMotor Lift = null;
    public DcMotor Extender = null;
    public DcMotor armTilt = null;

    //The robot's gyro
    public GyroSensor gyro;

    //Our servos
    public Servo Marker = null;
    public Servo Gate = null;
    public CRServo Sweeper = null;

    public ElapsedTime runtime = new ElapsedTime();
    //--------------------------------------------------------------------------------------------------
    HardwareMap hwMap  =  null;
    public LinearMap(){}

    public void init(HardwareMap ahwMap)
    {
        hwMap  = ahwMap;
        gyro   = hwMap.get(GyroSensor.class, "gyro");

        //The Motors for the drive train
        FL = hwMap.get(DcMotor.class, "FL");
        BL = hwMap.get(DcMotor.class, "BL");
        FR = hwMap.get(DcMotor.class, "FR");
        BR = hwMap.get(DcMotor.class, "BR");

        //The Motor That Attaches to the Lander
        Grapple   = hwMap.get(DcMotor.class, "grapple");

        //The Motors for the arm
        Lift   = hwMap.get(DcMotor.class, "lift");
        Extender   = hwMap.get(DcMotor.class, "extender");
        armTilt   = hwMap.get(DcMotor.class, "armTilt");

        //The Servos the run the End Effector
        Gate       = hwMap.get(Servo.class, "gate");
        Sweeper    = hwMap.get(CRServo.class, "sweeper");

        //The Servo for the team marker
        Marker     = hwMap.get(Servo.class, "marker");

        //Declare the direction of all of our motors
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);
        Extender.setDirection(DcMotorSimple.Direction.FORWARD);
        Grapple.setDirection(DcMotorSimple.Direction.FORWARD);
        armTilt.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        Grapple.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Grapple.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
    }
}
