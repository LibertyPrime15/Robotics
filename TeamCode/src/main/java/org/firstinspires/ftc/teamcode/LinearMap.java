//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Disabled
//public class LinearMap
//{
//    /* Public OpMode members */
//    public DcMotor FL = null;
//    public DcMotor FR = null;
//    public DcMotor BR = null;
//    public DcMotor BL = null;
//
//    public DcMotor EndDefector = null;
//    public DcMotor HookMotor = null;
//
//    public DcMotor lift = null;
//    public DcMotor angle = null;
//
//    public GyroSensor gyro;
//    public CRServo D;
//
//    public double dPosition = 100;
//    public double ePos = 100;
//
//    public ElapsedTime runtime = new ElapsedTime();
//    //--------------------------------------------------------------------------------------------------
//    HardwareMap hwMap  =  null;
//    public LinearMap(){}
//
//    public void init(HardwareMap ahwMap)
//    {
//        hwMap  = ahwMap;
//        gyro   = hwMap.get(GyroSensor.class, "gyro");
//
//        FL = hwMap.get(DcMotor.class, "FL");
//        BL = hwMap.get(DcMotor.class, "BL");
//        FR = hwMap.get(DcMotor.class, "FR");
//        BR = hwMap.get(DcMotor.class, "BR");
//
//        EndDefector = hwMap.get(DcMotor.class, "Sweeper");
//        HookMotor   = hwMap.get(DcMotor.class, "Hook");
//
//        lift = hwMap.get(DcMotor.class, "one");
//        angle = hwMap.get(DcMotor.class, "two");
//
//        D    = hwMap.get(CRServo.class, "d");
//        gyro = hwMap.get(GyroSensor.class, "gyro");
//
//        lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        angle.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        FL.setDirection(DcMotorSimple.Direction.REVERSE);
//        FR.setDirection(DcMotorSimple.Direction.FORWARD);
//        BL.setDirection(DcMotorSimple.Direction.REVERSE);
//        BR.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        gyro.resetZAxisIntegrator();
//        gyro.calibrate();
//    }
//
//    public void FaB(double pow)
//    {
//        FL.setPower(pow);
//        FR.setPower(pow);
//        BL.setPower(pow);
//        BR.setPower(pow);
//    }
//    public void Turn(double pow)
//    {
//        FL.setPower(pow);
//        FR.setPower(-pow);
//        BL.setPower(pow);
//        BR.setPower(-pow);
//    }
//
//    public void angle(int power)
//    {
//        angle.setPower(power);
//    }
//
//    public void Halt()
//    {
//        FL.setPower(0);
//        FR.setPower(0);
//        BL.setPower(0);
//        BR.setPower(0);
//        lift.setPower(0);
//        angle.setPower(0);
//    }
//}
