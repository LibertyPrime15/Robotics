package org.firstinspires.ftc.teamcode;

import android.database.DatabaseErrorHandler;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.DomFront;

@TeleOp(name="TeleOp", group = "test")
@Disabled
public class LinearOp extends LinearOpMode {

    private ElapsedTime runTime = new ElapsedTime();

    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BR = null;
    DcMotor BL = null;

    DcMotor EndDefectorSweeper = null;
    DcMotor HookMotor = null;

    DcMotor JointOne = null;
    DcMotor JointTwo = null;

    private CRServo D;


    public void FaB(double pow){
        FL.setPower(pow);
        FR.setPower(pow);
        BL.setPower(pow);
        BR.setPower(pow);
    }
    public void Turn(double pow){
        FL.setPower(pow);
        FR.setPower(-pow);
        BL.setPower(pow);
        BR.setPower(-pow);
    }

    public void moveArm(int power){

    }
    public void moveJointTwo(int power){
        JointTwo.setPower(power);
    }

    public void Cease(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        JointOne.setPower(0);
        JointTwo.setPower(0);
    }

    double dPosition = 100;
    double ePos = 100;


    public void runOpMode(){

        runTime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        EndDefectorSweeper = hardwareMap.get(DcMotor.class, "Sweeper");
        HookMotor = hardwareMap.get(DcMotor.class, "Hook");

        JointOne = hardwareMap.get(DcMotor.class, "one");
        JointTwo = hardwareMap.get(DcMotor.class, "two");

        D = hardwareMap.get(CRServo.class, "d");


        JointOne.setDirection(DcMotorSimple.Direction.FORWARD);
        JointTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();
        runTime.reset();

        while(opModeIsActive()){

            if(gamepad1.right_stick_y!=0){
                FaB(-gamepad1.right_stick_y / 2);
            }else if(gamepad1.left_stick_x != 0) {
                Turn(gamepad1.left_stick_x / 2);
            }else{
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
            }

            if(gamepad2.right_stick_y!=0){
                JointOne.setPower(gamepad2.right_stick_y);
            }else if(gamepad2.left_stick_y!=0) {
                JointTwo.setPower(gamepad2.left_stick_y);
            }else if(gamepad2.right_bumper) {
                D.setPower(.8);
            }else if(gamepad2.left_bumper) {
                D.setPower(-.8);
            }else if(gamepad2.right_trigger !=0) {
                EndDefectorSweeper.setPower(gamepad2.right_trigger);
            }else if(gamepad2.left_trigger !=0) {
                EndDefectorSweeper.setPower(-gamepad2.left_trigger);
            }else if(gamepad2.y) {
                HookMotor.setPower(1);
            }else if(gamepad2.a) {
                HookMotor.setPower(-1);
            }else{
                EndDefectorSweeper.setPower(0);
                HookMotor.setPower(0);

                D.setPower(0);
                JointOne.setPower(0);
                JointTwo.setPower(0);
            }

            telemetry.update();

        }
    }
}
