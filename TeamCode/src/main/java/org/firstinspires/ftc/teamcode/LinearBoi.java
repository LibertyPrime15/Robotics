package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Colee's TeleOp", group = "Main")
public class LinearBoi extends LinearOpMode
{
    DcMotor front_right;
    DcMotor front_left;
    DcMotor back_right;
    DcMotor back_left;

    //----------------------------------------------------------------------------------------------
    private void FowardPosBackNeg(double pow)
    {
        front_right.setPower(pow);
        front_left.setPower(pow);
        back_left.setPower(pow);
        back_right.setPower(pow);
    }
    //----------------------------------------------------------------------------------------------
    private void Halt(){
        front_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    //----------------------------------------------------------------------------------------------
    public void runOpMode(){

        robot.init(hardwareMap);

        telemetry.addData("Status", "Let's Go Boisssss");
        telemetry.update();

//        //------------------------------------------------------------------------------------------
//        front_right = hardwareMap.get(DcMotor.class, "front_right");
//        back_left = hardwareMap.get(DcMotor.class, "front_left");
//        front_left = hardwareMap.get(DcMotor.class, "back_right");
//        back_right = hardwareMap.get(DcMotor.class, "back_left");
//        //------------------------------------------------------------------------------------------
//        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
//        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
//        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
//        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        //------------------------------------------------------------------------------------------
//        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //------------------------------------------------------------------------------------------
        waitForStart();

        while(opModeIsActive()){

            back_left.setPower(0);
            front_right.setPower(0);
            back_right.setPower(0);
            front_left.setPower(0);

            if(gamepad1.right_stick_y!=0)
            {
                back_left.setPower(gamepad1.right_stick_y);
                front_right.setPower(gamepad1.right_stick_y);
            }

            if(gamepad1.left_stick_y!=0)
            {
                back_right.setPower(gamepad1.left_stick_y);
                front_left.setPower(gamepad1.left_stick_y);
            }
        }
    }
}
