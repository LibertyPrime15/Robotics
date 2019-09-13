package org.firstinspires.ftc.teamcode;

import android.database.DatabaseErrorHandler;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.internal.android.dx.ssa.DomFront;

@Autonomous(name="GyroTest", group = "test")
@Disabled
public class Gyro_Test extends LinearOpMode {

    private ElapsedTime runTime = new ElapsedTime();

    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BR = null;
    DcMotor BL = null;

    ModernRoboticsI2cGyro gyro;

    public void Turn(double pow) {
        FL.setPower(pow);
        FR.setPower(-pow);
        BL.setPower(pow);
        BR.setPower(-pow);

    }

    public void Stop(int sleep){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        sleep(sleep);

    }

    public void runOpMode() throws InterruptedException{

        runTime.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        gyro.calibrate();
        gyro.resetZAxisIntegrator();
        waitForStart();
        runTime.reset();

        while(opModeIsActive()){

            while(gyro.getHeading() < 340 && gyro.getHeading() > 350){
                Turn(-.1);
            }
            telemetry.addData("First Turn", "345");
            telemetry.update();
            Stop(1000);
            while(gyro.getHeading() < 30 && gyro.getHeading() > 40){
                Turn(.1);
            }
            telemetry.addData("Second Turn", "345");
            telemetry.update();
            Stop(1000);



            telemetry.update();

        }
    }
}
