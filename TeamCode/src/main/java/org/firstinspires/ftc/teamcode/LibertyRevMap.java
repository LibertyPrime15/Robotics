package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class LibertyRevMap
{
    /* Public OpMode members. */
    public DcMotor  front_right   = null;
    public DcMotor  front_left    = null;
    public DcMotor  back_left     = null;
    public DcMotor  back_right    = null;

    public DcMotor  lift          = null;
    public DcMotor  arm           = null;

    public Servo claw1 = null;
    public Servo claw2 = null;

    public GyroSensor gyro;
//--------------------------------------------------------------------------------------------------
    HardwareMap hwMap  =  null;
    public LibertyRevMap(){}

    public void init(HardwareMap ahwMap)
    {
        hwMap  = ahwMap;
        gyro   = hwMap.get(GyroSensor.class, "gyro");

        front_right = hwMap.get(DcMotor.class, "front_right");
        front_left  = hwMap.get(DcMotor.class, "front_left");
        back_right  = hwMap.get(DcMotor.class, "back_right");
        back_left   = hwMap.get(DcMotor.class, "back_left");

        lift  = hwMap.get(DcMotor.class, "back_right");
        arm   = hwMap.get(DcMotor.class, "back_left");

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        lift.setPower(0);
        arm.setPower(0);

        front_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.FORWARD);
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

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
    }
}
