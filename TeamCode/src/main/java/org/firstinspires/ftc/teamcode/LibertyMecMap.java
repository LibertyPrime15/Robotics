package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class LibertyMecMap
{
    /* Public OpMode members. */
    public DcMotor  front_right   = null;
    public DcMotor  front_left    = null;
    public DcMotor  back_left     = null;
    public DcMotor  back_right    = null;

    public GyroSensor gyro;
//--------------------------------------------------------------------------------------------------
    HardwareMap hwMap  =  null;
    public LibertyMecMap(){}

    public void init(HardwareMap ahwMap)
    {
        hwMap  = ahwMap;
        gyro   = hwMap.get(GyroSensor.class, "gyro");

        front_right = hwMap.get(DcMotor.class, "front_right");
        front_left  = hwMap.get(DcMotor.class, "front_left");
        back_right  = hwMap.get(DcMotor.class, "back_right");
        back_left   = hwMap.get(DcMotor.class, "back_left");

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        front_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
    }
}
