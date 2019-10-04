package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
public class oldMap
{
    /* Public OpMode members */
    public DcMotor  right_drive   = null;
    public DcMotor  left_drive    = null;

    public Servo claw = null;

    public ElapsedTime runtime = new ElapsedTime();
//--------------------------------------------------------------------------------------------------
    HardwareMap hwMap  =  null;
    public oldMap(){}

    public void init(HardwareMap ahwMap)
    {
        hwMap  = ahwMap;

        right_drive = hwMap.get(DcMotor.class, "right_drive");
        left_drive  = hwMap.get(DcMotor.class, "left_drive");

        claw = hwMap.get(Servo.class, "claw");

        right_drive.setPower(0);
        left_drive.setPower(0);

        claw.setPosition(0);

        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive.setDirection(DcMotor.Direction.FORWARD);
    }
//----------------------------------------//
}
