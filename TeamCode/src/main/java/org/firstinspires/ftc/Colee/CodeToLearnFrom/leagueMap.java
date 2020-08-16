package org.firstinspires.ftc.Colee.CodeToLearnFrom;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//This class is pretty straight forward, this is where we declare all of our different forms of hardware
//Such as motors, servos, sensors, we can create different methods, and we also establish the naming syntax
//Used by the phones when pairing the hardware to the hubs

//@Disabled
public class leagueMap
{
    //Here's a bunch of named stuff... it must always be public if using a hardware map

    /* Public OpMode members */
    public DcMotor front_right = null;
    public DcMotor front_left = null;
    public DcMotor back_left = null;
    public DcMotor back_right = null;

    public DcMotor intake1 = null;
    public DcMotor intake2 = null;
    public DcMotor liftPrimary = null;
    public DcMotor liftSecondary = null;

    public Servo flip1         = null;
    public Servo flip2         = null;
    public Servo wrist         = null;
    public Servo rotate        = null;
    public Servo grabber       = null;
    public Servo plateGrabber1 = null;
    public Servo plateGrabber2 = null;
    public Servo capStone      = null;
	public Servo sideGrabber   = null;
	public CRServo measuringTape = null;
	
	public ColorSensor sensorColor;
	
	public boolean canToggleIntake = true;
//--------------------------------------------------------------------------------------------------
    //This is a constructor for the hardware map such that it can be initialized inside of a running instance of another class
    HardwareMap hwMap = null;
    public leagueMap(){}

    public void init(HardwareMap ahwMap)
	{
	    //This is a notation used below, it really isn't entirely necessary if you know what you're doing
        hwMap = ahwMap;

        //This is how you set up a configuration for the phones!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Depending on the piece of hardware you're using the import name will change but the syntax remains the same regardless - this is a must!!!
        //You need to understand the relationship between these words and the rev hub \/ \/ \/ \/ \/
        sensorColor = hwMap.get(ColorSensor.class, "color_sensor");

        front_right = hwMap.get(DcMotor.class, "front_right");
        front_left  = hwMap.get(DcMotor.class, "front_left");
        back_right  = hwMap.get(DcMotor.class, "back_right");
        back_left   = hwMap.get(DcMotor.class, "back_left");

        intake1       = hwMap.get(DcMotor.class, "intake1");
        intake2       = hwMap.get(DcMotor.class, "intake2");
        liftPrimary   = hwMap.get(DcMotor.class, "liftPrimary");
        liftSecondary = hwMap.get(DcMotor.class, "liftSecondary");

        flip1 = hwMap.get(Servo.class, "flip1");
        flip2 = hwMap.get(Servo.class, "flip2");

        wrist   = hwMap.get(Servo.class, "wrist");
        rotate  = hwMap.get(Servo.class, "rotate");
        grabber = hwMap.get(Servo.class, "grabber");

        plateGrabber1  = hwMap.get(Servo.class, "plateGrabber1");
        plateGrabber2  = hwMap.get(Servo.class, "plateGrabber2");
        capStone       = hwMap.get(Servo.class, "capStone");
		sideGrabber    = hwMap.get(Servo.class, "sideGrabber");
		measuringTape  = hwMap.get(CRServo.class, "tape");
//------------------------------
        //We're setting the power to 0 for all of the motors so once they receive power they don't seize a little and scare the refs
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        intake1.setPower(0);
        intake2.setPower(0);
        liftPrimary.setPower(0);
        liftSecondary.setPower(0);
//------------------------------
        //You need to set the proper directions for all of the different motors otherwise they go the wrong direction... Big Brain - This does not affect negative signs when assigning power
        front_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        liftPrimary.setDirection(DcMotor.Direction.REVERSE);
        liftSecondary.setDirection(DcMotor.Direction.FORWARD);
//------------------------------
        //Encoders are necessary for most* motors but it really depends on what needs to be accomplished
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //THIS IS COMMENTED OUT SO WE CAN MOVE THE MOTOR TO A POSITION USING A PID LOOP
//        liftPrimary.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftSecondary.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//------------------------------
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPrimary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftSecondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //----------------------------------------//


    /*We have multiple methods below, however none of them use any real time data
    because this class cannot process data and return updated powers and times and such
    because the class never runs, not instance of this class is created, an import is done into another instance of a class
    and that's all - THIS CLASS DOES NOT RUN BECAUSE THERE IS NOT INSTANCE OF IT CALLED TO RUN
    You can however all these methods using the syntax below, ignore the quotation marks

    "robot.Halt()"

    Just because these methods do not use active data does not mean you cannot pass in arguments as parameters like in turnLeft()
    Also, not all of the methods below entirely work properly such as setDriveToBrake()

    */
    //Stop
    public void Halt() {
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
    }
    //----------------------------------------//
    //Reset all of the encoder values
    public void resetEncoder() {
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
    public void resetLift()
    {
        liftSecondary.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftSecondary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //LTurn
    public void turnLeft(double power) {
        front_right.setPower(-power);
        front_left.setPower(power);
        back_right.setPower(-power);
        back_left.setPower(power);
    }
    //----------------------------------------//
    //RTurn
    public void turnRight(double power) {
        front_right.setPower(power);
        front_left.setPower(-power);
        back_right.setPower(power);
        back_left.setPower(-power);
    }
    //----------------------------------------//
    public void intake(double power) {
        intake1.setPower(-power);
        intake2.setPower(-power);
        canToggleIntake = true;
    }
    //----------------------------------------//
    public void outtake(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
        canToggleIntake = false;
    }
    //----------------------------------------//
    public void stopIntake() {
        intake1.setPower(0);
        intake2.setPower(0);

        canToggleIntake = false;
    }
    //----------------------------------------//
    public void setDriveToBrake()
    {
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void grabPlate()
    {
        plateGrabber1.setPosition(0);
        plateGrabber2.setPosition(0.73);
    }

    public void ungrabPlate()
    {
        plateGrabber1.setPosition(0.73);
        plateGrabber2.setPosition(0);
    }

    public void disengageIntake()
    {
        plateGrabber1.setPosition(0.365);
        plateGrabber2.setPosition(0.365);
    }
}
//-----------------------------