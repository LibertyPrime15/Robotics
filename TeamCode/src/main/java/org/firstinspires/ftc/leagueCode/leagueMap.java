package org.firstinspires.ftc.leagueCode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
public class leagueMap
{
	/* Public OpMode members */
	public DcMotor front_right = null;
	public DcMotor front_left = null;
	public DcMotor back_left = null;
	public DcMotor back_right = null;
	
	public DcMotor intake1 = null;
	public DcMotor intake2 = null;
	public DcMotor liftPrimary = null;
	public DcMotor liftSecondary = null;
	
	public Servo flip1 = null;
	public Servo flip2 = null;
	public Servo wrist = null;
	public Servo rotate = null;
	public Servo grabber = null;
	public Servo plateGrabber1 = null;
	public Servo plateGrabber2 = null;
	
	
	public boolean canToggleIntake = true;
	
	ColorSensor sensorColor;
	
//    public Servo arm6  = null;
//    public Servo arm7  = null;
//    public Servo arm8  = null;
//    public Servo arm9  = null;
//    public Servo arm10 = null;
//
//    public Servo arm11 = null;
//    public Servo arm12 = null;
//--------------------------------------------------------------------------------------------------
	HardwareMap hwMap = null;
	
	public leagueMap() {}
	
	public void init(HardwareMap ahwMap)
	{
		hwMap = ahwMap;
		
		sensorColor = hwMap.get(ColorSensor.class, "color_sensor");
		
		front_right = hwMap.get(DcMotor.class, "front_right");
		front_left = hwMap.get(DcMotor.class, "front_left");
		back_right = hwMap.get(DcMotor.class, "back_right");
		back_left = hwMap.get(DcMotor.class, "back_left");
		
		intake1 = hwMap.get(DcMotor.class, "intake1");
		intake2 = hwMap.get(DcMotor.class, "intake2");
		liftPrimary = hwMap.get(DcMotor.class, "liftPrimary");
		liftSecondary = hwMap.get(DcMotor.class, "liftSecondary");
		
		flip1 = hwMap.get(Servo.class, "flip1");
		flip2 = hwMap.get(Servo.class, "flip2");
		
		wrist = hwMap.get(Servo.class, "wrist");
		rotate = hwMap.get(Servo.class, "rotate");
		grabber = hwMap.get(Servo.class, "grabber");
		
		plateGrabber1  = hwMap.get(Servo.class, "plateGrabber1");
		plateGrabber2  = hwMap.get(Servo.class, "plateGrabber2");
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
		
		intake1.setPower(0);
		intake2.setPower(0);
		liftPrimary.setPower(0);
		liftSecondary.setPower(0);
//------------------------------
		front_right.setDirection(DcMotor.Direction.FORWARD);
		front_left.setDirection(DcMotor.Direction.REVERSE);
		back_right.setDirection(DcMotor.Direction.FORWARD);
		back_left.setDirection(DcMotor.Direction.REVERSE);
		
		intake1.setDirection(DcMotor.Direction.REVERSE);
		intake2.setDirection(DcMotor.Direction.FORWARD);
		liftPrimary.setDirection(DcMotor.Direction.REVERSE);
		liftSecondary.setDirection(DcMotor.Direction.FORWARD);
//------------------------------
		front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		
		intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftPrimary.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		liftSecondary.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//------------------------------
		front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
		intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		liftPrimary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		liftSecondary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
	
	//----------------------------------------//
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
	
	public void initiatePlateGrabber()
	{
		plateGrabber1.setPosition(0);
		plateGrabber2.setPosition(0.73);
	}
	
	
}
//-----------------------------

//hello