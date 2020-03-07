package org.firstinspires.ftc.teamcode.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RevMap;

@Autonomous(name="GingerAuto", group="Ginger")
//@Disabled
public class GingerAuto extends LinearOpMode
{
    // Declare OpMode members
    RevMap robot = new RevMap();


















//-----------------------------------------------
public void driveDistance(double length)
{
    //Convert steps into inches
    //40:1 - 1160steps/per rotation
    //circumfrence = 2pir


    double circumfrence = 11.97;
    double stepsPerInch = 1160/circumfrence;
    double goal = stepsPerInch*length;



    if(length > 0)
    {
        while(robot.back_left.getCurrentPosition() < goal)
        {
            //Forward
            robot.back_left.setPower(.5);
            robot.back_right.setPower(.5);
            robot.front_left.setPower(.5);
            robot.front_right.setPower(.5);
        }
    }
    else if(length < 0)
    {
        while(robot.back_left.getCurrentPosition() < goal)
        {
            //Backward
            robot.back_left.setPower(-.5);
            robot.back_right.setPower(-.5);
            robot.front_left.setPower(-.5);
            robot.front_right.setPower(-.5);
        }
    }
}
//-------------------------------------------------------
public void openClaw()
{
    robot.claw1.setPosition(.5);
    robot.claw2.setPosition(.5);
}
//-------------------------------------------------------
public void closeClaw()
{

}
//-------------------------------------------------------
public void turn(double multiplier)
{
    robot.front_left.setPower(multiplier * .5);
    robot.back_left.setPower(.5);
    robot.front_right.setPower(-.5);
    robot.back_right.setPower((-.5));
}
//-----------------------------------------------






























//-----------------------------------------------
    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
//------------------------------------------------
        if(opModeIsActive())
        {
            while(opModeIsActive())
            {
                openClaw();
                driveDistance(26);
                stop();
            }
        }
    }
}