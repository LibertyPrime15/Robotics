package org.firstinspires.ftc.unused.linear;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="linear Tele", group = "Concept")
//Last Year's Robot
@Disabled
public class linearTele extends LinearOpMode
{
    linearMap robot = new linearMap();
//----------------------------------------------------------------------------------------------
    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Groovy to Go Bois");
        telemetry.update();

        waitForStart();
//--------------------------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
            double leftPower;
            double rightPower;

            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.left_stick_x;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//----------------------------------
            //This drives the robot forward
            robot.FR.setPower(rightPower);
            robot.FL.setPower(leftPower);
            robot.BR.setPower(rightPower);
            robot.BL.setPower(leftPower);
//----------------------------------
            if(gamepad1.x)
            {
                robot.Gate.setPosition(-.8);
            }
            else if(gamepad1.y)
            {
                robot.Gate.setPosition(.8);
            }
//            else
//            {
//                robot.Sweeper.setPower(1);
//            }
//----------------------------------
            if(gamepad1.a)
            {
                robot.Marker.setPosition(.8);
            }
            else if(gamepad1.b)
            {
                robot.Marker.setPosition(-.8);
            }
//------------------------------------------------------------------------------------------------//
            //THIS IS ALL GAMEPAD 2//
//------------------------------------------------------------------------------------------------//
            //Moves the Grapple Connector Up Towards the Lander
            if(gamepad2.x)
                robot.Grapple.setPower(1);
            else if(gamepad2.y)
                robot.Grapple.setPower(-1);
            else
                robot.Grapple.setPower(0);
//--------------------------------------------------------------------
            //Extends the length of the arm
            if(gamepad2.left_stick_y!=0)
                robot.Extender.setPower(gamepad2.left_stick_y);
            else
                robot.Extender.setPower(0);
//--------------------------------------------------------------------
            //Changes the angle of the arm
            if(gamepad2.right_stick_y!=0)
                robot.armTilt.setPower(gamepad2.right_stick_y);
            else
                robot.armTilt.setPower(0);
//--------------------------------------------------------------------
            //Lifts the base of the arm higher into the air
            if(gamepad2.left_trigger !=0)
                robot.Lift.setPower(1);
            else if(gamepad2.right_trigger !=0)
                robot.Lift.setPower(-1);
            else
                robot.Lift.setPower(0);
//--------------------------------------------------------------------
            //Opens and closes the gate of the end effector
            if(gamepad2.right_bumper)
                robot.Sweeper.setPower(.8);
            else if(gamepad2.left_bumper)
                robot.Sweeper.setPower(-.8);
            else
                robot.Sweeper.setPower(0);
//--------------------------------------------------------------------
            telemetry.update();
        }
    }
}
