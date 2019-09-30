package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Mec Tele", group = "Main")
//@Disabled
public class MecTele extends LinearOpMode
{
    MecMap robot = new MecMap();
//--------------------------------------------------------------------------------------------------
//----------------------------------------//
//----------------------------------------//
//---These are all of my Called Methods---//
//----------------------------------------//
//----------------------------------------//
//--------------------------------------------------------------------------------------------------
    public double angleCheck()
    {
        telemetry.addLine().addData("After", robot.curHeading);
        telemetry.update();
        robot.angles = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.robot.imu.getPosition();
        robot.curHeading = robot.angles.firstAngle;
        return robot.curHeading;
    }
//--------------------------------------------------------------------------------------------------











































//--------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        angleCheck();
        telemetry.addData("Status", "Let's Go Boisssss");
        telemetry.update();

        waitForStart();
//------------------------------------------------------------
        while(opModeIsActive() && (!(isStopRequested())))
        {
            telemetry.addLine().addData("Heading",robot.curHeading);
            telemetry.update();

            //This is for basic movement forward/backward
            if(gamepad1.left_stick_y !=0)
            {
                robot.front_right.setPower(gamepad1.left_stick_y);
                robot.front_left.setPower(gamepad1.left_stick_y);
                robot.back_right.setPower(gamepad1.left_stick_y);
                robot.back_left.setPower(gamepad1.left_stick_y);
            }
//--------------------------------------------------------------------------------
            //This moves the robot to the  left
            else if(gamepad1.left_stick_x < 0)
            {
                robot.front_right.setPower(-gamepad1.left_stick_x);
                robot.front_left.setPower(gamepad1.left_stick_x);
                robot.back_right.setPower(gamepad1.left_stick_x);
                robot.back_left.setPower(-gamepad1.left_stick_x);
            }
            //This moves the robot to the right
            else if(gamepad1.left_stick_x > 0)
            {
                robot.front_right.setPower(-gamepad1.left_stick_x);
                robot.front_left.setPower(gamepad1.left_stick_x);
                robot.back_right.setPower(gamepad1.left_stick_x);
                robot.back_left.setPower(-gamepad1.left_stick_x);
            }
//--------------------------------------------------------------------------------
//            This is for turning the robot to the left
            else if(gamepad1.right_stick_x < 0)
            {
                robot.front_right.setPower(gamepad1.right_stick_x);
                robot.front_left.setPower(-gamepad1.right_stick_x);
                robot.back_right.setPower(gamepad1.right_stick_x);
                robot.back_left.setPower(-gamepad1.right_stick_x);
            }
//            //This is for turning the robot to the left
            else if(gamepad1.right_stick_x > 0)
            {
                robot.front_right.setPower(gamepad1.right_stick_x);
                robot.front_left.setPower(-gamepad1.right_stick_x);
                robot.back_right.setPower(gamepad1.right_stick_x);
                robot.back_left.setPower(-gamepad1.right_stick_x);
            }
//--------------------------------------------------------------------------------
            else
            {
                robot.front_right.setPower(0);
                robot.front_left.setPower(0);
                robot.back_right.setPower(0);
                robot.back_left.setPower(0);
            }
        telemetry.update();
        }
    }
}
