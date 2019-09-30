//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name ="Linear Auto", group = "Concept")
//@Disabled
//public class LinearAuto extends LinearOpMode
//{
//    RevMap robot = new RevMap();
//    private ElapsedTime runtime = new ElapsedTime();
////----------------------------------------//
////----------------------------------------//
////---These are all of my Called Methods---//
////----------------------------------------//
////----------------------------------------//
//    //Reset all encoder values
//    public void resetEncoder()
//    {
//        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    //Stop
//    public void Halt()
//    {
//        robot.front_right.setPower(0);
//        robot.front_left.setPower(0);
//        robot.back_right.setPower(0);
//        robot.back_left.setPower(0);
//    }
//
//    //Forward
//    public void Forward(double power)
//    {
//        robot.front_right.setPower(power);
//        robot.front_left.setPower(power);
//        robot.back_right.setPower(power);
//        robot.back_left.setPower(power);
//    }
//
//    public void Backward(double power)
//    {
//        robot.front_right.setPower(-power);
//        robot.front_left.setPower(-power);
//        robot.back_right.setPower(-power);
//        robot.back_left.setPower(-power);
//    }
//
//    public void LTurn(double power)
//    {
//        robot.front_right.setPower(power);
//        robot.front_left.setPower(-power);
//        robot.back_right.setPower(power);
//        robot.back_left.setPower(-power);
//    }
//
//    public void RTurn(double power)
//    {
//        robot.front_right.setPower(-power);
//        robot.front_left.setPower(power);
//        robot.back_right.setPower(-power);
//        robot.back_left.setPower(power);
//    }
//
//    //This method moves the robot a certain distance in inches
//    private void moveDistance(double length)
//    {
//        double totDistInSteps = (((length / 11.97) * 1120) * -1);
//
//        //IF THE NUMBER IS A POSITIVE NUMBER WE GO FORWARD!
//        if (totDistInSteps < robot.front_right.getCurrentPosition())
//        {
//            while(totDistInSteps <= robot.front_right.getCurrentPosition() && (!(isStopRequested())))
//            {
//                telemetry.addData("Current Value",robot.front_right.getCurrentPosition());
//                telemetry.addData("totDistInSteps",totDistInSteps);
//                telemetry.update();
//                robot.Forward(.1);
//            }
//        }
//        //IF THE NUMBER IS A NEGATIVE NUMBER WE GO BACKWARD!
//        else if (totDistInSteps > robot.front_right.getCurrentPosition())
//        {
//            while (totDistInSteps >= robot.front_right.getCurrentPosition() && (!(isStopRequested())))
//            {
//                telemetry.addData("---Current Value",robot.front_right.getCurrentPosition());
//                telemetry.addData("---totDistInSteps",totDistInSteps);
//                telemetry.update();
//                robot.Backward(.1);
//            }
//        }
//        robot.Halt();
//        robot.resetEncoder();
//    }
////--------------------------------------------------------------------------------------------------------------
//    public void runOpMode() throws InterruptedException
//    {
//        robot.init(hardwareMap);
//        if (opModeIsActive())
//        {
////            robot.gyro.resetZAxisIntegrator();
////            robot.gyro.calibrate();
//
////            telemetry.addData("Heading", robot.gyro.getHeading());
//            telemetry.update();
//
//            waitForStart();
//            runtime.reset();
//
//            while(opModeIsActive())
//            {
//                runtime.reset();
////                while (!(robot.gyro.getHeading() >= 30 && robot.gyro.getHeading() <= 35))
//                {
//                    LTurn(.1);
//                }
//
//                moveDistance(-4);
//                sleep(1000);
//                resetEncoder();
//
////                while (!(robot.gyro.getHeading() >= 335 && robot.gyro.getHeading() <= 340))
//                {
//                    RTurn(.1);
//                }
//
//                moveDistance(-10);
//            }
////--------------------------------------------------------------------------------------------------------------
//        }
//    }
//}
