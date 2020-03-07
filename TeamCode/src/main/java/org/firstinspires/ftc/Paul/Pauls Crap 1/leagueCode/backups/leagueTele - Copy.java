//package org.firstinspires.ftc.leagueCode;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//@TeleOp(name="leagueSolo", group = "A")
////@Disabled
//public class leagueTele extends LinearOpMode
//{
//    leagueMap robot = new leagueMap();
//    Orientation angles;
//    BNO055IMU imu;
//
//    float currHeading = 0;
//    boolean isSpinningInward = false;
//    //    boolean canToggleIntake  = true;
//    boolean canToggleGrabber = true;
//    boolean canTogglePlateGrabber = true;
//
//    boolean hasBlock = false;
//
//    boolean canAddToLiftPos = true;
//    boolean canSubtractFromLiftPos = true;
//
//    int armHeight = 0;
//    int[] liftPositions = new int[10];
//    liftPositions[0] = 0;
//    liftPositions[1] = 0;
//    liftPositions[2] = 0;
//    liftPositions[3] = 0;
//    liftPositions[4] = 0;
//    liftPositions[5] = 0;
//    liftPositions[6] = 0;
//    liftPositions[7] = 0;
//    liftPositions[8] = 0;
//    liftPositions[9] = 0;
//
//    //these are the servo positions for the end effector - they allow us to change these values everywhere in the code at once
//    double flippedIn = 0.9;
//    double flippedOut = 0.15;
//    double wristWhenIn = 0.1;
//    double wristWhenOut = 0.9;
//    double rotateGrab = 0;
//    double rotateFar = 0;
//    double rotateLeft = 0;
//    double rotateClose = 0;
//    double grabbed = 0.8;
//    double ungrabbed = 0.1;
//
//    int currentLiftPos = 0; //This tells the code what position the lif should be in at the moment
//
//    //these track what our next place position on, to make it so our driver doesn't have to deal with as much
//    int nextLiftPos = 0;
//    int nextPlacePos = 0;
//
//    //--------------------------------------------------------------------------------------------------
////----------------------------------------//
////----------------------------------------//
////---These are all of my Called Methods---//
////----------------------------------------//
////----------------------------------------//
////--------------------------------------------------------------------------------------------------
//    private void turnIMU()
//    {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        robot.init(hardwareMap);
//        imu = hardwareMap.get(BNO055IMU.class,"imu1");
//        imu.initialize(parameters);
//    }
////--------------------------------------------------------------------------------------------------
//    private void driveIMU()
//    {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        robot.init(hardwareMap);
//        imu = hardwareMap.get(BNO055IMU.class,"imu2");
//        imu.initialize(parameters);
//    }
////--------------------------------------------------------------------------------------------------
//    private double angleBoi()
//    {
//        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
//        this.imu.getPosition();
//        currHeading = angles.firstAngle;
//        return currHeading;
//    }
////--------------------------------------------------------------------------------------------------
//    public void driveLiftToPosition()
//    {
//        int currentPos = robot.liftPrimary.getCurrentPosition();
//        int posDifference = currentPos - liftPositions[currentLiftPos];
//        double power = posDifference * 0.003;
//        robot.liftPrimary.setPower(-power);
//        robot.liftSecondary.setPower(power);
//    }
////--------------------------------------------------------------------------------------------------
//    public void setLiftPosition(int position)
//    {
//        if(position >= 0 && position <= 9)
//        {
//            currentLiftPos = position;
//        }
//    }
////--------------------------------------------------------------------------------------------------
//    public void initiateIntakeCycle()
//    {
//        setLiftPosition(1);
//        setFlipPosition(flippedIn);
//        robot.rotate.setPosition(rotateGrab);
//        robot.grabber.setPosition(ungrabbed);
//        robot.wrist.setPosition(wristWhenIn);
//        robot.intake(0.1);
//        while(!hasBlock && !isStopRequested())
//        {
//            normalTeleopStuff();
//        }
//        robot.stopIntake();
//        setLiftPosition(0);
//        sleep(500);
//        robot.grabber.setPosition(grabbed);
//    }
////--------------------------------------------------------------------------------------------------
//    public void normalTeleopStuff()
//    {
//        //--------------------------------------------------------------------------------------------------
//        double frontRight;
//        double frontLeft;
//        double backRight;
//        double backLeft;
//        //This is full Holonomic
//        frontRight = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) + (0.25 * gamepad1.left_stick_x) + gamepad1.right_stick_x;
//        frontLeft  = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) - (0.25 * gamepad1.left_stick_x) - gamepad1.right_stick_x;
//        backRight  = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) - (0.25 * gamepad1.left_stick_x) + gamepad1.right_stick_x;
//        backLeft   = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) + (0.25 * gamepad1.left_stick_x) - gamepad1.right_stick_x;
//        //This is holonomic drive
//        robot.front_right.setPower(frontRight);
//        robot.front_left.setPower(frontLeft);
//        robot.back_right.setPower(backRight);
//        robot.back_left.setPower(backLeft);
//        //--------------------------------------------------------------------------------------------------
//        if(robot.sensorColor.red() > 2 * robot.sensorColor.blue())
//        {
//            hasBlock = true;
//        }
//        else if(robot.sensorColor.red() <= 2 * robot.sensorColor.blue())
//        {
//            hasBlock = false;
//        }
//        //--------------------------------------------------------------------------------------------------
//        //this spits out a block if for some reason we need to
//        if(gamepad1.left_stick_button)
//        {
//            robot.outtake(1);
//            sleep(500);
//        }
//        //--------------------------------------------------------------------------------------------------
//        if(canTogglePlateGrabber && gamepad1.left_bumper)
//        {
//            if(robot.plateGrabber1.getPosition() == 0.73 && canTogglePlateGrabber)
//            {
//                robot.grabPlate();
//            }
//            else
//            {
//                robot.ungrabPlate();
//            }
//            canTogglePlateGrabber = false;
//        }
//        else if(!canTogglePlateGrabber && !gamepad1.left_bumper)
//        {
//            canTogglePlateGrabber = true;
//        }
//        //--------------------------------------------------------------------------------------------------
//        if(canAddToLiftPos && gamepad1.dpad_up && nextLiftPos < 9)
//        {
//            nextLiftPos++;
//            canAddToLiftPos = false;
//        }
//        else if(!canAddToLiftPos && !gamepad1.dpad_up)
//        {
//            canAddToLiftPos = true;
//        }
//        if(canSubtractFromLiftPos && gamepad1.dpad_down && nextLiftPos > 0)
//        {
//            nextLiftPos--;
//            canSubtractFromLiftPos = false;
//        }
//        else if(!canSubtractFromLiftPos && !gamepad1.dpad_down)
//        {
//            canSubtractFromLiftPos = true;
//        }
//        telemetry.addData("Next lift position", nextLiftPos);
//        //--------------------------------------------------------------------------------------------------
//        if(gamepad2.a)
//        {
//            nextPlacePos = 0;
//        }
//        if(gamepad1.x)
//        {
//            nextPlacePos = 1;
//        }
//        if(gamepad1.y)
//        {
//            nextPlacePos = 2;
//        }
//        if(gamepad1.b)
//        {
//            nextPlacePos = 3;
//        }
//        //--------------------------------------------------------------------------------------------------
//        driveLiftToPosition();
//        telemetry.update();
//
//    }
////--------------------------------------------------------------------------------------------------
//    public void setFlipPosition(double position)
//    {
//        robot.flip1.setPosition(position);
//        robot.flip2.setPosition(position);
//    }
////--------------------------------------------------------------------------------------------------
//    public void goToNextPosition()
//    {
//        double start = System.currentTimeMillis();
//        setLiftPosition(nextLiftPos);
//        while((System.currentTimeMillis() - start) < 1000 && !isStopRequested())
//        {
//            normalTeleopStuff();
//        }
//        setFlipPosition(flippedOut);
//        robot.wrist.setPosition(flippedOut);
//        if(nextPlacePos == 0)
//        {
//            robot.rotate.setPosition(rotateClose);
//        }
//        else if(nextPlacePos == 1)
//        {
//            robot.rotate.setPosition(rotateLeft);
//        }
//        else if(nextPlacePos == 2)
//        {
//            robot.rotate.setPosition(rotateFar);
//        }
//        else
//        {
//            robot.rotate.setPosition(rotateGrab);
//        }
//    }
////--------------------------------------------------------------------------------------------------
//    public void place()
//    {
//        setLiftPosition(currentLiftPos - 1);
//        double start = System.currentTimeMillis();
//        setLiftPosition(nextLiftPos);
//        while((System.currentTimeMillis() - start) < 1000 && !isStopRequested())
//        {
//            normalTeleopStuff();
//        }
//        robot.grabber.setPosition(ungrabbed);
//        while((System.currentTimeMillis() - start) < 200 && !isStopRequested())
//        {
//            normalTeleopStuff();
//        }
//        setFlipPosition(flippedIn);
//        robot.rotate.setPosition(rotateGrab);
//        robot.wrist.setPosition(wristWhenIn);
//        sleep(300);
//        setLiftPosition(0);
//        driveLiftToPosition();
//    }
////--------------------------------------------------------------------------------------------------
////--------------------------------------------------------------------------------------------------
////----------------------------------------//
////----------------------------------------//
////---No More Methods Are Made Past This---//
////----------------------------------------//
////----------------------------------------//
////--------------------------------------------------------------------------------------------------
//
////--------------------------------------------------------------------------------------------------
////------------------------------------------------------//
////------------------------------------------------------//
////---Here is my Actual Run Op where I call my methods---//
////------------------------------------------------------//
////------------------------------------------------------//
////--------------------------------------------------------------------------------------------------
//    public void runOpMode()
//    {
//        turnIMU();
//        driveIMU();
//        waitForStart();
//        robot.sensorColor.enableLed(true);
////--------------------------------------------------------------------------------------------------
//        while(opModeIsActive() && (!(isStopRequested())))
//        {
//
//
//            double frontRight;
//            double frontLeft;
//            double backRight;
//            double backLeft;
////--------------------------------------------------------------------------------------------------
//            //This is full Holonomic
//            frontRight = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) + (0.25 * gamepad1.left_stick_x) + gamepad1.right_stick_x;
//            frontLeft  = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) - (0.25 * gamepad1.left_stick_x) - gamepad1.right_stick_x;
//            backRight  = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) - (0.25 * gamepad1.left_stick_x) + gamepad1.right_stick_x;
//            backLeft   = gamepad1.right_stick_y + (0.25 * gamepad1.left_stick_y) + (0.25 * gamepad1.left_stick_x) - gamepad1.right_stick_x;
//            //This is holonomic drive
//            robot.front_right.setPower(frontRight);
//            robot.front_left.setPower(frontLeft);
//            robot.back_right.setPower(backRight);
//            robot.back_left.setPower(backLeft);
////--------------------------------------------------------------------------------------------------
//            if(robot.sensorColor.red() > 2 * robot.sensorColor.blue())
//            {
//                hasBlock = true;
//            }
//            else if(robot.sensorColor.red() <= 2 * robot.sensorColor.blue())
//            {
//                hasBlock = false;
//            }
//            telemetry.addData("Has Block is  ", hasBlock);
//
//            if(hasBlock)
//            {
//                robot.stopIntake();
//            }
//            else
//            {
//                robot.intake(0.1);
//            }
//
//
////--------------------------------------------------------------------------------------------------
//            //this spits out a block if for some reason we need to
//            if(gamepad1.left_stick_button)
//            {
//                robot.outtake(1);
//                sleep(500);
//            }
////--------------------------------------------------------------------------------------------------
//            robot.liftPrimary.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//            robot.liftSecondary.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
////--------------------------------------------------------------------------------------------------
//            if(gamepad1.a)
//            {
//                armHeight = armHeight + 1;//This might sprout a problem
//            }
//            else if(gamepad1.b)
//            {
//                armHeight = armHeight - 1;
//            }
////-------------------
//
////--------------------------------------------------------------------------------------------------
//            if(canToggleGrabber && gamepad1.right_bumper)
//            {
//                if(robot.grabber.getPosition() == 0.1 && canToggleGrabber)
//                {
//                    robot.grabber.setPosition(0.80);
//                }
//                else
//                {
//                    robot.grabber.setPosition(0.1);
//                }
//                canToggleGrabber = false;
//            }
//            else if(!canToggleGrabber && !gamepad1.right_bumper) {
//                canToggleGrabber = true;
//            }
////--------------------------------------------------------------------------------------------------
//            if(canTogglePlateGrabber && gamepad1.left_bumper)
//            {
//                if(robot.plateGrabber1.getPosition() == 0.73 && canTogglePlateGrabber)
//                {
//                    robot.grabPlate();
//                }
//                else
//                {
//                    robot.ungrabPlate();
//                }
//                canTogglePlateGrabber = false;
//            }
//            else if(!canTogglePlateGrabber && !gamepad1.left_bumper)
//            {
//                canTogglePlateGrabber = true;
//            }
////--------------------------------------------------------------------------------------------------
//
////--------------------------------------------------------------------------------------------------
//            if(gamepad1.dpad_up)
//            {
//                robot.flip1.setPosition(0.15);
//                robot.flip2.setPosition(0.15);
//                robot.wrist.setPosition(0.9);
//                robot.rotate.setPosition(0.666);
//            }
//            if(gamepad1.dpad_down)
//            {
//                robot.flip1.setPosition(0.15);
//                robot.flip2.setPosition(0.15);
//                robot.wrist.setPosition(0.9);
//                robot.rotate.setPosition(0);
//            }
//            if(gamepad1.dpad_left)
//            {
//                robot.flip1.setPosition(0.15);
//                robot.flip2.setPosition(0.15);
//                robot.wrist.setPosition(0.9);
//                robot.rotate.setPosition(1);
//            }
//            if(gamepad1.dpad_right)
//            {
//                robot.flip1.setPosition(0.15);
//                robot.flip2.setPosition(0.15);
//                robot.wrist.setPosition(0.9);
//                robot.rotate.setPosition(0.3333);
//            }
////--------------------------------------------------------------------------------------------------
//            telemetry.update();
//        }
//    }
//}
////--------------------------------------------------------------------------------------------------
////-------------------------------------------//
////-------------------------------------------//
////---There is No More Code Past This Point---//
////-------------------------------------------//
////-------------------------------------------//
////--------------------------------------------------------------------------------------------------