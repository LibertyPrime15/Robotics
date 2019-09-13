/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.text.method.MovementMethod;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "BlueDepotAuto", group = "Concept")
@Disabled
public class BlueDepotAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = " AT0ySZn/////AAABGVDoF4gdNkZugx61WeftYqVhwz6Leeu2a1cWYQR+08xsATI6GQf3vvrvynP8JpemukCajxoFg32bkspzJx8g6uNBgHlQsFPxmFMJ8b4V/fDFTRSpy+vMOzIMoV2CHuitvtyrn/a6AsPUWczm5rsTqcCzAUEL6YD0xrzXkvaNJBzm3Jq5BkUW2ualta+LldpZ0ho/rdkDuyp6xOjsSvAbsjIDkQt807jlfYLBJAsaJNqRnQUU4mZMzl5aJsr+VnUbTfeev943zeK34ENEQzW1jCeCnLTsuWGmOd7QP+gF1YhxUr0A0s5Tr6+v8QntHPwYHp7QRpRErFqkst/OxbA4omIuLTsC41FFwYOr5YxDHhyl";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private ModernRoboticsI2cGyro gyro;

    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;

    private boolean Left = false, Right = false, Center = false;

    double distPerRot = (Math.PI * 3.8125);
    double stepsPerRot = 1120;
    double lengthOfField = 12 * 12;

    public void Stop() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void Forward(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);
    }

    public void Backward(double power){
        FrontLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackLeft.setPower(-power);
        BackRight.setPower(-power);
    }

    public void RTurn(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }

    public void LTurn(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }

    double prevEncoder = 0;

    public void moveDistance(double length) {

        double distPerRot = Math.PI * 3.8125;
        double stepsPerRot= 1120;

        double totDistInSteps = ((length / distPerRot) * stepsPerRot);

        if(totDistInSteps > 0) {
            while (totDistInSteps >= FrontLeft.getCurrentPosition()) {

                telemetry.addData("Position", FrontLeft.getCurrentPosition());
                Forward(.5);

            }
        } else if(length < 0){
            while(totDistInSteps < FrontLeft.getCurrentPosition()) {
                telemetry.addData("Position", FrontLeft.getCurrentPosition());
                Backward(.5);
            }
        }
        Stop();
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()){
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            FrontLeft = hardwareMap.get(DcMotor.class, "FL");
            FrontRight = hardwareMap.get(DcMotor.class, "FR");
            BackLeft = hardwareMap.get(DcMotor.class, "BL");
            BackRight = hardwareMap.get(DcMotor.class, "BR");

            gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

            FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            gyro.calibrate();

            waitForStart();

            while (opModeIsActive()) {

                if(Right) {

                    //Turn to face the cube
                    while(gyro.getHeading() != 146)
                    {
                        telemetry.addData("heading", gyro.getHeading());

                        LTurn(.2);

                        if(gyro.getHeading() >= 147)
                        {
                            telemetry.addData("heading", gyro.getHeading());
                            break;
                        }
                    }

                    //Sample the cube on the right side
                    moveDistance(-40);

                    //turn left to face the crater
                    while(gyro.getHeading() != 36)
                    {
                        telemetry.addData("heading", gyro.getHeading());

                        LTurn(.2);

                        if(gyro.getHeading() >= 37)
                        {
                            telemetry.addData("heading", gyro.getHeading());
                            break;
                        }
                    }

                    //The robot will drive into the depot to drop off the team marker
                    moveDistance(-20);

                    //Drive into the red crater
                    moveDistance(-144);
                    Stop();
//----------------------------------------
                    //Ends the Program and updates the encoders
                    telemetry.addData("FrontLeftEncoderValue: ", FrontLeft.getCurrentPosition());
                    telemetry.update();
                    stop();

                }else if(Left) {

                    //turn to face the cube for sampeling
                    while(gyro.getHeading() != 34)
                    {
                        telemetry.addData("heading", gyro.getHeading());

                        LTurn(.2);

                        if(gyro.getHeading() >= 35)
                        {
                            telemetry.addData("heading", gyro.getHeading());
                            break;
                        }
                    }

                    //drive into the cube and drive the ball into the crater
                    moveDistance(30);

                    //turn the robot to face the crater
                    while(gyro.getHeading() != 146)
                    {
                        telemetry.addData("heading", gyro.getHeading());

                        LTurn(.2);

                        if(gyro.getHeading() >= 147)
                        {
                            telemetry.addData("heading", gyro.getHeading());
                            break;
                        }
                    }

                    //Drive into the depot and drop off the team marker
                    moveDistance(-20);

                    //Drive into Red Side Crater
                    moveDistance(144);
                    Stop();
//------------------------------
                    //Ends the Program and updates the encoders
                    telemetry.addData("FrontLeftEncoderValue: ", FrontLeft.getCurrentPosition());
                    telemetry.update();
                    stop();

                }else if(Center){

                    //----------------------------------------
                    //Move forward from origin into the sampling box
                    moveDistance(62);
                    sleep(200);

                    //Back up so the block is not stuck to the bot
                    moveDistance(-1);
                    sleep(300);

                    //Turn to face the red Crater after sampling
                    while(gyro.getHeading() != 135)
                    {
                        telemetry.addData("heading", gyro.getHeading());
                        LTurn(.2);

                        if(gyro.getHeading() >= 136 && gyro.getHeading() < 300)
                        {
                            telemetry.addData("heading", gyro.getHeading());
                            break;
                        }
                    }

                    //Got crater on the red side
                    moveDistance(144);
                    Stop();
//----------------------------------------
                    //Ends the Program and updates the encoders
                    telemetry.addData("FrontLeftEncoderValue: ", FrontLeft.getCurrentPosition());
                    telemetry.update();
                    stop();

                }else{

                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 1) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for(int i = 0; i < 50; i++){
                                    for (Recognition recognition : updatedRecognitions) {
                                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                            goldMineralX = (int) recognition.getLeft();
    //                                    } else if (silverMineral1X == -1) {
    //                                        silverMineral1X = (int) recognition.getLeft();
    //                                    } else {
    //                                        silverMineral2X = (int) recognition.getLeft();
                                          }
                                            telemetry.addData("i", i);
                                            telemetry.addData("getLeft", goldMineralX);
                                            telemetry.update();



                                    }
                                }
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                    telemetry.addData("getLeft", goldMineralX);
                                    telemetry.update();
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Left");
                                        Right = false;
                                        Center = false;
                                        Left = true;
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Right");
                                        Left = false;
                                        Center = false;
                                        Right = true;
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                        Left = false;
                                        Right = false;
                                        Center = true;
                                    }
                                }
                            }

                            telemetry.update();
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
