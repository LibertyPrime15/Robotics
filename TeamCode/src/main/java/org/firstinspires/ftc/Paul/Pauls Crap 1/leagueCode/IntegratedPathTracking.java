package org.firstinspires.ftc.leagueCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.lang.Thread;

public abstract class IntegratedPathTracking implements Runnable {

    LinearOpMode referenceOpMode;
    BNO055IMU IMU;
    ArrayList<DcMotor> driveTrain;
    Thread pathTracking;
    Orientation angles;


    private double xPos;
    private double yPos;
    private double currDistSteps;
    private double lastDistSteps;


    public IntegratedPathTracking(LinearOpMode opmode, ArrayList<DcMotor> driveTrain, BNO055IMU IMU)
    {
        this.xPos = 0;
        this.yPos = 0;
        this.currDistSteps = 0;
        this.referenceOpMode = opmode;
        this.IMU = IMU;
        this.driveTrain = driveTrain;
        pathTracking = new Thread(this);

    }

    public void run()
    {
        while(!referenceOpMode.isStopRequested())
        {
            this.updatePosition();
        }
    }

    public void updatePosition()
    {
        double currAngle;
        angles = this.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.IMU.getPosition();
        currAngle = angles.firstAngle;
        double numSteps = 0;
        double numWheels = 0;
        lastDistSteps = currDistSteps;
        for(DcMotor x : driveTrain)
        {
            double currWheelSteps = x.getCurrentPosition();
            numWheels++;
            numSteps += currWheelSteps;
        }
        currDistSteps = numSteps / numWheels;
        double distTravelled = currDistSteps - lastDistSteps;

        yPos += Math.cos(currAngle)* distTravelled;
        xPos += Math.sin(currAngle) * distTravelled;
    }

    public double getDistanceToTarget(double xTarget, double yTarget)
    {
        double deltaX = xTarget - xPos;
        double deltaY = yTarget - yPos;
        return Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
    }

    //This is gonna need to b a lot more complicated because that;'s not how math works - tangent does not differentate between positive and negtive, and this won't either
    public double getDirectionToTarget(double xTarget, double yTarget)
    {
        double deltaX = xTarget - xPos;
        double deltaY = yTarget - yPos;
        return Math.atan(deltaX/deltaY);
    }

    public void driveToTargetPosition(double xTarget, double yTarget)
    {

    }




}
