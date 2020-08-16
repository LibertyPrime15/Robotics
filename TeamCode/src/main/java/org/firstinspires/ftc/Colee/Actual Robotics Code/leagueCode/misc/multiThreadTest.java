package org.firstinspires.ftc.Colee.leagueCode.misc;

public class multiThreadTest extends Thread
{
	public double xPos = 0;
	public double yPos = 0;
	public double angle = 0;
	private double previousDistTravelled = 0;
	private double distTravelled;
	private blueBlockMultiThreadTest opMode = null;
	
	//--------------------------------------------------------------------------------------------------
	public void passInOpMode(blueBlockMultiThreadTest opMode)
	{
		//This comment line keeps this from reformatting
		this.opMode = opMode;
	}
//--------------------------------------------------------------------------------------------------
	public void updatePosition()
	{
		//This comment line keeps this from reformatting
		angle = opMode.returnAngle();
		distTravelled = opMode.returnEncoderValues();
		yPos += Math.sin(angle)*(distTravelled - previousDistTravelled);
		xPos += Math.cos(angle)*(distTravelled = previousDistTravelled);
		previousDistTravelled = distTravelled;
		opMode.passPosition(xPos, yPos);
	}
	//--------------------------------------------------------------------------------------------------
	public void run()
	{
		try
		{
			while(true)
			{
				updatePosition();
				
			}
		}
		catch(Exception e)
		{
			// Throwing an exception
		}
		this.stop();
	}
}
//--------------------------------------------------------------------------------------------------