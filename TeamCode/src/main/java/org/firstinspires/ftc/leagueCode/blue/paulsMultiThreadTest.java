package org.firstinspires.ftc.leagueCode.blue;
public class paulsMultiThreadTest extends Thread
{
	public double xPos = 0;
	public double yPos = 0;
	public double angle = 0;
	private blueLeagueBlock opMode = null;
	
	//--------------------------------------------------------------------------------------------------
	public void passInOpMode(blueLeagueBlock opMode)
	{
		this.opMode = opMode;
	}
//--------------------------------------------------------------------------------------------------
	
	public void updatePosition()
	{
		angle = opMode.returnAngle();
	}
	//--------------------------------------------------------------------------------------------------
	public void run()
	{
		try
		{
			while(true)
			{
			
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