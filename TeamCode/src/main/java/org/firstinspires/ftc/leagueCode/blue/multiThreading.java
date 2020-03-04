package org.firstinspires.ftc.leagueCode.blue;
public class multiThreading extends Thread
{
	public int cycles1 = 0;
	public int cycles2 = 0;
	private blueLeagueBlock opMode = null;

//--------------------------------------------------------------------------------------------------
	public void passInOpMode(blueLeagueBlock opMode)
	{
		this.opMode = opMode;
	}
//--------------------------------------------------------------------------------------------------
	public void run()
	{
		try
		{
			while(true)
			{
				cycles1++;
				cycles2--;
				opMode.passPosition(cycles1, cycles2);
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