package org.firstinspires.ftc.leagueCode.blue;
import org.firstinspires.ftc.leagueCode.misc.leagueMap;

public class multiThreading extends Thread
{
	leagueMap robot = new leagueMap();
	//This is what runs \/ \/ \/
	public void run()
	{
		try//The code we actually want to run goes right here \/ \/ \/ \/
		{
			// Displaying the thread that is running
			System.out.println("Thread " + Thread.currentThread().getId() + " is running");
		}
		catch(Exception e)
		{
			// Throwing an exception
			System.out.println("Exception is caught");
		}
	}
}
//--------------------------------------------------------------------------------------------------