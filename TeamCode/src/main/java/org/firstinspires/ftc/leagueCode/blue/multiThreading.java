package org.firstinspires.ftc.leagueCode.blue;
import org.firstinspires.ftc.leagueCode.misc.leagueMap;

public class multiThreading extends Thread
{
	leagueMap robot = new leagueMap();
	public void run()
	{
		try
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
	
	// Main Class
	public class Multithread
	{
		public void main(String[] args)
		{
			int n = 8;// Number of threads
			for (int i = 0; i < 8; i++)
			{
				multiThreading object = new multiThreading();
				object.start();
			}
		}
	}
}
//--------------------------------------------------------------------------------------------------