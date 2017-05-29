package org.usfirst.frc.team5123.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class RobotD extends IterativeRobot {
	
	/**
	 * Declarations are made here
	 */
	
	RobotDrive drive = new RobotDrive(0, 1, 2, 3);
	Spark climber = new Spark(4);
	
	/*
	 *  The 4 motors controllers used for driving use the first 4 PWM Ports, 0 - 3
	 *  0 and 1 are for the left drive motors, while 2 and 3 are for the right drive motors
	 *  PWM port 4 is used by the motor controller for the climber motor
	 */
	
	Joystick joystick1 = new Joystick(0);
	Joystick joystick2 = new Joystick(1);
	
	/*
	 * joystick1 is designated to the first USB slot in the DS, joystick2 is the second 
	 * joystick1 is the joystick used to drive the robot, joystick2 controls the climber
	 */

	CameraServer cam0 = CameraServer.getInstance();
	
	// cam0 is any USB camera plugged into the first USB port on the RIO
	
	GyroBase gyro = new ADXRS450_Gyro();
	
	// gyro plugs directly into the RIO
	
	Timer timer = new Timer(); 
	
	// timer is used in auto for dead reckoning with t=s/v
	
	double yInput, xInput, climberAxis, s1, v1, t1, angle;
	
	/*
	 * variables
	 * s v and t have a 1 to represent that they are the variables for a singular, linear portion of the autonomous routine
	 * If more sections were added, 2, 3, etc would be used to identify the variables for each part
	 */
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	
	@Override
	public void robotInit() {		
		
		CameraServer.getInstance().startAutomaticCapture(0);
		
		// sets up camera to send feed to the Dashboard
		
		yInput = 0;
		xInput = 0;
		climberAxis = 0;
		
		// clears any leftover values from last time robot was turned on
		
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
	
		s1 = 100;
		
		// distance robot must travel in cm
		
		v1 = 20;
		
		/* 
		 * velocity that robot moves at in cm/sec
		 * found by measuring the distance the robot travels in 5 seconds at the determined speed and calculating distance/time
		 */
		
		timer.reset();
		timer.start();
		
		// clears and restarts timer	
		
		gyro.reset();
		
		//clears gyro and sets whatever the current orientation is to 0
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
			angle = gyro.getAngle();
		
			// variable used to easily access angle
			
			t1 = s1/v1;
			
			/*
			 * determines amount of time robot will have to travel to go the distance desired at the determined rate
			 * disregards full SUVAT equation because of a lack of necessity of precision
			 */
			
			if(timer.get() < t1){
				
				// if the amount of time passed does not equal the amount of time needed to go full distance, drive
				
				drive.drive(0.25, -angle);
				
				/*
				 *  drive at 25% speed
				 *  Turns in the opposite direction that it is drifting in, keeping it straight
				 */
				
			}
			else {
				
				drive.drive(0, 0);
			
				// when robot has driven long enough to go the distance, stop
				
			}
		
			SmartDashboard.putNumber("Distance Traveled", (timer.get()/v1));
			
			// projected distance that should have been traveled at any moment
			
			SmartDashboard.putNumber("Angle", angle);
			
			// displays angle
		}

	
	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		
	
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		yInput = joystick1.getRawAxis(1);
		xInput = joystick1.getRawAxis(0);
		
		/*
		 * yInput takes the y axis of the joystick and xInput takes the x axis
		 */
		
		drive.arcadeDrive(0.95 * yInput, -0.7 * xInput);
		
		/* 
		 * yInput determines the speed at which both sides move forward or back (magnitude)
		 * cap is set to 95%
		 * xInput determines the speed at which one side moves compared to the other going left or right (rotation)
		 * cap is set to 70%, and it is negative to invert the direction of rotation
		 */
		
		climberAxis = joystick2.getRawAxis(1);
		
		// takes y axis of the second joystick
		
		climber.set(limit(climberAxis));
		
		// climberAxis sets the speed of the climber to the output of limit() with climberAxis being the input
		
		SmartDashboard.putNumber("Magnitude", yInput);
		SmartDashboard.putNumber("Rotation", xInput);
		SmartDashboard.putNumber("Climber", climberAxis);
		
		/*
		 * Displays numerical values on Dashboard for each axis
		 */
		
		
		
	}
		
	/**
	 * This function is called periodically during test mode
	 */
	
	protected double limit(double climbPower){
		
		if (climbPower < 0){
			
			return 0;
			
			//if the input is negative, don't move climber
		
		}
		if (climbPower > 0){
			
			return 0.70 * climbPower;
		
			/*
			 * if input is positive, allow climber to move
			 * cap is 70%
			 */
			
			
		}
		
		else return 0;	
		
		// purpose of function is to prevent the motor from turning the climber backwards and breaking the mechanism
		
	}
	
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
}


