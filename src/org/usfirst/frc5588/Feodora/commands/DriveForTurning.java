package org.usfirst.frc5588.Feodora.commands;

import org.usfirst.frc5588.Feodora.RobotMap;
import org.usfirst.frc5588.Feodora.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForTurning extends Command {

	private double leftSpeed;
	private double rightSpeed;
	private double distance;
	private String leftOrRight;
	private double leftDistanceTraveled;
	private double rightDistanceTraveled;
	
    public DriveForTurning(double lSpeed, double rSpeed, double distance, String side) {
        
    	requires(Drive.getInstance());
        
        this.leftSpeed = lSpeed;
        this.rightSpeed = rSpeed;
        this.distance = distance;
        this.leftOrRight = side;
     // distance per pulse = PI * wheel diameter in inches / pulse per revolution * fudge factor;
        final double distancePerPulse = (Math.PI * 5.85)/ 265 * 1; 
    	RobotMap.rightEncoder.setDistancePerPulse(distancePerPulse);
    	RobotMap.leftEncoder.setDistancePerPulse(distancePerPulse);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.leftEncoder.reset();
    	RobotMap.rightEncoder.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//double correctingRight = 0.0;
    	//double correctingLeft = 0.0;
    	
    	leftDistanceTraveled = -1.0 * RobotMap.leftEncoder.getDistance();
    	rightDistanceTraveled = RobotMap.rightEncoder.getDistance();
    	
    	//if(leftDistanceTraveled > rightDistanceTraveled)
    		//correctingRight = -0.2;
    	
    	//if(rightDistanceTraveled > leftDistanceTraveled)
    		//correctingLeft = 0.2;
    	
    	//if(leftDistanceTraveled - rightDistanceTraveled > 0.05)
    		//correctingRight = -0.15;
    	
    	//if(rightDistanceTraveled - leftDistanceTraveled > 0.05)
    		//correctingLeft = 0.10;
    	
    	Drive.getInstance().setSpeed(this.leftSpeed, this.rightSpeed); 
    	
    	//Use this to TEST if your encoder correcting is working properly
    	System.out.println("The right encoder is at " + rightDistanceTraveled);
    	System.out.println("The left encoder is at " + leftDistanceTraveled);
    	System.out.println("The encoder counts is " + RobotMap.rightEncoder.get());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	 if(leftOrRight.equals("right"))
    		 return (rightDistanceTraveled>= distance ); //and left distance - need to use math to calculate
    	 if(leftOrRight.equals("left"))
    		 return (leftDistanceTraveled >= distance);
    	 return (leftDistanceTraveled>= distance);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Drive.getInstance().stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
