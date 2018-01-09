package org.usfirst.frc5588.Feodora.commands;

import org.usfirst.frc5588.Feodora.subsystems.Drive;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightGyro extends Command {
	
	AnalogGyro gyro;
	private double initialAngle;
	private double distance;
	private double leftDistanceTraveled;
	private double rightDistanceTraveled;
	private double leftSpeed;
	private double rightSpeed;
	
    public DriveStraightGyro(double oneSpeed, double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Drive.getInstance());
    	 //gets heading
    	
    	this.leftSpeed = oneSpeed;
        this.rightSpeed = -1 * oneSpeed;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	gyro.reset();
    	gyro.calibrate();
    	initialAngle = gyro.getAngle(); 
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double correctingRight = 0.0;
    	double correctingLeft = 0.0;
    	if( initialAngle - gyro.getAngle() > 0.05)
    		correctingRight = -0.15;
    	if(initialAngle - gyro.getAngle() < 0.0)
    		correctingLeft = 0.10;
    	//idk if this will work i'm gonna check later
    	//if(gyro.getAngle() > initialAngle + .2)
    	
    		Drive.getInstance().setSpeed(this.leftSpeed + 0.1 , this.rightSpeed);
    		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
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
