package org.usfirst.frc5588.Feodora.commands;
import org.usfirst.frc5588.Feodora.subsystems.Drive;
import edu.wpi.first.wpilibj.command.Command;


public class DriveForTime extends Command {
		
	private double leftSpeed;
	private double rightSpeed;
    public DriveForTime(double lSpeed, double rSpeed, double time) 
    {
    	
        super(time);
        requires(Drive.getInstance());
        this.leftSpeed = lSpeed;
        this.rightSpeed = rSpeed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Drive.getInstance().setSpeed(this.leftSpeed, this.rightSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
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
