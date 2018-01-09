package org.usfirst.frc5588.Feodora.commands;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc5588.Feodora.Robot;
import org.usfirst.frc5588.Feodora.subsystems.Drive;
/**
 *
 */
public class DriveCommand extends Command {

    public DriveCommand() {

        requires(Robot.drive);
        requires(Drive.getInstance());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//"Squared" driving means that the read from the joystick is squared, making it go really fast, but have fine movements when it is slow
    	double fSpeed;
    	double readfSpeed = deadzone(Robot.oi.driverJoystick.getRawAxis(0));
    	//if the joystick is moving backwards, then the read needs to be squared and * by -1
    	if(readfSpeed >= 0)
    		fSpeed = readfSpeed * readfSpeed;
    	else
    		fSpeed = -1 * (readfSpeed * readfSpeed);
    	
    	
    	double sSpeed = -deadzone(Robot.oi.driverJoystick.getRawAxis(1));
    	double lSpeed = (fSpeed + sSpeed);
    	double rSpeed = (fSpeed - sSpeed);
    	Robot.drive.setSpeed(lSpeed, rSpeed);
    	//System.out.println("The left speed is " + lSpeed);
    	//System.out.println("The right speed is " + rSpeed);
    }
    //if the joystick is close enough to 0, the deadzone will make the robot stop moving
    private double deadzone(double in){
    	if(Math.abs(in)<0.1)
    		return 0.0;
    	return in;
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}