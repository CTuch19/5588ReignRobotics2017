package org.usfirst.frc5588.Feodora.subsystems;

import org.usfirst.frc5588.Feodora.RobotMap;
import org.usfirst.frc5588.Feodora.commands.*;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Drive extends Subsystem {

    private final SpeedController leftDrive = RobotMap.driveLeftDrive;
    private final SpeedController rightDrive = RobotMap.driveRightDrive;

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void setSpeed(double lSpeed,double rSpeed){
    	leftDrive.set(lSpeed);
    	rightDrive.set(rSpeed);
    }
    public void initDefaultCommand() {
       
        setDefaultCommand(new DriveCommand());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    
    //Below is very important for autonomous to work properly
    private static Drive instance = new Drive();
    
    public static Drive getInstance()
    {
    	return instance;
    }
    
   
    //Stop the drive from moving
    public void stop()
    {
    	leftDrive.set(0);
    	rightDrive.set(0);
    }
}
