package org.usfirst.frc5588.Feodora.subsystems;

import org.usfirst.frc5588.Feodora.RobotMap;
import org.usfirst.frc5588.Feodora.commands.ClimberCommand;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
	private final SpeedController climber = RobotMap.climberMotor;
	public void setSpeed(double speed){
		climber.set(speed);
	}
	public void initDefaultCommand() {
		setDefaultCommand(new ClimberCommand());
	}
	
	private static Climber instance = new Climber();
	
	public static Climber getInstance()
	{
		return instance;
	}

}
