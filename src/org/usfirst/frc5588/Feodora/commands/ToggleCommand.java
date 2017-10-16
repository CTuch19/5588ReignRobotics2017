package org.usfirst.frc5588.Feodora.commands;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class ToggleCommand extends Command {
	public static final Logger logger = Logger.getLogger(ToggleCommand.class.getSimpleName());
	
	public interface TogglableCommand{
		boolean isToggled();
		Command toCommand();
	}
	
	private final TogglableCommand[] commands;
	private int toggleCounter;
	public ToggleCommand(TogglableCommand...commands) {
		if(commands.length <= 0){
			logger.log(Level.WARNING, "Must have commands for toggle to work.");
		}
		else if(commands.length == 1){
			TogglableCommand[] tempCommands = new TogglableCommand[2];
			tempCommands[0] = commands[0];
			tempCommands[1] = null;
			commands = tempCommands;
		}
		
		this.commands = commands;
		toggleCounter = 0;
	}
	
	@Override
	protected synchronized void initialize(){
		if(commands[toggleCounter] == null)
			return;
		
		if(commands[toggleCounter].isToggled()){
			logger.log(Level.INFO, "Changing state.");
			
			if(commands[toggleCounter].toCommand().isRunning()){
				logger.log(Level.INFO, "Canceling previous command.");
				
				commands[toggleCounter].toCommand().cancel();
			}
			
			logger.log(Level.INFO, "Starting new command.");
			next();
		}
		
		Scheduler.getInstance().add(commands[toggleCounter].toCommand());
	}
	
	private synchronized void next(){
		if(toggleCounter + 1 < commands.length){
			toggleCounter++;
		}
		else{
			toggleCounter = 0;
		}
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}

}
