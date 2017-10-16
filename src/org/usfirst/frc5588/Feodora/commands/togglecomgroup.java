package org.usfirst.frc5588.Feodora.commands;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc5588.Feodora.commands.ToggleCommand.TogglableCommand;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class togglecomgroup extends CommandGroup implements TogglableCommand {
	private List<TogglableCommand> togglableCommands = new ArrayList<TogglableCommand>();
	
	public void addTogglableSequential(TogglableCommand command, double timeout){
		togglableCommands.add(command);
		addSequential(command.toCommand(), timeout);
	}
	
	public void addTogglableSequential(TogglableCommand command){
		togglableCommands.add(command);
		addSequential(command.toCommand());
	}
	
	public void addTogglableParallel(TogglableCommand command, double timeout){
		togglableCommands.add(command);
		addParallel(command.toCommand(), timeout);
		
	}
	
	public void addTogglableParallel(TogglableCommand command){
		togglableCommands.add(command);
		addParallel(command.toCommand());
		
	}
	
	@Override
	public boolean isToggled() {
		boolean toggled = false;
		for(TogglableCommand command : togglableCommands){
			toggled = command.isToggled() || toggled;
		}
		return toggled;
	}

	@Override
	public Command toCommand() {
		return this;
	}

}
