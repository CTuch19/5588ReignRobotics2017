package org.usfirst.frc5588.Feodora.commands;

import org.usfirst.frc5588.Feodora.commands.ToggleCommand.TogglableCommand;

import edu.wpi.first.wpilibj.command.Command;

public class SetVisionCommand extends Command implements TogglableCommand{	
	private final Vision.Mode mode;
	private final boolean waitUntilSwitched;
	private boolean successfullySwitched;
	
	private final Vision vision;
	public SetVisionCommand(Vision.Mode mode){
		this(mode, false);
	}
	
	public SetVisionCommand(Vision.Mode mode, boolean waitUntilSwitched){
		this.mode = mode;
		this.waitUntilSwitched = waitUntilSwitched;
		
		this.vision = Vision.getInstance();
		requires(vision);
	}
	
	@Override
	protected void initialize(){
		vision.setMode(mode);
	}
	
	@Override
	protected void execute(){
		if(!successfullySwitched){
			successfullySwitched = vision.getMode() == mode;
		}
	}
	
	@Override
	protected boolean isFinished() {
		return !waitUntilSwitched || (successfullySwitched && vision.getTargetData().isValid());
	}

	@Override
	public boolean isToggled() {
		return vision.getMode() == mode;
	}

	@Override
	public Command toCommand() {
		return this;
	}

}
