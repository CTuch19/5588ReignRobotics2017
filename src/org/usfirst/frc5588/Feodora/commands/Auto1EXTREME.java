package org.usfirst.frc5588.Feodora.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Auto1EXTREME extends CommandGroup {

    public Auto1EXTREME() {
    	
    	addSequential(new DriveForRotations(0.4, 50.0));
        addSequential(new DriveForRotations(0.1, 30.0));
        System.out.println("Stoping for 5 seconds");
        addSequential(new StopDontMOVE(5.0));
        addSequential(new DrivingBackwards(-0.4, 20.0));
        addSequential(new DriveForTime(0.25, 0.25, 0.6));
        addSequential(new DriveForRotations(0.5, 64.0));
        addSequential(new DriveForTime(-0.25, -0.25, 0.64));
        addSequential(new DriveForRotations(0.60, 58.0));
        
        
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
