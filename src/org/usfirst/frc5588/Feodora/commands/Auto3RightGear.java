package org.usfirst.frc5588.Feodora.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Auto3RightGear extends CommandGroup {

    public Auto3RightGear() {
    	
    	 addSequential(new DriveForRotations(0.25, 78));
         addSequential(new DriveForTime(-0.25, -0.25, 0.72));
         addSequential(new DriveForRotations(0.15, 31));
     	
       
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