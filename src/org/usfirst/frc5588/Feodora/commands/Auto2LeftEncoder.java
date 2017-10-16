package org.usfirst.frc5588.Feodora.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Auto2LeftEncoder extends CommandGroup {

    public Auto2LeftEncoder() {
    	 addSequential(new DriveForRotations(0.25, 80.5));
         addSequential(new DriveForTurning(0.25, 0.25, 14.85, "left")); //TURNS RIGHT
         addSequential(new DriveForRotations(0.15, 29.0)); 
         
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
