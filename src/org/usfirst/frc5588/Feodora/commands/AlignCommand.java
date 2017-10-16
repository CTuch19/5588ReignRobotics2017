package org.usfirst.frc5588.Feodora.commands;

import org.usfirst.frc5588.Feodora.subsystems.Drive;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AlignCommand extends Command {
	private VisionCom vision = VisionCom.getInstance();
		private Drive drive = Drive.getInstance();
		private final float stopThreshold = 0.9f;
		private final float stopYawSpeed = 1.0f;
		private float speed = 0.4f;
		private final TargetTracker tracker;
		private final boolean invert;
		public AlignCommand(TargetTracker tracker, boolean invert) {
			super("Align", 2.0f);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(drive);
		this.tracker = tracker;
		this.invert = invert;
			requires(drive);
		   }


		// Called repeatedly when this Command is scheduled to run
		protected void execute() {
		float error = tracker.getDeltaYaw();
		if (error > stopThreshold) {
		//System.out.println("Turn left " + error);
		drive.setSpeed((-speed * Math.pow(error * 0.05f, 2) - 0.3f) * (invert ? -1 : 1),(speed * Math.pow(error * 0.05f, 2) + 0.3f) * (invert ? -1 : 1));
		//drive.setRightMotor((speed * Math.pow(error * 0.05f, 2) + 0.3f) * (invert ? -1 : 1));
		       }
		       else if (error < -stopThreshold) {
		           //System.out.println("Turn right " + error);
		           chassis.setLeftMotor((speed * Math.pow(error * 0.05f, 2) + 0.3f) * (invert ? -1 : 1));
		           chassis.setRightMotor((-speed * Math.pow(error * 0.05f, 2) - 0.3f) * (invert ? -1 : 1));
		       }
		       else {
		           //System.out.println("The rebel base is within firing range.");
		           chassis.setLeftMotor(0.0);
		           chassis.setRightMotor(0.0);
		       }

		   // Make this return true when this Command no longer needs to run execute()
		   protected boolean isFinished() {
		       return Math.abs(tracker.getDeltaYaw()) < stopThreshold && Math.abs(chassis.getGyroYawSpeed()) < stopYawSpeed;
		   }



}
