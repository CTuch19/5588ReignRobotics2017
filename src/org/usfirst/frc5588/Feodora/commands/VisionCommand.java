package org.usfirst.frc5588.Feodora.commands;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.usfirst.frc5588.Feodora.Robot;
import org.usfirst.frc5588.Feodora.subsystems.Drive;
//import org.usfirst.frc5588.Feodora.subsystems.VisionTargetting;
import org.usfirst.frc5588.Feodora.commands.GripPipelinepipe;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class VisionCommand extends Command {

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}

	
	/**
	//CameraServer server;
    UsbCamera tarcam = Robot.getCam();
    private VisionThread visionThread;
    private double centerX = 0.0;
    private double turn = 0.0;
    private final Object imgLock = new Object();
    //Drive drive = Drive.getInstance();
    GripPipelinepipe grip = new GripPipelinepipe();
    
    public VisionCommand() {
    	//requires(Robot.drive);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//server = CameraServer.getInstance(); 
        //targetcam = server.startAutomaticCapture(0); 
        //tarcam.setBrightness(30);
        //targetcam.setResolution(320, 240);
        visionThread = new VisionThread(tarcam, grip, pipeline -> {
        	if(!pipeline.filterContoursOutput().isEmpty())
        	{
        		Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        		System.out.println("I just found a rectangle");
        	
        	synchronized (imgLock){
        		centerX = r.x + (r.width/2);
        		System.out.println("Center x at first is" + centerX);
        	}
        	}
        	else
        	{
        		System.out.println("I never found a contour");
        	}
        });
        visionThread.start();
        System.out.println("I started the thread");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//ArrayList<MatOfPoint> contours = grip.filterContoursOutput();
    	double centerX;
    	synchronized (imgLock) {
    		centerX = this.centerX;
    		System.out.println("Center x in excute is" + centerX);
    	}
    	
    	turn = centerX - (160);
    	System.out.println("the turn is" + turn);
    	Drive.getInstance().setSpeed(0.1+(turn*0.005), 0.1+(turn*0.005));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(turn<= 1)
        {
        	Drive.getInstance().setSpeed(0, 0);
        	return true;
        }
        if(isTimedOut())
        {
        	return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Drive.getInstance().stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }**/
}
