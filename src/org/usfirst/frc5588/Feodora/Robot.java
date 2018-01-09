package org.usfirst.frc5588.Feodora;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.Image;

import org.usfirst.frc5588.Feodora.commands.*;
import org.usfirst.frc5588.Feodora.subsystems.*;  
import edu.wpi.first.wpilibj.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.command.CommandGroup;




public class Robot extends IterativeRobot {

    Command autonomousCommand;

    public static OI oi;
    public static Drive drive;
    public static Climber climber;
    
    //Open the Smart dashboard application by opening file explorer > Windows(C:) > Users > Team 5588 > wpilib > tools
    public SendableChooser<CommandGroup> autoChooser;
    public SendableChooser<CommandGroup> autonomousDirectionChooser;

    //For camera setup
    int session;
    Image frame;
    CameraServer server;
    static UsbCamera targetcam;
    UsbCamera targetcam2;
    int imaqError;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	System.out.println("entering robotInit()");
    	RobotMap.init();
        
        drive = Drive.getInstance();
        climber = Climber.getInstance();
        
        
        oi = new OI();
        
        
       
       //used with sendable chooser
        autoChooser = new SendableChooser<CommandGroup>();
        autoChooser.addDefault("Auto2 LeftEncod default", new Auto2LeftEncoder());
        autoChooser.addObject("Auto 4 or 5, drive past baseline", new Auto45DriveBaseline());
        autoChooser.addObject("Auto2 Left Gear", new Auto2LeftGear());
        //autoChooser.addObject("Auto2 Vision left", new AutoLeftGearVisionAlign());
        //autoChooser.addObject("Auto 1 EXTREME", new Auto1EXTREME());
        autoChooser.addObject("Auto3 Right Gear", new Auto3RightGear());
        //autoChooser.addObject("Auto2 LeftGear EXTREME", new Auto2LeftGearEXTREME());
        //autoChooser.addObject("Auto3 Right Gear EXTREME", new Auto3RightGearEXTREME());
        autoChooser.addObject("Auto 1 Center Gear", new Auto1CloseGear());
        autoChooser.addObject("Auto2 left ENCODER", new Auto2LeftEncoder());
        autoChooser.addObject("Auto3 Right ENCODER", new Auto3RightEncoder());
        //dont forget this line!
        SmartDashboard.putData("Automode", autoChooser);
       
        //Camera code
        server = CameraServer.getInstance(); 
        
        targetcam = server.startAutomaticCapture(0); 
        targetcam.setBrightness(1);
        
        targetcam2 = server.startAutomaticCapture(1);
        targetcam2.setBrightness(1);
        
        
    }

    public static UsbCamera getCam()
    {
    	return targetcam;
    }
    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){

    }

    public void disabledPeriodic() {
    	
        Scheduler.getInstance().run();
    }

    public void autonomousInit() {
        // schedule the autonomous command (example)
    	autonomousCommand = (Command)autoChooser.getSelected();
            	
    	RobotMap.leftEncoder.reset();
    	RobotMap.rightEncoder.reset();
    	
    	autonomousCommand.start();
		
    }
    
 

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
    	
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }

	
}
