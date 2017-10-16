package org.usfirst.frc5588.Feodora.commands;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.Properties;
import java.util.logging.Level;
import java.util.logging.Logger;

public class RobotProperties{
	private static final Logger logger = Logger.getLogger(RobotProperties.class.getSimpleName());
	private static final String DIRECTORY = "//home/lvuser/";
	private static final String EXTENSION = ".roboprops";
	
	private final Properties properties;
	private final String fullPath;
	public RobotProperties(String name){
		Properties properties = null;
		try {
			File tempFile = new File(DIRECTORY + name + EXTENSION);
			tempFile.createNewFile();
			tempFile.setWritable(true);
			properties = new Properties();
		} catch (Exception e){
			logger.log(Level.SEVERE, "Failed to construct.", e);
		}
		this.properties = properties;
		this.fullPath = DIRECTORY + name + EXTENSION;
	}
	
	public boolean load(){
		try (FileInputStream file = new FileInputStream(fullPath)){
			properties.loadFromXML(file);
			return true;
		} catch (Exception e){
			logger.log(Level.WARNING, "Failed to load properties file.", e);
			e.printStackTrace();
		}
		return false;
	}
	
	public boolean save(){
		try (FileOutputStream file = new FileOutputStream(fullPath)){
			properties.storeToXML(file, null);
			return true;
		} catch (Exception e){
			logger.log(Level.WARNING, "Failed to save properties file.", e);
			e.printStackTrace();
		}
		return false;
	}
	
	public String getProperty(String key){
		return properties.getProperty(key);
	}
	
	public void setProperty(String key, String value){
		properties.setProperty(key, value);
	}
}
