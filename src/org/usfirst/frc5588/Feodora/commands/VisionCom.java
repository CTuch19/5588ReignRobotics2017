package org.usfirst.frc5588.Feodora.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5588.Feodora.RobotMap;
import org.usfirst.frc5588.Feodora.subsystems.VisionTargetting;

public class VisionCom extends VisionTargetting{
	
	private static final VisionCom singleton = new VisionCom();
	//private static final double YAW_BOILER_FUDGE_FACTOR_DEFAULT = 1.37;
	private static final double YAW_GEAR_FUDGE_FACTOR_DEFAULT = 1.37;
	//private static final double PITCH_BOILER_FUDGE_FACTOR_DEFAULT = 1.0;
	private static final double PITCH_GEAR_FUDGE_FACTOR_DEFAULT = 1.0;
	private static final RobotProperties properties = new RobotProperties(VisionCom.class.getSimpleName());
	private static final long WAIT_INVALID_IMAGE = 1000;
	
	public static VisionCom getInstance() {
		return singleton;
	}

	public enum Mode {
		OFF, GEAR_TARGETING, BOILER_TARGETING
	}

	public enum Resolution {
		HD(1280, 720), SVGA(800, 600), SHD(800, 448), VGA(640, 480), HHD(640, 360), QVGA(320, 240);
		public final int w, h;

		Resolution(int w, int h) {
			this.w = w;
			this.h = h;
		}
	}

	private volatile Mode mode = Mode.OFF;
	private final CameraServer cs = CameraServer.getInstance();
	private CvSource robotCam;
	private VideoSink server;

	private final LEDRing frontRing;
	//private final LEDRing backRing;
	private final DigitalOutput frontBigRing;

	private final Pipeline gearVision = new GearVision();
	private final Pipeline boilerVision = new BoilerVision();
	private final Pipeline gearDriving = new GearDriving();

	private final TargetData targetData = new TargetData();
	private final NetworkTable visionIndicatorTable = NetworkTable.getTable("VisionIndicator");

	private Pipeline pipeline = gearDriving;

	private interface RoboCameraIF {
		void open(String string, int cameraId);

		void close();

		void setResolution(int w, int h);

		void setFPS(int fps);

		boolean getFrame(Mat image);

		void setExposureManual(int exposure);
	}

	private boolean opencv = false;

	private RoboCameraIF createCamera() {
		return opencv ? new OpenCVCamera() : new WPICamera();
	}

	private class OpenCVCamera implements RoboCameraIF {

		private VideoCapture videoCapture = new VideoCapture();

		@Override
		public void open(String string, int cameraId) {
			videoCapture.open(cameraId);
		}

		@Override
		public void setResolution(int w, int h) {
			videoCapture.set(Videoio.CAP_PROP_FRAME_WIDTH, w);
			videoCapture.set(Videoio.CAP_PROP_FRAME_HEIGHT, h);
		}

		@Override
		public void setFPS(int fps) {
			videoCapture.set(Videoio.CAP_PROP_FPS, fps);
		}

		@Override
		public boolean getFrame(Mat image) {
			return videoCapture.read(image);
		}

		@Override
		public void close() {
			videoCapture.release();
		}

		@Override
		public void setExposureManual(int exposure) {
			videoCapture.set(exposure <= 0 ? Videoio.CAP_PROP_AUTO_EXPOSURE : Videoio.CAP_PROP_EXPOSURE, exposure);
		}
	}

	private class WPICamera implements RoboCameraIF {

		private UsbCamera camera = null;
		private CvSink cvSink = null;

		@Override
		public void open(String name, int cameraId) {
			camera = new UsbCamera("cam_" + name, cameraId);
			cvSink = new CvSink("sink_" + name);
			cvSink.setSource(camera);
		}

		@Override
		public void setResolution(int w, int h) {
			camera.setResolution(w, h);
		}

		@Override
		public void setFPS(int fps) {
			camera.setFPS(fps);
		}

		@Override
		public boolean getFrame(Mat image) {
			long time = cvSink.grabFrame(image);
			return time != 0;
		}

		@Override
		public void close() {
			cvSink.free();
			camera.free();
			cvSink = null;
			camera = null;
		}

		@Override
		public void setExposureManual(int exposure) {
			camera.setExposureManual(exposure);
		}

	}

	public double getYawBoilerFudgeFactor() {
		return Double.parseDouble(properties.getProperty("yawBoilerFudgeFactor"));
	}

	public void setYawBoilerFudgeFactor(double fudgeFactor) {
		properties.setProperty("yawBoilerFudgeFactor", Double.toString(fudgeFactor));
		properties.save();
	}

	public double getYawGearFudgeFactor() {
		return Double.parseDouble(properties.getProperty("yawGearFudgeFactor"));
	}

	public void setYawGearFudgeFactor(double fudgeFactor) {
		properties.setProperty("yawGearFudgeFactor", Double.toString(fudgeFactor));
		properties.save();
	}

	public double getPitchBoilerFudgeFactor() {
		return Double.parseDouble(properties.getProperty("pitchBoilerFudgeFactor"));
	}

	public void setPitchBoilerFudgeFactor(double fudgeFactor) {
		properties.setProperty("pitchBoilerFudgeFactor", Double.toString(fudgeFactor));
		properties.save();
	}

	public double getPitchGearFudgeFactor() {
		return Double.parseDouble(properties.getProperty("pitchGearFudgeFactor"));
	}

	public void setPitchGearFudgeFactor(double fudgeFactor) {
		properties.setProperty("pitchGearFudgeFactor", Double.toString(fudgeFactor));
		properties.save();
	}

	public class TargetData {
		boolean isValid;
		double distance;
		double deltaYaw;
		double timeStamp;
		int targetX, targetY;

		private Point contoursPoint = new Point(), delta = new Point();
		private Resolution resolution;
		private double deltaPitch, deltaHeight, angle, targetHeight, cameraHeight, cameraAngle, cameraOffset;
		private double minContours, maxContours, minDistance, maxDistance;
		private double yawFudgeFactor, pitchFudgeFactor;

		public void updateData(List<MatOfPoint> contours, double timeStamp) {
			synchronized (this) {
				switch (mode) {
				/** case BOILER_TARGETING:
					targetHeight = RobotMap.TARGET_BOILER_HEIGHT;
					cameraHeight = RobotMap.CAMERA_BOILER_HEIGHT;
					cameraAngle = RobotMap.CAMERA_BOILER_ANGLE;
					cameraOffset = -RobotMap.CAMERA_BOILER_OFFSET;
					minContours = RobotMap.BOILER_MIN_CONTOURS;
					maxContours = RobotMap.BOILER_MAX_CONTOURS;
					minDistance = RobotMap.BOILER_MIN_DISTANCE;
					maxDistance = RobotMap.BOILER_MAX_DISTANCE;
					yawFudgeFactor = getYawBoilerFudgeFactor();
					pitchFudgeFactor = getPitchBoilerFudgeFactor();
					break; **/
				case GEAR_TARGETING:
				case OFF:
				default:
					//ASK ANDY WHAT ALL MEANS
					targetHeight = RobotMap.TARGET_GEAR_HEIGHT;
					cameraHeight = RobotMap.CAMERA_GEAR_HEIGHT;
					cameraAngle = RobotMap.CAMERA_GEAR_ANGLE;
					cameraOffset = RobotMap.CAMERA_GEAR_OFFSET;
					minContours = RobotMap.GEAR_MIN_CONTOURS;
					maxContours = RobotMap.GEAR_MAX_CONTOURS;
					minDistance = RobotMap.GEAR_MIN_DISTANCE;
					maxDistance = RobotMap.GEAR_MAX_DISTANCE;
					yawFudgeFactor = getYawGearFudgeFactor();
					pitchFudgeFactor = getPitchGearFudgeFactor();
					break;
				}

				if (contours == null || contours.size() < minContours || contours.size() > maxContours) {
					isValid = false;
					deltaYaw = 0;
					distance = -1;
					return;
				}

				this.timeStamp = timeStamp;
				resolution = pipeline.res;

				contoursPoint.x = 0;
				contoursPoint.y = 0;
				MatOfPoint allContours = new MatOfPoint();
				for (MatOfPoint contour : contours) {
					allContours.push_back(contour);
				}

				Rect allBounds = Imgproc.boundingRect(allContours);
				contoursPoint.x = allBounds.x + allBounds.width * 0.5f;
				contoursPoint.y = allBounds.y + allBounds.height * 0.5f;
				targetX = (int) contoursPoint.x;
				targetY = (int) contoursPoint.y;

				delta.x = resolution.w * 0.5f - contoursPoint.x;
				delta.y = resolution.h * 0.5f - contoursPoint.y;
				//ASK ANDY WHAT MEANS
				deltaYaw = delta.x * RobotMap.CAMERA_FOV_X / resolution.w * yawFudgeFactor;
				deltaPitch = delta.y * RobotMap.CAMERA_FOV_Y / resolution.h * pitchFudgeFactor;
				deltaHeight = targetHeight - cameraHeight;
				angle = cameraAngle + deltaPitch;
				//NECESSARY FOR ONLY GEARS?
				distance = deltaHeight / Math.tan(angle) + cameraOffset + (mode == Mode.BOILER_TARGETING ? FieldData.Map.BOILER_RADIUS : 0.0);

				if (distance < minDistance || distance > maxDistance) {
					isValid = false;
				} else {
					isValid = true;
				}
			}
		}

		public double getDeltaYaw() {
			synchronized (this) {
				return deltaYaw;
			}
		}

		public double getDistance() {
			synchronized (this) {
				return distance;
			}
		}

		public boolean isValid() {
			synchronized (this) {
				return isValid;
			}
		}

		public double getTimeStamp() {
			synchronized (this) {
				return timeStamp;
			}
		}

		public boolean isNewImage(double oldTimeStamp) {
			synchronized (this) {
				return Double.compare(timeStamp, oldTimeStamp) != 0;
			}
		}

		public boolean isDistanceSane(double distance) {
			synchronized (this) {
				return distance < maxDistance && distance > minDistance;
			}
		}
	}
    //?????????
	private VisionCom() {
		super(VisionCom.class);
		frontRing = new LEDRing(RobotMap.LED_FRONT);
		frontRing.setLoading();
		//backRing = new LEDRing(RobotMap.LED_BACK);
		//backRing.setLoading();
		frontBigRing = new DigitalOutput(RobotMap.LED_FRONT_BIG);
		frontBigRing.set(false);
	}
	//?????????
	@Override
	protected void initSubsystem() {
		if (!properties.load()) {
			setYawBoilerFudgeFactor(YAW_BOILER_FUDGE_FACTOR_DEFAULT);
			setYawGearFudgeFactor(YAW_GEAR_FUDGE_FACTOR_DEFAULT);
			setPitchBoilerFudgeFactor(PITCH_BOILER_FUDGE_FACTOR_DEFAULT);
			setPitchGearFudgeFactor(PITCH_GEAR_FUDGE_FACTOR_DEFAULT);
		}

		// setup stream to Dash-board
		Resolution res = Resolution.HHD;
		Size size = new Size(res.w, res.h);
		robotCam = new CvSource("RobotCam", VideoMode.PixelFormat.kMJPEG, res.w, res.h, 30);
		server = cs.addServer("serve_RobotCam");
		server.setSource(robotCam);

		Thread videoThread = new Thread("Video Cam") {

			private Mode prevMode = null;

			public void run() {
				Mat image = new Mat();

				long prev = System.currentTimeMillis();
				long time, dt, t0;
				while (!Thread.interrupted()) {
					try{
						// check and switch modes
						switchMode();

						// update parameters
						pipeline.update();
						time = System.currentTimeMillis();

						// grab image
						pipeline.getImage(image);
						t0 = time;
						time = System.currentTimeMillis();
						SmartDashboard.putNumber("grab image", time - t0);
						
						// check to see if image is invalid
						if(image.cols() <= 0 || image.rows() <= 0){
							logger.log(Level.WARNING, "Image grabbed has zero rows or cols.");
							try {
								Thread.sleep(WAIT_INVALID_IMAGE);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}
							continue;
						}

						// process image
						List<MatOfPoint> output = pipeline.process(image);
						targetData.updateData(output, System.currentTimeMillis());
						t0 = time;
						time = System.currentTimeMillis();
						SmartDashboard.putNumber("process image", time - t0);

						// publish image
						pipeline.publish(image, size);
						t0 = time;
						time = System.currentTimeMillis();
						SmartDashboard.putNumber("public image", time - t0);

						// report FPS
						time = System.currentTimeMillis();
						dt = time - prev;
						prev = time;
						double actualFPS = Math.round(10000.0 / dt) / 10.0;
						SmartDashboard.putNumber("FPS", actualFPS);
					}
					catch(Exception e){
						logger.log(Level.SEVERE, "Exception encountered.", e);
						e.printStackTrace();
						try {
							Thread.sleep(50);
						} catch (InterruptedException e1) {
							e1.printStackTrace();
						}
					}
				}
			}

			private void switchMode() {

				if (prevMode != null && prevMode == mode) {
					return;
				}

				pipeline.deInit();
				targetData.isValid = false;

				switch (mode) {

				case OFF:
					pipeline = gearDriving;
					break;

				case GEAR_TARGETING:
					pipeline = gearVision;
					break;

				case BOILER_TARGETING:
					pipeline = boilerVision;
					break;
				}

				pipeline.init();

				prevMode = mode;
			}

		};

		videoThread.start();
	}

	public void setMode(Mode mode) {
		logger.log(Level.INFO, mode.toString());
		this.mode = mode;
	}
	
	public void addAutoSetupAssistance(Point...points){
		pipeline.addAutoSetupAssistance(points);
	}
	
	public void clearAutoSetupAssistance(){
		pipeline.clearAutoSetupAssistance();
	}
	
	public Mode getMode(){
		return mode;
	}

	@Override
	public void reset() {
		setMode(Mode.OFF);
	}

	@Override
	protected void runningUpdate() {
		visionIndicatorTable.putNumber("Distance", targetData.getDistance());
		visionIndicatorTable.putNumber("DeltaYaw", targetData.getDeltaYaw());
		visionIndicatorTable.putBoolean("IsValid", targetData.isValid() && mode != Mode.OFF);
	}

	@Override
	protected void debugUpdate() {
		SmartDashboard.putNumber("Vision Distance", targetData.getDistance() / 0.0254);
		SmartDashboard.putNumber("Vision Delta Yaw", Math.toDegrees(targetData.getDeltaYaw()));
		SmartDashboard.putBoolean("Vision Valid", targetData.isValid());
		SmartDashboard.putNumber("Vision Delta Pitch", targetData.deltaPitch);
	}

	@Override
	protected void startDebug() {
		SmartDashboard.putData("Vision Yaw Calibration", new Command() {
			private static final double CALIBRATION_POINT_SIZE = 10;
			private List<Point> calibrationPoints = new ArrayList<Point>();
			private boolean yawReady = false;

			{
				this.setRunWhenDisabled(true);
				
				SmartDashboard.putData("Vision Yaw Calibration Take Data Point", new Command() {
					private final Pose2D pose = new Pose2D(0.0, 0.0, 0.0);

					{
						this.setRunWhenDisabled(true);
					}
					
					@Override
					protected boolean isFinished() {
						return targetData.isValid() && yawReady;
					}

					@Override
					protected void end() {
						double estimatedYaw = -targetData.getDeltaYaw();
						double actualYaw = RobotState.getInstance().getPose(pose).heading;
						actualYaw = (actualYaw > Math.PI) ? (Math.PI * -2.0) + actualYaw : actualYaw;
						
						logger.log(Level.INFO, "Adding calibration point: Vision: " + estimatedYaw + "| Actual: " + actualYaw);
						
						calibrationPoints.add(
								new Point(estimatedYaw, actualYaw));
					}
					
					@Override
					protected void interrupted(){
						
					}
				});
			}

			@Override
			protected void initialize() {
				if(Math.toDegrees(Math.abs(Vision.getInstance().getTargetData().getDeltaYaw())) < 0.5){
					Scheduler.getInstance().add(new RobotState.ResetRobotState());
					RobotState.getInstance().setRotationMode(RotationMode.IMU_ONLY);
					calibrationPoints.clear();
					yawReady = true;
				}
			}
			
			@Override
			protected void execute(){
				if(!yawReady){
					initialize();
				}
			}

			@Override
			protected boolean isFinished() {
				return calibrationPoints.size() >= CALIBRATION_POINT_SIZE && yawReady;
			}

			@Override
			protected void end() {
				RobotState.getInstance().setRotationMode(RotationMode.BLENDED);
				yawReady = false;
				LinearRegression line = new LinearRegression(calibrationPoints);

				switch (mode) {
				case BOILER_TARGETING:
					setYawBoilerFudgeFactor(line.beta1 * getYawBoilerFudgeFactor());
					break;
				case GEAR_TARGETING:
				case OFF:
				default:
					setYawGearFudgeFactor(line.beta1 * getYawGearFudgeFactor());
					break;
				}
			}
			
			@Override
			protected void interrupted(){
				
			}
		});

		SmartDashboard.putData("Vision Pitch Calibration", new Command() {
			private static final double CALIBRATION_POINT_SIZE = 10;
			private List<Point> calibrationPoints = new ArrayList<Point>();

			{
				this.setRunWhenDisabled(true);
				
				SmartDashboard.putData("Vision Pitch Calibration Take Data Point", new Command() {
					private double actualDistance;

					{
						this.setRunWhenDisabled(true);
					}
					
					@Override
					protected boolean isFinished() {
						return targetData.isValid() && SmartDashboard.getBoolean("Pitch Calibration Ready", false);
					}

					@Override
					protected void end() {
						actualDistance = SmartDashboard.getNumber("Pitch Calibration Actual Distance", 0.0);
						SmartDashboard.putBoolean("Pitch Calibration Ready", false);
						SmartDashboard.putNumber("Pitch Calibration Actual Distance", 0.0);
						
						logger.log(Level.INFO, "Adding calibration point: Vision: " + (targetData.deltaPitch) + "| Actual: " + Math.atan2(targetData.deltaHeight, actualDistance));
						
						calibrationPoints.add(
								new Point(targetData.deltaPitch, Math.atan2(targetData.deltaHeight, actualDistance)));
					}
					
					@Override
					protected void interrupted(){
						
					}
				});
			}

			@Override
			protected void initialize() {
				calibrationPoints.clear();

				SmartDashboard.putNumber("Pitch Calibration Actual Distance", 0.0);
				SmartDashboard.putBoolean("Pitch Calibration Ready", false);
			}

			@Override
			protected boolean isFinished() {
				return calibrationPoints.size() >= CALIBRATION_POINT_SIZE;
			}

			@Override
			protected void end() {
				LinearRegression line = new LinearRegression(calibrationPoints);

//				switch (mode) {
//				case BOILER_TARGETING:
//					setPitchBoilerFudgeFactor(1.0 / line.beta1);
//					break;
//				case GEAR_TARGETING:
//				case OFF:
//				default:
//					setPitchGearFudgeFactor(1.0 / line.beta1);
//					break;
//				}
			}
			
			@Override
			protected void interrupted(){
				
			}
		});
		
		SmartDashboard.putData("Vision-Off", new SetVisionCommand(Mode.OFF));
		SmartDashboard.putData("Vision-Gear", new SetVisionCommand(Mode.GEAR_TARGETING));
		SmartDashboard.putData("Vision-Boiler", new SetVisionCommand(Mode.BOILER_TARGETING));
	}

	@Override
	protected void stopDebug() {
	}

	@Override
	protected void initDefaultCommand() {
	}

	public TargetData getTargetData() {
		return targetData;
	}

	private class GearDriving extends Pipeline {

		public GearDriving() {
			super(Resolution.HHD);
			cameraId = RobotMap.CAMERA_BACKWARDS;
			fps = 15;
			exposure = 80;
			inverted = true;
		}

		@Override
		public List<MatOfPoint> process(Mat image) {
			return filteredContours;
		}

		@Override
		public void update() {
			updateExposure();
		}
	}

	private class BoilerVision extends Pipeline {

		public BoilerVision() {
			super(Resolution.HD);
			cameraId = RobotMap.CAMERA_FORWARDS;
			fps = 15;
			exposure = 0;
			inverted = false;

			lowerColorLimit = new Scalar(65, 200, 53);
			upperColorLimit = new Scalar(100, 255, 255);

			minArea = 300;
			dilateIterations = 2;
		}

		@Override
		public void init() {
			super.init();
			frontRing.setGold();
			frontBigRing.set(true);
		}

		@Override
		public void deInit() {
			super.deInit();
			frontRing.setRainbow();
			frontBigRing.set(false);
		}
	}

	private class GearVision extends Pipeline {

		public GearVision() {
			super(Resolution.HD);
			cameraId = RobotMap.CAMERA_BACKWARDS;
			fps = 15;
			exposure = 20;
			inverted = true;

			lowerColorLimit = new Scalar(70, 180, 160);
			upperColorLimit = new Scalar(100, 255, 255);
			
			minArea = 1000;
			maxWidth = 1000;
			maxHeight = 1000;
		}

		@Override
		public void init() {
			super.init();
			backRing.setGold();
		}

		@Override
		public void deInit() {
			super.deInit();
			backRing.setRainbow();
		}
	}

	private abstract class Pipeline {

	int cameraId = RobotMap.CAMERA_FORWARDS;
	int fps;
	final Resolution res;
	RoboCameraIF camera = null;

	List<MatOfPoint> convexContours = new ArrayList<MatOfPoint>();
	List<MatOfPoint> filteredContours = new ArrayList<MatOfPoint>();
	List<MatOfPoint> initialContours = new ArrayList<MatOfPoint>();
	List<MatOfPoint> autoSetupAssistanceContours = new ArrayList<MatOfPoint>();
	Mat cvtColorMat = new Mat();
	Mat hierarchy = new Mat();
	MatOfInt hull = new MatOfInt();
	Mat invertedImage = new Mat();

	Scalar lowerColorLimit = new Scalar(0, 0, 0);
	Scalar upperColorLimit = new Scalar(255, 255, 255);
	double minArea = 100;
	double minPerimeter = 0;
	double minWidth = 0;
	double maxWidth = 1000;
	double minHeight = 0;
	double maxHeight = 1000;
	double[] solidity = new double[] { 0, 100 };
	double maxVertices = 1000000;
	double minVertices = 0;
	double minRatio = 0;
	double maxRatio = 1000;
	int dilateIterations = 3;
	final Mat cvDilateKernel = new Mat();
	final Point cvDilateAnchor = new Point(-1, -1);
	final Scalar cvDilateBordervalue = new Scalar(-1);

	static final String hueMin = "Hue Min";
	static final String hueMax = "Hue Max";
	static final String satMin = "Sat Min";
	static final String satMax = "Sat Max";
	static final String valMin = "Val Min";
	static final String valMax = "Val Max";
	static final String exposureString = "Exposure";
	static final String invertedString = "Inverted";

	int exposure = -1;
	boolean inverted;
	Mat sendImage = new Mat();
	Scalar color = new Scalar(0, 0, 255);
	final Size size;
	int thickness = 5;

	public Pipeline(Resolution res) {
		this.res = res;
		size = new Size(res.w, res.h);
	}

	public void getImage(Mat image) {
		if (camera.getFrame(inverted ? invertedImage : image)) {
			if (inverted) {
				Core.flip(invertedImage, image, -1);
			}
		}
	}

	public void publish(Mat image, Size size) {
		// draw any contours
		for (int i = 0; i < filteredContours.size(); i++) {
			Imgproc.drawContours(image, filteredContours, i, color, thickness);
		}
		
		for (int i = 0; i < autoSetupAssistanceContours.size(); i++) {
			Imgproc.drawContours(image, autoSetupAssistanceContours, i, new Scalar(0, 255, 0), 2);
		}

		Imgproc.drawMarker(image, new Point(targetData.targetX, targetData.targetY), new Scalar(0, 255, 0));

		// resize and publish image
		if (this.size == size) {
			robotCam.putFrame(image);
		} else {
			// Imgproc.cvtColor(cvtColorMat, image, Imgproc.COLOR_GRAY2BGR);
			Imgproc.resize(image, sendImage, size);
			robotCam.putFrame(sendImage);
		}
	}
	
	public void addAutoSetupAssistance(Point...points){
		autoSetupAssistanceContours.add(new MatOfPoint(points));
	}
	
	public void clearAutoSetupAssistance(){
		autoSetupAssistanceContours.clear();
	}

	public void init() {
		SmartDashboard.putNumber(hueMin, lowerColorLimit.val[0]);
		SmartDashboard.putNumber(satMin, lowerColorLimit.val[1]);
		SmartDashboard.putNumber(valMin, lowerColorLimit.val[2]);
		SmartDashboard.putNumber(hueMax, upperColorLimit.val[0]);
		SmartDashboard.putNumber(satMax, upperColorLimit.val[1]);
		SmartDashboard.putNumber(valMax, upperColorLimit.val[2]);
		SmartDashboard.putNumber(exposureString, exposure);
		SmartDashboard.putBoolean(invertedString, inverted);

		camera = createCamera();
		camera.open(this.getClass().getSimpleName(), cameraId);
		camera.setResolution(res.w, res.h);
		camera.setFPS(fps);
		camera.setExposureManual(exposure);
	}

	public void deInit() {
		if (camera != null) {
			camera.close();
		}
	}

	public void update() {
		lowerColorLimit.val[0] = SmartDashboard.getNumber(hueMin, 0);
		lowerColorLimit.val[1] = SmartDashboard.getNumber(satMin, 0);
		lowerColorLimit.val[2] = SmartDashboard.getNumber(valMin, 0);
		upperColorLimit.val[0] = SmartDashboard.getNumber(hueMax, 0);
		upperColorLimit.val[1] = SmartDashboard.getNumber(satMax, 0);
		upperColorLimit.val[2] = SmartDashboard.getNumber(valMax, 0);
		updateExposure();
	}

	protected void updateExposure() {
		// do not set over and over again since it slows FPS
		int exposure = (int) SmartDashboard.getNumber(exposureString, 80);
		if (this.exposure != exposure) {
			this.exposure = exposure;
			camera.setExposureManual(exposure);
		}
	}

	public List<MatOfPoint> process(Mat image) {

		// isolate color to binary image
		Imgproc.cvtColor(image, cvtColorMat, Imgproc.COLOR_BGR2HSV);// Imgproc.COLOR_BGR2RGB);//Imgproc.COLOR_BGR2HSV);
		Core.inRange(cvtColorMat, lowerColorLimit, upperColorLimit, cvtColorMat);

		// dilate
		Imgproc.dilate(cvtColorMat, cvtColorMat, cvDilateKernel, cvDilateAnchor, dilateIterations,
				Core.BORDER_CONSTANT, cvDilateBordervalue);

		// find initial set of contours
		initialContours.clear();
		Imgproc.findContours(cvtColorMat, initialContours, hierarchy, Imgproc.RETR_EXTERNAL,
				Imgproc.CHAIN_APPROX_SIMPLE);
				// System.out.println("findContour " +
				// initialContours.size());

		// convert to convex hull contours
		convexContours.clear();
		for (MatOfPoint contour : initialContours) {

			MatOfPoint mopHull = new MatOfPoint();

			Imgproc.convexHull(contour, hull);

			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);

			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				mopHull.put(j, 0, new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] });
			}
			convexContours.add(mopHull);
		}
		// System.out.println("convexContours " + convexContours.size());

		// filter contours
		filteredContours.clear();
		filterContours(convexContours, minArea, minPerimeter, minWidth, maxWidth, minHeight, maxHeight, solidity,
				maxVertices, minVertices, minRatio, maxRatio, filteredContours);

		SmartDashboard.putNumber("Contours", filteredContours.size());

		return filteredContours;
	}

	/**
	 * Filters out contours that do not meet certain criteria.
	 * 
	 * @param inputContours
	 *            is the input list of contours
	 * @param output
	 *            is the the output list of contours
	 * @param minArea
	 *            is the minimum area of a contour that will be kept
	 * @param minPerimeter
	 *            is the minimum perimeter of a contour that will be kept
	 * @param minWidth
	 *            minimum width of a contour
	 * @param maxWidth
	 *            maximum width
	 * @param minHeight
	 *            minimum height
	 * @param maxHeight
	 *            maximimum height
	 * @param Solidity
	 *            the minimum and maximum solidity of a contour
	 * @param minVertexCount
	 *            minimum vertex Count of the contours
	 * @param maxVertexCount
	 *            maximum vertex Count
	 * @param minRatio
	 *            minimum ratio of width to height
	 * @param maxRatio
	 *            maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea, double minPerimeter,
			double minWidth, double maxWidth, double minHeight, double maxHeight, double[] solidity,
			double maxVertexCount, double minVertexCount, double minRatio, double maxRatio,
			List<MatOfPoint> output) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		// operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth)
				continue;
			if (bb.height < minHeight || bb.height > maxHeight)
				continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea)
				continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter)
				continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] };
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1])
				continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)
				continue;
			final double ratio = bb.width / (double) bb.height;
			if (ratio < minRatio || ratio > maxRatio)
				continue;
			output.add(contour);
		}
	}

