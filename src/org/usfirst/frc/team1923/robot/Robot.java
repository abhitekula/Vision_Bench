
package org.usfirst.frc.team1923.robot;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1923.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	
	private VisionThread visionThread;
	public static double centerX = 0.0;
	public RobotDrive drive;
	
	public static double[] centerx;
	public static double[] defaultV = new double[0];

	NetworkTable table;
	
	public static final Object imgLock = new Object();

	// Arduino
	static I2C Wire = new I2C(Port.kOnboard, 4);

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		//chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		table=NetworkTable.getTable("GRIP/table");
		Arduino();
		vision();	
		//log();
		//network table code
		
	}

	private void log() {
		// TODO Auto-generated method stub
		table=NetworkTable.getTable("GRIP/table");
		centerx=table.getNumberArray("centerX", defaultV);
		for(double center:centerx){
			System.out.print(center + "\t");
		}
		System.out.println();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand  new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		log();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}

	private void Arduino() {
		byte[] bits = new byte[1];
		bits[0] = 1;
		Wire.transaction(bits, bits.length, null, 0);
	}

	private void vision(){
		new Thread(() -> {
            AxisCamera camera;// = CameraServer.getInstance().startAutomaticCapture();
            camera=CameraServer.getInstance().addAxisCamera("Axis", "10.19.23.76");
            //camera.setResolution(640, 480);
            //camera.setFPS(30);
            //camera.setExposureManual(20);
            //camera.setWhiteBalanceManual(0);
            //camera.setExposureManual(0);
            //camera.setExposureAuto();
            CvSource outputStream = CameraServer.getInstance().putVideo("HSL", 640, 480);
            
            visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
            
            //if(visionStart){
//                if (!pipeline.filterContoursOutput().isEmpty()) {
//                    Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
//                    //synchronized (imgLock) {
//                    //    centerX = r.x + (r.width / 2);
//                    //}
//                }
            	
                outputStream.putFrame(pipeline.hslThresholdOutput());
           //}
           // else{
            	//CvSink cvSink = CameraServer.getInstance().getVideo();
            	//Mat source = new Mat();
            	//cvSink.grabFrame(source);
            	//ArrayList<MatOfPoint> points = new ArrayList<MatOfPoint>();
            	
            	//Imgproc.boundingRect(points);
            	
            	//outputStream.putFrame(source);
           // }
           }
            
           );
            visionThread.start();
            
            
//            CvSink cvSink = CameraServer.getInstance().getVideo();
//            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
//            
//            Mat source = new Mat();
//            Mat output = new Mat();
//            
//            while(true) {
//                cvSink.grabFrame(source);
//                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
//                outputStream.putFrame(output);
//            }
        }).start();
	}

}
