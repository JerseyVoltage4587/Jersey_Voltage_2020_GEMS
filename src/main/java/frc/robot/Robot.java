package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
//GIT test
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.paths.*;
import java.util.Arrays;

import frc.robot.commands.Hab2Left2Hatch;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.util.CrashTracker;
import frc.robot.util.DriveSignal;
import frc.robot.util.VisionMath;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	// The SubsystemManager handles logging and looping for all registered subsystems.
	// I think it would be better to have the SubsystemManager own the looper and control all interactions
	// with the subsystems, but for now this is OK.
	private SubsystemManager mSubsystemManager = null;
	private Looper mEnabledLooper = null;

	// The subsystem that manages the drive base.
	// Again, it would be better for SubsystemManager to control the interactions with the subsystem.
	public static Drive getDrive(){
		return Drive.getInstance();
	}
	public static Climb getClimb(){
		return Climb.getInstance();
	}
	public static Arm getArm(){
		return Arm.getInstance();
	}
	public static Lift getLift(){
		return Lift.getInstance();
	}
	public static Intake getIntake(){
		return Intake.getInstance();
	}
	private static PowerDistributionPanel m_PDP;
	public static PowerDistributionPanel getPDP(){
		return m_PDP;
	}
	
	public static void setVisionPipeline(int pipeline){
		NetworkTable ll = NetworkTableInstance.getDefault().getTable("limelight");
		ll.getEntry("pipeline").forceSetNumber(pipeline);
	}
	public static void setVisionLEDs(boolean on){
		NetworkTable ll = NetworkTableInstance.getDefault().getTable("limelight");
		if(on){
			ll.getEntry("ledMode").forceSetNumber(0);
		}else{
			ll.getEntry("ledMode").forceSetNumber(1);
		}	
	}
	/**
	 * Constructor
	 */
	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	private boolean m_robotInit_loggedError = false;
	private static PathManager m_pathManager;
	public static PathManager getPathManager() {
		return m_pathManager;
	}

	private static boolean m_killAuto = false;
	public static void setKillAuto(boolean x){
		m_killAuto = x;
	}
	public static boolean getKillAuto(){
		return m_killAuto;
	}

	@Override
	public void robotInit() {
		
		try {
			CrashTracker.logRobotInit();
		    //m_PDP = new PowerDistributionPanel(0);
			// Create all subsystems and register them with the subsystem manager.
			mEnabledLooper = new Looper();
			mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(),Arm.getInstance(),Climb.getInstance(),Lift.getInstance(),Intake.getInstance()));
		    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			// Initialize the Operator Interface
			OI.getInstance();

			Compressor c = new Compressor(0);
			c.setClosedLoopControl(true);
			getArm().zeroSensors();
			
			//Robot.getDrive().setVisionPath();
			//CalcPathToTarget calcPathToTarget = new CalcPathToTarget();
			//calcPathToTarget.calcPath(0.0);
			SmartDashboard.putNumber("kCameraYaw", Constants.kCameraYaw);
        	SmartDashboard.putNumber("kCameraTilt", Constants.kCameraRotation);
        	SmartDashboard.putNumber("kCameraToCenter", Constants.kCamToCenter);
       		SmartDashboard.putNumber("kCameraToFront", Constants.kCamToFront);
			
			JVPathCreator pc = new JVPathCreator();
			//hab 2 to left cargo
			pc.calcDriveStraight(80, 0, 4.5,false);
			pc.calcArc(52, -30, 4.5, 5.0,false);
			pc.calcDriveStraight(70.0, 5.0, 6.5,false);
			pc.calcArc(62.5, 120, 6.5, 4,false);
			//vision place
			pc.writePathToFile("hab2ToLeftNearCargo");

			JVPathCreator pc1 = new JVPathCreator();
			//left cargo to loading
			pc1.calcArc(45, -65, 0, 0,true);//this needs to go backwards
			pc1.calcArc(106, 73, 0, 7.5,false);
			pc1.calcDriveStraight(65.0, 7.5, 6.0,false);
			pc1.calcArc(106, -40, 6.0, 3.0,false);
			//pc1.calcDriveStraight(5.0, 6.0, 3.0,false);
			//vision pickup
			pc1.writePathToFile("leftNearCargoToLoading");

			JVPathCreator pc2 = new JVPathCreator();
			//loading to left cargo
			pc2.calcDriveStraight(90, 0, 6,true);//backwards
			pc2.calcArc(52,-22,6,6,true);
			pc2.calcDriveStraight(138, 6, 5.5, true);
			pc2.calcArc(36, 112, 5.5, 0, true);
			pc2.writePathToFile("loadingToLeftMiddleCargo");
			


			m_pathManager = new PathManager();
			//comment this so the robot doesn't write paths every power cycle
			//m_pathManager.writePaths();
			m_pathManager.savePaths();
			
			setVisionLEDs(false);
		    CameraServer.getInstance().startAutomaticCapture();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"robotInit");
			if ( m_robotInit_loggedError == false ) {
				m_robotInit_loggedError = true ;
				System.out.println("robotInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}

		double test0 = Robot.getDrive().getLeftEnc();
		double test1 = Robot.getDrive().getRightEnc();

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	private boolean m_disabledInit_loggedError = false;
	@Override
	public void disabledInit() {
		try {
			CrashTracker.logDisabledInit();

			// Stop all subsystem loops.
			mEnabledLooper.stop();

			// Call stop() on all our registered Subsystems.
			mSubsystemManager.stop();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"disabledInit");
			if ( m_disabledInit_loggedError == false ) {
				m_disabledInit_loggedError = true;
				System.out.println("disabledInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Disabled mode.
	 */

	private boolean m_disabledPeriodic_loggedError = false;
	@Override
	public void disabledPeriodic() {
		try {
			VisionMath vm = new VisionMath();
			vm.findRobotPos();
			double xRobot = vm.getRobotX();
			double yRobot = vm.getRobotY();
			SmartDashboard.putNumber("xRobot", xRobot);
			SmartDashboard.putNumber("yRobot", yRobot);
			SmartDashboard.putNumber("visionAngle", Math.atan(yRobot/xRobot)*(180.0/Math.PI));
			//setVisionPipeline(1);

			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"disabledPeriodic");
			if ( m_disabledPeriodic_loggedError == false ) {
				m_disabledPeriodic_loggedError = true;
				System.out.println("disabledPeriodic Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Autonomous mode.
	 * You can use it to set the subsystems up to run the autonomous commands.
	 */
	Command autonomousCommand;
	private boolean m_autonomousInit_loggedError = false;
	private static boolean mInTeleop = false;
	public static boolean getInTeleop(){
		return mInTeleop;
	}
	static int pathsRan = 0;
	public static int getPathsRan(){
		return pathsRan;
	}
	int delayCount;
	double radius,theta;
	@Override
	public void autonomousInit() {
		try {
			CrashTracker.logAutonomousInit();
			// Start the subsystem loops.
			mEnabledLooper.start();

			getDrive().setOpenLoop(DriveSignal.NEUTRAL);
			//getClimb().setDesiredDist(10);
			//getClimb().startClimbing();
			//getClimb().startHold();

			//Robot.getDrive().setVisionPath();
			//Robot.getDrive().startPath();

			//Command autonomousCommand = new AutoTest();
			//Command autonomousCommand = new AutoTest();
			//Command autonomousCommand = new DriveDist(36);
			//Command autonomousCommand = new TurnToAngle(50);
			//Command autonomousCommand = new FollowPath(Robot.getPathManager().getTrajMap().get("leftNearCargoToLoadingLeft"), Robot.getPathManager().getTrajMap().get("leftNearCargoToLoadingRight"));
			Command autonomousCommand = new Hab2Left2Hatch();
			autonomousCommand.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"autonomousInit");
			if ( m_autonomousInit_loggedError == false ) {
				m_autonomousInit_loggedError = true;
				System.out.println("autonomousInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Autonomous mode.
	 */
	private boolean m_autonomousPeriodic_loggedError = false;
	boolean done = false;
	@Override
	public void autonomousPeriodic() {
		try {
			allPeriodic();
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"autonomousPeriodic");
			if ( m_autonomousPeriodic_loggedError == false ) {
				m_autonomousPeriodic_loggedError = true;
				System.out.println("autonomousPeriodic Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Teleop mode.
	 * You can use it to set the subsystems up to run under operator control.
	 */
	private boolean m_teleopInit_loggedError = false;
	@Override
	public void teleopInit() {
		try {
			mInTeleop = true;
			CrashTracker.logTeleopInit();

			// Start the subsystem loops.
			mEnabledLooper.start();

			// Change the Drive subsystem to manual control.
			getDrive().setOpenLoop(DriveSignal.NEUTRAL);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"teleopInit");
			if ( m_teleopInit_loggedError == false ) {
				m_teleopInit_loggedError = true;
				System.out.println("teleopInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Teleop mode.
	 */
	private boolean m_teleopPeriodic_loggedError = false;
	@Override
	public void teleopPeriodic() {
		try {
			allPeriodic();
			
			VisionMath vm = new VisionMath();
			vm.findRobotPos();
			double xRobot = vm.getRobotX();
			double yRobot = vm.getRobotY();
			SmartDashboard.putNumber("xRobot", xRobot);
			SmartDashboard.putNumber("yRobot", yRobot);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"teleopPeriodic");
			if ( m_teleopPeriodic_loggedError == false ) {
				m_teleopPeriodic_loggedError = true;
				System.out.println("teleopPeriodic Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called once each time the robot enters Test mode.
	 * You can use it to set the subsystems up to run their self-test routines.
	 */
	private boolean m_testInit_loggedError = false;
	@Override
	public void testInit() {
		try {
			CrashTracker.logTestInit();

			// ===== TEMPORARY CODE - REMOVE THIS =====
	        mEnabledLooper.start();
	        getDrive().runTest();
	        // ========================================

	        // ... Start a separate thread that runs through the self-test for each registered subsystem.
	        // ... Create and manage the thread in SubsystemManager.

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"testInit");
			if ( m_testInit_loggedError == false ) {
				m_testInit_loggedError = true;
				System.out.println("testInit Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}

	/**
	 * This function is called periodically while the robot is in Test mode.
	 */
	private boolean m_testPeriodic_loggedError = false;
	@Override
	public void testPeriodic() {
		try {
			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t,"testPeriodic");
			if ( m_testPeriodic_loggedError == false ) {
				m_testPeriodic_loggedError = true;
				System.out.println("testPeriodic Crash: "+t.toString());
				t.printStackTrace();
			}
		}
	}
	
	/*
	 * This is the method called periodically during every periodic mode.
	 * It runs all the logging methods, and then runs the WPI scheduler.
	 */
	public void allPeriodic() {
		SmartDashboard.putBoolean("killAuto", m_killAuto);
		mSubsystemManager.outputToSmartDashboard();
		mSubsystemManager.writeToLog();
		mEnabledLooper.outputToSmartDashboard();
		Scheduler.getInstance().run();
	}
}