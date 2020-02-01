package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.util.Gyro;
import frc.robot.util.VisionMath;
import frc.robot.util.AsyncStructuredLogger;
import frc.robot.util.CalcPathToTarget;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

import java.io.File;
import java.io.FileWriter;

import frc.robot.paths.*;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.util.ArcMath;
import frc.robot.util.AsyncAdHocLogger;
import frc.robot.util.AsyncStructuredLogger;
import frc.robot.util.DriveSignal;

public class Climb extends Subsystem {
	
	private long startTime;

    private static Climb mInstance = null;
    public static Climb getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Drive.class ) {
    			mInstance = new Climb();
    		}
    	}
    	return mInstance;
    }

    // The robot drivetrain's various states.
    public enum ClimbControlState {
        RESTING, // open loop voltage control
		CLIMBING, // used for autonomous driving
		HALF_ON,
		HOLD,
    }
    public ClimbControlState getState(){
    	return mClimbControlState;
    }
	private int m_frontEncoderLast=0,m_backEncoderLast = 0;
	private long m_lastTime;
    public int getFrontEnc(){
    	return mFrontTalon.getSelectedSensorPosition(0);
    }
    public int getBackEnc(){
    	return mBackTalon.getSelectedSensorPosition(0);
    }
    
    double m_frontPathPos, m_backPathPos;
    public void setPathPos(double backPathPos,double frontPathPos){
    	synchronized(Drive.class){
    		m_frontPathPos = frontPathPos * Constants.kInchesPerTic / 12.0;
    		m_backPathPos = backPathPos * Constants.kInchesPerTic / 12.0;
    		//System.out.println(m_frontPathPos);
    	}
    }
    public double getFrontPathPos(){
    	synchronized(Drive.class){
    		return m_frontPathPos;
    	}
    }
    public double getBackPathPos(){
    	synchronized(Drive.class){
    		return m_backPathPos;
    	}
	}

	double m_desiredDist, m_xNext;
	int m_zeroLeftEncoder, m_zeroRightEncoder;
	double m_acceleration;
	public void setDesiredDist(double distInches){
    	synchronized(Drive.class){
			m_desiredDist = distInches / 12.0;//feet
			m_desiredDist *= 19.0/15.0;//TODO get rid of this fudge?
			/*m_zeroLeftEncoder = getLeftEnc();
			m_zeroRightEncoder = getRightEnc();
			m_xNext = 0;*/
			double halfTime = Math.sqrt(m_desiredDist/Constants.kClimbMaxAcceleration);
			int numIntervals = (int)(100*halfTime)+1;
			halfTime = numIntervals * 0.01;
			m_acceleration = m_desiredDist / (halfTime*halfTime);

			Segment[] path = new Segment[(numIntervals*2)+2];
			double roll = Gyro.getRoll();
			double xMax=0;
			double vMax=0;
			for(int i = 0;i<numIntervals+1;i++){
				double v = m_acceleration * (i * 0.01);
				double x = 0.5 * m_acceleration * ((i * 0.01)*(i * 0.01));
				//System.out.println("x: "+x+" v: "+v+" a: "+m_acceleration+" yaw: "+yaw);
				path[i] = new Segment(0.01, 0, 0, x, v, m_acceleration, 0, 0);
				xMax = x;
				vMax = v;
			}
			for(int i = 0;i<numIntervals+1;i++){
				double v = vMax - (m_acceleration * (i * 0.01));
				double x = xMax + (vMax*(i*0.01)) - (0.5 * m_acceleration * ((i * 0.01)*(i * 0.01)));
				//System.out.println("x: "+x+" v: "+v+" a: "+m_acceleration+" yaw: "+yaw);
				path[i+numIntervals+1] = new Segment(0.01, 0, 0, x, v, -m_acceleration, 0, 0);
			}
			m_leftBackTrajectory = new Trajectory(path);
			m_rightFrontTrajectory = new Trajectory(path);
			for(int i =0;i<m_leftBackTrajectory.length();i++){
				Segment s = m_leftBackTrajectory.segments[i];
				//System.out.println("step: "+i+" pos: "+s.position+" vel: "+s.velocity+" acc: "+s.acceleration);
			}
    	}
    }

    // Control states
    private ClimbControlState mClimbControlState = ClimbControlState.RESTING;

    // Hardware
    private final WPI_TalonSRX mFrontTalon, mBackTalon;
    private final WPI_VictorSPX mDriveVictor;
    private final Gyro mNavXBoard;

    // Logging
    private DebugOutput mDebugOutput;
    private final AsyncStructuredLogger<DebugOutput> mCSVWriter;
	
	private boolean mStartingPath=false;
	private String pathFilename;
    public void setPathFilename(String x){
    	pathFilename = x;
	}
	private Trajectory m_leftBackTrajectory,m_rightFrontTrajectory;
    
    public void startClimbing() {
    	//System.out.println("in startPath");
    	synchronized (Climb.this) {
			mStartingPath = true;
    		mClimbControlState = ClimbControlState.CLIMBING;
    	}
	}
	public void startResting() {
    	//System.out.println("in startPath");
    	synchronized (Climb.this) {
			mStartingPath = true;
    		mClimbControlState = ClimbControlState.RESTING;
    	}
	}
	public void startHalfOn() {
    	//System.out.println("in startPath");
    	synchronized (Climb.this) {
			mStartingPath = true;
    		mClimbControlState = ClimbControlState.HALF_ON;
    	}
	}
	public void startHold() {
    	//System.out.println("in startPath");
    	synchronized (Climb.this) {
			mStartingPath = true;
    		mClimbControlState = ClimbControlState.HOLD;
    	}
	}
	
	private double getFrontPIDOutput(double encSetpoint, double rollSetpoint, boolean hold){
		double error = encSetpoint - getFrontEnc();
		double rollError = rollSetpoint - Gyro.getRoll();
		double output = (error * Constants.kClimbHoldKp) - (rollError * Constants.kClimbHoldKg);
		if(hold){
			output += Constants.kClimbHoldGravityMotorLevel;
		}
		return -output;
	}
	private double getBackPIDOutput(double encSetpoint, double rollSetpoint, boolean hold){
		double error = encSetpoint - getBackEnc();
		double rollError = rollSetpoint - Gyro.getRoll();
    	double output = (error * Constants.kClimbHoldKp) + (rollError * Constants.kClimbHoldKg);
		if(hold){
			output += Constants.kClimbHoldGravityMotorLevel;
		}
		return output;
    }
	
	AsyncAdHocLogger asyncAdHocLogger;
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Climb.this) {
            	startTime = System.nanoTime();
				asyncAdHocLogger = new AsyncAdHocLogger("");
            }
        }

        @Override
        public void onLoop(double timestamp) {
			iCall++;
			if(iCall % 1000 == 0){
        		//System.out.println("onLoop " + iCall + " " + mDriveControlState + " " + mLeftMaster.getControlMode());
        	}
            synchronized (Climb.this) {
                switch (mClimbControlState) {
                case RESTING:
					//do nothing
					doRest();
                    break;
                case CLIMBING:
					doClimbUp();
					break;
				case HALF_ON:
					doHalfOn();
					break;
				case HOLD:
					//pid
					setMotorLevels(0.1, -0.5);
					break;
                default:
                    System.out.println("Unexpected climb control state: " + mClimbControlState);
                    break;
				}
			}
			m_backEncoderLast = getBackEnc();
			m_frontEncoderLast = getFrontEnc();
			m_lastTime = System.nanoTime();
        	logValues();
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
	};

	public void setClimbDrive(double motorLevel){
		mDriveVictor.set(motorLevel);
	}

	PathFollower follower = null;
	double m_frontHoldSetpoint, m_backHoldSetpoint,m_rollHoldSetpoint;
	double m_frontStartPos, m_backStartPos;
	boolean m_startedClimb;
	private void doClimbUp(){
    	if (mStartingPath) {
    		mStartingPath = false;
			setPathPos(0,0);
    		follower = new PathFollower(m_leftBackTrajectory,m_rightFrontTrajectory,false);
			follower.initialize();
			m_frontStartPos = getFrontEnc();
			m_backStartPos = getBackEnc();
			m_startedClimb = true;
			asyncAdHocLogger.q("startBack: ").q(m_backStartPos).q(" startFront: ").q(m_frontStartPos).go();
    	}
    	if (follower.isFinished() == false){
			follower.execute();
			asyncAdHocLogger.q("left: ").q(follower.getLeftMotorSetting()).q(" right: ").q(follower.getRightMotorSetting()).go();
			setMotorLevels(follower.getLeftMotorSetting(), follower.getRightMotorSetting());
			m_frontHoldSetpoint = follower.getRightPos();
			m_backHoldSetpoint = follower.getLeftPos();
			m_rollHoldSetpoint = follower.getDesiredAngle();
    	}else{
			//HOLD
			setMotorLevels(Constants.kClimbHoldGravityMotorLevel, -Constants.kClimbHoldGravityMotorLevel);
    		//System.out.println("done with path");
    		
    	}
    	
	}
	public void doHalfOn(){
		setMotorLevels(Constants.kClimbHoldGravityMotorLevel, getFrontPIDOutput(m_frontStartPos, m_rollHoldSetpoint,false));
	}
	public void doRest(){
		if(m_startedClimb){
			setMotorLevels(getBackPIDOutput(m_backStartPos, m_rollHoldSetpoint,false), getFrontPIDOutput(m_frontStartPos, m_rollHoldSetpoint,false));
		}
	}

	private void setMotorLevels(double back, double front){
    	if(front < -1){
    		front =-1;
    	}
    	if(front > 1){
    		front = 1;
    	}
    	if(back < -1){
    		back =-1;
    	}
    	if(back > 1){
    		back = 1;
    	}
    	mFrontTalon.set(ControlMode.PercentOutput, front);
    	mBackTalon.set(ControlMode.PercentOutput, back);
	}
	
	VisionMath vm;
	private Climb() {
		// Start all Talons in open loop mode.
		mDriveVictor = new WPI_VictorSPX(RobotMap.CLIMB_DRIVE_VICTOR);

        mFrontTalon = new WPI_TalonSRX(RobotMap.CLIMB_FRONT_TALON);
        mFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mFrontTalon.setSensorPhase(false);
        mFrontTalon.changeMotionControlFramePeriod(5);
        mFrontTalon.setNeutralMode(NeutralMode.Brake);
        
		mFrontTalon.configNeutralDeadband(0.01, 10);
        
        mBackTalon = new WPI_TalonSRX(RobotMap.CLIMB_BACK_TALON);
        mBackTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mBackTalon.changeMotionControlFramePeriod(5);
        mBackTalon.setSensorPhase(true);
        mBackTalon.setNeutralMode(NeutralMode.Brake);
        
		mBackTalon.configNeutralDeadband(0.01, 10);

		mDriveVictor.configNominalOutputForward(0, 10);
		mDriveVictor.configNominalOutputReverse(0, 10);
		mDriveVictor.configPeakOutputForward(1, 10);
		mDriveVictor.configPeakOutputReverse(-1, 10);
		
		mBackTalon.configNominalOutputForward(0, 10);
		mBackTalon.configNominalOutputReverse(0, 10);
		mBackTalon.configPeakOutputForward(1, 10);
		mBackTalon.configPeakOutputReverse(-1, 10);
		
		mFrontTalon.configNominalOutputForward(0, 10);
		mFrontTalon.configNominalOutputReverse(0, 10);
		mFrontTalon.configPeakOutputForward(1, 10);
		mFrontTalon.configPeakOutputReverse(-1, 10);

        // Path Following stuff
        mNavXBoard = new Gyro();


        mDebugOutput = new DebugOutput();
        mCSVWriter = new AsyncStructuredLogger<DebugOutput>("ClimbLog" ,DebugOutput.class);
        
		vm = new VisionMath();

		
		calcPath = new CalcPathToTarget();
        calcPath.calcPath(0.0);
	}
	CalcPathToTarget calcPath;

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    @Override
    public synchronized void stop() {
        
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("front percent output", mFrontTalon.getMotorOutputPercent());
        SmartDashboard.putNumber("back percent output", mBackTalon.getMotorOutputPercent());
        SmartDashboard.putNumber("front position (rotations)", mFrontTalon.getSelectedSensorPosition(0));///4096);
        SmartDashboard.putNumber("back position (rotations)", mBackTalon.getSelectedSensorPosition(0));///4096);
		SmartDashboard.putNumber("gyro pitch", Gyro.getPitch());
		SmartDashboard.putNumber("gyro roll", Gyro.getRoll());
    }
    
    public static class DebugOutput{
    	public long sysTime;
    	public String driveMode;
    	public double gyroRoll;
    	public double frontEncoder;
    	public double backEncoder;
    	public double frontEncoderVel;
    	public double backEncoderVel;
    	public double frontMotorPercent;
    	public double backMotorPercent;
    	public double frontMotorVoltage;
    	public double backMotorVoltage;
    	public double frontMotorCurrent;
    	public double backMotorCurrent;
    	public double driveStick;
    	public double turnStick;
    	public double frontPathPos;
    	public double frontPathVel;
    	public double frontPathAcc;
    	public double backPathPos;
    	public double backPathVel;
    	public double backPathAcc;
    	public double pathHdg;
    	public int pathStep0;
    	public int pathStep1;
    	public double frontBusVoltage;
    	public double backBusVoltage;
    	public double frontTemp;
    	public double backTemp;
    	public boolean frontHasResetOccurred;
    	public boolean backHasResetOccurred;
    	public boolean frontIsSafetyEnabled;
    	public boolean backIsSafetyEnabled;
    }
    
    private void logValues(){
    	mDebugOutput.sysTime = System.nanoTime()-startTime;
    	mDebugOutput.driveMode = mClimbControlState.name();
    	mDebugOutput.gyroRoll = Gyro.getRoll();
    	mDebugOutput.frontEncoder = mFrontTalon.getSelectedSensorPosition(0);
    	mDebugOutput.backEncoder = mBackTalon.getSelectedSensorPosition(0);
    	mDebugOutput.frontEncoderVel = mFrontTalon.getSelectedSensorVelocity(0)/10;//convert from ticks/100ms to ticks/10ms
    	mDebugOutput.backEncoderVel = mBackTalon.getSelectedSensorVelocity(0)/10;
    	mDebugOutput.frontMotorPercent = mFrontTalon.getMotorOutputPercent();
    	mDebugOutput.backMotorPercent = mBackTalon.getMotorOutputPercent();
    	mDebugOutput.frontMotorVoltage = mFrontTalon.getMotorOutputVoltage();
    	mDebugOutput.backMotorVoltage = mBackTalon.getMotorOutputVoltage();
    	mDebugOutput.frontMotorCurrent = mFrontTalon.getOutputCurrent();
    	mDebugOutput.backMotorCurrent = mBackTalon.getOutputCurrent();
    	if(mClimbControlState == ClimbControlState.RESTING){
	    	mDebugOutput.driveStick = OI.getInstance().getDrive();
	    	mDebugOutput.turnStick = OI.getInstance().getTurn();
	    	mDebugOutput.frontPathPos = 0;
	    	mDebugOutput.frontPathVel = 0;
	    	mDebugOutput.frontPathAcc = 0;
	    	mDebugOutput.backPathPos = 0;
	    	mDebugOutput.backPathVel = 0;
	    	mDebugOutput.backPathAcc = 0;
	    	mDebugOutput.pathHdg = 0;
	    	mDebugOutput.pathStep0 = 0;
	    	mDebugOutput.pathStep1 = 0;
    	}else if(follower != null){
			mDebugOutput.driveStick = 0;
	    	mDebugOutput.turnStick = 0;
	    	mDebugOutput.frontPathPos = follower.getRightPos();
	    	mDebugOutput.frontPathVel = follower.getVRight();
	    	mDebugOutput.frontPathAcc = follower.getARight();
	    	mDebugOutput.backPathPos = follower.getLeftPos();
	    	mDebugOutput.backPathVel = follower.getVLeft();
	    	mDebugOutput.backPathAcc = follower.getALeft();
	    	mDebugOutput.pathHdg = follower.getDesiredAngle();
	    	mDebugOutput.pathStep0 = follower.getStep0();
	    	mDebugOutput.pathStep1 = follower.getStep1();
		}
    	mDebugOutput.frontBusVoltage = mFrontTalon.getBusVoltage();
    	mDebugOutput.backBusVoltage = mBackTalon.getBusVoltage();
    	mDebugOutput.frontTemp = mFrontTalon.getTemperature();
    	mDebugOutput.backTemp = mBackTalon.getTemperature();
    	mDebugOutput.frontHasResetOccurred = mFrontTalon.hasResetOccurred();
    	mDebugOutput.backHasResetOccurred = mBackTalon.hasResetOccurred();
    	mDebugOutput.frontIsSafetyEnabled = mFrontTalon.isSafetyEnabled();
    	mDebugOutput.backIsSafetyEnabled = mBackTalon.isSafetyEnabled();
		mCSVWriter.queueData(mDebugOutput);
    }

    public synchronized void resetEncoders() {
    	mFrontTalon.setSelectedSensorPosition(0, 0, 10);
    	mBackTalon.setSelectedSensorPosition(0, 0, 10);
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
    }

    public synchronized Gyro getNavXBoard() {
        return mNavXBoard;
    }

       @Override
    public void writeToLog() {
        //mCSVWriter.write();
    }

@Override
protected void initDefaultCommand() {
	// TODO Auto-generated method stub
	
}       
}