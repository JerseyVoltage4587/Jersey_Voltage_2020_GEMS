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
import frc.robot.util.VisionMath.scoringZone;
import frc.robot.util.AsyncStructuredLogger;
import frc.robot.util.CalcPathToTarget;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

import java.io.File;
import java.io.FileWriter;
import java.util.TimerTask;

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

public class Drive extends Subsystem {
	
	private long startTime;

    private static Drive mInstance = null;
    private DifferentialDrive _drive;
    static int timesInOnStart;
    public static Drive getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Drive.class ) {
    			timesInOnStart=0;
    			mInstance = new Drive();
    		}
    	}
    	return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // used for autonomous driving
		TEST_MODE, // to run the testSubsystem() method once, then return to OPEN_LOOP
		SIMPLE_VISION_DRIVE,
		VISION_DRIVE,
		TURN_ANGLE,
		DRIVE_DISTANCE,
    }
    public DriveControlState getState(){
    	return mDriveControlState;
    }
	private int m_leftEncoderLast,m_rightEncoderLast = 0;
	private double m_leftMotorLevelLast=0;
	private double m_rightMotorLevelLast = 0;
	private long m_lastTime;
    public int getLeftEnc(){
    	return mLeftMaster.getSelectedSensorPosition(0);
    }
    public int getRightEnc(){
    	return mRightMaster.getSelectedSensorPosition(0);
    }
    
    double m_leftPathPos, m_rightPathPos;
    public void setPathPos(double leftPathPos,double rightPathPos){
    	synchronized(Drive.class){
    		m_leftPathPos = leftPathPos * Constants.kInchesPerTic / 12.0;
    		m_rightPathPos = rightPathPos * Constants.kInchesPerTic / 12.0;
    		//System.out.println(m_leftPathPos);
    	}
    }
    public double getLeftPathPos(){
    	synchronized(Drive.class){
    		return m_leftPathPos;
    	}
    }
    public double getRightPathPos(){
    	synchronized(Drive.class){
    		return m_rightPathPos;
    	}
	}
	double m_desiredAngle;
	public void setDesiredAngle(double angle){
    	synchronized(Drive.class){
			m_desiredAngle = angle;
			m_turnDone = false;
    	}
	}

	public void setDesiredArc(double outerRadiusInches, double degrees){
		//+degrees = turn right, -degrees = turn left
		ArcMath am = new ArcMath();
		am.calcArc(outerRadiusInches, degrees);
		m_leftTrajectory = am.getLeftTrajectory();
		m_rightTrajectory = am.getRightTrajectory();
	}

	double m_desiredDist, m_xNext;
	int m_zeroLeftEncoder, m_zeroRightEncoder;
	double m_acceleration;
	public void setDesiredDist(double distInches){
    	synchronized(Drive.class){
			m_desiredDist = distInches / 12.0;//feet
			/*m_zeroLeftEncoder = getLeftEnc();
			m_zeroRightEncoder = getRightEnc();
			m_xNext = 0;*/
			double halfTime = Math.sqrt(m_desiredDist/Constants.kMaxAcceleration);
			int numIntervals = (int)(100*halfTime)+1;
			halfTime = numIntervals * 0.01;
			m_acceleration = m_desiredDist / (halfTime*halfTime);

			Segment[] path = new Segment[(numIntervals*2)+2];
			double yaw = Gyro.getYaw();
			double xMax=0;
			double vMax=0;
			for(int i = 0;i<numIntervals+1;i++){
				double v = m_acceleration * (i * 0.01);
				double x = 0.5 * m_acceleration * ((i * 0.01)*(i * 0.01));
				//System.out.println("x: "+x+" v: "+v+" a: "+m_acceleration+" yaw: "+yaw);
				path[i] = new Segment(0.01, 0, 0, x, v, m_acceleration, 0, yaw);
				xMax = x;
				vMax = v;
			}
			for(int i = 0;i<numIntervals+1;i++){
				double v = vMax - (m_acceleration * (i * 0.01));
				double x = xMax + (vMax*(i*0.01)) - (0.5 * m_acceleration * ((i * 0.01)*(i * 0.01)));
				//System.out.println("x: "+x+" v: "+v+" a: "+m_acceleration+" yaw: "+yaw);
				path[i+numIntervals+1] = new Segment(0.01, 0, 0, x, v, -m_acceleration, 0, yaw);
			}
			m_leftTrajectory = new Trajectory(path);
			m_rightTrajectory = new Trajectory(path);
			for(int i =0;i<m_leftTrajectory.length();i++){
				Segment s = m_leftTrajectory.segments[i];
				//System.out.println("step: "+i+" pos: "+s.position+" vel: "+s.velocity+" acc: "+s.acceleration);
			}
    	}
    }

    // Control states
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;

    // Hardware
    private final WPI_TalonSRX mLeftMaster, mRightMaster, _rightSlave1;
    private final WPI_VictorSPX _leftSlave1, _leftSlave2, _rightSlave2;
    private final Gyro mNavXBoard;

    // Hardware states
    private boolean mIsBrakeMode;

    // Logging
    private DebugOutput mDebugOutput;
    private final AsyncStructuredLogger<DebugOutput> mCSVWriter;
    
	private double mDrive,mTurn;
	
	private boolean mStartingPath=false;
	private String pathFilename;
    public void setPathFilename(String x){
    	pathFilename = x;
	}
	private Trajectory m_leftTrajectory,m_rightTrajectory;

	public void setTrajectories(Trajectory left, Trajectory right) {
    	synchronized (Drive.this) {
			m_leftTrajectory = left;
			m_rightTrajectory = right;
    	}
    }
    
    public void startPath() {
    	//System.out.println("in startPath");
    	synchronized (Drive.this) {
			mStartingPath = true;
    		mDriveControlState = DriveControlState.PATH_FOLLOWING;
    	}
    }
	
	public void startSimpleVisionDrive() {
    	//System.out.println("in startSimpleVisionDrive");
    	synchronized (Drive.this) {
    		mDriveControlState = DriveControlState.SIMPLE_VISION_DRIVE;
    	}
	}
	
	private boolean m_turnDone = false;
	public boolean getTurnDone(){
    	synchronized(Drive.class){
    		return m_turnDone;
    	}
	}
	public void startTurnAngle() {
    	//System.out.println("in startTurnAngle");
    	synchronized (Drive.this) {
			mDriveControlState = DriveControlState.TURN_ANGLE;
			m_turnDone = false;
    	}
	}
	
	private boolean m_driveDone = false;
	public boolean getDriveDone(){
    	synchronized(Drive.class){
    		return m_driveDone;
    	}
	}
	private double m_tHalfway;
	private double m_tStart;
	public void startDriveDist() {
    	//System.out.println("in startDriveDist");
    	synchronized (Drive.this) {
			mDriveControlState = DriveControlState.DRIVE_DISTANCE;
			m_driveDone = false;
			dCount = 0;
			driveMaybeDone = false;
			m_tHalfway = Math.sqrt(m_desiredDist/Constants.kMaxAcceleration);
			m_tStart = System.nanoTime() / 1000000000.0;
    	}
	}
	
	public void startVisionDrive() {
    	//System.out.println("in startVisionDrive");
    	synchronized (Drive.this) {
			mDriveControlState = DriveControlState.VISION_DRIVE;
			notMovingCount = -999;
    	}
	}

	public void setVisionPath(){
		synchronized (Drive.this) {	
			calcPath.calcPath(0.0);
			m_leftTrajectory = calcPath.getLeftTrajectory();
			m_rightTrajectory = calcPath.getRightTrajectory();
    	}
	}

    public void runTest() {
    	//System.out.println("in runTest");
    	synchronized (Drive.this) {
    		mDriveControlState = DriveControlState.TEST_MODE;
    	}
    }
	
	AsyncAdHocLogger asyncAdHocLogger;
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
            	timesInOnStart+=1;
            	SmartDashboard.putNumber("timesInOnStart", timesInOnStart);
            	startTime = System.nanoTime();
                setOpenLoop(DriveSignal.NEUTRAL);
				setBrakeMode(false);
				asyncAdHocLogger = new AsyncAdHocLogger("");
            }
        }

        @Override
        public void onLoop(double timestamp) {
			iCall++;
			if(iCall % 1000 == 0){
        		//System.out.println("onLoop " + iCall + " " + mDriveControlState + " " + mLeftMaster.getControlMode());
        	}
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                case OPEN_LOOP:
					Robot.setVisionLEDs(false);
                	mDrive = OI.getInstance().getDrive();
                	mTurn = OI.getInstance().getTurn();
                	_drive.arcadeDrive(mDrive, mTurn, false);//bool = squaredInputs
                    //mLeftMaster.setInverted(false);
                    invertRightSide(false);
                    _drive.setSafetyEnabled(true);
                    break;
                case PATH_FOLLOWING:
					Robot.setVisionLEDs(true);
					_drive.setSafetyEnabled(false);
					doPathFollowing();
                    break;
                case TEST_MODE:
                	testSubsystem();
                	mDriveControlState = DriveControlState.OPEN_LOOP;
					break;
				case SIMPLE_VISION_DRIVE:
					Robot.setVisionLEDs(true);
					/*mDrive = OI.getInstance().getDrive();
					mTurn = OI.getInstance().getTurn();
					NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
					NetworkTableEntry tx = limelightTable.getEntry("tx");
					double x = tx.getDouble(0.0);

					if(Math.abs(mTurn) < 0.1){
						mTurn = x * Constants.kVisionXToMotorStrong;
					}else{
						mTurn += x * Constants.kVisionXToMotorWeak;
					}

					_drive.arcadeDrive(mDrive, mTurn, false);//bool = squaredInputs
					//mLeftMaster.setInverted(false);
					invertRightSide(false);
					_drive.setSafetyEnabled(true);*/
					_drive.setSafetyEnabled(false);
					doSimpleVisionDrive();
					break;
				case VISION_DRIVE:
					_drive.setSafetyEnabled(false);
					Robot.setVisionLEDs(true);
					doVisionDrive();
					break;
				case TURN_ANGLE:
					doTurnAngle();
					break;
				case DRIVE_DISTANCE:
					doDriveDistance();
					break;
                default:
                    //System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
                }
			}
			
			m_lastTime = System.nanoTime();
        	logValues();
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            mCSVWriter.flush();
        }
	};
	double m_lastLeftVel=0,m_lastRightVel=0;

	double lastDesiredHdg=0;
	private void doVisionDrive(){
		NetworkTable limelightTable;
		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

		NetworkTableEntry tv = limelightTable.getEntry("tv");
		double v = tv.getDouble(0.0);
		NetworkTableEntry tx = limelightTable.getEntry("tx");
		double x = tx.getDouble(0.0);

		vm.findRobotPos();
		double xRobot = vm.getRobotX();
		double yRobot = vm.getRobotY();
		x = Math.atan(yRobot/xRobot)*180.0/Math.PI;
		SmartDashboard.putNumber("visionAngle",x);
		double distRobot = Math.sqrt((xRobot*xRobot)+(yRobot*yRobot));
		distRobot -= Constants.kVisionDistToStop;
		if(xRobot < 900 && yRobot < 900){
			//have pic
			lastDesiredDist = distRobot;
			lastDesiredHdg = Gyro.getYaw() - x;
			distMoved = 0;
			//notMovingCount = 0;
			m_leftEncoderLast = getLeftEnc();
			m_rightEncoderLast = getRightEnc();
		}else{
			//don't have pic
			x=0;
		}
		distRobot = lastDesiredDist - distMoved;//no change if we have pic

		double angleError = Gyro.getYaw() - lastDesiredHdg;

		double left = 0;
		double right = 0;

		left = (distRobot * Constants.kVisionDistToMotor) + (x * Constants.kVisionXToMotor);// * (distRobot / Constants.kVisionDistFullTurn));//OI.getInstance().getDrive()*0.5 + (angleError * Constants.kVisionXToMotor);
		right = (distRobot * Constants.kVisionDistToMotor) - (x * Constants.kVisionXToMotor);// * (distRobot / Constants.kVisionDistFullTurn));//(distRobot * Constants.kVisionDistToMotor) - (angleError * Constants.kVisionXToMotor);
		left*=0.75;
		right*=0.75;
		if(Math.abs(left)<Constants.kVisionMinMotorLevel && Math.abs(right) < Constants.kVisionMinMotorLevel){
			left = Constants.kVisionMinMotorLevel * Math.signum(left);
			right = Constants.kVisionMinMotorLevel * Math.signum(right);
		}

		if(Math.abs(yRobot) < Constants.kVisionYTolerance){
			boolean done = false;
			if(Math.abs(getLeftVel()) < 100 && Math.abs(getRightVel()) < 100){
				notMovingCount++;
			}else{
				notMovingCount = 0;
			}
			
			if(notMovingCount>5){
				done = true;
			}
			if(Math.abs(xRobot) <= Constants.kVisionToleranceToStop){
				done = true;
			}
			
			if(done){
				setOpenLoop(DriveSignal.NEUTRAL);
				left=0;
				right=0;
				limelightTable.getEntry("ledMode").forceSetNumber(2);
					
				m_timer.schedule(new FlashTimer(),2000);
			}
		}
		/*
		if(Math.abs(x) < Constants.kVisionXTolerance){
			boolean done = false;
			if(v == 0.0){
				double deltaDist = (getLeftEnc() - m_leftEncoderLast) * Constants.kInchesPerTic;
				if(Math.abs(deltaDist) < 0.1){
					notMovingCount++;
				}else{
					notMovingCount = 0;
				}
				distMoved += deltaDist;
				asyncAdHocLogger.q("distMoved: ").q(distMoved).q(" lastDesiredDist: ").q(lastDesiredDist).go();
				if(distMoved >= (lastDesiredDist - Constants.kVisionToleranceToStop)){
					done = true;
				}
				if(notMovingCount>5){
					done = true;
				}
			}else{
				double deltaDist = (getLeftEnc() - m_leftEncoderLast) * Constants.kInchesPerTic;
				SmartDashboard.putNumber("deltaDist",deltaDist);
				SmartDashboard.putNumber("notMovingCt",notMovingCount);
				if(Math.abs(deltaDist) < 0.1){
					notMovingCount++;
				}else{
					notMovingCount = 0;
				}
				if(notMovingCount>5){
					done = true;
				}
				if(Math.abs(xRobot) <= Constants.kVisionToleranceToStop){
					done = true;
				}
			}
			if(done){
				setOpenLoop(DriveSignal.NEUTRAL);
				left=0;
				right=0;
				limelightTable.getEntry("ledMode").forceSetNumber(2);
					
				m_timer.schedule(new FlashTimer(),2000);
			}
		}
		*/
		
		asyncAdHocLogger.q("left: ").q(left).q(" leftLast: ").q(m_leftMotorLevelLast).q(" right: ").q(right).q(" rightLast: ").q(m_rightMotorLevelLast).go();
		double leftRightRatio = Math.abs(left/right);
		if((left - m_leftMotorLevelLast)>Constants.kVisionMotorLevelAccMax){
			left = m_leftMotorLevelLast + (Constants.kVisionMotorLevelAccMax * leftRightRatio);
		}
		if((right - m_rightMotorLevelLast)>Constants.kVisionMotorLevelAccMax){
			right = m_rightMotorLevelLast + (Constants.kVisionMotorLevelAccMax / leftRightRatio);
		}

		setMotorLevels(left, -right);
	}

	public class FlashTimer extends TimerTask{
		public void run(){
			NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
			limelightTable.getEntry("ledMode").forceSetNumber(0);
		}
	}

	double lastDesiredDist;
	double distMoved;
	int notMovingCount;
	private void doSimpleVisionDrive(){
		NetworkTable limelightTable;
		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

		NetworkTableEntry tv = limelightTable.getEntry("tv");
		double v = tv.getDouble(0.0);
		NetworkTableEntry tx = limelightTable.getEntry("tx");
		double x = tx.getDouble(0.0);

		vm.findRobotPos();
		double xRobot = vm.getRobotX();
		double yRobot = vm.getRobotY();
		double distRobot = Math.sqrt((xRobot*xRobot)+(yRobot*yRobot));
		distRobot -= Constants.kVisionDistToStop;
		if(xRobot < 900 && yRobot < 900){
			//have pic
			lastDesiredDist = distRobot;
			distMoved = 0;
			notMovingCount = 0;
		}else{
			//don't have pic
			x=0;
		}
		distRobot = lastDesiredDist - distMoved;//no change if we have pic

		double left = 0;
		double right = 0;

		left = OI.getInstance().getDrive()*0.5 + (x * Constants.kVisionXToMotor);//(distRobot * Constants.kVisionDistToMotor) + (x * Constants.kVisionXToMotor);
		right = OI.getInstance().getDrive()*0.5 - (x * Constants.kVisionXToMotor);//(distRobot * Constants.kVisionDistToMotor) - (x * Constants.kVisionXToMotor);
		if(Math.abs(left)<Constants.kVisionMinMotorLevel && Math.abs(right) < Constants.kVisionMinMotorLevel){
			left = Constants.kVisionMinMotorLevel * Math.signum(left);
			right = Constants.kVisionMinMotorLevel * Math.signum(right);
		}

		if(Math.abs(x) < Constants.kVisionXTolerance){
			boolean done = false;
			if(v == 0.0){
				double deltaDist = (getLeftEnc() - m_leftEncoderLast) * Constants.kInchesPerTic;
				if(Math.abs(deltaDist) < 0.01){notMovingCount++;}
				distMoved += deltaDist;
				asyncAdHocLogger.q("distMoved: ").q(distMoved).q(" lastDesiredDist: ").q(lastDesiredDist).go();
				if(distMoved >= (lastDesiredDist - Constants.kVisionDistToStop)){
					done = true;
				}
				if(notMovingCount>5){
					done = true;
				}
			}else{
				if(Math.abs(xRobot) <= Constants.kVisionDistToStop){
					done = true;
				}
			}
			if(done){
				setOpenLoop(DriveSignal.NEUTRAL);
				left=0;
				right=0;
				limelightTable.getEntry("ledMode").forceSetNumber(2);
					
				m_timer.schedule(new FlashTimer(),2000);
			}
		}
		
		asyncAdHocLogger.q("left: ").q(left).q(" leftLast: ").q(m_leftMotorLevelLast).q(" right: ").q(right).q(" rightLast: ").q(m_rightMotorLevelLast).go();
		double leftRightRatio = Math.abs(left/right);
		if((left - m_leftMotorLevelLast)>Constants.kVisionMotorLevelAccMax){
			left = m_leftMotorLevelLast + (Constants.kVisionMotorLevelAccMax * leftRightRatio);
		}
		if((right - m_rightMotorLevelLast)>Constants.kVisionMotorLevelAccMax){
			right = m_rightMotorLevelLast + (Constants.kVisionMotorLevelAccMax / leftRightRatio);
		}

		setMotorLevels(left, -right);
	}


	private double lastG = 0;
	private double lastTime = 0;
	private boolean turnMaybeDone = false;
	private int count = 0;
	private void doTurnAngle(){
		double g = Gyro.getYaw();
		double time = System.nanoTime();
		double motorLevel = 0;

		if(turnMaybeDone == true){
			count+=1;
		}else{
			count = 0;
		}
		if(count > 15){
			m_turnDone = true;
		}

		//turn component
		if(lastG != 0.0 && lastTime != 0.0){
			double deltaAngle = m_desiredAngle - g;

			double pComponent = deltaAngle * Constants.kTurnToAngleKp;
			double dComponent = (deltaAngle - (m_desiredAngle - lastG)) * Constants.kTurnToAngleKd;
			double fComponent = 0;
			if(Math.abs(deltaAngle) < 5){
				fComponent = 1.75 * Constants.kTurnToAngleKf;
			}else if(Math.abs(deltaAngle) < 25){
				fComponent = 2.0 * Constants.kTurnToAngleKf;
			}else if(Math.abs(deltaAngle) < 40){
				fComponent = 3.0 * Constants.kTurnToAngleKf;
			}else{
				fComponent = 6.0 * Constants.kTurnToAngleKf;
			}
			if(deltaAngle < 0){
				fComponent *= -1;
			}

			motorLevel = fComponent;
								
			if(Math.abs(deltaAngle)<Constants.kTurnToAngleTolerance){
				//turn done
				turnMaybeDone = true;
			}else{
				turnMaybeDone = false;
			}
		}
		lastG = g;
		lastTime = time;

		if(m_turnDone){
			//done!
			motorLevel = 0.0;
		}
		//System.out.print(motorLevel);
		setMotorLevels(motorLevel, motorLevel);
	}

	double m_desiredVel;
	boolean driveMaybeDone=false;
	int dCount = 0;
	private void doDriveDistance(){
		if(driveMaybeDone){
			dCount += 1;
		}else{
			dCount = 0;
		}
		if(dCount > 15){
			m_driveDone = true;
		}

		double tNow = (System.nanoTime() / 1000000000.0);// - m_tStart;
		if(tNow < m_tHalfway){
			m_desiredVel = Constants.kMaxAcceleration * tNow;
			m_xNext = 0.5 * Constants.kMaxAcceleration * tNow * tNow;
		}else{
			m_desiredVel = Constants.kMaxAcceleration * ((m_tHalfway*2) - tNow);
			m_xNext = m_desiredDist - (0.5 * m_desiredVel * ((m_tHalfway*2) - tNow));
		}
		//System.out.println("tNow: "+tNow+" m_tHalfway: "+m_tHalfway+" m_tStart: "+m_tStart);

		double currentDistLeft = (getLeftEnc() - m_zeroLeftEncoder) * Constants.kInchesPerTic / 12.0;//ft
		double currentDistRight = (getRightEnc() - m_zeroRightEncoder) * Constants.kInchesPerTic / 12.0;//ft
		double averageDist = (currentDistLeft + currentDistRight) / 2.0;
		/*System.out.println("avgDist: "+averageDist+" desiredDist: "+m_desiredDist);
		if(averageDist < m_desiredDist/2){
			m_desiredVel += Constants.kMaxAcceleration * 0.01;
			if(m_desiredVel > Constants.kMaxFeetPerSecond){
				m_desiredVel = Constants.kMaxFeetPerSecond;
			}
		}else{
			m_desiredVel -= Constants.kMaxAcceleration * 0.01;
		}
		*/
		if(Math.abs(averageDist - m_desiredDist)<Constants.kDriveDistTolerance){
			driveMaybeDone = true;
		}
		if(m_driveDone){
			m_desiredVel = 0.0;
			m_xNext = m_desiredDist;
		}

		double mLeft = calculateDriveMotorLevel(currentDistLeft);
		double mRight = -calculateDriveMotorLevel(currentDistRight);
		//System.out.println("vel: "+m_desiredVel+" left: "+mLeft+" right: "+mRight);
		//m_xNext += m_desiredVel * 0.01;
		setMotorLevels(mLeft, mRight);
	}

	double calculateDriveMotorLevel(double currentDist){
		return m_desiredVel * Constants.kDriveDistKv + (m_xNext - currentDist) * Constants.kDriveDistKp;
	}

	PathFollower follower = null;
	
	private void doPathFollowing(){
    	if (mStartingPath) {
    		mStartingPath = false;
			setPathPos(0,0);
    		follower = new PathFollower(m_leftTrajectory,m_rightTrajectory,true);
    		//System.out.println("follower != null");
    		follower.initialize();
    	}
    	if (follower.isFinished() == false){
    		//System.out.println("doing path "+System.nanoTime()/1000000000.0);
			follower.execute();
			//System.out.println("after execute: "+System.nanoTime()/1000000000.0);
    		setMotorLevels(follower.getLeftMotorSetting(), follower.getRightMotorSetting());
    	}else{
    		//System.out.println("done with path");
    		setOpenLoop(DriveSignal.NEUTRAL);
    	}
    	
	}

	private void setMotorLevels(double left, double right){
    	if(left < -1){
    		left =-1;
    	}
    	if(left > 1){
    		left = 1;
    	}
    	if(right < -1){
    		right =-1;
    	}
    	if(right > 1){
    		right = 1;
		}
		m_leftMotorLevelLast = left;
		m_rightMotorLevelLast = -right;
    	mLeftMaster.set(ControlMode.PercentOutput, left);
    	mRightMaster.set(ControlMode.PercentOutput, right);
    }
	VisionMath vm;
	private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_TALON);
        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mLeftMaster.setSensorPhase(true);
        mLeftMaster.changeMotionControlFramePeriod(5);
        mLeftMaster.setNeutralMode(NeutralMode.Brake);
        
		mLeftMaster.configNeutralDeadband(0.01, 10);
		mLeftMaster.configMotionProfileTrajectoryPeriod(10, 10); 

        _leftSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_1);
        _leftSlave1.follow(mLeftMaster);
        _leftSlave1.setNeutralMode(NeutralMode.Brake);
        _leftSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_2);
        _leftSlave2.follow(mLeftMaster);
        _leftSlave2.setNeutralMode(NeutralMode.Brake);
        
        mRightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_TALON);
        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        mRightMaster.changeMotionControlFramePeriod(5);
        mRightMaster.setSensorPhase(false);
        mRightMaster.setNeutralMode(NeutralMode.Brake);
        
		mRightMaster.configNeutralDeadband(0.01, 10);

		mRightMaster.configMotionProfileTrajectoryPeriod(10, 10); 
		
        //_rightSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_1);
        _rightSlave1 = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_TALON_SLAVE);
        _rightSlave1.follow(mRightMaster);
        _rightSlave1.setNeutralMode(NeutralMode.Brake);
        _rightSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_2);
        _rightSlave2.follow(mRightMaster);
        _rightSlave2.setNeutralMode(NeutralMode.Brake);

		mRightMaster.config_kF(0, 0.275, 10);
		mRightMaster.config_kP(0, 0.25, 10);
		mRightMaster.config_kI(0, 0.0, 10);
		mRightMaster.config_kD(0, 0.0, 10);
		
		mLeftMaster.config_kF(0, 0.275, 10);
		mLeftMaster.config_kP(0, 0.25 , 10);
		mLeftMaster.config_kI(0, 0.0, 10);
		mLeftMaster.config_kD(0, 0.0, 10);
		
		mRightMaster.configNominalOutputForward(0, 10);
		mRightMaster.configNominalOutputReverse(0, 10);
		mRightMaster.configPeakOutputForward(1, 10);
		mRightMaster.configPeakOutputReverse(-1, 10);
		
		mLeftMaster.configNominalOutputForward(0, 10);
		mLeftMaster.configNominalOutputReverse(0, 10);
		mLeftMaster.configPeakOutputForward(1, 10);
		mLeftMaster.configPeakOutputReverse(-1, 10);

		mLeftMaster.configMotionCruiseVelocity(800, 10);
		mLeftMaster.configMotionAcceleration(400, 10);

		mRightMaster.configMotionCruiseVelocity(800, 10);
		mRightMaster.configMotionAcceleration(400, 10);
        
        setOpenLoop(DriveSignal.NEUTRAL);

        // Path Following stuff
        mNavXBoard = new Gyro();

        // Force a CAN message across.
        mIsBrakeMode = true;
		setBrakeMode(false);

		m_timer = new java.util.Timer();
		
		/*
		_leftSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_1);
		_leftSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_VICTOR_2);
		_rightSlave1 = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_TALON_SLAVE);
		_rightSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_VICTOR_2);
		WPI_TalonSRX testTalon1 = new WPI_TalonSRX(RobotMap.CLIMB_FRONT_TALON);
		WPI_TalonSRX testTalon2 = new WPI_TalonSRX(RobotMap.CLIMB_BACK_TALON);
		WPI_TalonSRX testTalon3 = new WPI_TalonSRX(40);
		WPI_TalonSRX testTalon4 = new WPI_TalonSRX(50);
		WPI_TalonSRX testTalon5 = new WPI_TalonSRX(51);
		WPI_VictorSPX testVictor1 = new WPI_VictorSPX(RobotMap.CLIMB_DRIVE_VICTOR);
		WPI_VictorSPX testVictor2 = new WPI_VictorSPX(41);
		WPI_VictorSPX testVictor3 = new WPI_VictorSPX(30);
		WPI_VictorSPX testVictor4 = new WPI_VictorSPX(31);

		_leftSlave1.follow(mLeftMaster);
		_leftSlave2.follow(mLeftMaster);
		_rightSlave1.follow(mLeftMaster);
		_rightSlave2.follow(mLeftMaster);
		testTalon1.follow(mLeftMaster);
		testTalon2.follow(mLeftMaster);
		testTalon3.follow(mLeftMaster);
		testTalon4.follow(mLeftMaster);
		testTalon5.follow(mLeftMaster);
		testVictor1.follow(mLeftMaster);
		testVictor2.follow(mLeftMaster);
		testVictor3.follow(mLeftMaster);
		testVictor4.follow(mLeftMaster);
*/
        mDebugOutput = new DebugOutput();
        mCSVWriter = new AsyncStructuredLogger<DebugOutput>("DriveLog" ,DebugOutput.class);
        
		vm = new VisionMath();

        _drive = new DifferentialDrive(mLeftMaster, mRightMaster);
		
		calcPath = new CalcPathToTarget();
	}
	java.util.Timer m_timer = null;
	CalcPathToTarget calcPath;

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
	}
	
	public double getLeftVel(){
		return mLeftMaster.getSelectedSensorVelocity();
	}
	public double getRightVel(){
		return mRightMaster.getSelectedSensorVelocity();
	}

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
            mRightMaster.set(ControlMode.PercentOutput, -signal.getRight());
            mDriveControlState = DriveControlState.OPEN_LOOP;
            setBrakeMode(false);
        }
        else
        {
	        mRightMaster.set(-signal.getRight());
	        mLeftMaster.set(signal.getLeft());
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
//            mRightMaster.enableBrakeMode(on);
//            mRightSlave.enableBrakeMode(on);
//            mLeftMaster.enableBrakeMode(on);
//            mLeftSlave.enableBrakeMode(on);
        }
        
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("left percent output", mLeftMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("right percent output", mRightMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("left position (rotations)", mLeftMaster.getSelectedSensorPosition(0));///4096);
        SmartDashboard.putNumber("right position (rotations)", mRightMaster.getSelectedSensorPosition(0));///4096);
		SmartDashboard.putNumber("gyro pos", Gyro.getYaw());
		SmartDashboard.putNumber("left vel", mLeftMaster.getSelectedSensorVelocity());
		SmartDashboard.putNumber("right vel", mRightMaster.getSelectedSensorVelocity());
    }
    
    public static class DebugOutput{
    	public long sysTime;
    	public String driveMode;
    	public double gyroYaw;
    	public double leftEncoder;
    	public double rightEncoder;
    	public double leftEncoderVel;
    	public double rightEncoderVel;
    	public double leftMotorPercent;
    	public double rightMotorPercent;
    	public double leftMotorVoltage;
    	public double rightMotorVoltage;
    	public double leftMotorCurrent;
    	public double rightMotorCurrent;
    	public double driveStick;
    	public double turnStick;
    	public double leftPathPos;
    	public double leftPathVel;
    	public double leftPathAcc;
    	public double rightPathPos;
    	public double rightPathVel;
    	public double rightPathAcc;
    	public double pathHdg;
    	public int pathStep0;
    	public int pathStep1;
    	public double leftBusVoltage;
    	public double rightBusVoltage;
    	public double leftTemp;
    	public double rightTemp;
    	public boolean leftHasResetOccurred;
    	public boolean rightHasResetOccurred;
    	public boolean leftIsSafetyEnabled;
    	public boolean rightIsSafetyEnabled;
    }
    
    private void logValues(){
    	mDebugOutput.sysTime = System.nanoTime()-startTime;
    	mDebugOutput.driveMode = mDriveControlState.name();
    	mDebugOutput.gyroYaw = Gyro.getYaw();
    	mDebugOutput.leftEncoder = mLeftMaster.getSelectedSensorPosition(0);
    	mDebugOutput.rightEncoder = mRightMaster.getSelectedSensorPosition(0);
    	mDebugOutput.leftEncoderVel = mLeftMaster.getSelectedSensorVelocity(0)/10;//convert from ticks/100ms to ticks/10ms
    	mDebugOutput.rightEncoderVel = mRightMaster.getSelectedSensorVelocity(0)/10;
    	mDebugOutput.leftMotorPercent = mLeftMaster.getMotorOutputPercent();
    	mDebugOutput.rightMotorPercent = mRightMaster.getMotorOutputPercent();
    	mDebugOutput.leftMotorVoltage = mLeftMaster.getMotorOutputVoltage();
    	mDebugOutput.rightMotorVoltage = mRightMaster.getMotorOutputVoltage();
    	mDebugOutput.leftMotorCurrent = mLeftMaster.getOutputCurrent();
    	mDebugOutput.rightMotorCurrent = mRightMaster.getOutputCurrent();
    	if(mDriveControlState == DriveControlState.OPEN_LOOP){
	    	mDebugOutput.driveStick = OI.getInstance().getDrive();
	    	mDebugOutput.turnStick = OI.getInstance().getTurn();
	    	mDebugOutput.leftPathPos = 0;
	    	mDebugOutput.leftPathVel = 0;
	    	mDebugOutput.leftPathAcc = 0;
	    	mDebugOutput.rightPathPos = 0;
	    	mDebugOutput.rightPathVel = 0;
	    	mDebugOutput.rightPathAcc = 0;
	    	mDebugOutput.pathHdg = 0;
	    	mDebugOutput.pathStep0 = 0;
	    	mDebugOutput.pathStep1 = 0;
    	}
    	mDebugOutput.leftBusVoltage = mLeftMaster.getBusVoltage();
    	mDebugOutput.rightBusVoltage = mRightMaster.getBusVoltage();
    	mDebugOutput.leftTemp = mLeftMaster.getTemperature();
    	mDebugOutput.rightTemp = mRightMaster.getTemperature();
    	mDebugOutput.leftHasResetOccurred = mLeftMaster.hasResetOccurred();
    	mDebugOutput.rightHasResetOccurred = mRightMaster.hasResetOccurred();
    	mDebugOutput.leftIsSafetyEnabled = mLeftMaster.isSafetyEnabled();
    	mDebugOutput.rightIsSafetyEnabled = mRightMaster.isSafetyEnabled();
		mCSVWriter.queueData(mDebugOutput);
    }

    public synchronized void resetEncoders() {
    	mLeftMaster.setSelectedSensorPosition(0, 0, 10);
    	mRightMaster.setSelectedSensorPosition(0, 0, 10);
    }
    
    private void invertRightSide(boolean x){
    	mRightMaster.setInverted(x);
    	_rightSlave1.setInverted(x);
    	_rightSlave2.setInverted(x);
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

   public boolean testSubsystem(){

	   boolean all_ok = false;
	   try{
		   FileWriter w = null;//new FileWriter(new File("/home/lvuser/testLog.csv"));
	   all_ok = true;

	   _drive.setSafetyEnabled(false);
	   
	   mLeftMaster.setNeutralMode(NeutralMode.Coast);
	   _leftSlave1.setNeutralMode(NeutralMode.Coast);
	   _leftSlave2.setNeutralMode(NeutralMode.Coast);
	   mRightMaster.setNeutralMode(NeutralMode.Coast);
	   _rightSlave1.setNeutralMode(NeutralMode.Coast);
	   _rightSlave2.setNeutralMode(NeutralMode.Coast);

	   mLeftMaster.set(ControlMode.PercentOutput,0.0);
	   _leftSlave1.set(ControlMode.PercentOutput,0.0);
	   _leftSlave2.set(ControlMode.PercentOutput,0.0);
	   mRightMaster.set(ControlMode.PercentOutput,0.0);
	   _rightSlave1.set(ControlMode.PercentOutput,0.0);
	   _rightSlave2.set(ControlMode.PercentOutput,0.0);

	   //System.out.println("Right Master...");
	   //mRightMaster.setSelectedSensorPosition(0, 0, 10);
	   //mRightMaster.set(-0.5);
	   Timer.delay(0.5);
	   //double rightMaster_end = mRightMaster.getSelectedSensorPosition(0);
	   //double rightMaster_vel = mRightMaster.getSelectedSensorVelocity(0);
	   //mRightMaster.set(0);
	   
	   //System.out.println("... end="+rightMaster_end+",vel="+rightMaster_vel);
	   //Timer.delay(3.0);
	   
	   //System.out.println("Right Master...");
	   mRightMaster.setSelectedSensorPosition(0, 0, 10);
	   mRightMaster.set(-0.5);
	   Timer.delay(3.0);
	   double rightMaster_end = mRightMaster.getSelectedSensorPosition(0);
	   double rightMaster_vel = mRightMaster.getSelectedSensorVelocity(0);
	   mRightMaster.set(0);
	   
	   w.write("rightMaster,"+rightMaster_end+","+rightMaster_vel+"\n");
	   //System.out.println("... end="+rightMaster_end+",vel="+rightMaster_vel);
	   //System.out.println("rightMasterDistError="+(rightMaster_end-Constants.kTestDistTarget)+" rightMasterVelError="+(rightMaster_vel-Constants.kTestVelTarget));
	   //System.out.println("passed="+((Math.abs(rightMaster_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(rightMaster_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   //System.out.println("Right Slave 1...");
	   mRightMaster.setSelectedSensorPosition(0, 0, 10);
	   _rightSlave1.set(-0.5);
	   Timer.delay(3.0);
	   double rightSlave1_end = mRightMaster.getSelectedSensorPosition(0);
	   double rightSlave1_vel = mRightMaster.getSelectedSensorVelocity(0);
	   _rightSlave1.set(0);

	   w.write("_rightSlave1,"+rightSlave1_end+","+rightSlave1_vel+"\n");
	   //System.out.println("... end="+rightSlave1_end+",vel="+rightSlave1_vel);
	   //System.out.println("_rightSlave1DistError="+(rightSlave1_end-Constants.kTestDistTarget)+" _rightSlave1VelError="+(rightSlave1_vel-Constants.kTestVelTarget));
	   //System.out.println("passed="+((Math.abs(rightSlave1_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(rightSlave1_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   //System.out.println("Right Slave 2...");
	   mRightMaster.setSelectedSensorPosition(0, 0, 10);
	   _rightSlave2.set(-0.5);
	   Timer.delay(3.0);
	   double rightSlave2_end = mRightMaster.getSelectedSensorPosition(0);
	   double rightSlave2_vel = mRightMaster.getSelectedSensorVelocity(0);
	   _rightSlave2.set(0);

	   w.write("_rightSlave2,"+rightSlave2_end+","+rightSlave2_vel+"\n");
	   //System.out.println("... end="+rightSlave2_end+",vel="+rightSlave2_vel);
	   //System.out.println("_rightSlave2DistError="+(rightSlave2_end-Constants.kTestDistTarget)+" _rightSlave2VelError="+(rightSlave2_vel-Constants.kTestVelTarget));
	   //System.out.println("passed="+((Math.abs(rightSlave2_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(rightSlave2_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   //System.out.println("Left Master...");
	   mLeftMaster.setSelectedSensorPosition(0, 0, 10);
	   mLeftMaster.set(0.5);
	   Timer.delay(3.0);
	   double leftMaster_end = mLeftMaster.getSelectedSensorPosition(0);
	   double leftMaster_vel = mLeftMaster.getSelectedSensorVelocity(0);
	   mLeftMaster.set(0);

	   w.write("leftMaster,"+leftMaster_end+","+leftMaster_vel+"\n");
	   //System.out.println("... end="+leftMaster_end+",vel="+leftMaster_vel);
	   //System.out.println("leftMasterDistError="+(leftMaster_end-Constants.kTestDistTarget)+" leftMasterVelError="+(leftMaster_vel-Constants.kTestVelTarget));
	   //System.out.println("passed="+((Math.abs(leftMaster_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(leftMaster_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   //System.out.println("Left Slave 1...");
	   mLeftMaster.setSelectedSensorPosition(0, 0, 10);
	   _leftSlave1.set(0.5);
	   Timer.delay(3.0);
	   double leftSlave1_end = mLeftMaster.getSelectedSensorPosition(0);
	   double leftSlave1_vel = mLeftMaster.getSelectedSensorVelocity(0);
	   _leftSlave1.set(0);

	   w.write("_leftSlave1,"+leftSlave1_end+","+leftSlave1_vel+"\n");
	   //System.out.println("... end="+leftSlave1_end+",vel="+leftSlave1_vel);
	   //System.out.println("leftSlave1DistError="+(leftSlave1_end-Constants.kTestDistTarget)+" leftSlave1VelError="+(leftSlave1_vel-Constants.kTestVelTarget));
	   //System.out.println("passed="+((Math.abs(leftSlave1_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(leftSlave1_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   Timer.delay(3.0);
	   
	   //System.out.println("Left Slave 2...");
	   mLeftMaster.setSelectedSensorPosition(0, 0, 10);
	   _leftSlave2.set(0.5);
	   Timer.delay(3.0);
	   double leftSlave2_end = mLeftMaster.getSelectedSensorPosition(0);
	   double leftSlave2_vel = mLeftMaster.getSelectedSensorVelocity(0);
	   _leftSlave2.set(0);

	   w.write("_leftSlave2,"+leftSlave2_end+","+leftSlave2_vel+"\n");
	   //System.out.println("... end="+leftSlave2_end+",vel="+leftSlave2_vel);
	   //System.out.println("leftSlave2DistError="+(leftSlave2_end-Constants.kTestDistTarget)+" leftSlave2VelError="+(leftSlave2_vel-Constants.kTestVelTarget));
	   //System.out.println("passed="+((Math.abs(leftSlave2_end-Constants.kTestDistTarget)<Constants.kTestDistError)&&(Math.abs(leftSlave2_vel-Constants.kTestVelTarget)<Constants.kTestVelError)));
	   
	   
	   
	   _leftSlave1.follow(mLeftMaster);
	   _leftSlave2.follow(mLeftMaster);
	   _rightSlave1.follow(mRightMaster);
	   _rightSlave2.follow(mRightMaster);
	   
	   mLeftMaster.setNeutralMode(NeutralMode.Brake);
	   _leftSlave1.setNeutralMode(NeutralMode.Brake);
	   _leftSlave2.setNeutralMode(NeutralMode.Brake);
	   mRightMaster.setNeutralMode(NeutralMode.Brake);
	   _rightSlave1.setNeutralMode(NeutralMode.Brake);
	   _rightSlave2.setNeutralMode(NeutralMode.Brake);

	   // Let the onLoop() method enable safety mode again...
	   w.close();
	   }catch(Exception e){}
	   return all_ok;
   }

@Override
protected void initDefaultCommand() {
	// TODO Auto-generated method stub
	
}       
}