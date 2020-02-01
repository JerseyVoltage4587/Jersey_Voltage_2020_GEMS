package frc.robot.paths;

import java.io.BufferedReader;
import java.io.FileWriter;
import java.io.IOException;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.util.Gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import frc.robot.util.AsyncAdHocLogger;
import frc.robot.util.AsyncStructuredLogger;

public class PathFollower {

	BufferedReader m_bufferedReader;
	boolean quit;
	int m_startEncoderLeft;
	int m_startEncoderRight;
	double m_startAngle;
	double m_startTime;
	double Ka = Constants.kPathFollowKa;
	double Kv = Constants.kPathFollowKv;
	double Kp = Constants.kPathFollowKp;
	double Kg = Constants.kPathFollowKg;
	
	//FileWriter m_logWriter=null;
    private DebugOutput mDebugOutput;
	private AsyncStructuredLogger<DebugOutput> mCSVWriter;
	private AsyncAdHocLogger mAdHocLogger;
	String m_namePath;
	Trajectory m_leftPath;
	Trajectory m_rightPath;

	double aLeft;
	public double getALeft(){
		return aLeft;
	}
	double vLeft;
	public double getVLeft(){
		return vLeft;
	}
	double xLeft;
	public double getXLeft(){
		return xLeft;
	}
	double aRight;
	public double getARight(){
		return aRight;
	}
	double vRight;
	public double getVRight(){
		return vRight;
	}
	double xRight;
	public double getXRight(){
		return xRight;
	}
	int step0;
	public int getStep0(){
		return step0;
	}
	int step1;
	public int getStep1(){
		return step1;
	}
	double desiredAngle;
	public double getDesiredAngle(){
		return desiredAngle;
	}
	
	double m_finalPositionRight;
	public double getFinalPositionRight(){
		return m_finalPositionRight;
	}
	double m_finalPositionLeft;
	public double getFinalPositionLeft(){
		return m_finalPositionLeft;
	}
	double m_leftMotorSetting;
	public double getLeftMotorSetting(){
		return m_leftMotorSetting;
	}
	double m_rightMotorSetting;
	public double getRightMotorSetting(){
		return m_rightMotorSetting;
	}
	double m_leftPos;
	public double getLeftPos(){
		synchronized(Drive.class){
			return m_leftPos;
		}
	}
	double m_rightPos;
	public double getRightPos(){
		synchronized(Drive.class){
			return m_rightPos;
		}
	}
	
	private void setMotorLevels(double l, double r){
		m_leftMotorSetting = l;
		m_rightMotorSetting = r;
	}
	boolean m_useYaw;
    public PathFollower(Trajectory leftPath, 
			Trajectory rightPath, boolean useYaw) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	m_leftPath = leftPath;
		m_rightPath = rightPath;
		m_useYaw = useYaw;
    }

	double m_inchesPerTic;
    // Called just before this Command runs the first time
    public void initialize() {
    	quit = false;
		mDebugOutput = new DebugOutput();
		mAdHocLogger = new AsyncAdHocLogger("");
		mCSVWriter = new AsyncStructuredLogger<DebugOutput>("PathLog",false,DebugOutput.class);
		
		if(m_useYaw){
			m_startAngle = Gyro.getYaw();
			m_inchesPerTic = Constants.kInchesPerTic;
			Ka = Constants.kPathFollowKa;
			Kv = Constants.kPathFollowKv;
			Kp = Constants.kPathFollowKp;
			Kg = Constants.kPathFollowKg;
			m_startEncoderLeft = Robot.getDrive().getLeftEnc();
			m_startEncoderRight = Robot.getDrive().getRightEnc();
		}else{
			m_startAngle = Gyro.getRoll();
			m_inchesPerTic = Constants.kClimbInchesPerTic;
			Ka = Constants.kClimbKa;
			Kv = Constants.kClimbKv;
			Kp = Constants.kClimbKp;
			Kg = Constants.kClimbKg;
			m_startEncoderLeft = Robot.getClimb().getBackEnc();
			m_startEncoderRight = Robot.getClimb().getFrontEnc();
		}
		m_startTime = System.nanoTime();
		

    	m_finalPositionLeft = m_leftPath.get(m_leftPath.length()-1).position * 12 / m_inchesPerTic;
    	m_finalPositionRight = m_rightPath.get(m_leftPath.length()-1).position * 12 / m_inchesPerTic;
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {

    	double time = System.nanoTime();
    	double dt = (time - m_startTime) / 1000000;//ms
    	step0 = (int)(dt / 10);
    	step1 = step0 + 1;
    	double offset = dt - 10 * step0;
    	
    	
    	if(step1 >= m_leftPath.length())
    	{
    		quit = true;
    		step0 = step1 = m_leftPath.length()-1;
		}
		while(m_leftPath.get(step1).dt == 0){
			//finished the filled in path
    		quit = true;
			step1 -= 1;
			step0 -= 1;
		}
	    		Trajectory.Segment left0;
	        	Trajectory.Segment right0;
	    		Trajectory.Segment left1;
	        	Trajectory.Segment right1;
	        	
            	left0 = m_leftPath.get(step0);
            	left1 = m_leftPath.get(step1);
            	right0 = m_rightPath.get(step0);
				right1 = m_rightPath.get(step1);
		
				//System.out.println("p: "+left0.position+" v: "+left0.velocity+" a: "+left0.acceleration+" yaw: "+left0.heading);
				//System.out.println("p: "+right0.position+" v: "+right0.velocity+" a: "+right0.acceleration+" yaw: "+right0.heading);

	        	xLeft = (left0.position + ((offset / 10) * (left1.position - left0.position))) * 12 / m_inchesPerTic;
	        	xRight = (right0.position + ((offset / 10) * (right1.position - right0.position))) * 12 / m_inchesPerTic;
	        	if(step0 == step1){
	            	/*aLeft = 0;
	            	vLeft = 0;
	            	aRight = 0;
		        	vRight = 0;
		        	Ka = 0;
		        	Kv = 0;
		        	Kp = Constants.kPathHoldKp;
		        	Kg = Constants.kPathHoldKg;*/
	        	}else{
	        		aLeft = (left0.acceleration + ((offset / 10) * (left1.acceleration - left0.acceleration))) * 12 / m_inchesPerTic / 1000 * 10 / 1000 * 10;
	        		vLeft = (left0.velocity + ((offset / 10) * (left1.velocity - left0.velocity))) * 12 / m_inchesPerTic / 1000 * 10;//convert ft/sec to ticks/10ms
	        		aRight = (right0.acceleration + ((offset / 10) * (right1.acceleration - right0.acceleration))) * 12 / m_inchesPerTic / 1000 * 10 / 1000 * 10;
	        		vRight = (right0.velocity + ((offset / 10) * (right1.velocity - right0.velocity))) * 12 / m_inchesPerTic / 1000 * 10;
	        		//Ka = Constants.kPathFollowKa;
	        		//Kv = Constants.kPathFollowKv;
	        		//Kp = Constants.kPathFollowKp;
	        		//Kg = Constants.kPathFollowKg;
				}
				
				desiredAngle = -right0.heading * 180 / Math.PI; //* -1;
				double currentAngle;
				int realLeftEncoder, realRightEncoder;
				if(m_useYaw){
					currentAngle = Gyro.getYaw();
					realLeftEncoder = Robot.getDrive().getLeftEnc();
					realRightEncoder = Robot.getDrive().getRightEnc();
					m_leftPos = realLeftEncoder - m_startEncoderLeft;
					m_rightPos = realRightEncoder - m_startEncoderRight;
					Robot.getDrive().setPathPos(m_leftPos, m_rightPos);
				}else{
					currentAngle = Gyro.getRoll();
					realLeftEncoder = Robot.getClimb().getBackEnc();
					realRightEncoder = Robot.getClimb().getFrontEnc();
					m_leftPos = realLeftEncoder - m_startEncoderLeft;
					m_rightPos = realRightEncoder - m_startEncoderRight;
					Robot.getClimb().setPathPos(m_leftPos, m_rightPos);
				}
        		desiredAngle += m_startAngle;
        		while(desiredAngle > 180)
        		{
        			desiredAngle -= 360;
        		}
        		while(desiredAngle < -180)
        		{
        			desiredAngle += 360;
        		}
		
        		
        		xLeft += m_startEncoderLeft;
        		xRight += m_startEncoderRight;
        		//---
		
        		double angleError = currentAngle - desiredAngle;
        		while(angleError>180.0){
        			angleError-=360.0;
        		}
        		while(angleError<-180.0){
        			angleError+=360.0;
				}
				
        		double leftMotorLevel = Ka * aLeft + Kv * vLeft - Kp * (realLeftEncoder - xLeft) - Kg * angleError;
        		double rightMotorLevel = Ka * aRight + Kv * vRight - Kp * (realRightEncoder - xRight) + Kg * angleError;
        		//String leftMotorThings = (Ka*aLeft)+","+(Kv*vLeft)+","+ (-Kp * (realLeftEncoder - xLeft))+ ","+(-Kg*angleError)+","+leftMotorLevel;
        		//String rightMotorThings = (Ka*aRight)+","+(Kv*vRight)+","+ (-Kp * (realRightEncoder - xLeft))+ ","+(Kg*angleError)+","+rightMotorLevel;
				//mAdHocLogger.q("Left: Ka: ").q(Ka*aLeft).q(" Kv: ").q(Kv*vLeft).q(" Kp: ").q(-Kp * (realLeftEncoder - xLeft)).q(" Kg: ").q((-Kg*angleError)).q(" leftM: ").q(leftMotorLevel).go();
				//mAdHocLogger.q("Right: Ka: ").q(Ka*aRight).q(" Kv: ").q(Kv*vRight).q(" Kp: ").q(-Kp * (realRightEncoder - xLeft)).q(" Kg: ").q((Kg*angleError)).q(" rightM: ").q(rightMotorLevel).go();
        		if(Math.abs(realLeftEncoder - m_finalPositionLeft)<Constants.kPathDoneTicsTolerance&&Math.abs(realRightEncoder - m_finalPositionRight)<Constants.kPathDoneTicsTolerance){
        			//quit = true;
				}
				
        			//---
        		setMotorLevels(leftMotorLevel, -rightMotorLevel);
        		//SmartDashboard.putNumber("left motor set to: ", leftMotorLevel);
        		//SmartDashboard.putNumber("right motor set to: ", -rightMotorLevel);
            	
        		logValues();
				
        	
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return quit;
    }

    // Called once after isFinished returns true
    protected void end() {
    	mCSVWriter.flush();
	}
	
	public static class DebugOutput{
    	public double time;
    	public double aLeft;
    	public double vLeft;
    	public double xLeft;
    	public double aRight;
    	public double vRight;
    	public double xRight;
    	public double desiredAngle;
    	public double currentAngle;
    	public double realLeftEncoder;
    	public double realRightEncoder;
    	public double leftMotorLevel;
    	public double rightMotorLevel;
	}
	
	private void logValues(){
		mDebugOutput.time = System.nanoTime();
		mDebugOutput.aLeft = getALeft();
		mDebugOutput.vLeft = getVLeft();
		mDebugOutput.xLeft = getXLeft();
		mDebugOutput.aRight = getARight();
		mDebugOutput.vRight = getVRight();
		mDebugOutput.xRight = getXRight();
		mDebugOutput.desiredAngle = getDesiredAngle();
		if(m_useYaw){
			mDebugOutput.currentAngle = Gyro.getYaw();
			mDebugOutput.realLeftEncoder = Robot.getDrive().getLeftEnc();
			mDebugOutput.realRightEncoder = Robot.getDrive().getRightEnc();
		}else{
			mDebugOutput.currentAngle = Gyro.getRoll();
			mDebugOutput.realLeftEncoder = Robot.getClimb().getBackEnc();
			mDebugOutput.realRightEncoder = Robot.getClimb().getFrontEnc();
		}
		mDebugOutput.leftMotorLevel = getLeftMotorSetting();
		mDebugOutput.rightMotorLevel = getRightMotorSetting();

    	mCSVWriter.queueData(mDebugOutput);
    }

}