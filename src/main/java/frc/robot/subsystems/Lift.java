package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.util.AsyncStructuredLogger;

public class Lift extends Subsystem {
	
    private static Lift mInstance = null;

     public static Lift getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Lift.class ) {
    			if ( mInstance == null ) {
    				mInstance = new Lift();
    			}
    		}
    	}
    	return mInstance;
    }
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
			mPosLast = getPosFeet();
			
        	if(mHaveCalledOnStart == false ){
        		//liftEncoder.reset();  //add this back for talon maybe?
            	mStartTime = System.nanoTime();
            	mLastTime = mStartTime;
            	mHaveCalledOnStart = true;
        	}
    		//setDebug();
        	//setLiftMotorLevels(0.0);
        	//setOpenLoop();
        }

        @Override
        public void onLoop(double timestamp) {
            mEncoder = liftEncoder.get();//liftTalon.getSelectedSensorPosition();
        	mCurrentTime = System.nanoTime();
        	mPos = getPosFeet();
        	mVel = getVelFPS();
        	/*if(Math.abs(mPos - mLiftSetpoint) < Constants.kLiftTolerance){
        		mIsAtSetpoints = true;
        	}else{
        		mIsAtSetpoints = false;
        	}*/
            synchronized (Lift.class) {
            	xPos = mPos;
            	xVel = mVel;
        		xIsAtSetpoints = mIsAtSetpoints;
            	mLiftSetpoint = xLiftSetpoint;
            }
            SmartDashboard.putNumber("mCurrentTime", mCurrentTime/(1000*1000));
			
			//determine lift motor level
			double lift_error = mLiftSetpoint - mPos;
			double lift_motor_level = 0;

			if (lift_error>1.0){
				lift_motor_level = Constants.kLiftMaxMotorUp;
			}else if (lift_error>0.025){
				lift_motor_level = Constants.kLiftSlowMotorUp;
			}else if(lift_error<-1.0){
				lift_motor_level = Constants.kLiftMaxMotorDown;
			}else if(lift_error<-0.025){
				lift_motor_level = Constants.kLiftSlowMotorDown;
			}else{
				lift_motor_level = lift_error * Constants.kLiftKp;
			}

			if(mLiftSetpoint > 0.1){
				//don't add hold power if at bottom
				if(mPos > Constants.kLiftStage2Pos){
					lift_motor_level += Constants.kLiftHoldHighPower;
				}else{
					lift_motor_level += Constants.kLiftHoldLowPower;
				}
			}
			setLiftMotorLevels(lift_motor_level);
			
            logValues();
        }

        @Override
        public void onStop(double timestamp) {
            onStart(timestamp);
            mCSVWriter.flush();
        }
	};
    
    private void setLiftMotorLevels(double x){
    	if(x < Constants.kLiftMaxMotorDown){
    		x = Constants.kLiftMaxMotorDown;
    	}
    	if(x > Constants.kLiftMaxMotorUp){
    		x = Constants.kLiftMaxMotorUp;
    	}
    	x = -x;
    	liftTalon.set(x);
    }

	// S H A R E D   A C C E S S
	// These member variables can be accessed by either thread, but only by calling the appropriate getter method.

    private double mPos;
    private double mPosLast;
    private double xPos;
    public double getPos(){
    	synchronized (Lift.class){
    		return xPos;
    	}
    }

    private double mVel;
    private double xVel;
    public double getVel(){
    	synchronized (Lift.class){
    		return xVel;
    	}
    }
    
    private double mLiftSetpoint;
    private double xLiftSetpoint;
    public void setLiftSetpoint(double setpoint){
    	synchronized (Lift.class){
			xLiftSetpoint = setpoint;
			if(setpoint>Constants.kLiftMaxHeight){
				xLiftSetpoint = Constants.kLiftMaxHeight;
			}else if(setpoint<0){
				xLiftSetpoint = 0;
			}
			if(Robot.getIntake().getHasHatch() == false){
				if(setpoint == Constants.kLiftRocket1){
					xLiftSetpoint = Constants.kLiftBallRocket1;
					Robot.getArm().setArmSetpoint(/*Constants.kArmCargoShipDeg*/ 0.0);
				}else if(setpoint == Constants.kLiftRocket2){
					xLiftSetpoint = Constants.kLiftBallRocket2;
					Robot.getArm().setArmSetpoint(/*Constants.kArmHoldBallDeg*/ 0.0);
				}else if(setpoint == Constants.kLiftRocket3){
					xLiftSetpoint = Constants.kLiftBallRocket3;
					Robot.getArm().setArmSetpoint(/*Constants.kArmHoldBallDeg*/ 0.0);
				}
				if(setpoint == Constants.kLiftCargoShip){
					Robot.getArm().setArmSetpoint(/*Constants.kArmCargoShipDeg+30*/ 0.0);
				}
			}
    	}
    }
    public double getLiftSetpoint(){
    	synchronized (Lift.class){
    		return xLiftSetpoint;
    	}
    }
    
    private boolean mIsAtSetpoints = false;
    private boolean xIsAtSetpoints = false;
    public boolean isAtSetpoint(){
    	return xIsAtSetpoints;
    }
    
	// S U B S Y S T E M   A C C E S S
	// These member variables can be accessed only by the subsystem

    // Hardware
	private final WPI_TalonSRX liftTalon; 
	private final WPI_VictorSPX liftVictor;
	private final Encoder liftEncoder;
    
    private Lift() {
		liftTalon = new WPI_TalonSRX(RobotMap.LIFT_TALON);
		liftTalon.setInverted(true);
        //liftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        liftTalon.changeMotionControlFramePeriod(5);
        //liftTalon.setSensorPhase(true);
        liftTalon.setNeutralMode(NeutralMode.Brake);
		liftTalon.configNeutralDeadband(0.01, 10);
		//liftTalon.setSelectedSensorPosition(0);

		liftVictor = new WPI_VictorSPX(RobotMap.LIFT_VICTOR);
		liftVictor.setInverted(true);
		liftVictor.follow(liftTalon);

		liftEncoder = new Encoder(RobotMap.LIFT_ENC_A, RobotMap.LIFT_ENC_B);
		
        mDebugOutput = new DebugOutput();
        mCSVWriter = new AsyncStructuredLogger<DebugOutput>("LiftLog" ,DebugOutput.class);
    }
    
    private long mCurrentTime, mLastTime, mStartTime;
	private boolean mHaveCalledOnStart = false;
	private double mLiftDrive, mEncoder;
    
    private double getPosFeet(){
		//return mEncoder * Constants.kLiftInchesPerTicHighGear / 12.0;
		return /*liftTalon.getSelectedSensorPosition()*/liftEncoder.get() * Constants.kLiftInchesPerTicHighGear / 12.0;
    }
    
    private double getVelFPS(){
 	   return ((mPos - mPosLast) / (mCurrentTime - mLastTime)) * Math.pow(10, 9);
    }
    
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }
        
    @Override
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Lift Height (ft)", getPosFeet());
    	SmartDashboard.putNumber("Lift Setpoint", getLiftSetpoint());
		SmartDashboard.putNumber("Lift encoder", liftEncoder.get());//liftTalon.getSelectedSensorPosition());
		SmartDashboard.putNumber("Lift Motor", liftTalon.get());
    }

    // Logging
    private DebugOutput mDebugOutput;
    private final AsyncStructuredLogger<DebugOutput> mCSVWriter;
    
    public static class DebugOutput{
    	public long sysTime;
    	public double liftEncoder;
    	public double posFeet;
    	public double motorPercent;
    	public double driveStick;
    }
    
    public void logValues(){
	    mDebugOutput.sysTime = System.nanoTime()-mStartTime;
	    mDebugOutput.liftEncoder = mEncoder;
	    mDebugOutput.posFeet = mPos;
	    mDebugOutput.motorPercent = liftTalon.get();
		mDebugOutput.driveStick = mLiftDrive;
		mCSVWriter.queueData(mDebugOutput);
		
    	SmartDashboard.putNumber("Lift Motor Percent", liftTalon.get());
       	SmartDashboard.putNumber("liftPosFeet: ", mPos);
    }

	//unused overrides
    @Override
    public void writeToLog() {}
	@Override
	protected void initDefaultCommand() {}
	@Override
	public void zeroSensors() {}
    @Override
    public synchronized void stop() {}
}