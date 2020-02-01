package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.util.AsyncStructuredLogger;

public class Arm extends Subsystem {
	
    private static Arm mInstance = null;

     public static Arm getInstance() {
    	if ( mInstance == null ) {
    		synchronized ( Arm.class ) {
    			if ( mInstance == null ) {
    				mInstance = new Arm();
    			}
    		}
    	}
		return mInstance;
	}
	
	private boolean m_holdArmForDefense = false;
	public void setHoldArmForDefense(boolean x){
		m_holdArmForDefense = x;
	}
	
	private boolean doNonManual = false;
    
    int iCall = 0;
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
        	if(mHaveCalledOnStart == false ){
            	mStartTime = System.nanoTime();
            	mHaveCalledOnStart = true;
        	}
        }

        @Override
        public void onLoop(double timestamp) {
            mArmEncoder = can.getQuadraturePosition();//armMotor.getSelectedSensorPosition();
        	mArmPos = getPosDegrees();
			mArmError = mArmSetpoint - mArmPos;
			if(mArmPos <= Constants.kArmDegSafeHatch){
				mHatchSafe = true;
			}else{
				mHatchSafe = false;
			}
        	/*if(Math.abs(mArmError) < Constants.kLiftTolerance){
        		mIsAtSetpoints = true;
        	}else{
        		mIsAtSetpoints = false;
			}*/
            synchronized (Arm.class) {
				if(mArmSetpoint != xArmSetpoint){doNonManual = true;}
            	xArmPos = mArmPos;
        		xIsAtSetpoints = mIsAtSetpoints;
            	mArmSetpoint = xArmSetpoint;
            }
			/*if(backStop.get() == false){
				//can.setQuadraturePosition(convertDegToEnc(Constants.kArmSoftStopLow), 10);
			}*/
			/*if(frontStop.get() == false){
				can.setQuadraturePosition(convertDegToEnc(Constants.kArmSoftStopHigh), 10);
			}*/
			//doPathFollowing();
			if(can.getGeneralInput(GeneralPin.LIMF) == false){
				//at limit
				if(can.getQuadraturePosition() != 0){
					can.setQuadraturePosition(0,10);
				}
			}

			double motorLevel = OI.getInstance().getDrive2();
			if(Math.abs(motorLevel) >= 0.1){
				armMotor.set(motorLevel);
				doNonManual = false;
			}
			if(doNonManual == true){
				doPathFollowing();
			}else{
				armMotor.set(motorLevel);
			}
			/*if(backStop.get() == false){
				if(motorLevel <= 0){
					//motorLevel = 0;
				}
			}*/
			/*if(frontStop.get() == false){
				if(motorLevel >= 0){
					motorLevel = 0;
				}
			}*/


            logValues();
            mLastArmError = mArmError;
        }

        @Override
        public void onStop(double timestamp) {
            onStart(timestamp);
            mCSVWriter.flush();
        }
    };
    
    private void setArmMotorLevels(double x){
    	if(x < Constants.kArmMaxMotorDown){
    		x = Constants.kArmMaxMotorDown;
    	}
    	if(x > Constants.kArmMaxMotorUp){
    		x = Constants.kArmMaxMotorUp;
    	}
    	x = -x;
    	armMotor.set(x);
    }
	boolean slowDown = false;
	private void doPathFollowing(){
		if(Robot.getIntake().getArmSafe() == false){
			//not safe to move arm
			mArmSetpoint = 0.0;
		}

    	double error = mArmSetpoint - mArmPos;
    	double arm_motor_level;
    	//double armRealPos = mArmPos + 12;
		/*if (error>40.0){
			slowDown = false;
			arm_motor_level = Constants.kArmMaxMotorUp;
			arm_motor_level -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else if (error>5.0){
			slowDown = false;
			arm_motor_level = Constants.kArmSlowMotorUp;
			arm_motor_level -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else if(error<-40.0){
			slowDown = false;
			arm_motor_level = Constants.kArmMaxMotorDown;
			arm_motor_level -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else if(error<-5.0){
			slowDown = false;
			arm_motor_level = Constants.kArmSlowMotorDown;
			arm_motor_level -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}else{
			slowDown = true;
			arm_motor_level = 0.0;//getArmPIDOutput(mArmSetpoint);
			arm_motor_level -= 3*Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}

		if(slowDown){
			arm_motor_level = getArmPIDOutput(mArmSetpoint);
			arm_motor_level -= 3*Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		}*/
		arm_motor_level = getArmPIDOutput(mArmSetpoint);

		/*if(mArmSetpoint > 45 && frontStop.get() == false){
			//at front stop
			arm_motor_level = 0;
		}*/
		//if(mArmSetpoint < -45 && backStop.get() == false){
			//at back stop
			//arm_motor_level = 0;
		//}
		
		SmartDashboard.putNumber("arm setpoint", mArmSetpoint);
		SmartDashboard.putNumber("arm motorlevel", arm_motor_level);
		
		setArmMotorLevels(arm_motor_level);
	}

	// S H A R E D   A C C E S S
	// These member variables can be accessed by either thread, but only by calling the appropriate getter method.
    
    private double mArmPos;
    private double xArmPos;
    public double getArmPos(){
    	synchronized (Arm.class){
    		return xArmPos;
    	}
    }
    
    private double mArmSetpoint=Constants.kArmSoftStopLow;
    private double xArmSetpoint=Constants.kArmSoftStopLow;
    public void setArmSetpoint(double setpoint){
		System.out.println("setArmSetpoint: " + setpoint);
    	synchronized (Arm.class){
			xArmSetpoint = setpoint;

			double lowStop = Constants.kArmSoftStopLow;
			double highStop = Constants.kArmSoftStopHigh;

			if(setpoint < lowStop){
				xArmSetpoint = lowStop;
			}else if(setpoint > highStop){
				xArmSetpoint = highStop;
			}
    	}
    }
    public double getArmSetpoint(){
    	synchronized (Arm.class){
    		return xArmSetpoint;
    	}
    }
    
    private boolean mIsAtSetpoints = false;
    private boolean xIsAtSetpoints = false;
    public boolean isAtSetpoint(){
    	return xIsAtSetpoints;
    }
    
	// S U B S Y S T E M   A C C E S S
	// These member variables can be accessed only by the subsystem
	private boolean mHatchSafe = true;
	public boolean getHatchSafe(){
		return mHatchSafe;
	}

    // Hardware
	private final WPI_VictorSPX armMotor;
	private final CANifier can;
	//private final DigitalInput backStop;//frontStop, backStop;
    
    private Arm() {
		armMotor = new WPI_VictorSPX(RobotMap.ARM_VICTOR);
		armMotor.configFactoryDefault();
        armMotor.setNeutralMode(NeutralMode.Brake);
		armMotor.configNeutralDeadband(0.01, 10);
		armMotor.setInverted(false);
		
		can = new CANifier(RobotMap.CANIFIER);
		can.configFactoryDefault();

		armMotor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, 
												LimitSwitchNormal.NormallyOpen, can.getDeviceID(), 10);
		armMotor.configClearPositionOnLimitF(true, 10);
		//frontStop = new DigitalInput(RobotMap.FRONT_ARM_STOP);
		//backStop = new DigitalInput(RobotMap.BACK_ARM_STOP);
		
        mDebugOutput = new DebugOutput();
        mCSVWriter = new AsyncStructuredLogger<DebugOutput>("ArmLog", DebugOutput.class);
    }
    
    private long mStartTime;
	private boolean mHaveCalledOnStart = false;
	private double mArmEncoder;
    private double mArmError, mLastArmError;

    private double getPosDegrees(){
 	   return can.getQuadraturePosition() * Constants.kArmDegreesPerTic;
	}
	
	private int convertDegToEnc(double deg){
		return (int)(deg * -1 / Constants.kArmDegreesPerTic);
	}

    private double getArmPIDOutput(double setpoint_to_use){
    	double error = setpoint_to_use - mArmPos;
    	double output = error * Constants.kArmHoldKp + Math.min(error, mLastArmError) * Constants.kArmHoldKi - (error - mLastArmError) * Constants.kArmHoldKd;
		output -= Math.sin(mArmPos*Math.PI/180.0)*Constants.kArmHoldPower;
		mLastArmError = error;
		if(output >= Constants.kArmMaxMotor){
			output = Constants.kArmMaxMotor;
		}
		return output;
    }
    
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }
        
    @Override
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("Arm Setpoint", getArmSetpoint());
    	SmartDashboard.putNumber("Arm Degrees", getPosDegrees());
		SmartDashboard.putNumber("Arm encoder", can.getQuadraturePosition());
		SmartDashboard.putNumber("re-zero forward pt",convertDegToEnc(Constants.kArmSoftStopHigh));
		SmartDashboard.putNumber("Arm Motor Percent", armMotor.get());
		SmartDashboard.putBoolean("Arm Front Stop", can.getGeneralInput(GeneralPin.LIMF));
		//SmartDashboard.putBoolean("Arm Back Stop",backStop.get());
	}
	
	PinValues pv = new PinValues();

    // Logging
    private DebugOutput mDebugOutput;
    private final AsyncStructuredLogger<DebugOutput> mCSVWriter;
    
    public static class DebugOutput{
    	public long sysTime;
    	public double armEncoder;
    	public double armPosDeg;
    	public double motorPercent;
    }
    
    public void logValues(){
	    mDebugOutput.sysTime = System.nanoTime()-mStartTime;
	    mDebugOutput.armEncoder = mArmEncoder;
	    mDebugOutput.armPosDeg = mArmPos;
	    mDebugOutput.motorPercent = armMotor.get();
		mCSVWriter.queueData(mDebugOutput);
    }

	//unused overrides
    @Override
    public void writeToLog() {}
	@Override
	protected void initDefaultCommand() {}
	@Override
	public void zeroSensors() {
		can.setQuadraturePosition(0, 10);
	}
    @Override
    public synchronized void stop() {}
}