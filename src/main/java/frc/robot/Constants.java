package frc.robot;


/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants{
    public static double kLooperDt = 0.01;

    public static double kSensorUnitsPerRotation = 4096;
    
    public static double kStepSizeSeconds = 0.01;
    public static double kMaxFeetPerSecond = 9.0;
    public static double kMaxAcceleration = 7.0;//15.0;
    public static double kMaxJerk = 40.0;
    public static double kWheelBaseFeet = 29.0 / 12.0;//25.75
    
    public static final double kNearRocketHatchAngle = 0;//TODO tune this
    public static final double kFarRocketHatchAngle = 0;//TODO tune this
    public static final double kRocketVisionAngleTolerance = 0;//TODO tune this

    public static final double kInchesPerVisionY = 4.273662947; // VisionY = 27 - (limelight ty output)
    public static final double kDegPerVisionX = 0.971;
    public static final double kVisionDistToMotor = 0.02;//0.03
    public static final double kVisionXToMotor = 0.01;
    public static final double kVisionDistToStop = 1.0;//inches
    public static final double kVisionToleranceToStop = 3.0;
    public static final double kVisionFullArea = 10.0;
    public static final double kVisionMinMotorLevel = 0.0;
    public static final double kVisionMotorLevelAccMax = 0.025;//max motor change in 10ms

    public static final double kVisionXToMotorWeak = 0.02;
    public static final double kVisionXToMotorStrong = 0.04;
    public static final double kVisionDeltaAngleTolerance = 1.0;//degrees
    public static final double kVisionApproachDist = -1.0;//ft
    public static final double kVisionApproachVel = 0.0;//ft per s
    public static final double kMinimumRadiusTurn = kWheelBaseFeet + 0.5;//ft
    public static final double kCamToBumper = 12;//inches
    public static final double kVisionMidPtVel = 2.0;//ft per s
    public static final double kVisionLargestApproachAngle = 10*Math.PI/180.0;//radians
    public static final double kVisionAngleHdgTolerance = 4*Math.PI/180.0;//radians

    public static final double kVisionXTolerance = 2.5;
    public static final double kVisionYTolerance = 3.5;

    public static final double kVisionDistFullTurn = 72;//inches

    public static final double kZRobotInches = -11.0;
    public static final double kCameraRotation = -12;// * (Math.PI / 180.0);
    public static final double kCameraYaw = 0;// * (Math.PI / 180.0);
    public static final double kCamToFront = 19;
    public static final double kCamToCenter = 0;

    public static final double kTurnToAngleKp = 0.04;
    public static final double kTurnToAngleKd = 0.05;
    public static final double kTurnToAngleTolerance = 1.0;
    public static final double kTurnToAngleKf = 0.125;// 10ms per 8deg
    
    public static final double kDriveDistKp = 0.0;
    public static final double kDriveDistKv = 0.01;
    public static final double kDriveDistTolerance = 0.2;//feet

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelDiameterInches = 4;
    public static double kTrackWidthInches = 26.655;
    public static double kTrackScrubFactor = 0.924;
    public static final double kInchesPerTic = Constants.kDriveWheelDiameterInches * Math.PI / Constants.kSensorUnitsPerRotation;

    public static final double kDriveMaxBackAccPerILiftLow = 0.02;
    public static final double kDriveMaxBackAccPerILiftHigh = 0.01;
    public static final double kPathDoneTicsTolerance = 4 / kInchesPerTic;

    //Climb
    public static final double kClimbPulleyDiameter = 0.75;//in
    public static final double kClimbInchesPerTic = Constants.kClimbPulleyDiameter * Math.PI / Constants.kSensorUnitsPerRotation;
    public static final double kClimbMaxAcceleration = 0.5;//ft per s

    public static final double kClimbKa = 0;
    public static final double kClimbKv = 0.005;
    public static final double kClimbKp = 0.001;
    public static final double kClimbKg = 0.02;
    
    public static final double kClimbHoldKp = 0.0005;
    public static final double kClimbHoldKg = 0.075;
    public static final double kClimbHoldGravityMotorLevel = 0.1;

    //Lift
    public static final double kLiftTicsPerRev = 256.0;//4096.0;
    public static final double kLiftInchesPerRevHighGear = 2.0 * Math.PI;// * (24.0/72.0);
    public static final double kLiftInchesPerTicHighGear = Constants.kLiftInchesPerRevHighGear / Constants.kLiftTicsPerRev;//2635
    
    public static final double kLiftMaxMotorUp = 1.0;
    public static final double kLiftMaxMotorDown = -0.45;
    public static final double kLiftSlowMotorUp = 0.2;
    public static final double kLiftSlowMotorDown = -0.2;
    
    public static final double kLiftHoldLowPower = 0.3;
    public static final double kLiftHoldHighPower = 0.3;
    public static final double kLiftKp = 0.05;//0.001;
    public static final double kLiftHoldBallPower = 0;

    public static final double kLiftCargoShip = 1.35;

    public static final double kLiftRocket1 = 0.01;
    public static final double kLiftRocket2 = 2.25;
    public static final double kLiftRocket3 = 4.3;

    public static final double kLiftBallRocket1 = 0.001;
    public static final double kLiftBallRocket2 = 1.45;
    public static final double kLiftBallRocket3 = 3.75;//arm 30 deg

    public static final double kLiftStage2Pos = 2.1;//ft
    public static final double kLiftMaxHeight = 4.5;//ft

    //Arm
    public static final double kArmTicsPerRev = 4096.0;
    public static final double kArmDegreesPerRev = 360.0 * (15.0/26.0); // gear ratios
    public static final double kArmDegreesPerTic = Constants.kArmDegreesPerRev / Constants.kArmTicsPerRev; //-425
    public static final double kArmMaxAmps = 45;
    public static final long kArmTimeSinceHitMax = 1000*1000*1000;
    public static final double kArmMaxMotorUp = 0.75;
    public static final double kArmMaxMotorDown = -0.5;
    public static final double kArmSlowMotorUp = 0.2;
    public static final double kArmSlowMotorDown = -0.2;
    public static final double kArmJoystickDeadband = 0.1;
    public static final double kArmSoftStopHigh = 120.0;//100
    public static final double kArmSoftStopLow = -15.0;//-100
    public static final double kArmDegTolerance = 5.0;
    public static final double kArmDegSafeHatch = 30.0;
    public static final double kArmHoldBallDeg = 30.0;
    public static final double kArmIntakeBallDeg = 117.0;
    public static final double kArmCargoShipDeg = 50.0;

    public static final double kArmMaxMotor = 0.75;

    public static final double kArmIntakeDeg = -183.5;
    public static final double kArmClimbDeg = 0.0;
    
    //Intake
    public static final double kIntakeStallCurrent = 60;//amps
    public static final double kHatchStallCurrent = 15;//amps
    
    //Tines
    public static final double kTinesMotorLevelPerIntervalUp = 0.01;
    public static final double kTinesMotorLevelPerIntervalDown = 0.01;
    
    // Geometry
    public static double kCenterToFrontBumperDistance = 16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 16.99;
    public static double kCenterToSideBumperDistance = 17.225;

    /* CONTROL LOOP GAINS */
    
    public static final double kODesiredFinalMotor = 0.8;
    public static final double kSimpleArcTolerance = 10.0;

    public static final double kPathFollowKa = 0;//0.065;
    public static final double kPathFollowKv = 0.0055;
    public static final double kPathFollowKp = 0.00025;
    public static final double kPathFollowKg = 0.01;

    public static final double kPathHoldKp = 0.0005;
    public static final double kPathHoldKg = 0.0001;
    
    public static final double kArmHoldKp = 0.035;
    public static final double kArmHoldKi = 0.0;
    public static final double kArmHoldKd = 0.0;
    public static final double kArmHoldPower = 0.2;//0.35

    public static final double kTestVelTarget = 1700;
    public static final double kTestDistTarget = 50000;
    public static final double kTestVelError = 250;
    public static final double kTestDistError = 6000;
    
}
