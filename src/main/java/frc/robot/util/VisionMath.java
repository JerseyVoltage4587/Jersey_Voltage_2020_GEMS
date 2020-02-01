package frc.robot.util;
import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionMath {

    private double zRobot = Constants.kZRobotInches;
    //private double cameraRotation = Constants.kCameraRotation;

    private double m_xRobot,m_yRobot;
    public double getRobotX(){
        return m_xRobot;
    }
    public double getRobotY(){
        return m_yRobot;
    }
    public enum scoringZone{
        LEFT_ROCKET,
        RIGHT_ROCKET,
        CARGO_SIDES,
        CARGO_FRONT,
    }
    private scoringZone m_scoringZone = scoringZone.CARGO_FRONT;
    public void setScoringZone(scoringZone sZone){
        m_scoringZone = sZone;
    }
    private boolean m_haveCargo = false;
    public void setHaveCargo(boolean haveCargo){
        m_haveCargo = haveCargo;
    }
    private double m_g;
    public double getRotatedHdg(){
        return m_g;
    }

    public void findRobotPos(){
        
        double cameraYaw = SmartDashboard.getNumber("kCameraYaw", Constants.kCameraYaw) * (Math.PI/180.0);
        double cameraTilt = SmartDashboard.getNumber("kCameraTilt", Constants.kCameraRotation) * (Math.PI/180.0);
        double cameraToCenter = SmartDashboard.getNumber("kCameraToCenter", Constants.kCamToCenter);
        double cameraToFront = SmartDashboard.getNumber("kCameraToFront", Constants.kCamToFront);

        NetworkTable limelightTable;
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tve = limelightTable.getEntry("tv");
        double tv = tve.getDouble(0.0);
        if(tv == 0){
            //don't have target
            m_xRobot = 999;
            m_yRobot = 999;
            return;
        }

		NetworkTableEntry txe = limelightTable.getEntry("tx");
        double tx = txe.getDouble(0.0);
        tx *= (Math.PI/180.0);
		NetworkTableEntry tye = limelightTable.getEntry("ty");
        double ty = tye.getDouble(0.0) - 90;
        ty *= (Math.PI/180.0);
        
        double r = zRobot / (
            (Math.cos(ty)*Math.cos(cameraTilt))
            - (Math.sin(ty)*Math.cos(tx)*Math.sin(cameraTilt))
            );

        SmartDashboard.putNumber("visionR", r);
        double xCam = r * (
            (Math.sin(ty)*Math.cos(tx)*Math.cos(cameraTilt))
            +(Math.cos(ty)*Math.sin(cameraTilt))
        );

        double yCam = r * (Math.sin(ty)*Math.sin(tx));

        m_g = Gyro.getYaw();
        double m_hdg = m_g;
        switch(m_scoringZone){
            case CARGO_FRONT:
                //no change, 0 degrees
                break;
            case CARGO_SIDES:
                if(m_g<0){
                    //right side, -90 deg
                    m_g -= 90;
                }else{
                    //left side, +90 deg
                    m_g += 90;
                }
                break;
            case RIGHT_ROCKET:
                if(m_haveCargo){
                    //+90 deg
                    m_g += 90;
                }else{
                    //have hatch
                    if(Math.abs(m_g-Constants.kNearRocketHatchAngle)<Constants.kRocketVisionAngleTolerance){
                        //near hatch
                        m_g+=Constants.kNearRocketHatchAngle;
                    }else if(Math.abs(m_g-Constants.kFarRocketHatchAngle)<Constants.kRocketVisionAngleTolerance){
                        //far hatch
                        m_g+=Constants.kFarRocketHatchAngle;
                    }else{
                        //TODO flash LEDs "bad state"
                    }
                }
                break;
            case LEFT_ROCKET:
                if(m_haveCargo){
                    //-90 deg
                    m_g -= 90;
                }else{
                    //have hatch
                    if(Math.abs(m_g+Constants.kNearRocketHatchAngle)<Constants.kRocketVisionAngleTolerance){
                        //near hatch
                        m_g-=Constants.kNearRocketHatchAngle;
                    }else if(Math.abs(m_g+Constants.kFarRocketHatchAngle)<Constants.kRocketVisionAngleTolerance){
                        //far hatch
                        m_g-=Constants.kFarRocketHatchAngle;
                    }else{
                        //TODO flash LEDs "bad state"
                    }
                }
                break;
        }
        m_g *= (Math.PI/180.0);//convert to radians for trig functions
        m_yRobot = (xCam * Math.sin(cameraYaw)) + (yCam * Math.cos(cameraYaw));
        m_xRobot = (xCam * Math.cos(cameraYaw)) - (yCam * Math.sin(cameraYaw));
        m_yRobot -= cameraToCenter;
        m_xRobot += cameraToFront;
        
        //m_yRobot += (Constants.kCamToFront * Math.sin(m_hdg)) + (Constants.kCamToCenter * Math.cos(m_hdg));
        //m_xRobot += (Constants.kCamToFront * Math.cos(m_hdg)) + (Constants.kCamToCenter * Math.sin(m_hdg));
        //m_yRobot = yCam;
        //m_xRobot = xCam;
    }
}
