package frc.robot.paths;

import java.io.File;
import java.io.FileWriter;

import frc.robot.Constants;
import frc.robot.util.Gyro;
import frc.robot.util.AsyncAdHocLogger;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

public class JVPathCreator {

    private Trajectory m_leftTrajectory;
    public Trajectory getLeftTrajectory() {
        return m_leftTrajectory;
    }
    private Trajectory m_rightTrajectory;
    public Trajectory getRightTrajectory() {
        return m_rightTrajectory;
    }
    AsyncAdHocLogger asyncAdHocLogger;
    private int m_currentIndex;//TODO initialize to zero when writing new path

    public JVPathCreator(){
        asyncAdHocLogger = new AsyncAdHocLogger("");
        m_leftTrajectory = new Trajectory(new Segment[1000]);
        m_rightTrajectory = new Trajectory(new Segment[1000]);
        for(int i=0;i<m_leftTrajectory.length();i++){
            m_leftTrajectory.segments[i] = new Segment(0,0,0,0,0,0,0,0);
            m_rightTrajectory.segments[i] = new Segment(0,0,0,0,0,0,0,0);
        }
    }

    public void writePathToFile(String filename){
        try{
            FileWriter w = new FileWriter(new File("/home/lvuser/"+filename+".csv") );
    
            w.write(m_leftTrajectory.length()+"\n");
            w.write("dt,x,y,pos,vel,acc,jerk,heading\n");
            for (int i = 0; i < m_leftTrajectory.length(); i++) {
                Trajectory.Segment seg = m_leftTrajectory.get(i);
                w.write(seg.dt+","+ seg.x+","+ seg.y+","+ seg.position+","+ seg.velocity+","+ 
                        seg.acceleration+","+ seg.jerk+","+ seg.heading+"\n");
                }
            for (int i = 0; i < m_rightTrajectory.length(); i++) {
                Trajectory.Segment seg = m_rightTrajectory.get(i);
    
                w.write(seg.dt+","+ seg.x+","+ seg.y+","+ seg.position+","+ seg.velocity+","+ 
                        seg.acceleration+","+ seg.jerk+","+ seg.heading+"\n");
                }
            w.close();
            asyncAdHocLogger.q("wrote: ").q(filename).go();
        }catch(Exception e){
                
        }
    }

	public void calcDriveStraight(double distInches, double startVel, double endVel, boolean backwards){
        double desiredDist = distInches / 12.0;//feet
        double currentDist = 0;
        double lastVel = startVel;

        while(currentDist<desiredDist){
            double currentVel,currentAcc;
            double timeToSlow = Math.abs(endVel - lastVel) / Constants.kMaxAcceleration;
            double distToSlow = (endVel + lastVel) / 2.0 * timeToSlow;
            //asyncAdHocLogger.q("tSlow: ").q(timeToSlow).q(" dSlow: ").q(distToSlow).q(" distLeft: ").q((desiredDist - currentPos)).go();
            if(distToSlow >= (desiredDist - currentDist)){
                //deccel
                currentVel = lastVel - (Constants.kMaxAcceleration * Constants.kLooperDt);
                currentAcc = -Constants.kMaxAcceleration;
                if(currentVel <= endVel){
                    currentVel = endVel;
                    currentAcc = 0;
                }
            }else{
                //accel
                currentVel = lastVel + (Constants.kMaxAcceleration * Constants.kLooperDt);
                currentAcc = Constants.kMaxAcceleration;
                if(currentVel >= Constants.kMaxFeetPerSecond){
                    currentVel = Constants.kMaxFeetPerSecond;
                    currentAcc = 0;
                }
            }
            double lastLeftPos,lastRightPos;
            double lastHdg;
            if(m_currentIndex>0){
                lastRightPos = m_rightTrajectory.segments[m_currentIndex-1].position;
                lastLeftPos = m_leftTrajectory.segments[m_currentIndex-1].position;
                //hdg doesn't matter left/right
                lastHdg = m_leftTrajectory.segments[m_currentIndex-1].heading;
            }else{
                lastLeftPos = 0;
                lastRightPos = 0;
                lastHdg = 0;
            }
            double deltaPos = currentVel * Constants.kLooperDt;
            double currentLeftPos, currentRightPos;

            currentDist += deltaPos;
            lastVel = currentVel;

            if(backwards == false){
                currentLeftPos = lastLeftPos + deltaPos;
                currentRightPos = lastRightPos + deltaPos;
            }else{
                currentLeftPos = lastLeftPos - deltaPos;
                currentRightPos = lastRightPos - deltaPos;
                currentVel *= -1;
                currentAcc *= -1;
            }
            

            m_rightTrajectory.segments[m_currentIndex].position = currentRightPos;
            m_rightTrajectory.segments[m_currentIndex].velocity = currentVel;
            m_rightTrajectory.segments[m_currentIndex].acceleration = currentAcc;
            m_rightTrajectory.segments[m_currentIndex].heading = lastHdg;
            m_rightTrajectory.segments[m_currentIndex].dt = Constants.kLooperDt;

            m_leftTrajectory.segments[m_currentIndex].position = currentLeftPos;
            m_leftTrajectory.segments[m_currentIndex].velocity = currentVel;
            m_leftTrajectory.segments[m_currentIndex].acceleration = currentAcc;
            m_leftTrajectory.segments[m_currentIndex].heading = lastHdg;
            m_leftTrajectory.segments[m_currentIndex].dt = Constants.kLooperDt;

            m_currentIndex++;
        }
    }

    public void calcArc(double outerRadiusInches, double degrees, double startVel, double endVel, boolean backwards){//+degrees = turn right, -degrees = turn left
        double desiredDist = ((2*Math.PI*outerRadiusInches*Math.abs(degrees))/360) / 12.0;//feet
        double currentPos = 0;
        double currentDist = 0;
        double lastVel = startVel;
        asyncAdHocLogger.q("desiredDist: ").q(desiredDist).q(" outerRadiusInches: ").q(outerRadiusInches).q(" deg: ").q(degrees).go();
        
        while(currentDist<desiredDist){
            double currentVel,currentAcc;
            double timeToSlow = Math.abs(endVel - lastVel) / Constants.kMaxAcceleration;
            double distToSlow = (endVel + lastVel) / 2.0 * timeToSlow;
            //asyncAdHocLogger.q("tSlow: ").q(timeToSlow).q(" dSlow: ").q(distToSlow).q(" distLeft: ").q((desiredDist - currentPos)).go();
            if(distToSlow >= (desiredDist - currentDist)){
                //deccel
                currentVel = lastVel - (Constants.kMaxAcceleration * Constants.kLooperDt);
                currentAcc = -Constants.kMaxAcceleration;
                if(currentVel <= endVel){
                    currentVel = endVel;
                    currentAcc = 0;
                }
            }else{
                //accel
                currentVel = lastVel + (Constants.kMaxAcceleration * Constants.kLooperDt);
                currentAcc = Constants.kMaxAcceleration;
                if(currentVel >= Constants.kMaxFeetPerSecond){
                    currentVel = Constants.kMaxFeetPerSecond;
                    currentAcc = 0;
                }
            }
            double lastPos;
            double lastHdg;
            if(m_currentIndex>0){
                if(degrees < 0){
                    //turn left
                    lastPos = m_rightTrajectory.segments[m_currentIndex-1].position;
                }else{
                    //turn right
                    lastPos = m_leftTrajectory.segments[m_currentIndex-1].position;
                }
                //hdg doesn't matter left/right
                lastHdg = m_leftTrajectory.segments[m_currentIndex-1].heading;
            }else{
                lastPos = 0;
                lastHdg = 0;
            }
            double deltaPos = currentVel * Constants.kLooperDt;
            currentPos = lastPos + deltaPos;
            currentDist += deltaPos;
            lastVel = currentVel;
            double deltaHdg = (deltaPos) / (outerRadiusInches / 12.0);//radians
            deltaHdg *= degrees>0?-1:1;
            double hdg;

            if(backwards == false){
                currentPos = lastPos + deltaPos;
                hdg = lastHdg + deltaHdg;
            }else{
                currentPos = lastPos - deltaPos;
                hdg = lastHdg - deltaHdg;
                currentVel *= -1;
                currentAcc *= -1;
            }

            //asyncAdHocLogger.q("lastHdg: ").q(lastHdg*180/Math.PI).q(" deltaHdg: ").q(deltaHdg*180/Math.PI).go();
            //asyncAdHocLogger.q("pos: ").q(currentPos).q(" vel: ").q(currentVel).q(" index: ").q(m_currentIndex).go();

            if(degrees < 0){
                //turn left
                m_rightTrajectory.segments[m_currentIndex].position = currentPos;
                m_rightTrajectory.segments[m_currentIndex].velocity = currentVel;
                m_rightTrajectory.segments[m_currentIndex].acceleration = currentAcc;
                m_rightTrajectory.segments[m_currentIndex].heading = hdg;
                m_rightTrajectory.segments[m_currentIndex].dt = Constants.kLooperDt;

                double ratio = (outerRadiusInches - (Constants.kWheelBaseFeet * 12)) / outerRadiusInches;
                int indexToUse = m_currentIndex>0?m_currentIndex-1:0;
                m_leftTrajectory.segments[m_currentIndex].position = m_leftTrajectory.segments[indexToUse].position + (deltaPos * ratio * (backwards?-1:1));
                m_leftTrajectory.segments[m_currentIndex].velocity = currentVel * ratio;
                m_leftTrajectory.segments[m_currentIndex].acceleration = currentAcc * ratio;
                m_leftTrajectory.segments[m_currentIndex].heading = hdg;
                m_leftTrajectory.segments[m_currentIndex].dt = Constants.kLooperDt;
            }else{
                //turn right
                m_leftTrajectory.segments[m_currentIndex].position = currentPos;
                m_leftTrajectory.segments[m_currentIndex].velocity = currentVel;
                m_leftTrajectory.segments[m_currentIndex].acceleration = currentAcc;
                m_leftTrajectory.segments[m_currentIndex].heading = hdg;
                m_leftTrajectory.segments[m_currentIndex].dt = Constants.kLooperDt;

                double ratio = (outerRadiusInches - (Constants.kWheelBaseFeet * 12)) / outerRadiusInches;
                int indexToUse = m_currentIndex>0?m_currentIndex-1:0;
                m_rightTrajectory.segments[m_currentIndex].position = m_rightTrajectory.segments[indexToUse].position + (deltaPos * ratio * (backwards?-1:1));
                m_rightTrajectory.segments[m_currentIndex].velocity = currentVel * ratio;
                m_rightTrajectory.segments[m_currentIndex].acceleration = currentAcc * ratio;
                m_rightTrajectory.segments[m_currentIndex].heading = hdg;
                m_rightTrajectory.segments[m_currentIndex].dt = Constants.kLooperDt;
            }
            m_currentIndex++;
        }
    }
}