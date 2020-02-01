package frc.robot.paths;

import java.io.File;
import java.io.FileWriter;


import frc.robot.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class PathWriter {
	public static void writePath(String filename, Waypoint[] points, boolean isReversed){
		//System.out.println("writing path");
		Trajectory left;
		Trajectory right;
		
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, Constants.kStepSizeSeconds, 
				Constants.kMaxFeetPerSecond, Constants.kMaxAcceleration, Constants.kMaxJerk);

		// Generate the trajectory
		Trajectory trajectory = Pathfinder.generate(points, config);
		
		// The distance between the left and right sides of the wheelbase is 0.6m
		double wheelbase_width = Constants.kWheelBaseFeet;

		// Create the Modifier Object
		TankModifier modifier = new TankModifier(trajectory);

		// Generate the Left and Right trajectories using the original trajectory
		// as the centre
		modifier.modify(wheelbase_width);

		left  = modifier.getLeftTrajectory();       // Get the Left Side
		right = modifier.getRightTrajectory();      // Get the Right Side
		
		if(isReversed){
			Segment[] leftSegments = new Segment[left.length()];
			Segment[] rightSegments = new Segment[right.length()];
			
			//double dt, double x, double y, double position, double velocity, double acceleration, double jerk, double heading
			
			for(int i = 0; i < left.length(); i++){
				leftSegments[i] = new Segment(
				right.get(i).dt,
				right.get(i).x * -1,
				right.get(i).y * -1,
				right.get(i).position * -1,
				right.get(i).velocity * -1,
				right.get(i).acceleration * -1,
				right.get(i).jerk * -1,
				right.get(i).heading
				);
			}
			for(int i = 0; i < right.length(); i++){
				rightSegments[i] = new Segment(
				left.get(i).dt,
				left.get(i).x * -1,
				left.get(i).y * -1,
				left.get(i).position * -1,
				left.get(i).velocity * -1,
				left.get(i).acceleration * -1,
				left.get(i).jerk * -1,
				left.get(i).heading
				);
			}
			
			
			
			left = new Trajectory(leftSegments);
			right = new Trajectory(rightSegments);
		}

		for(int i = 2; i < left.length()-1; i++){
			double a0 = left.get(i-2).acceleration;
			double a1 = left.get(i-1).acceleration;
			double a2 = left.get(i).acceleration;
			double dir = a1 - a0;
			double aMax;
			double aMin;
			
			if(dir<0){
				aMax=a1+Constants.kStepSizeSeconds*Constants.kMaxJerk;
				aMin=a1-2*Constants.kStepSizeSeconds*Constants.kMaxJerk;
			}else{
				aMax=a1+2*Constants.kStepSizeSeconds*Constants.kMaxJerk;
				aMin=a1-Constants.kStepSizeSeconds*Constants.kMaxJerk;
			}
			
			if(a2<aMin){
				left.get(i).acceleration=aMin;
			}
			if(a2>aMax){
				left.get(i).acceleration=aMax;
			}

			double v0 = left.get(i-2).velocity;
			double v1 = left.get(i-1).velocity;
			double v2 = left.get(i).velocity;
			double vdir = v1 - v0;
			double vMax;
			double vMin;
			
			if(vdir<0){
				vMax=v1+Constants.kStepSizeSeconds*Constants.kMaxAcceleration;
				vMin=v1-2*Constants.kStepSizeSeconds*Constants.kMaxAcceleration;
			}else{
				vMax=v1+2*Constants.kStepSizeSeconds*Constants.kMaxAcceleration;
				vMin=v1-Constants.kStepSizeSeconds*Constants.kMaxAcceleration;
			}
			
			if(v2<vMin){
				left.get(i).velocity=vMin;
			}
			if(v2>vMax){
				left.get(i).velocity=vMax;
			}
		}
		
		for(int i = 2; i < right.length()-1; i++){
			double a0 = right.get(i-2).acceleration;
			double a1 = right.get(i-1).acceleration;
			double a2 = right.get(i).acceleration;
			double dir = a1 - a0;
			double aMax;
			double aMin;
			
			if(dir<0){
				aMax=a1+Constants.kStepSizeSeconds*Constants.kMaxJerk;
				aMin=a1-2*Constants.kStepSizeSeconds*Constants.kMaxJerk;
			}else{
				aMax=a1+2*Constants.kStepSizeSeconds*Constants.kMaxJerk;
				aMin=a1-Constants.kStepSizeSeconds*Constants.kMaxJerk;
			}
			
			if(a2<aMin){
				right.get(i).acceleration=aMin;
			}
			if(a2>aMax){
				right.get(i).acceleration=aMax;
			}

			double v0 = right.get(i-2).velocity;
			double v1 = right.get(i-1).velocity;
			double v2 = right.get(i).velocity;
			double vdir = v1 - v0;
			double vMax;
			double vMin;
			
			if(vdir<0){
				vMax=v1+Constants.kStepSizeSeconds*Constants.kMaxAcceleration;
				vMin=v1-2*Constants.kStepSizeSeconds*Constants.kMaxAcceleration;
			}else{
				vMax=v1+2*Constants.kStepSizeSeconds*Constants.kMaxAcceleration;
				vMin=v1-Constants.kStepSizeSeconds*Constants.kMaxAcceleration;
			}
			
			if(v2<vMin){
				right.get(i).velocity=vMin;
			}
			if(v2>vMax){
				right.get(i).velocity=vMax;
			}
		}
		
		try{
		FileWriter w = new FileWriter(new File("/home/lvuser/"+filename+".csv") );

		w.write(left.length()+"\n");
	    w.write("dt,x,y,pos,vel,acc,jerk,heading\n");
		for (int i = 0; i < left.length(); i++) {
		    Trajectory.Segment seg = left.get(i);
		    w.write(seg.dt+","+ seg.x+","+ seg.y+","+ seg.position+","+ seg.velocity+","+ 
		            seg.acceleration+","+ seg.jerk+","+ seg.heading+"\n");
			}
		for (int i = 0; i < right.length(); i++) {
		    Trajectory.Segment seg = right.get(i);

		    w.write(seg.dt+","+ seg.x+","+ seg.y+","+ seg.position+","+ seg.velocity+","+ 
		            seg.acceleration+","+ seg.jerk+","+ seg.heading+"\n");
			}
		w.close();
		}catch(Exception e){
			
		}
	}
	
	
}
