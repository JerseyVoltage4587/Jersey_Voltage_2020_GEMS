package frc.robot.paths;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

public class PathReader {
	private Trajectory left;
	private Trajectory right;
	public Trajectory getLeftTraj() {
		return left;
	}
	public Trajectory getRightTraj() {
		return right;
	}
	public PathReader(String filename){

		
		try{
		BufferedReader r = new BufferedReader(new FileReader(new File("/home/lvuser/"+filename+".csv") ));
		String line = r.readLine();
		int length = Integer.parseInt(line);
		line = r.readLine();
		
		//	    w.write(seg.dt+","+ seg.x+","+ seg.y+","+ seg.position+","+ seg.velocity+","+ 
	    //        seg.acceleration+","+ seg.jerk+","+ seg.heading+"\n");
		Segment[] leftSegments = new Segment[length];
		Segment[] rightSegments = new Segment[length];
		for(int i = 0; i < length; i++){
			line = r.readLine();
			String[] fields = line.split(",");
			leftSegments[i] = new Segment(
				Double.parseDouble(fields[0]),
				Double.parseDouble(fields[1]),
				Double.parseDouble(fields[2]),
				Double.parseDouble(fields[3]),
				Double.parseDouble(fields[4]),
				Double.parseDouble(fields[5]),
				Double.parseDouble(fields[6]),
				Double.parseDouble(fields[7])
			);
		}
		for(int i = 0; i < length; i++){
			line = r.readLine();
			String[] fields = line.split(",");
			rightSegments[i] = new Segment(
				Double.parseDouble(fields[0]),
				Double.parseDouble(fields[1]),
				Double.parseDouble(fields[2]),
				Double.parseDouble(fields[3]),
				Double.parseDouble(fields[4]),
				Double.parseDouble(fields[5]),
				Double.parseDouble(fields[6]),
				Double.parseDouble(fields[7])
			);
		}

		left = new Trajectory(leftSegments);
		right = new Trajectory(rightSegments);
		
		r.close();
		}catch(Exception e){
			
		}
	}
	
	
}
