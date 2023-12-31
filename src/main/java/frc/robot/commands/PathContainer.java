// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
/** Add your docs here. */
public class PathContainer {
    String pathString;
    PathPlannerTrajectory trajectory;
    double maxSpeed, maxAcceleration;
    static double timeout;
    boolean first, resetOnEnd;

    public PathContainer(String pathString, double[] speeds, double timeout, boolean first, boolean resetOnEnd){
        this.pathString = pathString;
        this.timeout = timeout;
        maxSpeed = speeds[0];
        maxAcceleration = speeds[1];
        trajectory = PathPlanner.loadPath(pathString, speeds[0], speeds[1]);

        

    }

    public String getPathString(){
        return pathString;
    }

    public PathPlannerTrajectory getTrajectory(){
        return trajectory;
    }

    public double getMaxSpeed(){
        return maxSpeed;
    }

    public double getMaxAcceleration(){
        return maxAcceleration;
    }

    public double getTimeout(){
        return timeout;
    }

    public boolean getFirst(){
        return first;
    }

    public boolean getResetOnEnd(){
        return resetOnEnd;
    }

}
