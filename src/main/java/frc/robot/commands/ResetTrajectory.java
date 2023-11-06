// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

import com.revrobotics.SparkMaxAbsoluteEncoder;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetTrajectory extends InstantCommand {
  Swerve mSwerve;
  PathContainer pathContainer;
  PathPlannerTrajectory trajectory;
  PoseAtTime poseAtTime;

  
  public enum PoseAtTime{
    START,END;
  }


  public ResetTrajectory(Swerve mSwerve, PathContainer pathContainer, PoseAtTime poseAtTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    this.pathContainer= pathContainer;
    this.poseAtTime=poseAtTime;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    trajectory = PathPlanner.loadPath(pathContainer.getPathString(), pathContainer.getMaxSpeed(), pathContainer.getMaxAcceleration());
if(poseAtTime.equals(PoseAtTime.START)){
  if(pathContainer.getFirst()){
  //set offset del modulo 
  //mSwerve.setOffset(-trajectory.getInitialState().holonomicRotation.getDegrees());
  
  //}

  mSwerve.resetOdometry(new Pose2d(trajectory.getInitialState().poseMeters.getTranslation(), trajectory.getInitialState().holonomicRotation));
}
else{
  mSwerve.resetOdometry(new Pose2d(trajectory.getEndState().poseMeters.getTranslation(), trajectory.getEndState().holonomicRotation));
}
  }
}
}


