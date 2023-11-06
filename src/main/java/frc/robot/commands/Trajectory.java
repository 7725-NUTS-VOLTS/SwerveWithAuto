// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Trajectory extends CommandBase {

  private ProfiledPIDController rotPID;
  private HolonomicDriveController hController;
  private PathPlannerState state; 
  private Pose2d currentPose;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private PathContainer pathContainer;
  private final Timer timer = new Timer();
  private Swerve m_Swerve;
  private PathPlannerTrajectory path;

  /** Creates a new Trajectory. */
  public Trajectory(Swerve m_Swerve, PathContainer pathContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Swerve = m_Swerve;
    this.pathContainer = pathContainer;
    rotPID = Constants.AutoConstants.thetaPidController;
    path = pathContainer.getTrajectory();
    addRequirements(m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotPID.enableContinuousInput(-Math.PI, Math.PI);
    hController = new HolonomicDriveController(Constants.AutoConstants.X_CONTROLLER, Constants.AutoConstants.Y_CONTROLLER, rotPID);
    timer.stop();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var currentTime = timer.get();
    state = (PathPlannerState) path.sample(currentTime);
    currentPose = m_Swerve.getPose();
    speeds = hController.calculate(currentPose, state, state.holonomicRotation);
    m_Swerve.setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds));

  }

  public double getTime(){
    return timer.get();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
