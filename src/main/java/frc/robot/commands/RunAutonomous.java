// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetTrajectory.PoseAtTime;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutonomous extends SequentialCommandGroup {
  /** Creates a new RunAutonomous. */
  Swerve mSwerve;
  PathPlannerTrajectory trajectory;
  PathContainer pathContainer;
  public RunAutonomous(Swerve mSwerve, PathContainer pathContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.mSwerve = mSwerve;
    this.pathContainer = pathContainer;
    addRequirements(mSwerve);
    addCommands(
      new ResetTrajectory(mSwerve, pathContainer, PoseAtTime.START),
      new Trajectory(mSwerve, pathContainer).withTimeout(pathContainer.getTimeout()),
      new ConditionalCommand(new ResetTrajectory(mSwerve, pathContainer, PoseAtTime.END), new InstantCommand(), pathContainer:: getResetOnEnd)
    );
  }
}
