// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MecanumDrive;

public class ExamplePathAuto extends SequentialCommandGroup {

  PathPlannerTrajectory trajectory = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));

  public ExamplePathAuto(MecanumDrive drivetrain) {
    addCommands(
      drivetrain.followTrajectoryCommand(trajectory, true).withTimeout(trajectory.getTotalTimeSeconds())
    );
  }
}
