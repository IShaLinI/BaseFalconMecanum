// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MecanumDrive;

public class RobotContainer {

  private final CommandXboxController m_Controller; //This is now part of mainline wpilib now yay!

  private final MecanumDrive m_Drivetrain;

  public RobotContainer() {

    m_Controller = new CommandXboxController(0);

    m_Drivetrain = new MecanumDrive();

    //Field Relative is a must-have for omni-drivetrains.
    m_Drivetrain.setDefaultCommand(new RunCommand(
      () -> m_Drivetrain.drive(
        -m_Controller.getLeftY(), 
        -m_Controller.getLeftX(), 
        -m_Controller.getRightX(), 
        true
      ),
      m_Drivetrain
    ));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
