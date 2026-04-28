// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TriomniDrive;

public class RobotContainer {
  private TriomniDrive m_driveTrain;
  private CommandXboxController m_xboxController;
  private static final double inchesPerMeter = 39.37;
  private final double m_wheelDiameter = 8.0/inchesPerMeter ;  // in meters
  private final double m_robotRadius = 17.0/inchesPerMeter;    // in meters
  private final Pose2d m_initialPose = new Pose2d(5.0, 5.0, new Rotation2d(0.0));
  public RobotContainer() {
    m_xboxController = new CommandXboxController(0);
    m_driveTrain = new TriomniDrive(m_wheelDiameter, m_robotRadius, m_initialPose);
    SmartDashboard.putData("Drive Train", m_driveTrain);
    configureBindings();
  }

  private void configureBindings() {
    // for(int degrees = 45; degrees < 360; degrees += 45){
    //   m_xboxController.pov(degrees)
    //     .onTrue(m_driveTrain.goStraightCommand_voltage_radians(degrees/45.0, 0.0))
    //     .onFalse(m_driveTrain.goStraightCommand_voltage_radians(0.0, 0.0));
    // }
    m_driveTrain.setDefaultCommand(m_driveTrain.teleop(
      () -> -m_xboxController.getLeftY(),
      () -> -m_xboxController.getLeftX(),
      () -> -m_xboxController.getRightX(),
      () -> !m_xboxController.leftBumper().getAsBoolean() // field oriented by default, turn off by pressing left bumper
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
