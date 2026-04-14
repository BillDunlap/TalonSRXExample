// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GyroWrapper;
import frc.robot.kinematics.TriomniDriveKinematics;
import frc.robot.kinematics.TriomniDriveOdometry;
import frc.robot.kinematics.TriomniDrivePoseEstimator;
import frc.robot.kinematics.TriomniDriveWheelPositions;


public class TriomniDrive extends SubsystemBase {
  private TalonSRXWrapper m_frontMotor;
  private TalonSRXWrapper m_rightMotor;
  private TalonSRXWrapper m_leftMotor;

  private double m_wheelDiameter;
  private double m_robotRadius; // distance from center of robot to each wheel
  private final TriomniDriveWheelPositions m_wheelPositions;
  private final TriomniDriveKinematics m_driveKinematics;
  private final GyroWrapper m_gyro;
  private TriomniDriveOdometry m_odometry;
  private Pose2d m_pose2d_from_odometry;
  private TriomniDrivePoseEstimator m_poseEstimator;
  private Pose2d m_pose2d_from_poseEstimator;

  private double m_speed_voltage = 0.0;

  // feedforward constants
  private final double m_kS = 2.9; // volts needed to overcome static friction on medium pile carpet
  private final double m_kV = 0.01366; // voltage increase needed to increase speed by 1 encoder click per 100ms
  private final double m_kA = 0.0; // not considering acceleration in feedforward calculation
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(m_kS, m_kV, m_kA);
   
  /** Creates a new TriomniDrive. */
  public TriomniDrive(double wheelDiameter, double robotRadius) {
    m_wheelDiameter = wheelDiameter;
    m_robotRadius = robotRadius;
    m_frontMotor = new TalonSRXWrapper(1, m_wheelDiameter);
    SmartDashboard.putData("Front Motor", m_frontMotor);
    m_rightMotor = new TalonSRXWrapper(2, m_wheelDiameter );
    SmartDashboard.putData("Right Motor", m_rightMotor);
    m_leftMotor = new TalonSRXWrapper(3, m_wheelDiameter);
    SmartDashboard.putData("Left Motor", m_leftMotor);

    m_gyro = new GyroWrapper(0, GyroWrapper.WhichGyro.NavX, m_robotRadius);
    SmartDashboard.putData("Gyro", m_gyro);
    m_driveKinematics = new TriomniDriveKinematics(m_robotRadius);
    m_wheelPositions = new TriomniDriveWheelPositions(0.0, 0.0, 0.0);
    SmartDashboard.putData("Wheel Pos", m_wheelPositions);
    m_odometry = new TriomniDriveOdometry(m_driveKinematics, m_gyro.getRotation2d(), m_wheelPositions);
    m_poseEstimator = new TriomniDrivePoseEstimator(m_driveKinematics, m_gyro.getRotation2d(), m_wheelPositions, Pose2d.kZero);  // m_driveKinematics, m_gyro.getRotation2d(), m_wheelPositions);
  }

  /** Move in a straight line in a robot-relative direction
   * @param speed speed in volts
   * @param direction_radians direction in radians, counterclockwise from front of robot
   */
  public void goStraight_voltage_radians(double speed_voltage, double direction_radians){
    m_speed_voltage = speed_voltage;
    double frontSpeed = speed_voltage * Math.cos(direction_radians - 9.0/6.0 * Math.PI);
    double rightSpeed = speed_voltage * Math.cos(direction_radians - 5.0/6.0 * Math.PI);
    double leftSpeed = speed_voltage * Math.cos(direction_radians - 1.0/6.0 * Math.PI);
    m_frontMotor.setMotorVoltage(frontSpeed);
    m_rightMotor.setMotorVoltage(rightSpeed);
    m_leftMotor.setMotorVoltage(leftSpeed);
  }

  public Command goStraightCommand_voltage_radians(double speed_voltage, double direction_radians){
    return new InstantCommand(() -> goStraight_voltage_radians(speed_voltage, direction_radians));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Requested Voltage", m_speed_voltage);
    
    m_wheelPositions.update(m_frontMotor.getPosition(), m_leftMotor.getPosition(), m_rightMotor.getPosition());
    m_pose2d_from_odometry = m_odometry.update(m_gyro.getRotation2d(), m_wheelPositions);
    Logger.recordOutput("Pose(from Odometry)", m_pose2d_from_odometry);
    
    m_pose2d_from_poseEstimator = m_poseEstimator.update(m_gyro.getRotation2d(), m_wheelPositions);
    Logger.recordOutput("Pose(from PoseEstimator)", m_pose2d_from_poseEstimator);
  }
}
