// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GyroWrapper;
import frc.robot.kinematics.TriomniDriveKinematics;
import frc.robot.kinematics.TriomniDriveOdometry;
import frc.robot.kinematics.TriomniDrivePoseEstimator;
import frc.robot.kinematics.TriomniDriveWheelPositions;
import frc.robot.kinematics.TriomniDriveWheelSpeeds;


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
  private ChassisSpeeds m_desiredChassisSpeeds;
  private TriomniDriveWheelSpeeds m_desiredWheelSpeeds;
  // PERFORMANCE CONSTANTS
  private double m_maxWheelSpeed = 2.5; // m/s.  This should take about 10 volts on basement carpet
  private double m_maxRot = Math.PI/2;  // radians/s.  maximum rotation rate.

  private double m_speed_voltage = 0.0;
  // on my robot, running motor with positive voltage makes robot
  // twist clockwise.  We want positive twist to be counterclockwise,
  // so invert the voltage before giving it to motor.
  private final boolean m_invertMotorVoltage = true;
  
  /** Creates a new TriomniDrive. */
  public TriomniDrive(double wheelDiameter, double robotRadius, Pose2d initialPoseMeters) {
    m_wheelDiameter = wheelDiameter;
    m_robotRadius = robotRadius;
    m_frontMotor = new TalonSRXWrapper(1, m_wheelDiameter, m_invertMotorVoltage);
    SmartDashboard.putData("Front Motor", m_frontMotor);
    m_rightMotor = new TalonSRXWrapper(2, m_wheelDiameter, m_invertMotorVoltage);
    SmartDashboard.putData("Right Motor", m_rightMotor);
    m_leftMotor = new TalonSRXWrapper(3, m_wheelDiameter, m_invertMotorVoltage);
    SmartDashboard.putData("Left Motor", m_leftMotor);

    m_gyro = new GyroWrapper(0, GyroWrapper.WhichGyro.NavX, m_robotRadius);
    SmartDashboard.putData("Gyro", m_gyro);
    m_driveKinematics = new TriomniDriveKinematics(m_robotRadius);
    m_wheelPositions = new TriomniDriveWheelPositions(0.0, 0.0, 0.0);
    SmartDashboard.putData("Wheel Pos", m_wheelPositions);
    m_odometry = new TriomniDriveOdometry(m_driveKinematics, m_gyro.getRotation2d(), m_wheelPositions, initialPoseMeters);
    m_poseEstimator = new TriomniDrivePoseEstimator(m_driveKinematics, m_gyro.getRotation2d(), m_wheelPositions, initialPoseMeters);  // m_driveKinematics, m_gyro.getRotation2d(), m_wheelPositions);
  }

  /** Tell the robot to move in the given direction at the given speed (relative to robot)
   * If the given chassis speed cannot be done in the given direction,
   * lower that speeds so that the direction can be maintained.
   * 
   * @chassisSpeeds a ChassisSpeeds object giving the desired velocity
   */
  public void applySpeeds(ChassisSpeeds chassisSpeeds){
    m_desiredChassisSpeeds = chassisSpeeds;
    m_desiredWheelSpeeds = m_driveKinematics.toWheelSpeeds(m_desiredChassisSpeeds);
    m_desiredWheelSpeeds.desaturate(m_maxWheelSpeed);
    m_frontMotor.setSpeed(m_desiredWheelSpeeds.m_frontSpeed);
    m_leftMotor.setSpeed(m_desiredWheelSpeeds.m_leftSpeed);
    m_rightMotor.setSpeed(m_desiredWheelSpeeds.m_rightSpeed);
  }

   /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds =
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);
    applySpeeds(chassisSpeeds);
  }

  /** Move in a straight line in a robot-relative direction, specified by raw voltage
   * This is meant to be used to calibrate the motor constants, estimating
   * the relationship between voltage and robot speed, not for general use.
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

  public Command teleop(DoubleSupplier getXSpeed, DoubleSupplier getYSpeed, DoubleSupplier getYawSpeed, BooleanSupplier getFieldOriented){
      return new InstantCommand(() -> {
        // the DoubleSuppliers are expected to give values in [-1,1]: convert to meters/second
        double vX = m_maxWheelSpeed * MathUtil.applyDeadband(getXSpeed.getAsDouble(), 0.05, 1.0);
        double vY = m_maxWheelSpeed * MathUtil.applyDeadband(getYSpeed.getAsDouble(), 0.05, 1.0);
        double vYaw = m_maxRot * MathUtil.applyDeadband(getYawSpeed.getAsDouble(), 0.05, 1.0);
        drive(vX, vY, vYaw, getFieldOriented.getAsBoolean());
      }, this);
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

    Logger.recordOutput("Desired Chassis Speeds", m_desiredChassisSpeeds);

  }
}
