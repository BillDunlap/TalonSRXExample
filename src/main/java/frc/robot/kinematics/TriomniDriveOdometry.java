// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;

/** Add your docs here. */

public class TriomniDriveOdometry extends Odometry<TriomniDriveWheelPositions> {
  /**
   * Constructs a TriomniDriveOdometry object.
   *
   * @param kinematics The kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The distances driven by each wheel.
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public TriomniDriveOdometry(
      TriomniDriveKinematics kinematics,
      Rotation2d gyroAngle,
      TriomniDriveWheelPositions wheelPositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
  }
    /**
   * Constructs a TriomniDriveOdometry object with the default pose at the origin.
   *
   * @param kinematics The kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The distances driven by each wheel.
   */
  public TriomniDriveOdometry(
      TriomniDriveKinematics kinematics,
      Rotation2d gyroAngle,
      TriomniDriveWheelPositions wheelPositions) {
    this(kinematics, gyroAngle, wheelPositions, Pose2d.kZero);
  }
}
