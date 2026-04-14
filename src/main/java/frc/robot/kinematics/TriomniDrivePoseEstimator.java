// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This class wraps {@link TriomniDriveOdometry Triomni Drive Odometry} to fuse latency-compensated
 * vision measurements with mecanum drive encoder distance measurements. It will correct for noisy
 * measurements and encoder drift. It is intended to be a drop-in replacement for {@link
 * TriomniDriveOdometry}.
 *
 * <p>{@link TriomniDrivePoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link TriomniDrivePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave mostly like regular encoder odometry.
 */
public class TriomniDrivePoseEstimator extends PoseEstimator<TriomniDriveWheelPositions> {
  /**
   * Constructs a TriomniDrivePoseEstimator with default standard deviations for the model and
   * vision measurements.
   *
   * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
   * and 0.1 radians for heading. The default standard deviations of the vision measurements are
   * 0.45 meters for x, 0.45 meters for y, and 0.45 radians for heading.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param wheelPositions The distances driven by each wheel.
   * @param initialPoseMeters The starting pose estimate.
   */
  public TriomniDrivePoseEstimator(
      TriomniDriveKinematics kinematics,
      Rotation2d gyroAngle,
      TriomniDriveWheelPositions wheelPositions,
      Pose2d initialPoseMeters) {
    this(
        kinematics,
        gyroAngle,
        wheelPositions,
        initialPoseMeters,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.45, 0.45, 0.45));
  }

  /**
   * Constructs a TriomniDrivePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param wheelPositions The distance measured by each wheel.
   * @param initialPoseMeters The starting pose estimate.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
   *     in meters, and heading in radians). Increase these numbers to trust your state estimate
   *     less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public TriomniDrivePoseEstimator(
      TriomniDriveKinematics kinematics,
      Rotation2d gyroAngle,
      TriomniDriveWheelPositions wheelPositions,
      Pose2d initialPoseMeters,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super(
        kinematics,
        new TriomniDriveOdometry(kinematics, gyroAngle, wheelPositions, initialPoseMeters),
        stateStdDevs,
        visionMeasurementStdDevs);
  }
}
