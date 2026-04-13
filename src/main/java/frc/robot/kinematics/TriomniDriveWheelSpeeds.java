// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

/** Add your docs here. */
public class TriomniDriveWheelSpeeds {
    public double m_frontSpeed;
    public double m_leftSpeed;
    public double m_rightSpeed;

    TriomniDriveWheelSpeeds(double frontSpeed, double leftSpeed, double rightSpeed){
        m_frontSpeed = frontSpeed;
        m_leftSpeed = leftSpeed;
        m_rightSpeed = rightSpeed;
    }
    public TriomniDriveWheelSpeeds plus(TriomniDriveWheelSpeeds other){
        return new TriomniDriveWheelSpeeds(
            m_frontSpeed + other.m_frontSpeed,
            m_leftSpeed + other.m_leftSpeed,
            m_rightSpeed + other.m_rightSpeed);
    }
    public TriomniDriveWheelSpeeds minus(TriomniDriveWheelSpeeds other){
        return new TriomniDriveWheelSpeeds(
            m_frontSpeed - other.m_frontSpeed,
            m_leftSpeed - other.m_leftSpeed,
            m_rightSpeed - other.m_rightSpeed);
    }
    public TriomniDriveWheelSpeeds times(double scalar){
        return new TriomniDriveWheelSpeeds(
            m_frontSpeed * scalar,
            m_leftSpeed * scalar,
            m_rightSpeed * scalar);
    }
   /**
   * Renormalizes the wheel speeds if any either side is above the specified maximum.
   *
   * <p>Sometimes, after inverse kinematics, the requested speed from one or more wheels may be
   * above the max attainable speed for the driving motor on that wheel. To fix this issue, one can
   * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
   * absolute threshold, while maintaining the ratio of speeds between wheels.
   *
   * @param attainableMaxSpeed The absolute max speed that a wheel can reach.
   */
  public void desaturate(double attainableMaxSpeed) {
    double realMaxSpeed = Math.max(Math.abs(m_frontSpeed), Math.max(Math.abs(m_leftSpeed), Math.abs(m_rightSpeed)));

    if (realMaxSpeed > attainableMaxSpeed) {
      double factor = attainableMaxSpeed / realMaxSpeed;
      m_frontSpeed = m_frontSpeed * factor;
      m_rightSpeed = m_rightSpeed * factor;
      m_leftSpeed = m_leftSpeed * factor;
    }
  }

    public TriomniDriveWheelSpeeds unaryMinus(){
        return new TriomniDriveWheelSpeeds(-m_frontSpeed, -m_leftSpeed, -m_rightSpeed);
    }
    public String toString(){
        return "TriomniDriveWheelSpeeds(" + m_frontSpeed + ", " + m_leftSpeed + ", " + m_rightSpeed + ")";
    }
}
