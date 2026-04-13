// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class TriomniDriveWheelPositions implements Interpolatable<TriomniDriveWheelPositions>, Sendable{
    public double m_frontPosition;
    public double m_leftPosition;
    public double m_rightPosition;

    // Positions must give distance travelled (not rotations) for odometry to work.
    public TriomniDriveWheelPositions(double frontPosition, double leftPosition, double rightPosition){
        m_frontPosition = frontPosition;
        m_leftPosition = leftPosition;
        m_rightPosition = rightPosition;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Front", ()->m_frontPosition, null);
        builder.addDoubleProperty("Left", ()->m_leftPosition, null);
        builder.addDoubleProperty("Right", ()->m_rightPosition, null);
    }

    @Override
    public String toString(){
        return "TriomniDriveWheelPositions(" + m_frontPosition + ", " + m_leftPosition + ", " + m_rightPosition + ")"; 
    }
    @Override
    public boolean equals(Object obj){
       return obj instanceof TriomniDriveWheelPositions other
        && Math.abs(other.m_frontPosition - m_frontPosition) < 1E-9
        && Math.abs(other.m_leftPosition - m_leftPosition) < 1E-9
        && Math.abs(other.m_rightPosition - m_rightPosition) < 1E-9;
    }
    @Override
    public TriomniDriveWheelPositions interpolate(TriomniDriveWheelPositions endValue, double t){
        return new TriomniDriveWheelPositions(
            MathUtil.interpolate(m_frontPosition, endValue.m_frontPosition, t),
            MathUtil.interpolate(m_leftPosition, endValue.m_leftPosition, t),
            MathUtil.interpolate(m_rightPosition, endValue.m_rightPosition, t)
        );
    }

    public TriomniDriveWheelPositions minus(TriomniDriveWheelPositions other){
        return new TriomniDriveWheelPositions(m_frontPosition - other.m_frontPosition,
                                              m_leftPosition - other.m_leftPosition,
                                              m_rightPosition - other.m_rightPosition);
    }

    public void update(double frontPosition, double leftPosition, double rightPosition){
        m_frontPosition = frontPosition;
        m_leftPosition = leftPosition;
        m_rightPosition = rightPosition;
    }
}
