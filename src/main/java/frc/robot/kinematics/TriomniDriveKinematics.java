// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;

/** Kinematics for equal-sided triangular robot with 3 omniwheels with one wheel in front.
 * If wheel 1 is in front, 2 is back left and 3 is back right,
 * and robot's x axis is straight ahead (so front wheel is at (radius,0), center at (0,0))
 * and its y axis is 90 degrees counterclockwise from x axis,
 * the basic kinematic relations, in matrix form, are
 * 
 * (Vx    )     (        0  -sqrt(3)/2 sqrt(3)/2 )   (Vwheel1)
 * (Vy    )  =  (        1        -1/2      -1/2 ) * (Vwheel2)
 * (Vtheta)     ( 1/radius    1/radius  1/radius )   (Vwheel3)
 * 
 * The inverse matrix is
 * (          0         2/3  radius/3)
 * ( -sqrt(3)/3        -1/3  radius/3)
 * (  sqrt(3)/3        -1/3  radius/3)
 * 
 */
public class TriomniDriveKinematics implements Kinematics<TriomniDriveWheelSpeeds, TriomniDriveWheelPositions>{
    private final double m_robotRadius; // distance from robot center to any of the 3 omniwheels
    private final SimpleMatrix m_forwardKinematics;
    private final SimpleMatrix m_inverseKinematics;
    // the following two are scratch vectors
    private final SimpleMatrix m_chassisSpeedsVector = new SimpleMatrix(3,1);
    private final SimpleMatrix m_wheelSpeedsVector = new SimpleMatrix(3,1);

    public TriomniDriveKinematics(double robotRadius){
        m_robotRadius = robotRadius;
        m_forwardKinematics = new SimpleMatrix(3, 3);
        setForwardKinematics();
        m_inverseKinematics = m_forwardKinematics.pseudoInverse();
        System.out.println("forward kinematics = " + m_forwardKinematics);
        System.out.println("inverse kinematics = " + m_inverseKinematics);
    }

    private void setForwardKinematics(){
        double thirtyDegrees = Math.PI/6.0;
        double cos30 = Math.cos(thirtyDegrees); // sqrt(3)/2
        double sin30 = Math.sin(thirtyDegrees); // 1/2
        double reciprocalRadius = 1.0 / m_robotRadius;
        m_forwardKinematics.setRow(0, 0, 0.0, -cos30, cos30);
        m_forwardKinematics.setRow(1, 0, 1.0, -sin30, -sin30);
        m_forwardKinematics.setRow(2, 0, reciprocalRadius, reciprocalRadius, reciprocalRadius);
    }
    // We expect that computing the inverse of the forward kinematics will give the same as the following
    // m_inverseKinematics.setRow(0, 0, 0.0, 2.0/3.0, m_robotRadius/3.0);
    // m_inverseKinematics.setRow(1, 0, -Math.sqrt(3.0)/3, -1.0/3.0, m_robotRadius/3.0);
    // m_inverseKinematics.setRow(2, 0, Math.sqrt(3.0)/3.0, -1.0/3.0, m_robotRadius/3.0);

    @Override
    public TriomniDriveWheelPositions copy(TriomniDriveWheelPositions positions){
        return new TriomniDriveWheelPositions(positions.m_frontPosition, positions.m_leftPosition, positions.m_rightPosition);
    }
    @Override
    public void copyInto(TriomniDriveWheelPositions positions, TriomniDriveWheelPositions output){
        output.m_frontPosition = positions.m_frontPosition;
        output.m_leftPosition = positions.m_leftPosition;
        output.m_rightPosition = positions.m_rightPosition;
    }
    @Override
    public TriomniDriveWheelPositions interpolate(TriomniDriveWheelPositions start, TriomniDriveWheelPositions end, double t){
        return start.interpolate(end, t);
    }
    @Override
    public ChassisSpeeds toChassisSpeeds(TriomniDriveWheelSpeeds wheelSpeeds) {
        m_wheelSpeedsVector.setColumn(0, 0, 
                                    wheelSpeeds.m_frontSpeed,
                                    wheelSpeeds.m_leftSpeed,
                                    wheelSpeeds.m_rightSpeed);
        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(m_wheelSpeedsVector);
        return new ChassisSpeeds(chassisSpeedsVector.get(0, 0),
                                 chassisSpeedsVector.get(1, 0),
                                 chassisSpeedsVector.get(2, 0));
    }
    @Override
    public TriomniDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeedsVector.setColumn(0, 0, chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
        SimpleMatrix wheelSpeedsVector = m_inverseKinematics.mult(m_chassisSpeedsVector);
        return new TriomniDriveWheelSpeeds(wheelSpeedsVector.get(0, 0),
                                           wheelSpeedsVector.get(1, 0),
                                           wheelSpeedsVector.get(2, 0));
    }
    @Override
    public Twist2d toTwist2d(TriomniDriveWheelPositions start, TriomniDriveWheelPositions end) {
        return toTwist2d(end.minus(start));
    }

    public Twist2d toTwist2d(TriomniDriveWheelPositions delta){
        SimpleMatrix diffVector = new SimpleMatrix(3, 1);
        diffVector.setColumn(0, 0, 
                             delta.m_frontPosition,
                             delta.m_leftPosition,
                             delta.m_rightPosition);
        SimpleMatrix twistVector = m_forwardKinematics.mult(diffVector);
        return new Twist2d(twistVector.get(0, 0),
                           twistVector.get(1, 0),
                           twistVector.get(2, 0));
    }
}
