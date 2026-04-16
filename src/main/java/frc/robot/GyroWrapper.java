// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A common interface for two gyros, Pigeon 2.0 (from REV) and NavX (from Studica).
 * This will try to start both of them and use the one that it can start.
 * If both are responsive, use the WhichGyro enum say which one to get the 
 * data from.
 * 
 * TODO: the Pigeon2 branch is never used, fix that.
 */
public class GyroWrapper extends SubsystemBase {

  /** Creates a new Gyro. */
  private Pigeon2 m_pigeon;
  private AHRS m_navX;
  private boolean m_usePigeon;
  private double m_yawOffsetPigeon2;
  private double m_yawOffsetNavX;
  private double m_currentYawPigeon2;
  private double m_currentYawNavX;

  public enum WhichGyro{
    Pigeon2,
    NavX
  }

  private WhichGyro m_whichGyro;

  @Override
  public void periodic(){
    m_currentYawNavX = getYawNavX();
    m_currentYawPigeon2 = getYawPigeon2();
  }
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Yaw", this::getYaw, null);
    builder.addStringProperty("Gyro", () -> m_whichGyro.toString(), null);
  }
  public GyroWrapper(int pigeonCanID, WhichGyro whichGyro, double startingYaw_degrees) {
    
    m_pigeon = new Pigeon2(pigeonCanID);
    StatusCode statusCode = m_pigeon.getConfigurator().apply(new Pigeon2Configuration()); // replaces .configFactoryDefault()
    if (statusCode.isError()) {
      System.out.println("Pigeon2 has a problem: " + statusCode);
      m_pigeon = null;
      m_usePigeon = false;
    } else {
      m_pigeon.reset();
    }
    m_navX = new AHRS(NavXComType.kMXP_SPI);
    if (!calibrateNavX()){
      m_navX = null;
    }
    if (m_navX != null && m_pigeon != null){
      m_whichGyro = whichGyro;
    } else if (m_navX != null){
      m_whichGyro = WhichGyro.NavX;
    } else if (m_pigeon != null){
      m_whichGyro = WhichGyro.Pigeon2;
    }
    System.out.println("Asked to use Gyro " + whichGyro + "; using Gyro " + m_whichGyro);
    setYaw_degrees(startingYaw_degrees);
  }

  public void setYaw_degrees(double degreesCcw) {
    if (m_pigeon != null) {
      m_pigeon.reset();
      m_yawOffsetPigeon2 = degreesCcw;
    }
    if (m_navX != null) {
      m_navX.reset();
      m_yawOffsetNavX = degreesCcw + m_navX.getYaw();
    }
  }

  private double getYawPigeon2() {
    return m_pigeon != null
      ? m_pigeon.getYaw().getValueAsDouble() + m_yawOffsetPigeon2
      : 0.0;
  }

  private double getYawNavX() {
    return m_navX != null
      ? -m_navX.getYaw() + m_yawOffsetNavX
      : 0.0;
  }

  public double getYaw() {
    double retVal = 0;
    switch (m_whichGyro) {
      case Pigeon2:
        retVal = m_currentYawPigeon2; 
        break;
      case NavX:
        retVal = m_currentYawNavX;
        break;
    }
    return retVal;
  }

  public double getRoll() {
    if (m_usePigeon) {
      return m_pigeon != null ? m_pigeon.getRoll().getValueAsDouble() : 0.0;
    } else {
      return m_navX.getRoll();
    }
  }

  public double getPitch() {
    if (m_usePigeon) {
      return m_pigeon != null ? m_pigeon.getPitch().getValueAsDouble() : 0.0;
    } else {
      return m_navX.getPitch();
    }
  }

  public void resetGyro() {
    if (m_usePigeon) {
      m_pigeon.reset();
    } else {
      m_navX.reset();
    }
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  /**
   * Make sure gyro is done calibrating before using it
   * @return
   * true if calibration was successful, false if we could not connect to or could not calibrate NavX
   */
  private boolean calibrateNavX() {
    // calibration only needed for NavX
    int nTries = 1;
    boolean retval = true;
    while (m_navX.isCalibrating() && nTries < 100) { //wait to zero yaw if calibration is still running
      try {
        Thread.sleep(20);
        System.out.println("----calibrating gyro---- " + nTries);
      } catch (InterruptedException e) {}
      nTries++;
      if (nTries >= 50 && nTries % 10 == 0) {
        System.out.println("Having trouble calibrating NavX");
      }
    }
    try {
      Thread.sleep(60); // sometimes isConnected returns false immediately after calibration
    } catch (InterruptedException e) {
      // do nothing
    }
    if (m_navX.isCalibrating()) {
      System.out.println("Could not calibrate NavX, will use Pigeon2");
      retval = false;
    } else if (!m_navX.isConnected()) {
      System.out.println(
        "NavX is not connected (is SPI dip switch not ON?), will use Pigeon2"
      );
      retval = false;
    }
    return retval;
  }
}

