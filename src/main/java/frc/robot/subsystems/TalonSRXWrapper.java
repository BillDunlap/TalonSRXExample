// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonSRXWrapper extends SubsystemBase {
  private TalonSRX m_talonSRX;
  private double m_wheelDiameter_meters; // in meters
  private double m_voltageCompensationSaturation = 10.0; // 100% == 10V, 80% = 8V (if battery has 10V or 8V, respectively, in it)
  // feedforward gains determined by running robot on medium pile carpet
  // in my basement and recording voltage and speed at three voltages.
  // Approximate results were 80 clicks/100ms forward motion at 4 volts,
  // 225 at 6 and 300 at 7.
  private SimpleMotorFeedforward m_feedForward;
  // TODO: change units of m_kV to volts/(meters/second)
  // The Talon SRX motor controller uses native units for position and velocity to ensure maximum sensor resolution.
  // Position is measured in raw encoder edges (1440 per revolution for these us digits encoders),
  // while velocity is measured in raw encoder units per 100 milliseconds
  private double m_kS = 2.9; // Volts, determined experimentally
  private double m_kV_nativeUnits = 0.01366; // Volts/(clicks/100ms), determined experimentally
  private double m_kV;  //  volts / (meters/second)
  /** Creates a new TalonSRXWrapper.
   * @param canID the CAN number for this Talon SRX motor controller
   * @param wheelDiameter
   */
  public TalonSRXWrapper(int canID, double wheelDiameter_meters){
    m_talonSRX = new TalonSRX(canID);
    m_wheelDiameter_meters = wheelDiameter_meters;
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.peakCurrentLimit = 40; // the peak current, in amps
    config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
    config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
    m_talonSRX.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

    // robot init, set voltage compensation so 100% == 10V.
    // This is talon-specific API, perhas should come with generic WPIlib API.
    m_talonSRX.configVoltageCompSaturation(m_voltageCompensationSaturation);
    m_talonSRX.enableVoltageCompensation(true);
    m_talonSRX.set(ControlMode.PercentOutput, 0.0);
    m_feedForward = new SimpleMotorFeedforward(m_kS, m_kV);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addDoubleProperty("Position(raw)", this::getPosition_raw, null);
    builder.addDoubleProperty("Velocity", this::getVelocity, null);
    builder.addDoubleProperty("Motor Voltage", this::getMotorVoltage, null);
    builder.addDoubleProperty("Bus Voltage", m_talonSRX::getBusVoltage, null);
  }

  private double nativePosition2Meters(double nat){
    return nat                                // clicks
           / 1440.0                           // /(clicks/rotations) -> rotations
           * Math.PI * m_wheelDiameter_meters // *meters/rotation -> meters
          ;
  }

  private double nativeVelocity2metersPerSecond(double nat){
    return nativePosition2Meters(
      nat    // clicks/100ms
      * 10.0   // *100ms/seconds -> clicks/second
    );  // -> meters/second
  }

  public double getPosition_raw(){
    return m_talonSRX.getSelectedSensorPosition();
  }
  public double getPosition(){
    return nativePosition2Meters(getPosition_raw());
  }

  public double getVelocity_raw(){
    return m_talonSRX.getSelectedSensorVelocity();
  }
  public double getVelocity(){
    return nativeVelocity2metersPerSecond(getVelocity_raw());
  }

  public void setMotorVoltage(double voltage){
    m_talonSRX.set(TalonSRXControlMode.PercentOutput, voltage/m_voltageCompensationSaturation);
  }

  public double getMotorVoltage(){
    return m_talonSRX.getMotorOutputVoltage();
  }

  public Command setVoltageCommand(double voltage){
    return new InstantCommand(()->setMotorVoltage(voltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
