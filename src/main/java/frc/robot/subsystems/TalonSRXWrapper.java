// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonSRXWrapper extends SubsystemBase {
  private TalonSRX m_talonSRX;
  private double m_wheelDiameter_meters; // in meters
  private boolean m_invertMotorVoltage;
  private double m_voltageCompensationSaturation = 10.0; // 100% == 10V, 80% = 8V (if battery has 10V or 8V, respectively, in it)
  // feedforward gains determined by running robot on medium pile carpet
  // in my basement and recording voltage and speed at three voltages.
  // Approximate results were 80 clicks/100ms forward motion at 4 volts,
  // 225 at 6 and 300 at 7.
  // The Talon SRX motor controller uses native encoder units for position and velocity "to ensure maximum sensor resolution".
  // This triomnidrive robot has US Digital E4P-360-250 quadrature encoders.
  //    360 cycles (== 1440 clicks) per revolution (the 250 refers to hole for quarter inch shaft).
  // Position is measured in encoder edges (1440 per revolution),
  // while velocity is measured in position units per 100 milliseconds
  private double m_kS = 0.0; // 2.9; // Volts to overcome static friction medium pile carpet, determined experimentally
  private double m_kV_nativeUnits = 0.01; // 0.01366; // Volts/(clicks/100ms), determined experimentally
  private double m_desiredSpeed_meters_per_second ;
  private double m_desiredSpeed_nativeUnits;
  private final double m_clicks_per_revolution = 1440; 
  private double m_initialKP = 0.010;
  private double m_initialKI = 0.0;
  private double m_initialKD = 0.0;
  private PIDController m_pidController = new PIDController(m_initialKP, m_initialKI, m_initialKD);
  private SimpleMotorFeedforward m_feedForward;
  private double m_ffVolts;
  private double m_pidVolts;
  // private SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(100.0); // don't change motor voltage by more than that many volts each second.

  /** Creates a new TalonSRXWrapper.
   * @param canID the CAN number for this Talon SRX motor controller
   * @param wheelDiameter
   */
  public TalonSRXWrapper(int canID, double wheelDiameter_meters, boolean invertMotorVoltage){
    m_talonSRX = new TalonSRX(canID);
    m_wheelDiameter_meters = wheelDiameter_meters;
    m_invertMotorVoltage = invertMotorVoltage;

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
    m_feedForward = new SimpleMotorFeedforward(m_kS, m_kV_nativeUnits);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addDoubleProperty("Position(raw)", this::getPosition_raw, null);
    builder.addDoubleProperty("Velocity", this::getvelocity_nativeUnits, null);
    builder.addDoubleProperty("Desired Velocity", ()->m_desiredSpeed_nativeUnits, null);
    builder.addDoubleProperty("Motor Voltage", this::getMotorVoltage, null);
    builder.addDoubleProperty("Bus Voltage", m_talonSRX::getBusVoltage, null);
    builder.addDoubleProperty("ffVolts", ()->m_ffVolts, null);
    builder.addDoubleProperty("pidVolts", ()->m_pidVolts, null);
    builder.addDoubleProperty("kP", () -> m_pidController.getP(), (kP) -> { m_pidController.setP(kP); });
    builder.addDoubleProperty("kS", () -> m_feedForward.getKs(), (kS) -> { m_feedForward.setKs(kS); });
  }

  private double nativePosition2Meters(double nat){
    return nat                                // clicks
           / m_clicks_per_revolution          // /(clicks/rotations) -> rotations
           * Math.PI * m_wheelDiameter_meters // *meters/rotation -> meters
          ;
  }

  private double nativeVelocity2metersPerSecond(double nat){
    return nativePosition2Meters(
      nat    // clicks/100ms
      * 10.0   // *100ms/seconds -> clicks/second
    );  // -> meters/second
  }

  private double metersToNativePosition(double meters){
    return meters                                       // meters
           * 1.0 / (Math.PI * m_wheelDiameter_meters)   // revolutions/meter
           * m_clicks_per_revolution                    // clicks/revolution
    ;                                                   // -> clicks
  }

  private double metersPerSecondToNativeVelocity(double metersPerSecond){
    return metersToNativePosition(
      metersPerSecond    // meters/second
      * 0.1              // seconds/100ms
    );                   //  -> clicks/100ms
  }

  public double getPosition_raw(){
    return m_talonSRX.getSelectedSensorPosition();
  }
  public double getPosition(){
    return nativePosition2Meters(getPosition_raw());
  }

  public double getvelocity_nativeUnits(){
    return m_talonSRX.getSelectedSensorVelocity();
  }
  public double getVelocity(){
    return nativeVelocity2metersPerSecond(getvelocity_nativeUnits());
  }

  /**
   * run motor at given voltage
   * @param voltage
   */
  public void setMotorVoltage(double voltage){
    if (m_invertMotorVoltage){
      voltage = -voltage;
    }
    m_talonSRX.set(TalonSRXControlMode.PercentOutput, voltage/m_voltageCompensationSaturation);
  }

  public double getMotorVoltage(){
    double voltage = m_talonSRX.getMotorOutputVoltage();
    if (m_invertMotorVoltage){
      voltage = -voltage;
    }
    return voltage;
  }

  public Command setVoltageCommand(double voltage){
    return new InstantCommand(()->setMotorVoltage(voltage));
  }

  public void setSpeed(double speed){
    m_desiredSpeed_meters_per_second = speed;
    m_desiredSpeed_nativeUnits = metersPerSecondToNativeVelocity(m_desiredSpeed_meters_per_second);
  }

  public void setKP(double kP){
    m_pidController.setP(kP);
  }

  public double getKP(){
    return m_pidController.getP();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    // Get near desired speed.
    m_ffVolts = m_feedForward.calculate(m_desiredSpeed_nativeUnits);
    // use PID feedback loop to keep speed near desired speed.
    m_pidVolts = m_pidController.calculate(getvelocity_nativeUnits(), m_desiredSpeed_nativeUnits);
    double volts = m_ffVolts + m_pidVolts;
    volts = MathUtil.clamp(volts, -12.0, 12.0);
    // volts = m_slewRateLimiter.calculate(volts);
    setMotorVoltage(volts);
  }
}
