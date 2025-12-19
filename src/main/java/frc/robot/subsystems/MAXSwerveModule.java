// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units; // Added for unit conversion
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final CANcoder absEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, int deviceID, double chassisAngularOffsetDegrees) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getEncoder();
    absEncoder = new CANcoder(deviceID);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // FIX 1: Convert the offset from Degrees (Constants file) to Rotations (SparkMax unit)
    m_chassisAngularOffset = Units.degreesToRotations(chassisAngularOffsetDegrees);
    
    // FIX 2: Seed the relative encoder immediately. 
    // We subtract the offset here so we don't have to do it every loop.
    // Get absolute position (Rotations) - Offset (Rotations)
    double absolutePosition = absEncoder.getAbsolutePosition().getValueAsDouble();
    m_turningEncoder.setPosition(absolutePosition - m_chassisAngularOffset);
    
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   */
  public SwerveModuleState getState() {
    // FIX 3: Just read the encoder. It is already offset-corrected in the constructor.
    // Also, converted to Rotation2d using Rotations.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        Rotation2d.fromRotations(m_turningEncoder.getPosition()));
  }

  private double driveVolts = 0.0;
  private double turnVolts = 0.0;

  public void setDriveVoltage(double volts) {
    this.m_drivingSpark.setVoltage(volts);
    this.driveVolts = volts;
  }

  public void setTurnVoltage(double volts) {
    this.m_turningSpark.setVoltage(volts);
    this.turnVolts = volts;
  }

  /**
   * Returns the current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // FIX 4: Just read the encoder. No extra math needed.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        Rotation2d.fromRotations(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimization: avoid spinning further than 90 degrees.
    // We pass the CURRENT rotation to the optimize method.
    SwerveModuleState correctedDesiredState = new SwerveModuleState(
        desiredState.speedMetersPerSecond, desiredState.angle);
        
    correctedDesiredState.optimize(Rotation2d.fromRotations(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    
    // FIX 5: CRITICAL! Send ROTATIONS to the PID controller, not Radians.
    // The SparkMax is configured to wrap 0 to 1 (Rotations).
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRotations(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public double getTurnPositionInRotations() {
    return MathUtil.inputModulus(m_turningEncoder.getPosition(), 0, 1);
  }

  private int num = 0; // You might want to pass this in constructor for better debugging labels

  public void updateTelemetry() {
    SmartDashboard.putNumber("Turn Pos Rotations#" + num, getTurnPositionInRotations());
    SmartDashboard.putNumber("Raw turn pos " + num, m_turningEncoder.getPosition());
    SmartDashboard.putNumber("Drive Vel #" + num, m_drivingEncoder.getVelocity());
    SmartDashboard.putNumber("Drive Pos #" + num, m_drivingEncoder.getPosition());
    SmartDashboard.putNumber("drive voltage #" + num, m_drivingSpark.getBusVoltage());
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    // Re-seed turning encoder to account for drift if necessary
    double absolutePosition = absEncoder.getAbsolutePosition().getValueAsDouble();
    m_turningEncoder.setPosition(absolutePosition - m_chassisAngularOffset);
  }
}