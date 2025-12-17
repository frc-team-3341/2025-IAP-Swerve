// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;
import frc.robot.Constants;

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
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, int deviceID, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getEncoder();
    absEncoder = new CANcoder(deviceID);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    this.m_turningEncoder.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }
   public double getTurnPositionInRotations() {
        // Position should be already offsetted in constructor
        // Modulus is needed if the wheel is spun too much relative to the encoder's starting point
        // the min and max are taken from config.closedLoop.positionWrappingInputRange(0, 1);
        return MathUtil.inputModulus(m_turningEncoder.getPosition(), 0, 1);
    }

  private int num = 0;
 public void updateTelemetry() {
        // ESSENTIAL TELEMETRY
        // Show turning position and setpoints
        SmartDashboard.putNumber("Turn Pos Rotations#" + num, getTurnPositionInRotations());
        SmartDashboard.putNumber("Raw turn pos " + num, m_turningEncoder.getPosition());
        // Show driving velocity
        SmartDashboard.putNumber("Drive Vel #" + num, m_drivingEncoder.getVelocity());
        SmartDashboard.putNumber("Drive Pos #" + num, m_drivingEncoder.getPosition());

        SmartDashboard.putNumber("drive voltage #" + num, m_drivingSpark.getBusVoltage());

        // NON-ESSENTIAL TELEMETRY
        //if (Constants.enableSwerveMotorTelemetry && num == 1) {
            /** 
             *
            // Show driving velocity setpoints
            SmartDashboard.putNumber("Setpoint Drive Vel #" + this.num, state.speedMetersPerSecond);

            // Show turning position and setpoints
            SmartDashboard.putNumber("Radian Turn Pos #" + num, getTurnPositionInRad());
            SmartDashboard.putNumber("Rad Setpoint Turn Pos #" + this.num, state.angle.getRadians());
            SmartDashboard.putNumber("Setpoint Turn Pos Deg #" + this.num, Units.radiansToDegrees(state.angle.getRadians()));

            // Get RPMs
            SmartDashboard.putNumber("Turn RPM #" + this.num, (turnEncoder.getVelocity() / 360.0) * 60.0);
            SmartDashboard.putNumber("Drive RPS #" + this.num,
                    driveEncoder.getVelocity() / Constants.ModuleConstants.drivingEncoderPositionFactor);
            SmartDashboard.putNumber("CANCoder rotation#" + this.num, canCoder.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Drive Motor Voltage #" + this.num, driveSparkMax.getAppliedOutput());

            // Output of driving
            SmartDashboard.putNumber("Turn Volts #" + this.num, this.turnVolts);
            SmartDashboard.putNumber("Drive Volts #" + this.num, this.driveVolts);

            // Get Wheel Displacement
            SmartDashboard.putNumber("Wheel Displacement #" + this.num, getPosition().distanceMeters);
            */
        }       
    

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}