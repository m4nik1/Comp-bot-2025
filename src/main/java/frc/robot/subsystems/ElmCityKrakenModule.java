// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElmCityKrakenModule extends SubsystemBase {
  /** Creates a new ElmCityKrakenModule. */

  TalonFX driveMotor;
  TalonFX angleMotor;

  PositionDutyCycle anglePosition = new PositionDutyCycle(0);
  DutyCycleOut driveOpenLoop;
  VelocityVoltage driveVelocity;

  public int modNum;
  public double velocitySet;

  Rotation2d lastAngle;
  Rotation2d offset;
  InvertedValue driveInverted;

  public ElmCityKrakenModule(int moduleNumber, int driveID, int angleID,
      InvertedValue driveInvert,
      InvertedValue angleInvert) {
    this.modNum = moduleNumber;
    this.driveInverted = driveInvert;

    driveMotor = new TalonFX(driveID);
    angleMotor = new TalonFX(angleID);

    velocitySet = 0;

    // 0 is default position
    anglePosition.Slot = 0;

    driveOpenLoop = new DutyCycleOut(0);
    driveVelocity = new VelocityVoltage(0);
    driveVelocity.Slot = 0;

    configDriveMotor(driveInvert);
    configAngleMotor();

    driveMotor.setPosition(0.0);
    lastAngle = Rotation2d.fromDegrees(0);
    // resetToAbsolute();
    angleMotor.setPosition(0);
  }

  public void configDriveMotor(InvertedValue drive) {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    driveConfig.MotorOutput.Inverted = drive;
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 60;

    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.getConfigurator().setPosition(0.0);


  }

  public void configAngleMotor() {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());

    angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    angleConfig.CurrentLimits.StatorCurrentLimit = 60;

    angleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    angleConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    angleMotor.getConfigurator().apply(angleConfig);
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble();
  }

  public double getDriveVel() {
    return driveMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Mod angle " + modNum, angleMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Mod Pos " + modNum, driveMotor.getPosition().getValueAsDouble());
  }
}
