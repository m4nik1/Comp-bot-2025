// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  PositionDutyCycle climberDuty;
  TalonFX climberMotor = new TalonFX(13);


  public Climber() {
    configClimber();

    climberDuty = new PositionDutyCycle(0);
    climberMotor.setPosition(0);

    climberDuty.Slot = 0;
  }

  public void climbPosition(double pos) {
    climberDuty.Slot = 0;
    climberDuty.Position = climberMotor.getPosition().getValueAsDouble();
    climberMotor.setControl(climberDuty);
  }

  public void configClimber() {
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbConfig.Feedback.SensorToMechanismRatio =  100/1;

    climbConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    climbConfig.Slot0.kP = Constants.climbP;
    climbConfig.Slot0.kI = Constants.climbI;
    climbConfig.Slot0.kD = Constants.climbD;

    climberMotor.getConfigurator().apply(climbConfig);
  }

  public void runClimber(double speed) {
    climberMotor.set(speed);
  }

  public double getClimberPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  public double getClimberVolts() {
    return climberMotor.getMotorVoltage().getValueAsDouble();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
