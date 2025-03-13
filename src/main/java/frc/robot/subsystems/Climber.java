// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX climberMotor = new TalonFX(13);
  DutyCycleOut climbDuty;

  public Climber() {
    configClimber();
    climberMotor.setPosition(0);

    climbDuty = new DutyCycleOut(0);
  }

  public void configClimber() {
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbConfig.Feedback.SensorToMechanismRatio =  100/1;

    climbConfig.Slot0.kP = Constants.climberkP;
    climbConfig.Slot0.kI = Constants.climberkI;
    climbConfig.Slot0.kD = Constants.climberkD;

    climberMotor.getConfigurator().apply(climbConfig);
  }

  public void runClimber(double speed) {
    climberMotor.set(-speed);
  }



  @Override
  public void periodic() {
    Logger.recordOutput("Climber Motor", climberMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
