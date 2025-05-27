// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonFX algaeRun;
  DigitalInput coralDetector;
  DigitalInput algaeDetector;
  TalonFXConfiguration algaeConfig;
  // Make sure elevator is at 8 inches off top of bumper
  
  public AlgaeIntake() {
   algaeRun = new TalonFX(36);
   coralDetector = new DigitalInput(2);
   algaeDetector = new DigitalInput(3);

   algaeConfig = new TalonFXConfiguration();
   algaeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

   algaeRun.getConfigurator().apply(algaeConfig);
  }

  public void runIntake(double speed) {
    algaeRun.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Algae Detect", algaeDetector.get());
    Logger.recordOutput("Coral Detect", coralDetector.get());

  }
}
