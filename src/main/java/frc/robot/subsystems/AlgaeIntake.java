// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonFXS algaeRun;
  DigitalInput coralDetector;
  DigitalInput algaeDetector;
  TalonFXSConfiguration algaeConfig;
  // Make sure elevator is at 8 inches off top of bumper
  
  public AlgaeIntake() {
   algaeRun = new TalonFXS(36);
   coralDetector = new DigitalInput(2);
   algaeDetector = new DigitalInput(3);

   algaeConfig = new TalonFXSConfiguration();
   algaeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

   algaeRun.getConfigurator().apply(algaeConfig);
  }

  public void runIntake(double speed) {
    algaeRun.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Coral Detector", coralDetector.get());
    SmartDashboard.putBoolean("Algae Detector", algaeDetector.get());
  
    
    Logger.recordOutput("Coral Detector", coralDetector.get());
    Logger.recordOutput("Algae Detector", algaeDetector.get());
  }
}
