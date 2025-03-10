// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonFXS algaeRun;
  DigitalInput algaeDetector;
  DigitalInput algaeDetector2;
  // Make sure elevator is at 8 inches off top of bumper
  
  public AlgaeIntake() {
   algaeRun = new TalonFXS(36);
   algaeDetector = new DigitalInput(2);
   algaeDetector2 = new DigitalInput(3);


   algaeRun.getConfigurator().apply(new TalonFXSConfiguration());
  }

  public void runIntake(double speed) {
    algaeRun.set(speed);
  }

  public boolean algaeDetect() {
    return algaeDetector.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Detect 1", algaeDetect());
    SmartDashboard.putBoolean("Detect 2", algaeDetector2.get());
  }
}
