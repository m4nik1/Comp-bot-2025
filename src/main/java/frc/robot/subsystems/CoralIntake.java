// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  SparkMax coralMax;

  DigitalInput coralDetector;
  
  public CoralIntake() {
    // Change the pivot to its own subsystem
    coralMax = new SparkMax(31, MotorType.kBrushless);

    coralDetector = new DigitalInput(2);

    coralMax.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runCoral(double speed){
    coralMax.set(speed * 0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Coral Power", coralMax.getAppliedOutput());
    SmartDashboard.putNumber("Intake power", coralMax.getAppliedOutput());
    SmartDashboard.putBoolean("Coral Detector", coralDetector.get());
  }
}
