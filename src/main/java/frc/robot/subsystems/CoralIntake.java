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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SuppliedWait;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  SparkMax coralMax;
  
  public CoralIntake() {
    // Change the pivot to its own subsystem
    coralMax = new SparkMax(31, MotorType.kBrushless);

    coralMax.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runCoral(double speed){
    coralMax.set(speed * 0.5);
  }

  // .raceWith is the simpler way to state parallel command
  public Command coralOut() {
    return run(() -> runCoral(0.7))
      .raceWith(new SuppliedWait(() -> 0.7));
  }

  public Command coralIn() {
    return run(() -> runCoral(-0.7))
      .raceWith(new SuppliedWait(() -> 0.7));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Coral Power", coralMax.getAppliedOutput());
    SmartDashboard.putNumber("Intake power", coralMax.getAppliedOutput());
  }
}
