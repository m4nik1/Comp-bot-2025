// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  SparkMax coralMax;
  

  SparkFlex PivotPoint;
  public CoralIntake() {
    // Change the pivot to its own subsystem
    PivotPoint = new SparkFlex(26, MotorType.kBrushless);

    coralMax = new SparkMax(36, MotorType.kBrushless);
  }

  public void runCoral(double speed){
    coralMax.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
