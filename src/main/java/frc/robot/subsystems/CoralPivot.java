// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralPivot extends SubsystemBase {
  /** Creates a new CoralPivot. */
  SparkFlex pivotPoint;

  public CoralPivot() {
    pivotPoint = new SparkFlex(26, MotorType.kBrushless);
  }

  public void pivotConfig() {
    SparkFlexConfig configPivot = new SparkFlexConfig();

    pivotPoint.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    configPivot.inverted(false).idleMode(IdleMode.kCoast);

    // configPivot.encoder

    configPivot.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0, 0);

    pivotPoint.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
