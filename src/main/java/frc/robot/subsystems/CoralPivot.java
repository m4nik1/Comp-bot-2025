// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralPivot extends SubsystemBase {
  /** Creates a new CoralPivot. */
  SparkFlex pivotPoint;
  DigitalInput pivotLimit;
  SparkClosedLoopController pivotClosedLoop;
  RelativeEncoder pivotCoder;

  SimpleMotorFeedforward pivotFF;
  PIDController pivotPID;

  boolean referenceSet;

  public CoralPivot() {
    pivotPoint = new SparkFlex(35, MotorType.kBrushless);
    pivotLimit = new DigitalInput(1);
    pivotClosedLoop = pivotPoint.getClosedLoopController();
    pivotCoder = pivotPoint.getEncoder();

    pivotFF = new SimpleMotorFeedforward(0, 0, 0);
    pivotPID = new PIDController(0.001, 0, 0);

    referenceSet = false;

    pivotConfig();
  }

  public void pivotConfig() {
    SparkFlexConfig configPivot = new SparkFlexConfig();

    pivotPoint.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    configPivot.inverted(false).idleMode(IdleMode.kBrake);
    configPivot.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    // configPivot.encoder

    configPivot.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0, 0, 0, 0);

    pivotPoint.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runPivotManual(double speed) {
    referenceSet = false;
    pivotPoint.set(speed * 0.15);
  }

  public double getPivotCoder() {
    return pivotCoder.getPosition();
  }

  public boolean getPivotLimit() {
    return pivotLimit.get();
  }

  public void setPivot(double pos) {
    // double pidCalculate = pivotPID.calculate(pivotCoder.getPosition(), pos);
    double anglePivot = getPivotCoder() * (1/9) * 180;
    double kP = 0.001;
    double kG = 0.0195;
    double pidCalculate = kP * (Math.abs(pos - getPivotCoder())) + kG * Math.sin(anglePivot);

    SmartDashboard.putNumber("PIDCalculated", pidCalculate);

    // pivotPoint.setVoltage(pidCalculate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Pos", getPivotCoder());
    SmartDashboard.putNumber("Pivot Spd", pivotPoint.get());
    SmartDashboard.putBoolean("Pivot Limit", getPivotLimit());
    SmartDashboard.putNumber("Pivot Volts", pivotPoint.getAppliedOutput());
    SmartDashboard.putBoolean("Position Mode", referenceSet);

    if(getPivotLimit() == true) {
      pivotPoint.getEncoder().setPosition(0);
    }
  }
}
 