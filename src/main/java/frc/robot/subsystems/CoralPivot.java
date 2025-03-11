// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;

import org.littletonrobotics.junction.Logger;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralPivot extends SubsystemBase {
  /** Creates a new CoralPivot. */
  SparkFlex pivotPoint;
  DigitalInput pivotLimit;
  SparkClosedLoopController pivotClosedLoop;
  double pivotAngleConversion;

  boolean referenceSet;

  public CoralPivot() {
    pivotPoint = new SparkFlex(35, MotorType.kBrushless);
    pivotLimit = new DigitalInput(1);
    pivotClosedLoop = pivotPoint.getClosedLoopController();
    pivotAngleConversion = (1/9) * 180;

    pivotConfig();
  }

  public void pivotConfig() {
    SparkFlexConfig configPivot = new SparkFlexConfig();

    pivotPoint.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    configPivot.inverted(false).idleMode(IdleMode.kBrake);
    configPivot.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    // configPivot.encoder.positionConversionFactor(pivotAngleConversion).velocityConversionFactor(1);

    configPivot.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0, 0, 0, 0);

    pivotPoint.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runPivotManual(double speed) {
    referenceSet = false;
    pivotPoint.set(speed * 0.15);
  }

  public boolean getPivotLimit() {
    return pivotLimit.get();
  }

  public void setPivot(double pos) {
    double anglePivot = (pivotPoint.getEncoder().getPosition()) * (1/9) * 180;
    Logger.recordOutput("Pivot Convert Deg", anglePivot);
    double kP = 0.01;
    double kG = 0.0195;

    // Vtot = kp*(Rset - Rfb) + kg*sin(arm_angle)
    double pidCalculate = kP * (pos - pivotPoint.getEncoder().getPosition()) + kG * Math.sin(Rotation2d.fromDegrees(anglePivot).getRadians());

    Logger.recordOutput("Pivot PID calculated", pidCalculate);

    pivotPoint.setVoltage(pidCalculate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Pivot Position", pivotPoint.getEncoder().getPosition());
    Logger.recordOutput("Pivot Limit", getPivotLimit());

    SmartDashboard.putNumber("Pivot Pos", pivotPoint.getEncoder().getPosition());

    SmartDashboard.putNumber("Pivot Spd", pivotPoint.get());
    SmartDashboard.putBoolean("Pivot Limit", getPivotLimit());

    if(getPivotLimit() == true) {
      pivotPoint.getEncoder().setPosition(0);
    }
  }
}
 