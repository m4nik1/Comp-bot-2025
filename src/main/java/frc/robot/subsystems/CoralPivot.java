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
import frc.robot.Constants;

public class CoralPivot extends SubsystemBase {
  /** Creates a new CoralPivot. */
  SparkFlex pivotPoint;
  DigitalInput pivotLimit;
  RelativeEncoder pivotEncoder;

  double pivotAngleConversion;
  double pidCalculate = 0;

  public CoralPivot() {
    pivotPoint = new SparkFlex(35, MotorType.kBrushless);
    pivotLimit = new DigitalInput(1);
    pivotEncoder = pivotPoint.getEncoder();
    // pivotClosedLoop = pivotPoint.getClosedLoopController();
    pivotAngleConversion = (1/20) * 180;

    pivotPoint.getEncoder().setPosition(0);

    pivotConfig();
  }

  public void pivotConfig() {
    SparkFlexConfig configPivot = new SparkFlexConfig();

    pivotPoint.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    configPivot.idleMode(IdleMode.kBrake);
    configPivot.inverted(false).idleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("Conversion pivot", pivotAngleConversion);

    // configPivot.encoder.positionConversionFactor(pivotAngleConversion).velocityConversionFactor(1);
    configPivot.encoder.positionConversionFactor(18).velocityConversionFactor(1);

    // configPivot.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0, 0, 0, 0);

    pivotPoint.configure(configPivot, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runPivotManual(double speed) {
    pivotPoint.set(speed * 0.20);
  }

  public boolean getPivotLimit() {
    return pivotLimit.get();
  }

  public boolean isPivotDown() {
    return pivotPoint.getEncoder().getPosition() <= Constants.pivot_down;
  }

  public void setPivot(double pos) {
    double kP = 0.043; // retune this for new pivot
    double kG = 0.03; // change this to .027

    // Vtot = kp*(Rset - Rfb) + kg*sin(arm_angle)
    // Change the the fromDegrees(anglePivot) to fromDegrees(pivotPoint.getEncoder().getPosition())
    // If pivot tuning is not working just add -40 to the encoder and tune with that value
    pidCalculate = kP * (pos - (pivotPoint.getEncoder().getPosition() + -30)) + kG * Math.sin(Rotation2d.fromDegrees(pivotPoint.getEncoder().getPosition() + -30).getRadians());

    Logger.recordOutput("Pivot PID calculated", pidCalculate);

    pivotPoint.setVoltage(pidCalculate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.recordOutput("Pivot Position", pivotPoint.getEncoder().getPosition() + -30);
    SmartDashboard.putNumber("Pivot Pos", pivotPoint.getEncoder().getPosition() + -30);
    
    Logger.recordOutput("Pivot Limit", getPivotLimit());
    Logger.recordOutput("Pivot Volts", pivotPoint.getAppliedOutput());
    SmartDashboard.putNumber("Pivot Spd", pivotPoint.get());
    SmartDashboard.putBoolean("Pivot Limit", getPivotLimit());

    if(getPivotLimit() == true) {
      pivotPoint.getEncoder().setPosition(0);
    }
  }
}
 