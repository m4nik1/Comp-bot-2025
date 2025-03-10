// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  TalonFX elevatorMotor;
  VoltageOut voltage;

  DigitalInput TopElevatorLimit;
  DigitalInput LowerLimit;
  MotionMagicVoltage elevatorMagic;


  public Elevator() {
    voltage = new VoltageOut(0);

    elevatorMotor = new TalonFX(16);

    TopElevatorLimit = new DigitalInput(4);
    LowerLimit = new DigitalInput(0);

    elevatorMotor.setPosition(0);

    elevatorMagic = new MotionMagicVoltage(0);

    configElevatorMotor();
  }

  public void configElevatorMotor() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorConfig.Feedback.SensorToMechanismRatio = 1/25;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    

    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // These are the gains to tune
    elevatorConfig.Slot0.kS = .30;
    elevatorConfig.Slot0.kG = .34; // run the robot with voltage on a joystick and this is the voltage making the elevator stay in place
    elevatorConfig.Slot0.kV = 0.1; // runs robot at a up ward slope
    elevatorConfig.Slot0.kA = 0.01; // Makes the curve of the set position more curvier
    
    // Needed if we dont reach our set position
    elevatorConfig.Slot0.kP = 0.18;
    elevatorConfig.Slot0.kI = 0;
    elevatorConfig.Slot0.kD = 0; 

    
    // Set for speed of elevator
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 100;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;

    elevatorMotor.getConfigurator().apply(elevatorConfig);
    elevatorMotor.setPosition(0);

  }

  public double getElevatorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();  
  }

  public double getElevatorMotorVolts() {
    return elevatorMotor.getMotorVoltage().getValueAsDouble();
  }
  public double getElevatorSupply() {
    return elevatorMotor.getSupplyVoltage().getValueAsDouble();
  }

  public void driveElevatorPercent(double percent) {
    elevatorMotor.set(percent*.40);
  }

  public void setElevatorMagic(double pos) {
    elevatorMotor.setControl(elevatorMagic.withPosition(40));
  }

  public boolean getTopLimit() {
    return TopElevatorLimit.get();
  }

  public boolean getLowerLimit() {
    return LowerLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Position Elevator", getElevatorPosition());
    SmartDashboard.putNumber("Velocity", elevatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Volts", getElevatorMotorVolts());


    if(getLowerLimit() == true) {
      elevatorMotor.setPosition(0);
    }

    if(getTopLimit()) {
      elevatorMotor.set(0);
    }

    SmartDashboard.putBoolean("Top Limit", getTopLimit());
    SmartDashboard.putBoolean("Lower Limit", getLowerLimit());
  }
}
 