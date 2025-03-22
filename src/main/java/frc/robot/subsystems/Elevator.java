// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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
    

    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TopElevatorLimit;

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
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 300;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 300;

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
    elevatorMotor.setControl(elevatorMagic.withPosition(pos));
  }

  public boolean isElevatorL4() {
    double elevator_pos = getElevatorPosition();
    return elevator_pos >= (Constants.elevator_l4 + -7);
  }

  public BooleanSupplier isElevatorL4Supplier() {
    double elevator_pos = getElevatorPosition();

    BooleanSupplier l4_supplier = () -> elevator_pos >= Constants.elevator_l4 + -4;

    return l4_supplier;
  }

  public boolean isElevatorL3() {
    double elevator_pos = getElevatorPosition();
    return elevator_pos >= Constants.elevator_l3;
  }

  public boolean isElevatorHP() {
    double elevator_pos = getElevatorPosition();
    return elevator_pos >= (Constants.elevator_HP + -5);
  }
  
  public boolean isElevatorAt(double pos) {
    double elevator_pos = getElevatorPosition();
    return elevator_pos >= (pos + -5);
  }

  public Command setElevatorPos(double pos) {
    return run(() -> setElevatorMagic(pos))
      .until(() -> getElevatorPosition() <= pos + 5);
  }

  public Command setElevatorL4() {
    return run(() -> setElevatorMagic(Constants.elevator_l4))
      .until(isElevatorL4Supplier());
  }

  public Command scoreL4() {
    return new SequentialCommandGroup(
      setElevatorL4(),
      RobotContainer.coralIntake.coralOut()
    );
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


    if(getLowerLimit() == true && DriverStation.isDisabled() && getElevatorPosition() != 0) {
      elevatorMotor.getConfigurator().setPosition(0);
    }

    if(getLowerLimit() == true && DriverStation.isEnabled() && getElevatorPosition() != 0) {
      elevatorMotor.getConfigurator().setPosition(0);
    }

    Logger.recordOutput("Is at l4", isElevatorL4());
    SmartDashboard.putBoolean("Top Limit", getTopLimit());
    SmartDashboard.putBoolean("Lower Limit", getLowerLimit());
  }
}
 