// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrainKrakens extends SubsystemBase {
  /** Creates a new TestDrive. */

  ElmCityKrakenModule[] elmCityModules;

  Pigeon2 gyro;



  public DriveTrainKrakens() {
    elmCityModules = new ElmCityKrakenModule[] {
      new ElmCityKrakenModule(0, 1, 2, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityKrakenModule(1, 15, 14,  InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityKrakenModule(2, 1, 2, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityKrakenModule(3, 4, 3, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive)
    };
    gyro = new Pigeon2(21);
  }

  @Override
  public void periodic() {

    // First update pose with vision and other sensors
    // updatePose();

    // Updates the robot pose for the Robot itself
    // robotPose = getRobotPose2d();
    // field.setRobotPose(robotPose);

  }
}
 