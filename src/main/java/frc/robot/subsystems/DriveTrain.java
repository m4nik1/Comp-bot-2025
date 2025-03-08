// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new TestDrive. */

  ElmCityModule[] elmCityModules;

  Pigeon2 gyro;
  Pose2d robotPose;

  SwerveDriveOdometry odom;
  Field2d field;


  public DriveTrain() {
    elmCityModules = new ElmCityModule[] {
      new ElmCityModule(0, 8, 7, 0,Constants.angleOffsetMod0,InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(1, 20, 19, 2, Constants.angleOffsetMod1, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(2, 10, 9, 1, Constants.angleOffsetMod2 ,InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(3, 17, 18, 3, Constants.angleOffsetMod3, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
    };

    gyro = new Pigeon2(Constants.pigeonID);
    
    // odom = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), getPositions());

    resetGyro();
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(ElmCityModule k : elmCityModules) {
      positions[k.modNum] = elmCityModules[k.modNum].getPosition();
    }

    return positions;
  }


  public void resetGyro() {
    gyro.getConfigurator().setYaw(0.0);

    gyro.reset();
  }

    public void drive(Translation2d translation, double rotation) {
    SwerveModuleState[] moduleStates;

    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw());
    spds = ChassisSpeeds.discretize(spds, .02);
    moduleStates = Constants.swerveKinematics.toSwerveModuleStates(spds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.maxSpeed);

    for(ElmCityModule m : elmCityModules) {
      m.setDesiredState(moduleStates[m.modNum], true);
    }
  }

  public Pose2d getRobotPose2d() {
    Pose2d robotPose = odom.update(getYaw(), getPositions());

    return robotPose;
  }

  public double getRobotAngle() {
    return gyro.getYaw().getValueAsDouble();
  }

  public void setAngle(double deg){
  
    elmCityModules[0].goToAngle(deg);
  }

  public void setDriveVelocity(double vel) {
    for(ElmCityModule m : elmCityModules) {
      m.runVelocity(vel);
    }
  }


  @Override
  public void periodic() {

    // First update pose with vision and other sensors
    // updatePose();
    // odom.update(getYaw(), getPositions());

    // Updates the robot pose for the Robot itself
    SmartDashboard.putNumber("Robot Angle", getRobotAngle());



  }
}
 