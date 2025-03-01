// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrainKrakens extends SubsystemBase {
  /** Creates a new TestDrive. */

  ElmCityKrakenModule[] elmCityModules;

  Pigeon2 gyro;
  Pose2d robotPose;

  SwerveDrivePoseEstimator odom;
  Field2d field;


  public DriveTrainKrakens() {
    elmCityModules = new ElmCityKrakenModule[] {
      new ElmCityKrakenModule(0, 1, 2, 0, Constants.angleOffsetMod0, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      // new ElmCityKrakenModule(1, 15, 14, 25, Constants.angleOffsetMod1, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
      // new ElmCityKrakenModule(2, 1, 2, 26, Constants.angleOffsetMod2, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      // new ElmCityKrakenModule(3, 4, 3, 27, Constants.angleOffsetMod3, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive)
    };

    field = new Field2d();
    gyro = new Pigeon2(Constants.pigeonID);

    // odom = new SwerveDrivePoseEstimator(Constants.swerveKinematics, getYaw(), getPositions(), new Pose2d());
    
    resetGyro();

    // new Thread(() -> {
    //   try {
    //     Thread.sleep(1000);
    //     odom.resetPosition(new Rotation2d(), getPositions(), new Pose2d());
    //     } catch(Exception e) {}
    // }).start();
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public void setDriveVelocity() {
    for(ElmCityKrakenModule m : elmCityModules) {
      m.runVelocity(1);
    }
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(ElmCityKrakenModule k : elmCityModules) {
      positions[k.modNum] = elmCityModules[k.modNum].getPosition();
    }

    return positions;
  }


  public void resetGyro() {
    gyro.getConfigurator().setYaw(0.0);

    gyro.reset();
  }

  public Pose2d getRobotPose2d() {
    Pose2d robotPose = odom.update(getYaw(), getPositions());

    return robotPose;
  }

  public void resetPose(Pose2d pose) {
    odom.resetPose(pose);
  }

  public void updatePose() {
    odom.update(getYaw(), getPositions());

    Optional<EstimatedRobotPose> photonPose = RobotContainer.photonCamera.getEstimatedGlobalPose();

    // Adds Vision measurement for odometry
    odom.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 999999)); 
    odom.addVisionMeasurement(photonPose.get().estimatedPose.toPose2d(), photonPose.get().timestampSeconds);
  }

  public void setAngle(double deg) {
    elmCityModules[0].goToAngle(deg);
    // elmCityModules[4].goToAngle(deg);
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
 