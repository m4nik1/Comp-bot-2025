// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new TestDrive. */

  ElmCityModule[] elmCityModules;

  Pigeon2 gyro;
  Pose2d robotPose;

  private Matrix<N3, N1> kSingleTagStdDevs;
  private Matrix<N3, N1> kMultiTagStdDevs;

  SwerveDriveOdometry odom;
  Field2d field;
  RobotConfig autoConfig;


  public DriveTrain() {
    elmCityModules = new ElmCityModule[] {

      // TODO: If drive tuning takes longer than 10 minutes set all drive motors CounterClockwise positive
      // TODO: Then set the bevel to face left robot relative and then tunee
      new ElmCityModule(0, 8, 7, 0,Constants.angleOffsetMod0,InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(1, 20, 19, 2, Constants.angleOffsetMod1, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(2, 10, 9, 1, Constants.angleOffsetMod2 ,InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(3, 17, 18, 3, Constants.angleOffsetMod3, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
    };

    gyro = new Pigeon2(Constants.pigeonID);
    odom = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), getPositions());


    kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    try {
      autoConfig = RobotConfig.fromGUISettings();
    } catch(Exception e) {
      e.printStackTrace();
    }
    
    AutoBuilder.configure(
      this::getPose,
      this::resetPose,
      this::getRobotSpds,
      (speeds, feedforwards) -> driveRobotRelative(speeds),
      new PPHolonomicDriveController(
            new PIDConstants(0.5, 0, 0), 
            new PIDConstants(2.7, 0, 0)
      ),
      autoConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );

    resetGyro();
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public Pose2d getPose() {
    return odom.getPoseMeters(); // returns pose in meters
  }

  public void resetPose(Pose2d pose) {
    odom.resetPosition(getYaw(), getPositions(), pose);
  }

  public ChassisSpeeds getRobotSpds() {
    return Constants.swerveKinematics.toChassisSpeeds(
      elmCityModules[0].getState(),
      elmCityModules[1].getState(),
      elmCityModules[2].getState(),
      elmCityModules[3].getState()
    );
  }

  public void driveRobotRelative(ChassisSpeeds spds) {
    SwerveModuleState states[] = Constants.swerveKinematics.toSwerveModuleStates(spds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

    for(ElmCityModule m : elmCityModules) {
      m.setDesiredState(states[m.modNum], false);
    }
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

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[] {
      elmCityModules[0].getState(),
      elmCityModules[1].getState(),
      elmCityModules[2].getState(),
      elmCityModules[3].getState(),
    };

    return states;
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

  public void addVisionMeasurment(Pose2d visionRobotPose, double visionTimestamp, boolean isSingleTarget) {
    Matrix<N3, N1> visionStds = isSingleTarget ? kSingleTagStdDevs : kMultiTagStdDevs;

    // Uncomment these when swerveOdomPoseEstimation is added
    // odom.addVisionMeasurement(visionRobotPose, visionTimestamp, visionStds);

    // double visionToDriveTrainPose = visionRobotPose.getTranslation().getDistance(robotPose.getTranslation());

    // Logger.recordOutput("Distance to Vision measurement", visionToDriveTrainPose);
    // Logger.recordOutput("Is Vision Close to Drivetrain", visionToDriveTrainPose < 0.5);

  }

  public void zeroAngles() {
    for(ElmCityModule m : elmCityModules) {
      m.setAngleZero();
    }
  }

  @Override
  public void periodic() {

    // First update pose with vision and other sensors
    // updatePose();
    odom.update(getYaw(), getPositions());

    Logger.recordOutput("SwerveStates/Setpoints", getStates());
    Logger.recordOutput("Robot Yaw", getRobotAngle());

    

    // Updates the robot pose for the Robot itself
    SmartDashboard.putNumber("Robot Angle", getRobotAngle());



  }
}
 