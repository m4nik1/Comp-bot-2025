// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TeleopDrive extends Command {
  /** Creates a new TelopDrive. */
  SlewRateLimiter rotationLimiter, translateLimiter, strafeLimiter;

  Pose2d algaePose;

  PIDController xTranslation, yTranslation, rotOutput;

  double targetYaw = 0.0;
  double translationVal, strafeVal, rotationVal;
  boolean targetVisible = true;

  double xOutput, yOutput, rotation;

  public TeleopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);

    rotationLimiter = new SlewRateLimiter(3.0);
    translateLimiter = new SlewRateLimiter(3.0);
    strafeLimiter = new SlewRateLimiter(3.0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetYaw = 0.0;
    targetVisible = true;
    xTranslation = new PIDController(.5, 0, 0);
    yTranslation = new PIDController(.5, 0, 0);
    // rotation = new PIDController(.5, 0, 0);
    // rotation.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplier = Constants.speedMultiTeleop;
    double getX = -RobotContainer.getLeftX();
    double getY = -RobotContainer.getLeftY();
    double getRotation = -RobotContainer.getRightX();
    double turnKp = 0.015;



    // if(RobotContainer.getDriverA()) { // Driver presses the A button
    //   var results = RobotContainer.photonVision.getUnreadResults();
    //   Logger.recordOutput("Align on", RobotContainer.getDriverA());

    //   if(!results.isEmpty()) {
    //     var result = results.get(results.size() - 1);
    //     if(result.hasTargets()) {
    //       for (var target : result.getTargets()) {
    //         int tagId = target.getFiducialId();
    //         if(Constants.desiredTagIds.contains(tagId)) {
    //           targetYaw = target.getYaw();
    //           SmartDashboard.putNumber("Target found", target.getFiducialId());
    //           SmartDashboard.putNumber("18 Yaw", targetYaw);
    //           targetVisible = true;
    //         }
    //       }
    //     }
    //   }
    //   SmartDashboard.putNumber("target vision Yaw", targetYaw);
    //   translationVal = translateLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getY, .01)); // getY was negativeß
    //   strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getX, .01)); // getX was negative
    //   rotationVal = -1.0 * turnKp * targetYaw;
    // }
        var results = RobotContainer.photonVision.getUnreadResults();
    if(RobotContainer.getDriverX()) {
      if(!results.isEmpty()) {
        
        // Gets the latest frame since one has been processed since then
        var latestResult = results.get(results.size() - 1);
        if(latestResult.hasTargets()) { // At least one tag has been seen by the camera
          for (var target : latestResult.getTargets()) {
            int tagId = target.getFiducialId();

            // Finds the tags that are associated with the reef
            if(Constants.desiredTagIds.contains(tagId)) {
              double face = Constants.TagToFaceBlue.get(tagId);
              
              // Calculates the face angle in radians
              double thetaCalculate = ((2*Math.PI)*(face/6)+Math.PI) % (2*Math.PI); 

              Transform2d calculatedAlgae = Robot.reefPosesGenerate.calculateAlgaePose(thetaCalculate);

              algaePose = RobotContainer.driveTrain.getRobotPose2d().transformBy(calculatedAlgae);
              // Logger.recordOutput("calculated align Pose", algaePose);
            }
          }
        }
      }
      xOutput = xTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX()) * Constants.speedMultiTeleop;
      yOutput = yTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getY(), algaePose.getY()) * Constants.speedMultiTeleop;
      // rotOutput = rotation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX()) * Constants.speedMultiTeleop;

      translationVal = xOutput;
      strafeVal = yOutput;
      rotation = 0;
    }
    else {
      speedMultiplier = Constants.speedMultiTeleop;

      // Remember all these values from the stick are negative
      translationVal = translateLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getY, .01)); // getY was negativeß
      strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getX, .01)); // getX was negative
      rotationVal = rotationLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getRotation, .01)); // getRotation was negative  
    }


    

    Translation2d translation = new Translation2d(translationVal, strafeVal);

    RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotationVal * Constants.maxAngularSpd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
