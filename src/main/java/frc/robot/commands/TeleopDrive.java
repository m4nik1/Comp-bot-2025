// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TeleopDrive extends Command {
  /** Creates a new TelopDrive. */
  SlewRateLimiter rotationLimiter, translateLimiter, strafeLimiter;

  double targetYaw = 0.0;
  double translationVal, strafeVal, rotationVal;
  boolean targetVisible = true;

  public TeleopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);

    rotationLimiter = new SlewRateLimiter(1.8);
    translateLimiter = new SlewRateLimiter(1.8);
    strafeLimiter = new SlewRateLimiter(1.8);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetYaw = 0.0;
    targetVisible = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplier = Constants.speedMultiTeleop;
    double getX = -RobotContainer.getLeftX();
    double getY = -RobotContainer.getLeftY();
    double getRotation = -RobotContainer.getRightX();
    double turnKp = 0.025;



    // if(RobotContainer.getDriverA()) { // Driver presses the A button
      // var results = RobotContainer.photonVision.getUnreadResults();
      // Logger.recordOutput("Align on", RobotContainer.getDriverA());

      // if(!results.isEmpty()) {
      //   var result = results.get(results.size() - 1);
      //   if(result.hasTargets()) {
      //     for (var target : result.getTargets()) {
      //       if(target) {
      //         targetYaw = target.getYaw();
      //         SmartDashboard.putNumber("Target found", target.getFiducialId());
      //         SmartDashboard.putNumber("18 Yaw", targetYaw);
      //         targetVisible = true;
      //       }
      //     }
      //   }
      // }
      // translationVal = translateLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getY, .08)); // getY was negativeß
      // strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getX, .09)); // getX was negative
      // rotationVal = -1.0 * turnKp * targetYaw; 
    // }
    // else {
      speedMultiplier = Constants.speedMultiTeleop;

      // Remember all these values from the stick are negative
      translationVal = translateLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getY, .01)); // getY was negativeß
      strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getX, .01)); // getX was negative
      rotationVal = rotationLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getRotation, .01)); // getRotation was negative  
    // }


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
