// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import frc.robot.commands.TeleopDrive;
import frc.robot.commands.zeroGyro;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private LoggedDashboardChooser<Command> autoChooser;

  public static DriveTrain driveTrain;
  public static Vision photonVision;

  static CommandXboxController driver = new CommandXboxController(0);
  static CommandXboxController operator = new CommandXboxController(1);

  private final Field2d field;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
      
    driveTrain = new DriveTrain();
    photonVision =  new Vision();
    
    field = new Field2d();

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      Logger.recordOutput("Pathplanner pose", pose);
      field.setRobotPose(pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    SmartDashboard.putData("Field", field);

    driveTrain.setDefaultCommand(new TeleopDrive());

    configureBindings();

  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static double getLeftYOp() {
    return operator.getLeftY();
  }


  public static boolean getDriverA() {
    return driver.a().getAsBoolean();
  }
  
  public static double getRightYOp() {
    return operator.getRightY();
  }

  public static double getLeftY() {
    return driver.getLeftY();
  }

  public static double getRightX() {
    return driver.getRightX();
  }

  public static boolean getStartBtn() {
    return operator.start().getAsBoolean();
  }

  public static double getLeftX() {
    return driver.getLeftX();
  }
}
