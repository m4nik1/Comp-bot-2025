// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import frc.robot.commands.TeleopDrive;
import frc.robot.commands.zeroGyro;
import frc.robot.commands.Auto.CoralDownAuto;
import frc.robot.commands.Auto.ElevatorAuto_HP;
import frc.robot.commands.Auto.ElevatorAuto_l2;
import frc.robot.commands.Auto.ElevatorAuto_l4;
import frc.robot.commands.Auto.PivotAuto90;
import frc.robot.commands.Auto.RunCoralIntakeAuto;
import frc.robot.commands.Climber.ClimberStartSet;
import frc.robot.commands.Climber.Runclimber;
import frc.robot.commands.Climber.RunclimberBack;
import frc.robot.commands.Climber.StillClimber;
import frc.robot.commands.CoralPivot.CoralPivot90;
import frc.robot.commands.CoralPivot.CoralPivotDown;
import frc.robot.commands.CoralPivot.CoralPivotUp;
import frc.robot.commands.CoralPivot.RunPivotManual;
import frc.robot.commands.Elevator_Postions.ElevatorAl_l3;
import frc.robot.commands.Elevator_Postions.Elevator_HP;
import frc.robot.commands.Elevator_Postions.Elevator_L2;
import frc.robot.commands.Elevator_Postions.Elevator_L3;
import frc.robot.commands.Elevator_Postions.Elevator_L4;
import frc.robot.commands.Elevator_Postions.RunElevatorManual;
import frc.robot.commands.Intakes.AlgaeIn;
import frc.robot.commands.Intakes.AlgaeOut;
import frc.robot.commands.Intakes.CoralIn;
import frc.robot.commands.Intakes.CoralOut;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
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
  public static Elevator elevator;
  public static CoralIntake coralIntake;
  public static CoralPivot coralPivot;
  public static AlgaeIntake algaeIntake;
  // public static Vision photonVision;
  public static Climber climber = new Climber();

  static CommandXboxController driver = new CommandXboxController(0);
  static CommandXboxController operator = new CommandXboxController(1);

  private final Field2d field;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
      
    driveTrain = new DriveTrain();
    elevator = new Elevator();
    coralIntake = new CoralIntake();
    coralPivot = new CoralPivot();
    algaeIntake = new AlgaeIntake();
    // photonVision =  new Vision();
    
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

    NamedCommands.registerCommand("Coral_out", new RunCoralIntakeAuto());
    NamedCommands.registerCommand("Pivot_Down", new CoralDownAuto());
    NamedCommands.registerCommand("Pivot_90", new PivotAuto90());
    NamedCommands.registerCommand("Elevator_L4", new ElevatorAuto_l4());
    NamedCommands.registerCommand("Elevator_HP", new ElevatorAuto_HP());
    NamedCommands.registerCommand("Elevator_L2", new ElevatorAuto_l2());

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Right Turned Coral L4", new PathPlannerAuto("Right L4 Facing Coral"));
    autoChooser.addOption("Left Turned Coral L4", new PathPlannerAuto("Right L4 Facing Coral", true));
    autoChooser.addOption("Center L4 Coral", new PathPlannerAuto("Center L4 Coral"));
    autoChooser.addOption("Right Two Piece L4", new PathPlannerAuto("Right Two Piece L4"));
    autoChooser.addOption("Left Two Piece L4", new PathPlannerAuto("Right Two Piece L4", true ));
    autoChooser.addOption("Distance Tuning", new PathPlannerAuto("Distance Tuning"));

    SmartDashboard.putData("Field", field);

    driveTrain.setDefaultCommand(new TeleopDrive());
    elevator.setDefaultCommand(new RunElevatorManual());

    configureBindings();

  }

  private void configureBindings() {
    operator.povRight().onTrue(new CoralPivot90());
    operator.povUp().onTrue(new CoralPivotUp());
    operator.povLeft().onTrue(new RunPivotManual());
    operator.povDown().onTrue(new CoralPivotDown());
    operator.start().onTrue(new ElevatorAl_l3());
    

    // Elevator Positions - Find out what btn should be elevator ground
    operator.a().onTrue(new Elevator_L2());
    operator.b().onTrue(new Elevator_L3());
    operator.y().onTrue(new Elevator_L4());
    operator.x().onTrue(new Elevator_HP());

    // Coral and Algae Intakes
    operator.leftTrigger().whileTrue(new AlgaeIn());
    operator.leftBumper().whileTrue(new AlgaeOut());
    operator.rightTrigger().whileTrue(new CoralIn());
    operator.rightBumper().whileTrue(new CoralOut());

    driver.povUp().whileTrue(new RunclimberBack());
    driver.rightBumper().whileTrue(new zeroGyro());
    driver.povDown().whileTrue(new Runclimber());
    driver.a().whileTrue(new ClimberStartSet());
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
