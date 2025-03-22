// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import frc.robot.commands.AlgaeAlign;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.zeroGyro;
import frc.robot.commands.Auto.*;
import frc.robot.commands.Climber.*;
import frc.robot.commands.CoralPivot.*;
import frc.robot.commands.Elevator_Positions.*;
import frc.robot.commands.Intakes.*;
import frc.robot.subsystems.*;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;

  public static DriveTrain driveTrain;
  public static Elevator elevator;
  public static CoralIntake coralIntake;
  public static CoralPivot coralPivot;
  public static AlgaeIntake algaeIntake;
  public static Vision photonVision;
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
    photonVision =  new Vision();
    
    field = new Field2d();

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    // NamedCommands.registerCommand("Coral_out", new RunCoralIntakeAuto());
    NamedCommands.registerCommand("Coral_out", coralIntake.coralOut());
    NamedCommands.registerCommand("Pivot_Down", new CoralDownAuto());
    NamedCommands.registerCommand("Pivot_90", new PivotAuto90());
    NamedCommands.registerCommand("Elevator_L4", new ElevatorAuto_l4());
    // NamedCommands.registerCommand("Elevator_L4", elevator.setElevatorPos(Constants.elevator_l4));
    NamedCommands.registerCommand("Elevator_HP", new ElevatorAuto_HP());
    NamedCommands.registerCommand("Elevator_L2", new ElevatorAuto_l2());


    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("Right Turned Coral L4", new PathPlannerAuto("Right L4 Facing Coral"));
    autoChooser.addOption("Left Turned Coral L4", new PathPlannerAuto("Right L4 Facing Coral", true));
    autoChooser.addOption("Center L4 Coral", new PathPlannerAuto("Center L4 Coral"));
    autoChooser.addOption("Right Two Piece L4", new PathPlannerAuto("Right Two Piece L4"));
    autoChooser.addOption("Left Two Piece L4", new PathPlannerAuto("Right Two Piece L4", true ));
    autoChooser.addOption("Distance Tuning", new PathPlannerAuto("Distance Tuning"));
    autoChooser.addOption("Right turn test", new PathPlannerAuto("Right Turn Test"));

    SmartDashboard.putData("Field", field);
    SmartDashboard.putData("autoChooser", autoChooser);

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
    // driver.a().onTrue(driveTrain.velTest());
    // driver.x().onTrue(new AlgaeAlign());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static double getLeftYOp() {
    return operator.getLeftY();
  }


  public static boolean getDriverA() {
    return driver.a().getAsBoolean();
  }

  public static boolean getDriverX() {
    return driver.x().getAsBoolean();
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
