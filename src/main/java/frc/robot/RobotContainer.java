// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

//import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;

 import frc.robot.subsystems.Drivetrain;
 import frc.robot.subsystems.Shooter;
 import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here.
   private final Drivetrain m_drivetrain = new Drivetrain();
   private final Shooter m_shooter = new Shooter();
   private final Intake m_intake = new Intake();

  //Special PID Ball-Pit
   private Command spinUpShooterCommand = Commands.runOnce(m_shooter::enable, m_shooter);
   private Command  stopShooterCommand = Commands.runOnce(m_shooter::disable, m_shooter);
  

  //SendableChooser<Command> chooser = new SendableChooser<>();

  //private final SendableChooser<Command> chooser;
  SendableChooser<Command> chooser;

  /*The gamepad provided in the KOP shows up like an XBox controller if the mode switch is set to X mode using the
   * switch on the top.*/
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("Set Intake", m_intake.getIntakeCommand().withTimeout(3));
    NamedCommands.registerCommand("Shooter Raw", m_shooter.getShootCommand());
    NamedCommands.registerCommand("Shooter Spinup", spinUpShooterCommand);

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", chooser);

    // **Raw/Sensor Style**

    // Command FancyAuto = Autos.FancyAuto(m_drivetrain);
    // Command RawTaxi = Autos.TaxiAuto(m_drivetrain);
    // chooser.addOption("Fancy Auto", FancyAuto);  
    // chooser.addOption("Raw Taxi", RawTaxi);

    // Configure the trigger bindings
    //Old Way
    configureBindings();
    // chooser.addOption("Curvy path", loadPathToRamsete("C:/Users/foxlo/Documents/Crescendo-2024-Vet-Bot-Java-Toby-Changes/src/main/deploy/deploy/pathplanner/paths/Straight.path",
    //  true) );
    // chooser.addOption("Straight", loadPathToRamsete("C:/Users/foxlo/Documents/Crescendo-2024-Vet-Bot-Java-Toby-Changes/src/main/deploy/deploy/pathplanner/paths/Curved.path", true));

    // chooser.addOption("Taxi Raw", Autos.TaxiAuto(m_drivetrain));
    //SmartDashboard.putData(chooser);

    //Shuffleboard.getTab("Autonomous").add(chooser);

    // chooser.addOption("taxi path", getAutonomousCommand());

     //Shuffleboard.getTab("Autonomous options").add(chooser);
  }
  // **Old Way**
  //  public Command loadPathToRamsete(String filename, boolean resetOdometry){
  //    Trajectory trajectory;

  //    try{
  //      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
  //      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  //    } catch(IOException exception){
  //      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
  //      System.out.println("Unable to read from file" + filename);
  //      return new InstantCommand();
  //    }

  //       RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
  //   new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
  //   new SimpleMotorFeedforward(DrivetrainConstants.ksVolts, DrivetrainConstants.kvVoltSecondsPerMeter,
  //   DrivetrainConstants.kaVoltSecondsSquareMeter),
  //   DrivetrainConstants.kDriveKinematics, m_drivetrain::getWheelSpeeds,
  //   new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
  //   new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), m_drivetrain::tankDriveVolts,
  //   m_drivetrain);

  //   return ramseteCommand;

    //  if(resetOdometry){
    //   return new SequentialCommandGroup(
    //     new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    //  } else {
    //   return ramseteCommand;
    //  }

   //}
  /**
   * Use this method to define your trigger->command mappings. Triggers can be accessed via the
   * named factory methods in the Command* classes in edu.wpi.first.wpilibj2.command.button (shown
   * below) or via the Trigger constructor for arbitary conditions
   */
  private void configureBindings() {
    // Set the default command for the drivetrain to drive using the joysticks
    m_drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                m_drivetrain.arcadeDrive(
                    m_driverController.getRightX(), m_driverController.getLeftY()),
            m_drivetrain));

    // **SYSID should probably use**
    m_driverController
        .a()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .b()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController
        .x()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .y()
        .and(m_driverController.rightBumper())
        .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_driverController
        .a()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .b()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController
        .x()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .y()
        .and(m_driverController.leftBumper())
        .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    //m_guitarHero.button(0).whileTrue(m_intake.getIntakeCommand());

    // m_operatorController.a().onTrue(m_intake.getIntakeCommand());
    // m_operatorController.x().onTrue(m_intake.getReverseIntakeCommand());
    // m_operatorController.y().onTrue(m_intake.getStopIntakeCommand());
    m_operatorController.pov(225).whileTrue(m_intake.getIntakeCommand());
    m_operatorController.pov(315).whileTrue(m_intake.getReverseIntakeCommand());
    m_operatorController.a().whileTrue(spinUpShooterCommand);
    m_operatorController.x().whileTrue(stopShooterCommand);


    if(m_operatorController.getRightTriggerAxis() > 0.05){
      m_shooter.enable();
    }

    if(m_intake.getBeamBreak()){
      new RunCommand(() -> m_intake.stop(), m_intake)
        .withTimeout(0.5);
    }

    // **Boring X-BOX Controls**
    // if(m_operatorController.getLeftY() < 0.05){
    //   m_intake.getIntakeCommand();
    // } 
    // else if(m_operatorController.getLeftY() > 0.05){
    //   m_intake.getReverseIntakeCommand();
    // }
    // else{
    //   m_intake.stop();
    // }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // **Safe way!!**
    //return chooser.getSelected();

    //return new PathPlannerAuto("Example");
     // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("Straight");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);

    // An example command will be run in autonomous
  //   var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
  //     new SimpleMotorFeedforward(DrivetrainConstants.ksVolts, DrivetrainConstants.kvVoltSecondsPerMeter, DrivetrainConstants.kaVoltSecondsSquareMeter), DrivetrainConstants.kDriveKinematics, 10);
  //   return Autos.exampleAuto(m_drivetrain);

  // TrajectoryConfig config = new TrajectoryConfig(3, 1)
  //  .setKinematics(DrivetrainConstants.kDriveKinematics)
  //  .addConstraint(autoVoltageConstraint);

  // Trajectory exampleTraj = TrajectoryGenerator.generateTrajectory( new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);
  
  //   RamseteCommand ramseteCommand = new RamseteCommand(exampleTraj, m_drivetrain::getPose,
  //   new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
  //   new SimpleMotorFeedforward(DrivetrainConstants.ksVolts, DrivetrainConstants.kvVoltSecondsPerMeter,
  //   DrivetrainConstants.kaVoltSecondsSquareMeter),
  //   DrivetrainConstants.kDriveKinematics, m_drivetrain::getWheelSpeeds,
  //   new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
  //   new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), m_drivetrain::tankDriveVolts,
  //   m_drivetrain);


  }
}
