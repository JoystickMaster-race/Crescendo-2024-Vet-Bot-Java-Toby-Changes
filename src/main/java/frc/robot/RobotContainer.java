// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;

 import frc.robot.subsystems.Drivetrain;
//  import frc.robot.subsystems.Shooter;
//  import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here.
   private final Drivetrain m_drivetrain = new Drivetrain();
  //  private final Shooter m_shooter = new Shooter();
  //  private final Intake m_intake = new Intake();
  

  // SendableChooser<Command> chooser = new SendableChooser<>();

  /*The gamepad provided in the KOP shows up like an XBox controller if the mode switch is set to X mode using the
   * switch on the top.*/
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // chooser.addOption("taxi path", getAutonomousCommand());

    // Shuffleboard.getTab("Autonomous options").add(chooser);
  }

  // public Command loadPathToRamsete(String filename, boolean resetOdometry){
  //   Trajectory trajectory;

  //   try{
  //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
  //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  //   } catch(IOException exception){
  //     DriverStation.reportError("Unable to open trajectory dog" + filename, exception.getStackTrace());
  //     System.out.println("Unable to read from file" + filename);
  //     return new InstantCommand();
  //   }
    
  //   // RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain.getPose(),
  //   // new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
  //   // new SimpleMotorFeedforward(DrivetrainConstants.ksVolts, DrivetrainConstants.kvVoltSecondsPerMeter,
  //   // DrivetrainConstants.kaVoltSecondsSquareMeter),
  //   // DrivetrainConstants.kDriveKinematics, m_drivetrain.getWheelSpeeds(),
  //   // new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
  //   // new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), m_drivetrain.tankDriveVolts(0, 0),
  //   // m_drivetrain);
  // }



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


    // Set up a binding to run the intake command while the operator is pressing and holding the
    // left Bumper
    // m_operatorController.leftBumper().whileTrue(m_shooter.getShootCommand());
    // m_operatorController.rightBumper().whileTrue(m_intake.getIntakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_drivetrain);
  }
}
