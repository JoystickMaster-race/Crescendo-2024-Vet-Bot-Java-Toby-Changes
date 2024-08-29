// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;

// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DigitalInput;
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

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import java.util.function.BooleanSupplier;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;

 import frc.robot.subsystems.Drivetrain;
 import frc.robot.subsystems.Shooter;
 import frc.robot.subsystems.Intake;
 import frc.robot.subsystems.LEDDigitalOutputSubsystem;
 import frc.robot.commands.SetLEDDigitalOutputCommand;

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
   private final LEDDigitalOutputSubsystem m_led_output = new LEDDigitalOutputSubsystem(8);

  //Special PID Ball-Pit

//    private Command spinUpShooterCommand = Commands.runOnce(m_shooter::enable, m_shooter);
//    private Command  stopShooterCommand = Commands.runOnce(m_shooter::disable, m_shooter);

  /*The gamepad provided in the KOP shows up like an XBox controller if the mode switch is set to X mode using the
   * switch on the top.*/
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final DigitalInput m_beamSensor = new DigitalInput(7);

  public BooleanSupplier beambroken = () -> !m_beamSensor.get();
  public BooleanSupplier beamConnected = () -> m_beamSensor.get();
  SendableChooser<Command> autoChooser;

    Command a_CJ = Autos.CJ(m_drivetrain, m_shooter, m_intake);
    Command a_Anna = Autos.Anna(m_drivetrain, m_shooter, m_intake);
    Command a_Max = Autos.Max(m_drivetrain, m_intake, m_shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = new SendableChooser<>();
    //autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);

    autoChooser.addOption("CJ", a_CJ);
    autoChooser.addOption("Anna", a_Anna);
    autoChooser.addOption("Max", a_Max);


    // NamedCommands.registerCommand("Set Intake", m_intake.getIntakeCommand().withTimeout(3));
    // NamedCommands.registerCommand("Shooter Raw", m_shooter.getShootCommand());

    configureBindings();

  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be accessed via the
   * named factory methods in the Command* classes in edu.wpi.first.wpilibj2.command.button (shown
   * below) or via the Trigger constructor for arbitary conditions
   */
  private void configureBindings() {
    // Set the default command for the drivetrain to drive using the joysticks

    // VERY IMPORTANT
    m_drivetrain.setDefaultCommand(
      //new RunCommand( () -> m_drivetrain.getDriveContCommand(1)).withTimeout(100));
        new RunCommand(
            () ->
                m_drivetrain.arcadeDrive(
                    m_driverController.getRightX(), m_driverController.getLeftY()),
            m_drivetrain));
    
    CameraServer.startAutomaticCapture("9191 Cam", 0);
    m_operatorController.pov(315).whileTrue(m_intake.getIntakeCommand().until(beambroken).andThen(new SetLEDDigitalOutputCommand(m_led_output, true).alongWith(m_intake.getIndexCommand()).until(beamConnected)).andThen(new SetLEDDigitalOutputCommand(m_led_output, false)));
    //m_operatorController.x().whileTrue(m_intake.getIntakeCommand().until(beambroken).andThen(new SetLEDDigitalOutputCommand(m_led_output, true).alongWith(m_intake.getIndexCommand()).until(beamConnected)).andThen(new SetLEDDigitalOutputCommand(m_led_output, false)));
    // if(m_driverController.getLeftY() > 0.05){
    //   m_drivetrain.frontLeft.set(m_driverController.getLeftY());
    //   m_drivetrain.frontRight.set(m_driverController.getLeftY());
    //   m_drivetrain.backLeft.set(m_driverController.getLeftY());
    //   m_drivetrain.backRight.set(m_driverController.getLeftY());
    // }
    //CHECK THIS
    //m_driverController.a().onTrue(m_drivetrain.Safety());

    //new RunCommand( () -> m_drivetrain.arcadeDrive(m_driverController.getRightX(), m_driverController.getLeftY()), m_drivetrain);

    //m_driverController.getLeftY().whileTrue(m_drivetrain.arcadeDrive(1, 0));
    //m_driverController.getLeftY().whileTrue(m_drivetrain.getDriveCommand(m_driverController.getLeftY()));

    // if(m_driverController.getLeftY() > 0.05){
    //   m_drivetrain.getDriveCommand(m_driverController.getLeftY());
    // }

    m_operatorController.pov(225).whileTrue(m_intake.getReverseIntakeCommand());
    m_operatorController.pov(225).whileTrue(m_shooter.getReverseShootCommand());

    //m_operatorController.getRightBumper().whileTrue(m_shooter.getIntak)
    // m_operatorController.pov(225).whileTrue(new SetLEDDigitalOutputCommand(m_led_output, false));

    m_operatorController.b().whileTrue(m_shooter.getShootCommand());
    // m_operatorController.b().whileTrue(new SetLEDDigitalOutputCommand(m_led_output, false));
    m_operatorController.a().whileTrue(m_shooter.getReverseShootCommand());
    // m_operatorController.a().whileTrue(new SetLEDDigitalOutputCommand(m_led_output, false));
    m_operatorController.x().whileTrue(m_shooter.getShuttleCommand());
    // m_operatorController.x().whileTrue(new SetLEDDigitalOutputCommand(m_led_output, false));

    //**Boring X-BOX Controls**
    if(m_operatorController.getLeftY() < 0.05){
      m_intake.getIntakeCommand();
    } 
    else if(m_operatorController.getLeftY() > 0.05){
      m_intake.getReverseIntakeCommand();
    }
    else{
      m_intake.stop();
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
}
