// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import java.lang.Math;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;

import com.kauailabs.navx.frc.AHRS;

/* 
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class Drivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */

  //Drivebase moving components
  DifferentialDrive m_drivetrain;
  WPI_TalonSRX frontLeft;
  WPI_TalonSRX frontRight;
  WPI_TalonSRX backLeft;
  WPI_TalonSRX backRight;

  //Virtual components
  DifferentialDriveOdometry m_odometry;
  SysIdRoutine m_sysIdRoutine;

  //Sensors
  AHRS m_navX;
  Timer m_TimerLeft;
  Timer m_TimerRight;
  Encoder leftEncoder;
  Encoder rightEncoder;
  
  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public Drivetrain() {
    frontLeft = new WPI_TalonSRX(2);
    frontRight = new WPI_TalonSRX(3);
    backLeft = new WPI_TalonSRX(1);
    backRight = new WPI_TalonSRX(4);

    leftEncoder = new Encoder(0,1);
    rightEncoder = new Encoder(2, 3);

   final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
   final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
   final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
     m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                frontLeft.setVoltage(volts.in(Volts));
                frontRight.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(leftEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(leftEncoder.getRate(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRight.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(rightEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightEncoder.getRate(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
    
  //   //Set current limits to prevent brown-out during acceleration
  //   backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
  //   frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
    frontLeft.configPeakCurrentLimit(10, 10); /* 35 A */
    frontLeft.configPeakCurrentDuration(200, 10); /* 200ms */
    frontLeft.configContinuousCurrentLimit(10, 10); 

    frontRight.configPeakCurrentLimit(10, 10); 
    frontRight.configPeakCurrentDuration(200, 10); /* 200ms */
    frontRight.configContinuousCurrentLimit(10, 10); /* 30 */

    m_drivetrain = new DifferentialDrive(frontLeft, frontRight);
    m_navX = new AHRS(SPI.Port.kMXP);
    m_TimerLeft = new Timer();
    m_TimerRight = new Timer();
    leftEncoder.reset();
    rightEncoder.reset();
    
    m_TimerLeft.reset();
    m_TimerRight.reset();
    m_navX.reset();
    // leftEncoder.reset();
    // rightEncoder.reset();

    m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    
    //converting ticks to meters two ways (see which one works better?
    //idk how conversions are gonna work with new selectedSensor
    //leftEncoder.setDistancePerPulse(DrivetrainConstants.kLinearDistanceConversionFactor);

    //cpr = 5 if am-3441a
    //cpr = 64 if am-4027
    leftEncoder.setDistancePerPulse(Math.PI*(2*kWheelRadiusMeters)/64);
    rightEncoder.setDistancePerPulse(Math.PI*(2*kWheelRadiusMeters)/64);
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public double getRightEncoderDistance(){
    return rightEncoder.getDistance();
  }

  public double getLeftEncoderDistance(){
    return leftEncoder.getDistance();

  }

  //TalonSRX Breakout Board Method
  // public double getLeftEncoderVelocity(){    
  //   return backLeft.getSelectedSensorVelocity();
  // }

    public double getLeftEncoderVelocity(){
    return leftEncoder.getRate();
  }

  public double getRightEncoderVelocity(){
    return rightEncoder.getRate();
  }

  public double getHeading(){
    return m_navX.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_navX.getRate();
  }

  public Pose2d getPose(){
    //estimate
    return m_odometry.getPoseMeters();
  }

  //idk why I made another one
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  //already handled?
  // public void resetOdometry(Pose2d pose){
  //   leftEncoder.reset();
  //   rightEncoder.reset();
  //   m_odometry.resetPosition(pose, m_navX.getRotation2d());
  // } 

  public void tankDriveVolts(double leftVolts, double rightVolts){
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    m_drivetrain.feed();
  }

  public double getAverageEncoderDistance(){
    return (leftEncoder.getDistance() + rightEncoder.getDistance() / 2);
  }


  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
  return m_sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return m_sysIdRoutine.dynamic(direction);
}

  @Override
  public void periodic() {

    //TalonSRX breakout board method
    // SmartDashboard.putNumber("Left Encoder velocity ticks", backLeft.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Right Encoder velocity ticks", frontRight.getSelectedSensorVelocity()); 
    //m_odometry.update(m_navX.getRotation2d(), backLeft. , rightEncoder.getDistance());

    SmartDashboard.putNumber("Right Encoder distance", getRightEncoderDistance());
    SmartDashboard.putNumber("Right Encoder velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Left Encoder velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Left Encoder distance", getLeftEncoderDistance());
    SmartDashboard.putNumber("Left current", frontLeft.getSupplyCurrent());
    SmartDashboard.putNumber("Right current", frontRight.getSupplyCurrent());
    SmartDashboard.putNumber("Gyro heading", getHeading());
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}
