// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import java.lang.Math;

import edu.wpi.first.units.Voltage;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class Drivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;
  AHRS m_navX;
  Timer m_TimerLeft;
  Timer m_TimerRight;
  //private final DifferentialDriveOdometry m_odometry;
  // Encoder leftEncoder;
  // Encoder rightEncoder;
    WPI_TalonSRX frontLeft;
    WPI_TalonSRX frontRight;
    WPI_TalonSRX backLeft;
    WPI_TalonSRX backRight;


  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public Drivetrain() {
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(1);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(3);
    WPI_TalonSRX backLeft = new WPI_TalonSRX(2);
    WPI_TalonSRX backRight = new WPI_TalonSRX(4);
    
    
    // Encoder leftEncoder = new Encoder(backLeft.getDeviceID(), 7);
    // Encoder rightEncoder = new Encoder(frontRight.getDeviceID(), 5);

    backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);

    m_drivetrain = new DifferentialDrive(frontLeft, frontRight);
    m_navX = new AHRS(SPI.Port.kMXP);
    m_TimerLeft = new Timer();
    m_TimerRight = new Timer();
    
    m_TimerLeft.reset();
    m_TimerRight.reset();
    m_navX.reset();
    // leftEncoder.reset();
    // rightEncoder.reset();

   //m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    //already handled?
    //m_odometry.resetPosition(new Pose2d(), m_navX.getRotation2d());

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    // frontLeft.setInverted(false);
    // frontRight.setInverted(true);

    
    //converting ticks to meters two ways (see which one works better?
    //idk how conversions are gonna work with new selectedSensor
    //leftEncoder.setDistancePerPulse(DrivetrainConstants.kLinearDistanceConversionFactor);

    // leftEncoder.setDistancePerPulse(Math.PI*(2*kWheelRadiusMeters)/5);
    // rightEncoder.setDistancePerPulse(Math.PI*(2*kWheelRadiusMeters)/5);

  }


  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public double getRightEncoderDistance(){
    //return rightEncoder.getDistance();
    //Probably return garbage bc timer output is probably instantaneous and not continuous, why don't they have a getDistance() method, 
    //it is already inside the quadratureEncoder object
    m_TimerRight.reset();
    m_TimerRight.start();
    return backRight.getSelectedSensorVelocity() * m_TimerRight.get();
  }

  public double getLeftEncoderDistance(){
    //return leftEncoder.getDistance();
    m_TimerLeft.reset();
    m_TimerLeft.start();
    return backLeft.getSelectedSensorVelocity() * m_TimerLeft.get();
  }

  public double getLeftEncoderVelocity(){
    // m_Timer.start();
    // return (leftEncoder.getDistance() / m_Timer.get());
    //return leftEncoder.getRate();
    return backLeft.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocity(){
    // m_Timer.start();
    // return (rightEncoder.getDistance() / m_Timer.get());
    //return rightEncoder.getRate();
    return backRight.getSelectedSensorVelocity();
  }

  public double getHeading(){
    return m_navX.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_navX.getRate();
  }

  // public Pose2d getPose(){
  //   //estimate
  //   return m_odometry.getPoseMeters();
  // }

  //idk why I made another one
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(backLeft.getSelectedSensorVelocity(), frontRight.getSelectedSensorVelocity());
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

  // public double getAverageEncoderDistance(){
  //   //return ((getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0);
  //   return leftEncoder.getDistance() + rightEncoder.getDistance();
  // }

  public double getLeftEncoderPosition(){
    return backLeft.getSelectedSensorPosition();
    //HARRY I LEFT THIS HERE BC I DON'T KNOW WHAT THIS METHOD IS ON ABOUT, BUT WILL MOST LIKELY WORK SINCE IT'S INBUILT LOL, SEE WHAT IT GIVES
  }

  @Override
  public void periodic() {
    
    //m_odometry.update(m_navX.getRotation2d(), backLeft. , rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Encoder velocity ticks", backLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Encoder velocity ticks", backRight.getSelectedSensorVelocity()); 
    SmartDashboard.putNumber("Right Encoder distance", getRightEncoderDistance());
    SmartDashboard.putNumber("Left Encoder distance", getLeftEncoderDistance());
    SmartDashboard.putNumber("Left encoder position wtf", getLeftEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}
