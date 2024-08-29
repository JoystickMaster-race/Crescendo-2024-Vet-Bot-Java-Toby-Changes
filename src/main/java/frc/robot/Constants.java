// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 1;
    public static final int kLeftFrontID = 2;
    public static final int kRightRearID = 3;
    public static final int kRightFrontID = 4;

    //SYSID VALUES
    public static final double ksVolts = 0.20322;
    public static final double kvVoltSecondsPerMeter = 3.2976;
    public static final double kaVoltSecondsSquareMeter = 0.67542;
    public static final double kPDriveVel = 4.569;
    public static final double kTrackWidthMeters = 1.5;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    // Current limit for drivetrain motors

    //Pathplanning Data 
    public static final double cprKeft = 64; //am-4027 CIMCoder
    public static final double cprRight = 5; //am 3314-a CIMCoder

    public static final double kGearRatio = 8.4586466165;
    public static final double kWheelRadiusMeters = 0.0762; //3 inch
    public static final double kLinearDistanceConversionFactor = (1/(kGearRatio * 2 * Math.PI * (kWheelRadiusMeters))*10);

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static class ShooterConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kShooterID = 7;
    public static final int kShooterEncoderIDA = 4;
    public static final int kShooterEncoderIDB = 5;
    public static final double kShooterFreeRPS = 5300;
    public static final double kShooterTargetRPS = 400;
    public static final double kShooterToleranceRPS = 50;
    public static final double kVVoltSecondsPerRotation = 12 / kShooterFreeRPS;

    public static final double kShooterSpeed = 1;
    public static final double kReverseShooterSpeed = -1;
    public static final double kShuttleSpeed = 0.85;


  
  }

  public static class IntakeConstants {
    public static final int kIntakeID = 5;
    public static final int kIndexerID = 6;
    public static final int kBeamBreakID = 7;
    public static final double kIntakeVoltage = 6;
    public static final double kReverseIntakeVoltage = -6;
    public static final double kIntakeSpeed = 1;
    public static final double kIndexSpeed = 0.35;

    public static final double kReverseIntakeSpeed = -1;
  }
}
