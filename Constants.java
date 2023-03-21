// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double        ROBOT_MASS   = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double        CHASSIS_MASS = ROBOT_MASS;
  public static final Matter        CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double        LOOP_TIME    = 0.13; //s, 20ms + 110ms sprk max velocity lag
  
  
  
  //use this to not slam into an april tag. Based on half our length.
 
  public static final double        xOffset = .58;

public static final int SLOW_MODE_BUTTON = 1;

public static final int shoulderID = 16;
public static final int elevatorID = 4;
public static final int wristID = 24;
public static final int bottomLimitPort = 0;
public static final int wrist2ID = 18;
public static final int shoulderAbsoluteEncoderPort = 1;
public static final double wristKP = .1;
public static final double wristKI = 0;
public static final double wristKD = 0;
public static final double elevatorKP = .1;
public static final double elevatorKI = 0;
public static final double elevatorKD = 0;
public static final double wrist2KP = .1;
public static final double wrist2KI = 0;
public static final double wrist2KD = 0;
public static final double elevatorLockKP = .2;
public static final double shoulderLockKP = .2;

public static final double elevatorDownwardRampRate = .5;
public static final double elevatorUpwardRampRate = .5;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
     // Joystick Deadband
     public static final double LEFT_X_DEADBAND = 0.01;
     public static final double LEFT_Y_DEADBAND = 0.01;

  }

  public static class ArmConstants {

   

}

  // public static class VisionConstants {

  //   /**
  //    * Physical location of the camera on the robot, relative to the center of the robot.
  //    */
  //   public static final Transform3d CAMERA_TO_ROBOT =
  //       new Transform3d(new Translation3d(Units.inchesToMeters(14), 0.0, Units.inchesToMeters(8)), new Rotation3d());
  //   public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

  //   public static final double FIELD_LENGTH_METERS = 16.54175;
  //   public static final double FIELD_WIDTH_METERS = 8.0137;
  // }

  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0.0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0.0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.0);

    // public static final double MAX_SPEED        = 4;
    // public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

}
