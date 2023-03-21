// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
\\ */
public class RobotContainer {
//   private final Arm arm = new Arm();
//   private final MoveShoulderCommand moveShoulderCommand = new MoveShoulderCommand(arm);
//   private final MoveElevatorCommand moveElevatorCommand = new MoveElevatorCommand(arm);

  private final Arm arm = new Arm();
  // private final PhotonCamera photonCamera = new PhotonCamera("photonCamera");
  
  private static final SendableChooser<String> AutoPath = new SendableChooser<>();
  SwerveSubsystem driveSubsystem;

  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/falcon"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CommandJoystick driverController = new CommandJoystick(1);

  // private final PoseEstimatorSubsystem poser = new PoseEstimatorSubsystem(photonCamera, drivebase);

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  GenericHID driverXbox = new GenericHID(0);

  private static AprilTagFieldLayout aprilTagField = null;

  /* Driver Buttons */
  private final JoystickButton StartButton = new JoystickButton(driverXbox, XboxController.Button.kStart.value);
  private final JoystickButton BackButton = new JoystickButton(driverXbox, XboxController.Button.kBack.value);
  private final JoystickButton YButton = new JoystickButton(driverXbox, XboxController.Button.kY.value);
  private final JoystickButton XButton = new JoystickButton(driverXbox, XboxController.Button.kX.value);
  private final JoystickButton AButton = new JoystickButton(driverXbox, XboxController.Button.kA.value);
  private final JoystickButton LeftBumper = new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value);
  private final JoystickButton RightBumper = new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value);
  private final JoystickButton BButton = new JoystickButton(driverXbox, XboxController.Button.kB.value);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // poser.addDashboardWidgets(Shuffleboard.getTab("Vision"));

    AutoPath.setDefaultOption("Two Piece Blue NB", "Two Piece Blue NB");
    AutoPath.addOption("Two Piece Red NB", "Two Piece Red NB");

    AutoPath.addOption("Two Piece Red Balance", "Two Piece Red Balance");
    AutoPath.addOption("Two Piece Blue Balance", "Two Piece Blue Balance");

    AutoPath.addOption("Middle Blue", "Middle Blue");
    AutoPath.addOption("Middle Red", "Middle Red");

    AutoPath.addOption("Blue Right", "Blue Right");
    AutoPath.addOption("Red Left Balance", "Red Left Balance");

    SmartDashboard.putData("Auto Pathing", AutoPath);


    // AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
    //     // Applies deadbands and inverts controls because joysticks
    //     // are back-right positive while robot
    //     // controls are front-left positive
    //     () -> (Math.abs(driverXbox.getLeftY()) > OperatorConstants.LEFT_Y_DEADBAND)
    //         ? driverXbox.getLeftY()
    //         : 0,
    //     () -> (Math.abs(driverXbox.getLeftX()) > OperatorConstants.LEFT_X_DEADBAND)
    //         ? driverXbox.getLeftX()
    //         : 0,
    //     () -> -driverXbox.getRightX(),
    //     () -> -driverXbox.getRightY(),
    //     false);

    // AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
    //     () -> (Math.abs(driverXbox.getLeftY()) > OperatorConstants.LEFT_Y_DEADBAND)
    //         ? -driverXbox.getLeftY()
    //         : 0,
    //     () -> (Math.abs(driverXbox.getLeftX()) > OperatorConstants.LEFT_X_DEADBAND)
    //         ? -driverXbox.getLeftX()
    //         : 0,
    //     () -> -driverXbox.getRawAxis(4), false);
    // TeleopDrive closedFieldRel = new TeleopDrive(
    //     drivebase,
    //     () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? driverController.getY() : 0,
    //     () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? driverController.getX() : 0,
    //     () -> -driverController.getRawAxis(3), () -> true, false);

    TeleopDrive closedFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverXbox.getRawAxis(3)) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverXbox.getRawAxis(8) : 0,
      () -> (Math.abs(driverXbox.getRawAxis(4)) > OperatorConstants.LEFT_X_DEADBAND) ? -driverXbox.getRawAxis(2) : 0,
      () -> -driverXbox.getRawAxis(8), () -> true, false, false);

    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
      drivebase.setDefaultCommand(closedFieldRel);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   // Command extendArmCommand = new RunCommand(() -> arm.setArmPosition(117, 75), arm);
    // Command extendArmCommand = new RunCommand(() -> arm.setArmPosition(10, 15), arm);

    //Command retractArmCommand = new RunCommand(() -> arm.setArmPosition(0, 0), arm);
   // Command collectgamepiececommand = new RunCommand(() -> arm.setArmPosition(50, 70), arm);
   // Command IntakeInwards = new RunCommand(() -> arm.setIntakeSpeed(.5));
   // Command IntakeOutwards = new RunCommand(() -> arm.setIntakeSpeed(-.5));

   // Command cubeModeEngage = new RunCommand(() -> arm.setArmPosition(35, 245), arm);
   // Command cubeModeScore = new RunCommand(() -> arm.setArmPosition(87, 210), arm);
    // Create button to toggle slow mode

    // Toggle slow mode when button is pressed
  /**

    /* Driver Buttons */
    StartButton.onTrue((new InstantCommand(drivebase::zeroGyro)));
    BButton.whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
 //   XButton.onTrue(new InstantCommand((moveElevatorCommand)));

    // double shoulderSpeed = Robot._operator.getRawAxis(5);
    // double elevatorSpeed = Robot._operator.getLeftX();
    
    // if (shoulderSpeed == 0 && elevatorSpeed == 0) {
    //   // If no controls are being applied, lock the shoulder and elevator motors using encoders
    //   arm.lockShoulder();
    //   arm.lockElevator();
    // } else {
    //   // Set the speed of the shoulder and elevator motors based on raw joystick inputs and lock flags
    //   arm.setShoulderSpeed(shoulderSpeed);
    //   arm.setElevatorSpeed(elevatorSpeed);
    // }
  
 //---

//     The StartButton will trigger a single, instant command to zero the gyro used for navigation.
// The BButton will repeatedly execute an instant command to lock the drivebase while the button is held down.
// The YButton will repeatedly toggle a "slow mode" for the drivebase when pressed.
// The XButton will trigger a run command that sets the position of an arm to specific values.

    // LeftBumper.whileTrue(IntakeInwards);
    // RightBumper.whileTrue(IntakeOutwards);

    // AButton.onTrue(extendArmCommand);
    // AButton.onFalse(retractArmCommand);


    //Co-Driver Buttons on Dashboard


    // IT IS IMPORTANT THAT YOU SUBTRACT THE X OFFSET FROM TAGS 1, 2, 3, 4 AND ADD THE OFFSET TO 5, 6, 7, 8
    SmartDashboard.putData("Drive to 1", driveToAprilTag(drivebase, 1, Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(0), new Translation2d((-Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 2", driveToAprilTag(drivebase, 2, Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(0), new Translation2d((-Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 3", driveToAprilTag(drivebase, 3, Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(0), new Translation2d((-Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 4", driveToAprilTag(drivebase, 4, Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(0), new Translation2d(-Constants.xOffset, (Units.inchesToMeters(0)))));

    SmartDashboard.putData("Drive to 5", driveToAprilTag(drivebase, 5, Rotation2d.fromDegrees(180),
    Rotation2d.fromDegrees(180), new Translation2d((Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 6", driveToAprilTag(drivebase, 6, Rotation2d.fromDegrees(180),
    Rotation2d.fromDegrees(180), new Translation2d((Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 7", driveToAprilTag(drivebase, 7, Rotation2d.fromDegrees(180),
    Rotation2d.fromDegrees(180), new Translation2d((Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 8", driveToAprilTag(drivebase, 8, Rotation2d.fromDegrees(180),
    Rotation2d.fromDegrees(180), new Translation2d((Constants.xOffset), 0)));
 


    // auto.onTrue(Commands.sequence(autoToConePickup(drivebase, 4, new Translation2d(Units.inchesToMeters(-36), 0)),
    // IntakeOutwards,collectgamepiececommand));

    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox, 4).onTrue(driveToAprilTag(drivebase, 4, Rotation2d.fromDegrees(0),
    //     Rotation2d.fromDegrees(0), new Translation2d(Units.inchesToMeters(-36), 0)));
    // new JoystickButton(driverXbox, 5).onTrue();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAutonomousCommand() {
    Command reachout = new RunCommand(() -> Robot.wristleft.set(ControlMode.PercentOutput, .2));
    Command zero = new RunCommand(() -> Robot.wristleft.setSelectedSensorPosition(0));
    Command stop = new RunCommand(() -> Robot.wristleft.set(ControlMode.PercentOutput, -.09));
    Command fart = new RunCommand(() -> Robot.intake_robin.set(ControlMode.PercentOutput, 1));
    Command poop = new RunCommand(() -> Robot.intake_batman.set(ControlMode.PercentOutput, -1));



    // An example command will be run in autonomous
    return Commands.sequence(
    zero.withTimeout(.25),
    reachout.withTimeout(1.00),
    stop.withTimeout(.25),
    fart.withTimeout(.25),
    poop.withTimeout(.25),

    Autos.exampleAuto(drivebase, AutoPath.getSelected())
    );
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }


  /**
   * Create a {@link FollowTrajectory} command to go to the April Tag from the
   * current position.
   *
   * @param swerve            Swerve drive subsystem.
   * @param id                April Tag ID to go to.
   * @param rotation          Rotation to go to.
   * @param holonomicRotation Holonomic rotation to be at.
   * @param offset            Offset from the April Tag.
   * @return {@link FollowTrajectory} command. May return null if cannot load
   *         field.
   */
  public CommandBase driveToAprilTag(SwerveSubsystem swerve, int id, Rotation2d rotation,
  Rotation2d holonomicRotation, Translation2d offset) {
        
    if (aprilTagField == null) {
      try {
        aprilTagField = new AprilTagFieldLayout(
            Filesystem.getDeployDirectory() + "/apriltags/2023-chargedup.json");
      } catch (Exception ignored) {
        return null;
      }
    }
    PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(4, 3), false,
        PathPoint.fromCurrentHolonomicState(swerve.getPose(),swerve.getRobotVelocity()),
        // new PathPoint(new Translation2d(10,7), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        new PathPoint(aprilTagField.getTagPose(id).get().getTranslation()
            .toTranslation2d().plus(offset),
            rotation, holonomicRotation));
            swerve.postTrajectory(path);
    return Commands.sequence(new FollowTrajectory(swerve, path, false));
  }


  public CommandBase autoToConePickup(SwerveSubsystem swerve, int id, Translation2d offset){

    if (aprilTagField == null) {
      try {
        aprilTagField = new AprilTagFieldLayout(
            Filesystem.getDeployDirectory() + "/apriltags/2023-chargedup.json");
      } catch (Exception ignored) {
        return null;
      }
    }

    PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(4, 3), false,
        PathPoint.fromCurrentHolonomicState(swerve.getPose(),swerve.getRobotVelocity()),
        new PathPoint(new Translation2d(10,7), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
                new PathPoint(aprilTagField.getTagPose(id).get().getTranslation()
            .toTranslation2d().plus(offset),
            Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
            // swerve.postTrajectory(path);
    return Commands.sequence(new FollowTrajectory(swerve, path, false),
    new RunCommand(() -> arm.setArmPosition(47, 75, 9), arm));
  }

}
