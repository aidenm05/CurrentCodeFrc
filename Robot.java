// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

public class Robot extends TimedRobot {
  private static final double kUnitsPerRevolution = 360;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Timer timer;
  // public SN_Blinkin ledController;

  private Timer disabledTimer;
  /* Hardware */
  static XboxController _operator = new XboxController(2);
  private TalonFX elevator_crude = new TalonFX(4);
  private TalonFX _gearbox = new TalonFX(16);

  public static TalonFX wrist = new TalonFX(24);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  public DigitalInput bottomlimit = new DigitalInput(0);
  static TalonFX intake_batman = new TalonFX(23);
  static TalonFX intake_robin = new TalonFX(20);
  public final PneumaticHub phub = new PneumaticHub(15);
  private Solenoid batsolenoid = new Solenoid(15, PneumaticsModuleType.REVPH, 7);
  private Solenoid robsolenoid = new Solenoid(15, PneumaticsModuleType.REVPH, 0);
  public double _pos = wrist.getSelectedSensorPosition();
  public DutyCycleEncoder gearabsolute = new DutyCycleEncoder(1);
  public DutyCycleEncoder armabsolute = new DutyCycleEncoder(2);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  public SN_Blinkin SN_Blinkin = new SN_Blinkin(6);

  @Override
  public void robotInit() {
    phub.enableCompressorAnalog(80, 100);// TODO: check limits

    PathPlannerServer.startServer(5811);
    wrist.clearStickyFaults();
    intake_batman.clearStickyFaults();
    intake_robin.clearStickyFaults();
    _gearbox.clearStickyFaults();
    elevator_crude.clearStickyFaults();

    // ------------------------------------------
    elevator_crude.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    wrist.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    elevator_crude.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    // ------------------------------------------

    phub.clearStickyFaults();
    timer = new Timer();
    CameraServer.startAutomaticCapture("driver_camera", 0);
    armabsolute.setDistancePerRotation(360.0);
    gearabsolute.setDistancePerRotation(360.0);
    // SN_Blinkin.setPattern(PatternType.HotPink);
    m_robotContainer = new RobotContainer();
    disabledTimer = new Timer();

    intake_batman.setNeutralMode(NeutralMode.Brake);
    intake_robin.setNeutralMode(NeutralMode.Brake);
    elevator_crude.setNeutralMode(NeutralMode.Brake);
    _gearbox.setNeutralMode(NeutralMode.Brake);

    wrist.configFactoryDefault();
    intake_batman.configFactoryDefault();
    intake_robin.configFactoryDefault();
    elevator_crude.configFactoryDefault();
    _gearbox.configFactoryDefault();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    wrist.setNeutralMode(NeutralMode.Coast);
    wrist.setNeutralMode(NeutralMode.Coast);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

    if (_pos == 12000) {
      wrist.set(ControlMode.PercentOutput, 0.05);
    }

  }

  @Override
  public void teleopInit() {
    phub.enableCompressorAnalog(80, 100);// TODO: check limits
    batsolenoid.set(false);
    robsolenoid.set(true);
    wrist.setNeutralMode(NeutralMode.Brake);
    armabsolute.reset();

    // get the position offset from when the encoder was reset
    armabsolute.getPositionOffset();

    // set the position offset to half a rotation
    armabsolute.setPositionOffset(0);
    gearabsolute.reset();

    // get the position offset from when the encoder was reset
    gearabsolute.getPositionOffset();

    // set the position offset to half a rotation
    gearabsolute.setPositionOffset(0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.setDriveMode();
   // m_robotContainer.setMotorBrake(true);
  }

  @Override
  public void teleopPeriodic() {
    double appliedMotorOutput = wrist.getMotorOutputPercent();
    double selSenPos = wrist.getSelectedSensorPosition(0); /* position units */
    double selSenVel = wrist.getSelectedSensorVelocity(0); /* position units per 100ms */
    double pos_Rotations = (double) selSenPos / kUnitsPerRevolution;
    double vel_RotPerSec = (double) selSenVel / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
    double vel_RotPerMin = vel_RotPerSec * 60.0;
    boolean toggle = false; 
    SmartDashboard.putNumber("Motor-out: %.2f | ", appliedMotorOutput);
    SmartDashboard.putNumber("Pos-units: %.2f | ", selSenPos);
    SmartDashboard.putNumber("Vel-unitsPer100ms: %.2f | ", selSenVel);
    SmartDashboard.putNumber("Pos-Rotations:%.3f | ", pos_Rotations);
    SmartDashboard.putNumber("Vel-RotPerSec:%.3f | ", vel_RotPerSec);
    SmartDashboard.putNumber("Vel-RotPerMin:%.3f | ", vel_RotPerMin);

    /* get the raw sensor values */
    int rawPos = wrist.getSelectedSensorPosition(0);
    int rawVel = wrist.getSelectedSensorVelocity(0);
    int rawCur = wrist.getStatorCurrent();
    /* get the converted sensor values */
    double posRot = wrist.getSelectedSensorPosition(0) * 360.0 / 4096.0;
    double velRPM = wrist.getSelectedSensorVelocity(0) * 600.0 / 4096.0;
    double curAmp = wrist.getStatorCurrent() / 100.0;

    SmartDashboard.putNumber("RawPos: %.2f | ", rawPos);
    SmartDashboard.putNumber("RawVel: %.2f | ", rawVel);
    SmartDashboard.putNumber("RawCur: %.2f | ", rawCur);
    SmartDashboard.putNumber("PosRot: %.2f | ", posRot);
    SmartDashboard.putNumber("VelRPM: %.2f | ", velRPM);
    SmartDashboard.putNumber("CurAmp: %.2f | ", curAmp);

    if (_operator.getXButton()) {
      batsolenoid.set(true);
      robsolenoid.set(false);
    } else if (_operator.getYButton()) {
      batsolenoid.set(false);
      robsolenoid.set(true);
    }

    if (_operator.getAButton()) {
      if (toggle == false) {
        toggle = true;
      } else {
        toggle = false;
      }
    }
  if(toggle == true){
  batsolenoid.set(true);
  robsolenoid.set(false);
  } else {
  batsolenoid.set(false);
  robsolenoid.set(true);
  }

    if (_operator.getRightBumper()) {
      intake_batman.set(TalonFXControlMode.PercentOutput, .2);
      intake_robin.set(TalonFXControlMode.PercentOutput, -.2);
    } else if (_operator.getLeftBumper()) {
      intake_batman.set(TalonFXControlMode.PercentOutput, -.5);
      intake_robin.set(TalonFXControlMode.PercentOutput, .5);
    } else if (_operator.getStartButton()) {
      intake_batman.set(TalonFXControlMode.PercentOutput, -1);
      intake_robin.set(TalonFXControlMode.PercentOutput, 1);
    } else if (_operator.getBackButton()) {
      intake_batman.set(TalonFXControlMode.PercentOutput, .2);
      intake_robin.set(TalonFXControlMode.PercentOutput, -.2);
    } else {
      intake_batman.set(TalonFXControlMode.PercentOutput, 0);
      intake_robin.set(TalonFXControlMode.PercentOutput, 0);
    }
   

    if (_operator.getLeftTriggerAxis() > 0) {
      wrist.set(ControlMode.PercentOutput, _operator.getLeftTriggerAxis() / 4);
    } else if (_operator.getRightTriggerAxis() > 0) {
      wrist.set(ControlMode.PercentOutput, -_operator.getRightTriggerAxis() / 4);
    } else {
      wrist.set(ControlMode.Position, wrist.getSelectedSensorPosition() + 1);
    }

    Color detectedColor = m_colorSensor.getColor();

    if (detectedColor.blue > .3) {
      SN_Blinkin.setPattern(PatternType.BPMPartyPalette);
    } else if (detectedColor.blue > .32 && detectedColor.red <= 0.35) {
      SN_Blinkin.setPattern(PatternType.EndToEndBlendToBlackC2P);
    } else {
      SN_Blinkin.setPattern(PatternType.HotPink);
    }
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("armduty", armabsolute.getDistance());
    SmartDashboard.putNumber("gearduty", gearabsolute.getDistance());
    SmartDashboard.putNumber("Wrist left encoder value", wrist.getSelectedSensorPosition());

    _gearbox.set(TalonFXControlMode.PercentOutput, _operator.getLeftY());

    phub.enableCompressorAnalog(115, 120);

    if (bottomlimit.get()) {
      elevator_crude.set(TalonFXControlMode.PercentOutput, _operator.getRawAxis(5) / 3);
    } else {
      elevator_crude.set(TalonFXControlMode.PercentOutput,.1 + (- (_operator.getRawAxis(5) / 3)));
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
