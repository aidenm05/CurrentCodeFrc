// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  final double targetEncoder = -1000;
  private RobotContainer m_robotContainer;
  private Timer timer;

  private Timer disabledTimer;
/* Hardware */
static XboxController _operator = new XboxController(1);
 private TalonFX elevator_crude = new TalonFX(Constants.elevatorID);
 private TalonFX _gearbox = new TalonFX(Constants.shoulderID);
private TalonFX wristright = new TalonFX(Constants.wrist2ID);
public static TalonFX wristleft = new TalonFX(Constants.wristID);

static TalonFX intake_batman = new TalonFX(23);
static TalonFX intake_robin = new TalonFX(20);
public final PneumaticHub phub = new PneumaticHub(15);
private Solenoid batsolenoid = new Solenoid(15,PneumaticsModuleType.REVPH, 7);
private Solenoid robsolenoid = new Solenoid(15,PneumaticsModuleType.REVPH, 0);

private DutyCycleEncoder shoulderAbsoluteEncoder = new DutyCycleEncoder(Constants.shoulderAbsoluteEncoderPort);
final double currentEncoder = shoulderAbsoluteEncoder.getAbsolutePosition() ;

  @Override
  public void robotInit() {
    phub.enableCompressorAnalog(80, 115);//TODO: check limits

    PathPlannerServer.startServer(5811);
   // batsolenoid = new Solenoid(PneumaticsModuleType.REVPH, 7);
    //robsolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    timer = new Timer();
     // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
        // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    /* Factory default hardware to prevent unexpected behavior */
		// _arm.configFactoryDefault();
		// _elevator.configFactoryDefault();
		// /* Configure Sensor Source for Pirmary PID */
		// _arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
		// 		Constants.kTimeoutMs);
    // _elevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
		// 		Constants.kTimeoutMs);
		
    intake_batman.setNeutralMode(NeutralMode.Brake);
    intake_robin.setNeutralMode(NeutralMode.Brake);
     elevator_crude.setNeutralMode(NeutralMode.Brake);
    _gearbox.setNeutralMode(NeutralMode.Brake);

    wristleft.configFactoryDefault();
    wristright.configFactoryDefault();
    intake_batman.configFactoryDefault();
    intake_robin.configFactoryDefault();
     elevator_crude.configFactoryDefault();
     _gearbox.configFactoryDefault();
  }

  @Override
  public void robotPeriodic() {

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
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
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    wristleft.setNeutralMode(NeutralMode.Coast);
    wristright.setNeutralMode(NeutralMode.Coast);
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    // if ((Arm.) == 12000)
    // {
    //   wristleft.set(ControlMode.PercentOutput, 0.05);}

  }

  @Override
  public void teleopInit() {
    shoulderAbsoluteEncoder.setPositionOffset(0);
  //  phub.enableCompressorAnalog(80, 115);//TODO: check limits
  batsolenoid.set(false); 
  robsolenoid.set(true); 
  wristleft.setNeutralMode(NeutralMode.Brake);
  wristright.setNeutralMode(NeutralMode.Brake);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (_operator.getRawButton(7)) {
    // If the elevator is above the middle position, run the motor in reverse until the encoder value is close enough to the middle position
    if (currentEncoder > targetEncoder) {
      _gearbox.set(ControlMode.PercentOutput,-.5);
    }
    // If the elevator is below the middle position, run the motor in forward until the encoder value is close enough to the middle position
    else if (currentEncoder < targetEncoder) {
      _gearbox.set(ControlMode.PercentOutput,.5);
    }
    // If the elevator is at or very close to the middle position, stop the motor and lock the Falcon 500
    else {
     
    }}
   
    SmartDashboard.putNumber(" Position",shoulderAbsoluteEncoder.getAbsolutePosition());

  
 //--------------------------------------------------------------------------------------------------------------------------
    if (_operator.getXButton()) {
      batsolenoid.set(false); 
    } else if (_operator.getYButton()) {
      batsolenoid.set(true); 
    }
    if (_operator.getXButton()) {
      robsolenoid.set(true); 
    } else if (_operator.getYButton()) {
      robsolenoid.set(false); 
    }


    if (_operator.getRightBumper()) {
      intake_batman.set(TalonFXControlMode.PercentOutput, .4);
      intake_robin.set(TalonFXControlMode.PercentOutput, -.4);

    } else if (_operator.getLeftBumper()) {
      intake_batman.set(TalonFXControlMode.PercentOutput, -1);
      intake_robin.set(TalonFXControlMode.PercentOutput, 1);

    } else {
      intake_batman.set(TalonFXControlMode.PercentOutput, 0);
      intake_robin.set(TalonFXControlMode.PercentOutput, 0);

    }
    if (_operator.getAButton()) {
      wristright.set(ControlMode.PercentOutput,_operator.getRightTriggerAxis()/5);
       wristleft.set(ControlMode.PercentOutput,_operator.getRightTriggerAxis()/5);

    } else if (_operator.getBButton()) {
      wristright.set(ControlMode.PercentOutput,_operator.getRightTriggerAxis()/5);
       wristleft.set(ControlMode.PercentOutput,_operator.getRightTriggerAxis()/5);

    } else {
      wristright.set(TalonFXControlMode.PercentOutput, 0.04);
      wristleft.set(TalonFXControlMode.PercentOutput,-0.04);

    }


  phub.enableCompressorAnalog(115, 120);

//  if (_operator.getRightY()==(0)){
//    _gearbox.set(TalonFXControlMode.PercentOutput, 0.05);
//  }
//  if (_operator.getRawAxis(5)==(0)){
//   elevator_crude.set(TalonFXControlMode.PercentOutput, 0.05);
// }
//  elevator_crude.set(ControlMode.PercentOutput,-_operator.getRawAxis(5)/5);

//  _gearbox.set(ControlMode.PercentOutput,_operator.getLeftY());


//-----------------------------------------------------------------------------------------------------------------------------
//     /* Get gampad axis - forward stick is positive */
// 		double leftYstick = -1.0 * _operator.getLeftY(); /* left-side Y for Gamepad */
// 		double rghtYstick = -1.0 * _operator.getRightY(); /* right-side Y for Gamepad */
//     double rightystick = -1.0 * _operator.getRightX(); /* left-side Y for Gamepad */
// 		double leftxstick = -1.0 * _operator.getLeftX(); /* right-side Y for Gamepad */
// //-----------------------------------------------------------------------------
//   if (Math.abs(leftYstick) < 0.10) { leftYstick = 0; } /* deadband 10% */
// 		if (Math.abs(rghtYstick) < 0.10) { rghtYstick = 0; } /* deadband 10% */
// //-----------------------------------------------------------------------------
// 		/* Get current Talon FX motor output */
// 		double motorOutput = _arm.getMotorOutputPercent();	
//     double elevatormotorOutput = _elevator.getMotorOutputPercent();

// //-----------------------------------------------------------------------------
// 		/* Prepare line to print */
// 		_sb.append("\tOut%:");
// 		_sb.append(motorOutput);
// 		_sb.append("\tVel:");
// 		_sb.append(_arm.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
// 		/** Perform Motion Magic when Button 1 is held, else run Percent Output, which can
// 		 * be used to confirm hardware setup. */
// 		if (!_operator.getRawButton(1)) {
// 			/* Motion Magic */
// 			/* 2048 ticks/rev * 10 Rotations in either direction */
// 			double targetPos = rghtYstick * 2048 * 10.0;
// 			_arm.set(TalonFXControlMode.MotionMagic, targetPos);
// 			/* Append more signals to print when in speed mode */
// 			_sb.append("\terr:");
// 			_sb.append(_arm.getClosedLoopError(Constants.kPIDLoopIdx));
// 			_sb.append("\ttrg:");
// 			_sb.append(targetPos);
// 		} else {
// 			/* Percent Output */
// 			_arm.set(TalonFXControlMode.PercentOutput, leftYstick);
// 		}
// //-----------------------------------------------------------------------------
// 		if (_operator.getRawButton(2)) {
// 			/* Zero sensor positions */
// 			_arm.setSelectedSensorPosition(0);
// 		}
// 		int pov = _operator.getPOV();
// 		if (_pov == pov) {
// 			/* no change */
// 		} else if (_pov == 180) { // D-Pad down
// 			/* Decrease smoothing */
// 			_smoothing--;
// 			if (_smoothing < 0)
// 				_smoothing = 0;
// 			_arm.configMotionSCurveStrength(_smoothing);

// 			System.out.println("Smoothing is set to: " + _smoothing);
// 		} else if (_pov == 0) { // D-Pad up
// 			/* Increase smoothing */
// 			_smoothing++;
// 			if (_smoothing > 8)
// 				_smoothing = 8;
// 			_arm.configMotionSCurveStrength(_smoothing);

// 			System.out.println("Smoothing is set to: " + _smoothing);
// 		}
// 		_pov = pov; /* save the pov value for next time *
// 		 Instrumentation */
// 		Instrum.Process(_arm, _sb);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
