package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  public WPI_TalonFX mainMotor;
  public double calculatedPOutput = 0;
  public double motorPosition;
  public int smoothing = 0;
  int upTargetPos = 10000;
  int downTargetPosition = 100;
  int count = 0;

  public Elevator() {
    SupplyCurrentLimitConfiguration elevatorSupplyLimit = new SupplyCurrentLimitConfiguration(
      true,
      25,
      40,
      .1
    );

//need to add second motor configured simmlar to main that acts as the shoulder in conjunction with the eleavyion motor
//shoudlermotor = new WPI_TalonFX(16);
      mainMotor = new WPI_TalonFX(4); 
//shoudlermotor.configselectedfeedbacksensor(talonfxfeedbackdevice.integratedsensor, 0, 30);
//shoudlermotor.setinverted(talonfxinverttype.clockwise);
//shoudlermotor.setneutralmode(neutralmode.brake);
//shoudlermotor.configneutraldeadband(0.001);
//shoudlermotor.configsupplycurrentlimit(shoudlersupplylimit);
//shoudlermotor.setstatusframeperiod(statusframeenhanced.status_13_base_pidf0, 10, constants.ktimeoutms);
//shoudlermotor.setstatusframeperiod(statusframeenhanced.status_10_motionmagic, 10, constants.ktimeoutms);
//shoudlermotor.confignominaloutputforward(0, constants.ktimeoutms);
//shoudlermotor.confignominaloutputreverse(0, constants.ktimeoutms);
//shoudlermotor.configpeakoutputforward(1, constants.ktimeoutms);
//shoudlermotor.configpeakoutputreverse(-1, constants.ktimeoutms);
//shoudlermotor.config_kf(constants.kslotidx0, 0.060176, constants.ktimeoutms);
//shoudlermotor.config_kp(constants.kslotidx0, 0.2, constants.ktimeoutms);
//shoudlermotor.config_ki(constants.kslotidx0, 0, constants.ktimeoutms);
//shoudlermotor.config_kd(constants.kslotidx0, 0, constants.ktimeoutms);
//shoulder.config_IntegralZone(Constants.kSlotIdx0, 200);
//shoulder.configAllowableClosedloopError(Constants.kSlotIdx0, 0);
/* Set Motion Magic gains in slot1 - see documentation */
//shoulder.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
//shoulder.config_kF(Constants.kSlotIdx1, 0.0, Constants.kTimeoutMs);
//shoulder.config_kP(Constants.kSlotIdx1, 0.0, Constants.kTimeoutMs);
//shoulder.config_kI(Constants.kSlotIdx1, 0.0, Constants.kTimeoutMs);
//shoulder.config_kD(Constants.kSlotIdx1, 0.0, Constants.kTimeoutMs);
//shoulder.config_IntegralZone(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
//shoulder.configAllowableClosedloopError(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
//shoulder.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
//shoulder.configMotionAcceleration(6000, Constants.kTimeoutMs);
//shoulder.configForwardSoftLimitEnable(true);
//shoulder.configReverseSoftLimitEnable(true);
//shoulder.configForwardSoftLimitThreshold(10000);
//shoulder.configReverseSoftLimitThreshold(0);

mainMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        0,
        30
      );

  
      mainMotor.setInverted(TalonFXInvertType.Clockwise);
      mainMotor.setNeutralMode(NeutralMode.Brake);
      mainMotor.configNeutralDeadband(0.001);

      mainMotor.configSupplyCurrentLimit(elevatorSupplyLimit);
      mainMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0,
        10,
        Constants.kTimeoutMs
      );

      mainMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic,
        10,
        Constants.kTimeoutMs
      );

      /* Set the peak and nominal outputs */
      mainMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
      mainMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
      mainMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
      mainMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
      mainMotor.config_kF(Constants.kSlotIdx0, 0.060176, Constants.kTimeoutMs);
      mainMotor.config_kP(Constants.kSlotIdx0, 0.2, Constants.kTimeoutMs);
      mainMotor.config_kI(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      mainMotor.config_kD(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      mainMotor.config_IntegralZone(Constants.kSlotIdx0, 200);
      mainMotor.configAllowableClosedloopError(Constants.kSlotIdx0, 400);
      /* Set Motion Magic gains in slot1 - see documentation */
      mainMotor.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
      mainMotor.config_kF(Constants.kSlotIdx1, 0.060176, Constants.kTimeoutMs);
      mainMotor.config_kP(Constants.kSlotIdx1, 0.2, Constants.kTimeoutMs);
      mainMotor.config_kI(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
      mainMotor.config_kD(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
      mainMotor.config_IntegralZone(Constants.kSlotIdx1, 200);
      mainMotor.configAllowableClosedloopError(Constants.kSlotIdx1, 100);
      /* Set acceleration and vcruise velocity - see documentation */
      mainMotor.configMotionCruiseVelocity(17000, Constants.kTimeoutMs);
      mainMotor.configMotionAcceleration(17000, Constants.kTimeoutMs);
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
      /* Set acceleration and vcruise velocity - see documentation */
      mainMotor.configForwardSoftLimitEnable(true);
      mainMotor.configForwardSoftLimitThreshold(Constants.elevatorUpperLimit);


      //DISABLE MOTION MAGIC
     // mainMotor.set(ControlMode.PercentOutput, 0.03);
    }
  

  // public double feedForward() {
  //   double armPos = armMotor.getSelectedSensorPosition();
  //   double degrees =
  //     (armPos - Constants.horizontalPos) / Constants.ticksPerDegrees;
  //   double radians = java.lang.Math.toRadians(degrees);
  //   double cosineScalar = java.lang.Math.cos(radians);
  //   return Constants.maxFF * cosineScalar;
  //}

  //nice run up and down commands
  public CommandBase resetElevatorEncoder() {
    return run(() -> mainMotor.setSelectedSensorPosition(0));
  }

  public CommandBase runDown() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, -.2))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.03))
      .withName("runDown");
  }

  public CommandBase runUp() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, 0.27))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.03))
      .withName("runUp");
  }

  public CommandBase shoulderdown() {
    return run(() -> armMotor.set(TalonFXControlMode.PercentOutput, -0.1))
      .finallyDo(interrupted ->
        armMotor.set(ControlMode.PercentOutput, feedForward())
      )
      .withName("shoulderdown");
  }

  public CommandBase armUp() {
    return run(() -> armMotor.set(TalonFXControlMode.PercentOutput, 0.1))
      .finallyDo(interrupted ->
        armMotor.set(ControlMode.PercentOutput, feedForward())
      )
      .withName("armUp");
  }

  public void armAndElevatorStopPercentMode() {
    // if (!DriverStation.isAutonomous()) {
    armMotor.set(TalonFXControlMode.PercentOutput, feedForward());
    mainMotor.set(TalonFXControlMode.PercentOutput, 0.03);
    // }
  }

  public CommandBase sequentialSetPositions(
    final int elevatorPosition,
    int armPosition
  ) {
    mainMotor.selectProfileSlot(Constants.kSlotIdx0, Constants.kPIDLoopIdx);
    return runOnce(() ->
        armMotor.set(TalonFXControlMode.MotionMagic, Constants.armUpperLimit)
      )
      .andThen(
        Commands
          .waitUntil(() ->
            armMotor.getActiveTrajectoryPosition() >
            Constants.armUpperLimit -
            100
          )
          .withTimeout(1)
      )
      .andThen(
        runOnce(() ->
          mainMotor.set(
            TalonFXControlMode.MotionMagic,
            elevatorPosition,
            DemandType.ArbitraryFeedForward,
            0.03
          )
        )
      )
      .andThen(
        Commands
          .waitUntil(() ->
            mainMotor.getActiveTrajectoryPosition() <
            elevatorPosition +
            30000 &&
            mainMotor.getActiveTrajectoryPosition() > elevatorPosition - 30000
          )
          .withTimeout(1.5)
      )
      .andThen(
        runOnce(() -> armMotor.set(TalonFXControlMode.MotionMagic, armPosition))
      )
      .andThen(
        Commands
          .waitUntil(() ->
            armMotor.getActiveTrajectoryPosition() < armPosition + 20 &&
            armMotor.getActiveTrajectoryPosition() > armPosition - 20
          )
          .withTimeout(1)
      )
      .andThen(runOnce(() -> this.armAndElevatorStopPercentMode()));
  }

  // Test this
  public CommandBase setStow() {
    mainMotor.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
    return runOnce(() ->
        armMotor.set(TalonFXControlMode.MotionMagic, Constants.armUpperLimit)
      )
      // .andThen(
      //   Commands
      // .waitUntil(() ->
      //   armMotor.getActiveTrajectoryPosition() >
      //   Constants.armUpperLimit -
      //   100 &&
      //   armMotor.getActiveTrajectoryPosition() <
      //   Constants.armUpperLimit +
      //   100
      // )
      // .withTimeout(1)
      //) // set to current upperlimit
      .andThen(
        runOnce(() ->
          armMotor.configForwardSoftLimitThreshold(Constants.armStow)
        )
      ) // set soft limit to be stow position
      .andThen(
        runOnce(() ->
          mainMotor.set(
            TalonFXControlMode.MotionMagic,
            Constants.elevatorStow,
            DemandType.ArbitraryFeedForward,
            0.03
          )
        )
      ) // set elevator to 0
      .andThen(
        Commands
          .waitUntil(() ->
            mainMotor.getActiveTrajectoryPosition() <
            Constants.elevatorStow +
            5000 &&
            mainMotor.getActiveTrajectoryPosition() >
            Constants.elevatorStow -
            5000
          )
          .withTimeout(2.25)
      )
      .andThen(
        runOnce(() ->
          armMotor.set(TalonFXControlMode.MotionMagic, Constants.armStow)
        )
      ) //^wait until finished, set arm to stow
      .andThen(
        Commands
          .waitUntil(() ->
            armMotor.getActiveTrajectoryPosition() > Constants.armStow - 20 &&
            armMotor.getActiveTrajectoryPosition() < Constants.armStow + 20
          )
          .withTimeout(3)
      ) //wait until finished
      .andThen(
        runOnce(() ->
          armMotor.configForwardSoftLimitThreshold(Constants.armUpperLimit)
        )
      )
      // ) //set soft limit back to what it was
      .andThen(runOnce(() -> this.armAndElevatorStopPercentMode()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "elevatorEncoderVal",
      mainMotor.getSelectedSensorPosition()
    );
    SmartDashboard.putNumber(
      "armEncoderVal",
      armMotor.getSelectedSensorPosition()
    );
    SmartDashboard.putNumber(
      "Active Trajectory Position",
      armMotor.getActiveTrajectoryPosition()
    );
  }
}
