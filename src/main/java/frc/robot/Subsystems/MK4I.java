package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

public class MK4I extends SubsystemBase {

  private Rotation2d Offset;

  private SwerveModuleState State;
  private SwerveModuleState optimizedState;

  public CANcoder AbsoluteEncoder;
  public CANcoderConfiguration config;
  public CANcoderConfigurator CANconfig;

  private CANSparkMax DriveMotor;
  private CANSparkMax AzimuthMotor;

  private PIDController AzimuthController;

  private RelativeEncoder DriveEncoder;
  private double SimDistance = 0;

  private SimpleMotorFeedforward OpenLoopFF = new SimpleMotorFeedforward(
    0.046,
    2.67,
    0.113
  );

  public MK4I(
    int DriveID,
    int AzimuthID,
    int AbsoluteEncoderID,
    Rotation2d Offset
  ) {
    this.Offset = Offset;
    State = new SwerveModuleState();
    optimizedState = new SwerveModuleState();

    DriveMotor = new CANSparkMax(DriveID, MotorType.kBrushless);
    DriveEncoder = DriveMotor.getEncoder();
    DriveEncoder.setPosition(0);
    DriveMotor.setIdleMode(RobotConstants.kSwerve.kMotorConstants.DriveIdleMode);
    DriveMotor.setInverted(false);

    AzimuthMotor = new CANSparkMax(AzimuthID, MotorType.kBrushless);
    AzimuthMotor.setIdleMode(RobotConstants.kSwerve.kMotorConstants.AzumuthIdleMode);
    AzimuthMotor.setInverted(false);

    AzimuthController = new PIDController(2, 0, 0);

    AzimuthController.enableContinuousInput(0, 1);

    AbsoluteEncoder = new CANcoder(AbsoluteEncoderID);
    CANconfig = AbsoluteEncoder.getConfigurator();
    config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange =
      AbsoluteSensorRangeValue.Unsigned_0To1;
    CANconfig.apply(config);
    }

  public Rotation2d getAngle() {
    if(Robot.isSimulation()){
      return getTargetState().angle;
    }
    return Rotation2d.fromRotations(
      AbsoluteEncoder.getPosition().getValueAsDouble() - Offset.getRotations()
    );
  }

  public void setState(SwerveModuleState state) {
    State = state;
    optimizedState = SwerveModuleState.optimize(state, getAngle());

    if(Robot.isSimulation()){
      SimDistance += optimizedState.speedMetersPerSecond * 0.02;
      return;
    }

    DriveMotor.setVoltage(OpenLoopFF.calculate(optimizedState.speedMetersPerSecond));

    AzimuthMotor.setVoltage(
      AzimuthController.calculate(
        optimizedState.angle.getRotations(),
        getAngle().getRotations()
      )
    );
  }
  

  // Gets the targeted state before optimizing 
  public SwerveModuleState getTargetState(){
    return State;
  }

  // Gets the targeted state after optimizing 
  public SwerveModuleState getOptimizedState(){
    return optimizedState;
  }

  public SwerveModulePosition getPosition() {
    if(Robot.isSimulation()){
      return new SwerveModulePosition(SimDistance, getAngle());
    }
    return new SwerveModulePosition(gearReduction() , getAngle());
  }

  public SwerveModuleState getState() {
    if (Robot.isSimulation()){return getTargetState();}
    return new SwerveModuleState(RPM_TO_M_per_S(DriveEncoder.getVelocity()), getAngle());
  }

  // Converts RPM to M/S for any module
  private double RPM_TO_M_per_S(double RPM){
    double velocity = (((2 * Math.PI) * (RobotConstants.kSwerve.wheelDiameter /2 )) / 60) * RPM;
    return velocity;
  }

  // for SDS L1 module use 8.14, use 6.75 and 6.12 for L2 and L3 respectively 
  private double gearReduction(){
    double wheelRotations = DriveEncoder.getPosition() / 8.14;
    return wheelRotations * RobotConstants.kSwerve.wheelCircumference; // Distance in meters
  }
}
