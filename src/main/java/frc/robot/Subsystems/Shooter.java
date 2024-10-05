package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.kShooter;
import frc.robot.Subsystems.Shooter.ShooterModes.modes;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {

  public class ShooterModes {

    public enum modes {
      Shooting,
      Amp,
      Idle,
    }

    public modes mode;

    public void IntakeModes() {
      mode = modes.Idle;
    }

    public void set(modes modes) {
      this.mode = modes;
    }

    public String getmode() {
      return this.mode.toString();
    }
  }

  private ShooterModes mShooterModes = new ShooterModes();

  private CANSparkMax LNEO = new CANSparkMax(
    kShooter.kPorts.LeftNeoPort,
    MotorType.kBrushless
  );
  private CANSparkMax RNEO = new CANSparkMax(
    kShooter.kPorts.RightNeoPort,
    MotorType.kBrushless
  );

  private double DesiredSpeed;

  private PIDController SpeedsController;

  public Shooter() {
    LNEO.enableVoltageCompensation(kShooter.kLimits.NominalVoltage);
    LNEO.setSmartCurrentLimit(kShooter.kLimits.CurrentLimit);

    RNEO.enableVoltageCompensation(kShooter.kLimits.NominalVoltage);
    RNEO.setSmartCurrentLimit(20);

    SpeedsController =
      new PIDController(
        kShooter.kShooterPID.kP,
        kShooter.kShooterPID.kI,
        kShooter.kShooterPID.kD
      );
  }

  public Command RunShooter(Shooter mShooter) {
    return this.run(
      (
        () -> {
          switch (mShooterModes.mode) {
            case Shooting:
              DesiredSpeed = kShooter.kSpeeds.MaxSpeed;
              break;
            case Amp:
              DesiredSpeed = kShooter.kSpeeds.AmpSpeed;
              break;
            case Idle:
              DesiredSpeed = kShooter.kSpeeds.IdleSpeed;
              break;
            default:
              DesiredSpeed = 0;
              break;
          }
            setVoltage(DesiredSpeed);
        }
      )
    );
  }

  public Command setVoltage(double Volts) {
    return Commands.runOnce(() -> {
      LNEO.setVoltage(SpeedsController.calculate(LNEO.getBusVoltage(), Volts));
      RNEO.setVoltage(SpeedsController.calculate(RNEO.getBusVoltage(), Volts));
    }, this);
  }

  public Command ShootingMode() {
    return this.runOnce(() -> mShooterModes.set(modes.Shooting));
  }

  public Command AmpMode() {
    return this.runOnce(() -> mShooterModes.set(modes.Shooting));
  }

  public Command IdleMode() {
    return this.runOnce(() -> mShooterModes.set(modes.Shooting));
  }

  @Log.NT
  public String getMode() {
    return mShooterModes.getmode();
  }

  @Log.NT
  public double getCurrent() {
    return RNEO.getOutputCurrent();
  }

  @Log.NT
  public double getVolts() {
    return RNEO.getBusVoltage();
  }
}
