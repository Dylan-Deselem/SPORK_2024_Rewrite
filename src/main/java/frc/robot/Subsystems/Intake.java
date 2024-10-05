package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.kIntake;

public class Intake extends SubsystemBase {

  public CANSparkMax PivotNEO = new CANSparkMax(
    kIntake.kPorts.PivotNeoPort,
    MotorType.kBrushless
  );
  public CANSparkMax IntakeNeoMini = new CANSparkMax(
    kIntake.kPorts.IntakeNeoPort,
    MotorType.kBrushless
  );

  public ProfiledPIDController PivotController;

  public RelativeEncoder PivotEncoder;

  public Intake() {
    // set up voltage comp and reduce strain on the battery by seting a curent limit
    PivotNEO.enableVoltageCompensation(kIntake.kLimits.NominalVoltage);
    PivotNEO.setSmartCurrentLimit(kIntake.kLimits.CurrentLimit);
    PivotNEO.setIdleMode(IdleMode.kBrake);

    // Intake Neo 550 voltage comp and idle mode, coast so drive team can put in the first note
    IntakeNeoMini.enableVoltageCompensation(kIntake.kLimits.NominalVoltage);
    IntakeNeoMini.setIdleMode(IdleMode.kCoast);

    // Defines the Pivot Contorller with constrained Acceleration a deceleration
    PivotController =
      new ProfiledPIDController(
        kIntake.kPIDConstants.kP,
        0,
        0,
        kIntake.kSpeeds.Constraints
      );

    // track the pivot angle
    PivotEncoder = PivotNEO.getEncoder();
  }

  public Command FeedShooter() {
    return Commands.runOnce(() -> {
      IntakeNeoMini.setIdleMode(IdleMode.kCoast);
      IntakeNeoMini.set(kIntake.kSpeeds.FeedSpeed);
    });
  }

  public Command stopFeed() {
    return Commands.runOnce(() -> {
      IntakeNeoMini.setIdleMode(IdleMode.kBrake);
      IntakeNeoMini.set(0);
    });
  }

  public Command IntakeDown() {
    return this.runOnce(() -> {
        resetPID();
      })
      .andThen(() -> {
        PivotController.calculate(
          PivotEncoder.getPosition(),
          kIntake.kPositions.IntakePOS
        );
        IntakeNeoMini.set(kIntake.kSpeeds.IntakeSpeed);
      })
      .until(() -> PivotController.atGoal());
  }

  public Command IntakeUp() {
    return this.runOnce(() -> {
        resetPID();
      })
      .andThen(() -> {
        PivotController.calculate(
          PivotEncoder.getPosition(),
          kIntake.kPositions.ShootingPOS
        );
        IntakeNeoMini.set(0);
      })
      .until(() -> PivotController.atGoal());
  }

  public Command IntakeSpit() {
    return this.runOnce(() -> {
        resetPID();
      })
      .andThen(
        this.run(() ->
            PivotController.calculate(
              PivotEncoder.getPosition(),
              kIntake.kPositions.spitPOS
            )
          )
      )
      .until(() -> PivotController.atGoal())
      .andThen(() -> FeedShooter());
  }

  public void resetPID() {
    PivotController.reset(
      PivotEncoder.getPosition(),
      PivotEncoder.getVelocity()
    );
  }
}
