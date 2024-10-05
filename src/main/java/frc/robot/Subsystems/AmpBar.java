package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.kAmpBar;
import monologue.Logged;

public class AmpBar extends SubsystemBase implements Logged {

  public CANSparkMax PivotNEO = new CANSparkMax(
    kAmpBar.kPorts.PivotNeoPort,
    MotorType.kBrushless
  );
  public CANSparkMax IntakeNeoMini = new CANSparkMax(
    kAmpBar.kPorts.IntakeNeoPort,
    MotorType.kBrushless
  );

  public ProfiledPIDController PivotController;

  public RelativeEncoder PivotEncoder;

  public AmpBar() {
    // set up voltage comp and reduce strain on the battery by seting a curent limit
    PivotNEO.enableVoltageCompensation(kAmpBar.kLimits.NominalVoltage);
    PivotNEO.setSmartCurrentLimit(kAmpBar.kLimits.CurrentLimit);
    PivotNEO.setIdleMode(IdleMode.kBrake);

    // Intake Neo 550 voltage comp and idle mode, coast so drive team can put in the first note
    IntakeNeoMini.enableVoltageCompensation(kAmpBar.kLimits.NominalVoltage);
    IntakeNeoMini.setIdleMode(IdleMode.kCoast);

    // Defines the Pivot Contorller with constrained Acceleration a deceleration
    PivotController =
      new ProfiledPIDController(
        kAmpBar.kPIDConstants.kP,
        0,
        0,
        kAmpBar.kSpeeds.Constraints
      );

    // track the pivot angle
    PivotEncoder = PivotNEO.getEncoder();
  }

  public Command PassThru() {
    return this.runOnce(() -> {
        IntakeNeoMini.setIdleMode(IdleMode.kCoast);
        IntakeNeoMini.set(kAmpBar.kSpeeds.ShotSpeed);
      });
  }

  public Command AmpShot() {
    return this.runOnce(() -> {
        resetPID();
      })
      .andThen(
        this.run(() -> {
            PivotController.calculate(
              PivotEncoder.getPosition(),
              kAmpBar.kPositions.ShotPos
            );
          })
        .until(() -> PivotController.atGoal())
      ).andThen(
        PassThru()
      );
  }

  public Command Stow() {
    return this.runOnce(() -> {
        resetPID();
      })
      .andThen(() -> {
        PivotController.calculate(
          PivotEncoder.getPosition(),
          kAmpBar.kPositions.StowPos
        );
        IntakeNeoMini.set(0);
      })
      .until(() -> PivotController.atGoal());
  }

  public void resetPID() {
    PivotController.reset(
      PivotEncoder.getPosition(),
      PivotEncoder.getVelocity()
    );
  }
}
