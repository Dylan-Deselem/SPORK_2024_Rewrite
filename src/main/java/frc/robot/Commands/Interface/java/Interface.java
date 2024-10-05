package frc.robot.Commands.Interface.java;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.AmpBar;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class Interface {

  public Shooter mShooter;
  public Intake mIntake;
  public AmpBar mAmpBar;

  public Interface(Shooter mShooter, Intake mIntake, AmpBar mAmpBar) {
    this.mAmpBar = mAmpBar;
    this.mIntake = mIntake;
    this.mShooter = mShooter;
  }

  public Command AmpShot() {
    return Commands
      .runOnce(() -> {
        mAmpBar.AmpShot();
        mIntake.IntakeUp();
      })
      .withTimeout(1)
      .andThen(mShooter.AmpMode())
      .andThen(mIntake.FeedShooter())
      .withTimeout(2)
      .finallyDo(() -> this.returnToDefault());
  }

  public Command returnToDefault() {
    return Commands.runOnce(() -> {
      mAmpBar.Stow();
      mIntake.IntakeUp();
      mShooter.IdleMode();
    });
  }
}
