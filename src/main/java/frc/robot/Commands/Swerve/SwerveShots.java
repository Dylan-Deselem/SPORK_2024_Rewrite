package frc.robot.Commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Interface.java.Interface;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants.kSwerve;
import frc.robot.Subsystems.Swerve;

public class SwerveShots {

  public Swerve mSwerve;
  public Interface mInterface;

  public SwerveShots(Swerve mSwerve, Interface mInterface) {
    this.mSwerve = mSwerve;
    this.mInterface = mInterface;
  }

  public Command AutoAmp() {
    return Commands.sequence(
      Commands
        .either(
          mSwerve.drivetoPoint(
            FieldConstants.BLUE_AMP_TRANS,
            kSwerve.kControls.AmpRotation
          ),
          mSwerve.drivetoPoint(
            FieldConstants.RED_AMP_TRANS,
            kSwerve.kControls.AmpRotation
          ),
          () -> FieldConstants.IsBlueAllience()
        )
        .withTimeout(3),
      Commands
        .run(() -> mSwerve.TeleopDrive(new Translation2d(0.1, 0), 0, false))
        .withTimeout(0.2)
        .alongWith(
          Commands
            .waitUntil(mSwerve::getInAmpPosition)
            .andThen(mInterface.AmpShot())
        )
    );
  }

  public Command TrackSpeaker(DoubleSupplier xTrans, DoubleSupplier yTrans){
    return Commands.sequence(
        Commands.either(
        mSwerve.trackTarget(xTrans, yTrans, FieldConstants.BLUE_SPEAKER_TRANS),
        mSwerve.trackTarget(xTrans, yTrans, FieldConstants.RED_SPEAKER_TRANS),
        () -> FieldConstants.IsBlueAllience())
    );
  }
}
