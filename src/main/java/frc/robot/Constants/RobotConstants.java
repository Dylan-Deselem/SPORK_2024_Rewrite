package frc.robot.Constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class RobotConstants {

  public static class kSwerve {

    private static double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(
      24
    );
    private static double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(
      24
    );
    public static double wheelDiameter = Units.inchesToMeters(4);
    public static double wheelCircumference = wheelDiameter * Math.PI;
    public static double MaxSpeed = Units.feetToMeters(12);

    public static class kControlConstants {

      public static double kDeadband = 0.1;
    }

    public static class kVision {

      public static class kLimelightNames {

        public static String LimeLight1 = "Limelight";
        public static String Limelight2 = "Limelight2";
      }
    }

    public static class kControls {

      public static HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
        new PIDConstants(5),
        new PIDConstants(5),
        kSwerve.MaxSpeed,
        DRIVETRAIN_WHEELBASE_METERS / 2,
        new ReplanningConfig(),
        0.02
      );

      public static Rotation2d AmpRotation = Rotation2d.fromRadians(
        Math.PI / 2
      );

      public static double MaxPathSpeed = 1.5;
      public static double MaxPathAccel = 1.5;

      public static double MaxAngularSpeed = 12;
      public static double MaxAngularAccel = 5;

      public static TrapezoidProfile.Constraints Constraints = new TrapezoidProfile.Constraints(
        MaxPathSpeed,
        MaxPathAccel
      );

      public static TrapezoidProfile.Constraints AngularConstrants = new TrapezoidProfile.Constraints(
        MaxAngularSpeed,
        MaxAngularAccel
      );

      public static double slowMod = 0.5;
    }

    public static class kMotorConstants {

      public static IdleMode AzumuthIdleMode = IdleMode.kBrake;
      public static IdleMode DriveIdleMode = IdleMode.kBrake;

      public static int frontLeftSteer = 1;
      public static int backLeftSteer = 2;
      public static int frontRightSteer = 3;
      public static int backRightSteer = 4;

      public static int frontLeftDrive = 5;
      public static int backLeftDrive = 6;
      public static int frontRightDrive = 7;
      public static int backRightDrive = 8;
    }

    public static class kAbsoluteEncoders {

      public static int kFrontLeftDAbsoluteEncoderPort = 1;
      public static int kBackLeftAbsoluteEncoderPort = 2;
      public static int kFrontRightAbsoluteEncoderPort = 3;
      public static int kBackRightAbsoluteEncoderPort = 4;
    }

    public static class kOffsets {

      public static Rotation2d FlOffset = Rotation2d.fromDegrees(0);
      public static Rotation2d BlOffset = Rotation2d.fromDegrees(0);
      public static Rotation2d FrOffset = Rotation2d.fromDegrees(0);
      public static Rotation2d BrOffset = Rotation2d.fromDegrees(0);
    }

    public static class kPIDConstants {

      public static double TranslationkP = 12;
      public static double TranslationkI = 0;
      public static double TranslationkD = 0;

      public static double AngularkP = 4;
      public static double AngularkI = 0;
      public static double AngularkD = 0.5;
    }

    public static class kFeedForwardConstatnts {}

    public static class kMessurements {

      public static final double wheelDiameter = Units.inchesToMeters(4.0);
      public static final double WheelCircumference = wheelDiameter * Math.PI;
      public static final double RotationGearRatio = (150 / 7) / 1;
      public static final double DriveGearRatio = 8.14 / 1;
      public static final Translation2d ChassisOffset = new Translation2d(
        0,
        DRIVETRAIN_TRACKWIDTH_METERS * 0.75
      );
    }

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(
        kSwerve.DRIVETRAIN_WHEELBASE_METERS / 2.0,
        kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2.0
      ),
      // Front right
      new Translation2d(
        kSwerve.DRIVETRAIN_WHEELBASE_METERS / 2.0,
        -kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2.0
      ),
      // Back left
      new Translation2d(
        -kSwerve.DRIVETRAIN_WHEELBASE_METERS / 2.0,
        kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2.0
      ),
      // Back right
      new Translation2d(
        -kSwerve.DRIVETRAIN_WHEELBASE_METERS / 2.0,
        -kSwerve.DRIVETRAIN_TRACKWIDTH_METERS / 2.0
      )
    );
  }

  public static class kIntake {

    public static class kPorts {

      public static int IntakeNeoPort = 11;
      public static int PivotNeoPort = 12;
    }

    public static class kLimits {

      public static int NominalVoltage = 12;
      public static int CurrentLimit = 20;
      public static double MaxRPM = 30;
    }

    public static class kPIDConstants {

      public static double kP = 0.1;
    }

    public static class kPositions {

      public static double ShootingPOS = 0;
      public static double IntakePOS = 48;
      public static double spitPOS = 28;
    }

    public static class kSpeeds {

      public static double IntakeSpeed = 1;
      public static double FeedSpeed = -1;

      public static double MaxPathSpeed = 1;
      public static double MaxPathAccel = 0.3;
      public static TrapezoidProfile.Constraints Constraints = new TrapezoidProfile.Constraints(
        MaxPathSpeed,
        MaxPathAccel
      );
    }
  }

  public static class kAmpBar {

    public static class kPorts {

      public static int IntakeNeoPort = 15;
      public static int PivotNeoPort = 16;
    }

    public static class kLimits {

      public static int NominalVoltage = 12;
      public static int CurrentLimit = 20;
      public static double MaxRPM = 15;
    }

    public static class kPIDConstants {

      public static double kP = 0.1;
    }

    public static class kPositions {

      public static double StowPos = 0;
      public static double ShotPos = 0;
    }

    public static class kSpeeds {

      public static double ShotSpeed = 1;
      public static double StopSpeed = 0;

      public static double MaxPathSpeed = 1;
      public static double MaxPathAccel = 0.3;
      public static TrapezoidProfile.Constraints Constraints = new TrapezoidProfile.Constraints(
        MaxPathSpeed,
        MaxPathAccel
      );
    }
  }

  public static class kShooter {

    public static class kPorts {

      public static int LeftNeoPort = 9;
      public static int RightNeoPort = 10;
    }

    public static class kSpeeds {

      public static double MaxSpeed = 12;
      public static double AmpSpeed = 6;
      public static double IdleSpeed = 3;
    }

    public static class kLimits {

      public static double NominalVoltage = 12;
      public static int CurrentLimit = 20;
    }

    public static class kShooterPID {

      public static double kP = 2;
      public static double kI = 0;
      public static double kD = 0;
    }
  }
}
