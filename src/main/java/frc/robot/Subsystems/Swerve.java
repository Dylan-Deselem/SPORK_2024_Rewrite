package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.kSwerve;
import frc.robot.Constants.RobotConstants.kSwerve.kVision.kLimelightNames;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Swerve extends SubsystemBase implements Logged {

  // Modules

  public static MK4I FL = new MK4I(
    kSwerve.kMotorConstants.frontLeftSteer,
    kSwerve.kMotorConstants.frontLeftDrive,
    kSwerve.kAbsoluteEncoders.kFrontLeftDAbsoluteEncoderPort,
    kSwerve.kOffsets.FlOffset
  );
  public static MK4I FR = new MK4I(
    kSwerve.kMotorConstants.frontRightSteer,
    kSwerve.kMotorConstants.frontRightDrive,
    kSwerve.kAbsoluteEncoders.kFrontRightAbsoluteEncoderPort,
    kSwerve.kOffsets.FrOffset
  );
  public static MK4I BL = new MK4I(
    kSwerve.kMotorConstants.backLeftSteer,
    kSwerve.kMotorConstants.backLeftDrive,
    kSwerve.kAbsoluteEncoders.kBackLeftAbsoluteEncoderPort,
    kSwerve.kOffsets.BlOffset
  );
  public static MK4I BR = new MK4I(
    kSwerve.kMotorConstants.backRightSteer,
    kSwerve.kMotorConstants.backRightDrive,
    kSwerve.kAbsoluteEncoders.kBackRightAbsoluteEncoderPort,
    kSwerve.kOffsets.BrOffset
  );

  public SwerveDrivePoseEstimator PoseEstimator;

  public ProfiledPIDController Xcontrol;
  public ProfiledPIDController Ycontrol;
  public ProfiledPIDController THETAcontrol;

  @Log.NT
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  @Log.NT
  private final Field2d field2d = new Field2d();

  private final SimDeviceSim simNavX = new SimDeviceSim("navX-Sensor", 0);
  private final SimDouble simNavXYaw = simNavX.getDouble("Yaw");

  public class SwerveState {

    public enum Mode {
      Focus,
      Normal,
    }

    public Mode mode;

    public SwerveState() {
      mode = Mode.Normal;
    }

    public void set(Mode mode) {
      this.mode = mode;
    }
  }

  public SwerveState mSwerveState = new SwerveState();

  public Swerve() {
    PoseEstimator =
      new SwerveDrivePoseEstimator(
        kSwerve.kinematics,
        getGyroAngle(),
        getPositions(),
        new Pose2d()
      );

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::AutoDrive,
      kSwerve.kControls.config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );

    Xcontrol =
      new ProfiledPIDController(
        RobotConstants.kSwerve.kPIDConstants.TranslationkP,
        0,
        RobotConstants.kSwerve.kPIDConstants.TranslationkD,
        RobotConstants.kSwerve.kControls.Constraints
      );
    Ycontrol =
      new ProfiledPIDController(
        RobotConstants.kSwerve.kPIDConstants.TranslationkP,
        0,
        RobotConstants.kSwerve.kPIDConstants.TranslationkD,
        RobotConstants.kSwerve.kControls.Constraints
      );
    THETAcontrol =
      new ProfiledPIDController(
        RobotConstants.kSwerve.kPIDConstants.AngularkP,
        RobotConstants.kSwerve.kPIDConstants.AngularkI,
        RobotConstants.kSwerve.kPIDConstants.AngularkD,
        RobotConstants.kSwerve.kControls.AngularConstrants
      );

    Xcontrol.setTolerance(0.05);
    Ycontrol.setTolerance(0.05);

    THETAcontrol.enableContinuousInput(0, Math.PI * 2);
    THETAcontrol.setTolerance(0.05);
  }

  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      simNavXYaw.set(
        simNavXYaw.get() +
        chassisSpeeds.omegaRadiansPerSecond *
        -360 /
        (2 * Math.PI) *
        0.02
      );
      PoseEstimator.update(getGyroAngle(), getPositions());
    }

    /////////
    // Vision Pose est
    /////////

    if (Robot.isReal()) {
      boolean Reject = false;
      // seting the orentation of the robot based on gyro mesurements
      // used for megatag 2
      LimelightHelpers.SetRobotOrientation(
        kLimelightNames.LimeLight1,
        Robot.Gyro.getYaw(),
        Robot.Gyro.getRate(),
        Robot.Gyro.getPitch(),
        0,
        Robot.Gyro.getRoll(),
        0
      );

      // Update before vision measurement
      PoseEstimator.update(getGyroAngle(), getPositions());

      // for using Vision in pose estimation it requires
      // The pose of the limelight must be configured via the API or the webUI
      // a field map has been uploaded (.fmap) this is provided by FIRST
      // a Bot pose estimate is made per periodic cycle
      // Set robot orentaton has a blue corner origin

      // creates a BotPose est
      LimelightHelpers.PoseEstimate MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
        kLimelightNames.LimeLight1
      );

      // if the rate of rotation is above 720 degrees per second, reject the pose
      if (Math.abs(Robot.Gyro.getRate()) > 720) {
        Reject = true;
      }
      // if no tags, reject the pose
      if (MegaTag2.tagCount == 0) {
        Reject = true;
      }
      // if the pose was not rejected set the values to trust in pose est and update the pose
      if (!Reject) {
        PoseEstimator.setVisionMeasurementStdDevs(
          // the higher the number the less trust it will have
          VecBuilder.fill(0.7/* X */, 0.7/* Y */, 99999999/* Theta */)
        );
        PoseEstimator.addVisionMeasurement(
          MegaTag2.pose,
          MegaTag2.timestampSeconds
        );
      }
    }
    field2d.setRobotPose(getPose());
  }

  /////////
  // Pose
  /////////

  public void resetPose(Pose2d pose) {
    PoseEstimator.resetPosition(getGyroAngle(), getPositions(), pose);
  }

  /////////
  // set states
  /////////

  public void setStates(SwerveModuleState[] states) {
    FL.setState(states[0]);
    FR.setState(states[1]);
    BL.setState(states[2]);
    BR.setState(states[3]);
  }

  /////////
  // Drive Commands
  /////////

  public void AutoDrive(ChassisSpeeds Speeds) {
    Speeds = ChassisSpeeds.discretize(Speeds, 0.02);
    chassisSpeeds = Speeds;
    SwerveModuleState[] TargetStates = kSwerve.kinematics.toSwerveModuleStates(
      Speeds
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(TargetStates, kSwerve.MaxSpeed);
    setStates(TargetStates);
  }

  public void TeleopDrive(
    Translation2d translations,
    double OmegaRadiansPerSecond,
    boolean fieldOrentation
  ) {
    ChassisSpeeds Speeds = fieldOrentation
      ? ChassisSpeeds.fromFieldRelativeSpeeds(
        translations.getX() * kSwerve.MaxSpeed,
        translations.getY() * kSwerve.MaxSpeed,
        OmegaRadiansPerSecond,
        Robot.Gyro.getRotation2d()
      )
      : new ChassisSpeeds(
        translations.getX() * kSwerve.MaxSpeed,
        translations.getY() * kSwerve.MaxSpeed,
        OmegaRadiansPerSecond
      );
    chassisSpeeds = Speeds;

    Speeds = ChassisSpeeds.discretize(Speeds, 0.2);
    SwerveModuleState[] TargetStates = kSwerve.kinematics.toSwerveModuleStates(
      Speeds
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(TargetStates, kSwerve.MaxSpeed);
    setStates(TargetStates);
  }

  /////////
  // Get Methods
  /////////

  public Translation2d getTargetPoint() {
    if (FieldConstants.IsBlueAllience()) {
      return FieldConstants.BLUE_SPEAKER_TRANS;
    }
    return FieldConstants.RED_SPEAKER_TRANS;
  }

  public Translation2d getGoalPosition() {
    if (FieldConstants.IsBlueAllience()) {
      return FieldConstants.BLUE_AMP_TRANS;
    }
    return FieldConstants.RED_AMP_TRANS;
  }

  @Log.NT
  public Rotation2d getGyroAngle() {
    return Robot.Gyro.getRotation2d();
  }

  public double getYawRate() {
    return Robot.Gyro.getRate();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kSwerve.kinematics.toChassisSpeeds(getStates());
  }

  @Log.NT
  public Pose2d getPose() {
    return PoseEstimator.getEstimatedPosition();
  }

  @Log.NT
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      FL.getPosition(),
      FR.getPosition(),
      BL.getPosition(),
      BR.getPosition(),
    };
  }

  @Log.NT
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      FL.getState(),
      FR.getState(),
      BL.getState(),
      BR.getState(),
    };
  }

  @Log.NT
  public SwerveModuleState[] getTargetStates() {
    return new SwerveModuleState[] {
      FL.getTargetState(),
      FR.getTargetState(),
      BL.getTargetState(),
      BR.getTargetState(),
    };
  }

  @Log.NT
  public SwerveModuleState[] getOptimizedStates() {
    return new SwerveModuleState[] {
      FL.getOptimizedState(),
      FR.getOptimizedState(),
      BL.getOptimizedState(),
      BR.getOptimizedState(),
    };
  }

  public boolean getInAmpPosition() {
    if (FieldConstants.IsBlueAllience()) {
      return (
        FieldConstants.BLUE_AMP_TRANS
          .minus(getPose().getTranslation())
          .getNorm() <
        1
      );
    }
    return (
      FieldConstants.RED_AMP_TRANS.minus(getPose().getTranslation()).getNorm() <
      1
    );
  }

  public boolean getInAmpRange() {
    if (FieldConstants.IsBlueAllience()) {
      return (
        FieldConstants.BLUE_AMP_TRANS
          .minus(getPose().getTranslation())
          .getNorm() <
        5
      );
    }
    return (
      FieldConstants.RED_AMP_TRANS.minus(getPose().getTranslation()).getNorm() <
      3
    );
  }

  /////////
  // Interface Methods
  /////////

  public Command JoystickToChassis(
    DoubleSupplier xTrans,
    DoubleSupplier yTrans,
    DoubleSupplier OmegaRadiansPerSec,
    DoubleSupplier AngTrans,
    BooleanSupplier fieldOrentation,
    Swerve mSwerve
  ) {
    return this.run(() -> {
        // Adds the dead area to controller input ex: input = 0.01 output = 0, mitigates drift/small movements
        double xTranslation = MathUtil.applyDeadband(
          xTrans.getAsDouble(),
          kSwerve.kControlConstants.kDeadband
        );
        double yTranslation = MathUtil.applyDeadband(
          yTrans.getAsDouble(),
          kSwerve.kControlConstants.kDeadband
        );
        double OmegaRadiansPerSecond = MathUtil.applyDeadband(
          OmegaRadiansPerSec.getAsDouble(),
          kSwerve.kControlConstants.kDeadband
        );

        // adds curve to controller input ex: 0.5 * 0.5 = 0.25
        xTranslation = Math.copySign(xTranslation * xTranslation, xTranslation);
        yTranslation = Math.copySign(yTranslation * yTranslation, yTranslation);
        OmegaRadiansPerSecond =
          Math.copySign(
            OmegaRadiansPerSecond * OmegaRadiansPerSecond,
            OmegaRadiansPerSecond
          );

        Translation2d translations = new Translation2d(
          xTranslation,
          yTranslation
        );

        // get the rotation going max speed before MODs
        OmegaRadiansPerSecond *= kSwerve.MaxSpeed;

        switch (mSwerveState.mode) {
          case Focus:
            TeleopDrive(
              translations,
              OmegaRadiansPerSecond * RobotConstants.kSwerve.kControls.slowMod,
              fieldOrentation.getAsBoolean()
            );
            break;
          case Normal:
            TeleopDrive(
              translations,
              OmegaRadiansPerSecond,
              fieldOrentation.getAsBoolean()
            );
            break;
        }
      });
  }

  /////////
  // Auto Drive Commands
  /////////

  public Command angleControl(
    DoubleSupplier xTrans,
    DoubleSupplier yTrans,
    DoubleSupplier OmegaRadiansPerSec,
    DoubleSupplier AngTrans
  ) {
    return Commands
      .runOnce(
        () -> THETAcontrol.reset(getGyroAngle().getDegrees(), getYawRate()),
        this
      )
      .andThen(
        Commands.run(
          () -> {
            double xTranslation = MathUtil.applyDeadband(
              xTrans.getAsDouble(),
              kSwerve.kControlConstants.kDeadband
            );
            double yTranslation = MathUtil.applyDeadband(
              yTrans.getAsDouble(),
              kSwerve.kControlConstants.kDeadband
            );

            // adds curve to controller input ex: 0.5 * 0.5 = 0.25
            xTranslation =
              Math.copySign(xTranslation * xTranslation, xTranslation);
            yTranslation =
              Math.copySign(yTranslation * yTranslation, yTranslation);

            Translation2d translations = new Translation2d(
              xTranslation,
              yTranslation
            );

            Translation2d trasnslation = new Translation2d(
              OmegaRadiansPerSec.getAsDouble(),
              AngTrans.getAsDouble()
            );

            if (trasnslation.getNorm() < 0.2) {
              TeleopDrive(translations, 0, true);
              return;
            }

              TeleopDrive(
                translations,
                THETAcontrol.calculate(
                  getGyroAngle().getRadians(),
                  trasnslation.getAngle().getRadians()
                ),
                true
              );
            
          },
          this
        )
      );
  }

  public Command trackTarget(
    DoubleSupplier xTrans,
    DoubleSupplier yTrans,
    Translation2d goal
  ) {
    return Commands
      .runOnce(
        () -> THETAcontrol.reset(getPose().getRotation().getDegrees(), getYawRate()),
        this
      )
      .andThen(
        Commands.run(
          () -> {
            double xTranslation = MathUtil.applyDeadband(
              xTrans.getAsDouble(),
              kSwerve.kControlConstants.kDeadband
            );
            double yTranslation = MathUtil.applyDeadband(
              yTrans.getAsDouble(),
              kSwerve.kControlConstants.kDeadband
            );

            // adds curve to controller input ex: 0.5 * 0.5 = 0.25
            xTranslation =
              Math.copySign(xTranslation * xTranslation, xTranslation);
            yTranslation =
              Math.copySign(yTranslation * yTranslation, yTranslation);

            Translation2d translations = new Translation2d(
              xTranslation,
              yTranslation
            );

            TeleopDrive(
              translations,
              THETAcontrol.calculate(
                getPose().getRotation().getRadians(),
                getPose()
                  .getTranslation()
                  .minus(goal)
                  .getAngle()
                  .plus(Rotation2d.fromDegrees(180))
                  .getRadians()
              ),
              true
            );
          },
          this
        )
      );
  }

  public Command drivetoPoint(Translation2d goal, Rotation2d RobotRotation) {
    return Commands
      .runOnce(
        () -> {
          ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            getChassisSpeeds(),
            getGyroAngle()
          );
          Xcontrol.reset(getPose().getX(), speeds.vxMetersPerSecond);
          Ycontrol.reset(getPose().getY(), speeds.vyMetersPerSecond);
          THETAcontrol.reset(
            getPose().getRotation().getRadians(),
            speeds.omegaRadiansPerSecond
          );
        },
        this
      )
      .andThen(
        Commands.run(
          () -> {
            // Calculate full chassis Speeds
            ChassisSpeeds ChassisSpeeds = new ChassisSpeeds(
              Xcontrol.calculate(
                getPose().getX(),
                goal.minus(kSwerve.kMessurements.ChassisOffset).getX()
              ),
              Ycontrol.calculate(
                getPose().getY(),
                goal.minus(kSwerve.kMessurements.ChassisOffset).getY()
              ),
              THETAcontrol.calculate(
                getPose().getRotation().getRadians(),
                RobotRotation.getRadians()
              )
            );

            this.log("xControllerSetpoint", Xcontrol.getSetpoint().position);
            this.log("yControllerSetpoint", Ycontrol.getSetpoint().position);
            this.log("rotController", THETAcontrol.getSetpoint().position);
            this.log("xSetVel", Xcontrol.getSetpoint().velocity);
            this.log("ySetVel", Ycontrol.getSetpoint().velocity);
            this.log("rotSetVel", THETAcontrol.getSetpoint().velocity);
            this.log("x at goal", Xcontrol.atGoal());
            this.log("y at goal", Ycontrol.atGoal());
            this.log("z at goal", THETAcontrol.atGoal());
            TeleopDrive(
              new Translation2d(
                ChassisSpeeds.vxMetersPerSecond,
                ChassisSpeeds.vyMetersPerSecond
              ),
              ChassisSpeeds.omegaRadiansPerSecond,
              true
            );
          },
          this
        )
      )
      .until(() ->
        Xcontrol.atGoal() && Ycontrol.atGoal() && THETAcontrol.atGoal()
      );
  }
}
