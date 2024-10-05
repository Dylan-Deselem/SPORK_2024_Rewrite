// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Interface.java.Interface;
import frc.robot.Commands.Swerve.SwerveShots;
import frc.robot.Subsystems.AmpBar;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Swerve.SwerveState;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged {

  private Command m_autonomousCommand;

  public static AHRS Gyro = new AHRS(Port.kMXP);

  public Swerve mSwerve = new Swerve();
  public Intake mIntake = new Intake();
  public Shooter mShooter = new Shooter();
  public AmpBar mAmpBar = new AmpBar();
  public static SendableChooser<Command> autoChooser;

  public Interface mInterface = new Interface(mShooter, mIntake, mAmpBar);
  public SwerveShots mSwerveShots = new SwerveShots(mSwerve, mInterface);

  public CommandXboxController Driver = new CommandXboxController(0);
  public CommandXboxController Operator = new CommandXboxController(1);

  @Override
  public void robotInit() {
    Monologue.setupMonologue(this, "Robot", false, false);

        mSwerve.setDefaultCommand(
      mSwerve.JoystickToChassis(
        () -> Driver.getLeftX(),
        () -> -Driver.getLeftY(),
        () -> Driver.getRightX(),
        () -> -Driver.getRightY(),
        () -> !Driver.start().getAsBoolean(),
        mSwerve
      )
    );

    ConfigureButtonBindings();
    registerNameCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autos", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public void ConfigureButtonBindings() {

    Driver
      .a()
      .toggleOnTrue(
        Commands.runOnce(() -> mSwerve.mSwerveState.set(SwerveState.Mode.Focus))
      )
      .toggleOnFalse(
        Commands.runOnce(() -> mSwerve.mSwerveState.set(SwerveState.Mode.Normal)
        )
      );

    Driver
      .y()
      .whileTrue(
        mSwerve.angleControl(
          () -> Driver.getLeftX(),
          () -> -Driver.getLeftY(),
          () -> Driver.getRightX(),
          () -> -Driver.getRightY()
        )
      )
      .toggleOnFalse(
        Commands.runOnce(() -> mSwerve.mSwerveState.set(SwerveState.Mode.Normal)
        )
      );

      Driver
      .x()
      .whileTrue(
        mSwerveShots.AutoAmp().unless(() -> !mSwerve.getInAmpRange())
      )
      .toggleOnFalse(
        mInterface.returnToDefault()
      );

      Driver
      .rightBumper()
      .whileTrue(
        mSwerveShots.TrackSpeaker(
          () -> Driver.getLeftX(),
          () -> -Driver.getLeftY())
      );
  }

  public void registerNameCommands() {}
}
