// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;

public class RobotContainer {



  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My joystick
  private final CommandXboxController operaController = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.Swerve; // My drivetrain

    private DoubleSupplier speed = () -> TunerConstants.kSpeedAt12VoltsMps  - driverController.getHID().getLeftTriggerAxis() * 0.8;
  private double MaxAngularRate = 3 * Math.PI;//1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


  private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
  .withDeadband(MaxAngularRate * 0.1).withRotationalDeadband(MaxAngularRate * 0.1).
  withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
  .withDeadband(MaxAngularRate * 0.1).withRotationalDeadband(MaxAngularRate * 0.1).
  withDriveRequestType(DriveRequestType.OpenLoopVoltage);


                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = Misc.m_Telemetry;

  private void configureBindings() {
    // swerve command
    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(driverController.getRightX() * speed.getAsDouble()) //fieldCentric
        .withVelocityY(driverController.getLeftY() * speed.getAsDouble())
        .withRotationalRate(driverController.getRightY() * MaxAngularRate)));
    //TODO: add a changer for the drive type with a button between driveFieldCentric and driveRobotCentric and driveFacingAngle
      

    driverController.leftBumper().whileTrue(drivetrain.applyRequest(() ->
      brake));


      // reset the field-centric heading on left bumper press
      driverController.y().onTrue(drivetrain.runOnce(() ->
      drivetrain.seedFieldRelative()));
    drivetrain.registerTelemetry(logger::telemeterize);
    



  }

  public RobotContainer() {
    configureBindings();


		// driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		// driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		// driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		// driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }
}
