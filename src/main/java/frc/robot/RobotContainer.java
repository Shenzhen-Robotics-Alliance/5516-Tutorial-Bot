// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private static final double percentDeadband = 0.03;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * percentDeadband).withRotationalDeadband(MaxAngularRate * percentDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();

    public RobotContainer() {
        configureBindings();

        drivetrain.configureAutoBuilder();
        NamedCommands.registerCommand("runIntakeForever", intake.runIntake().alongWith(shooter.runReverse()).asProxy());
        NamedCommands.registerCommand("intakeFinish", intake.runInverseALittle().deadlineFor(shooter.runReverse()).asProxy());
        NamedCommands.registerCommand("runShooterForever", shooter.runForward().asProxy());
        NamedCommands.registerCommand("shoot", intake.runIntake().alongWith(shooter.runForward()).withTimeout(0.5).asProxy());
        NamedCommands.registerCommand("shootSequence", shooter.runForward().withTimeout(0.8).andThen(
            intake.runIntake().alongWith(shooter.runForward()).withTimeout(0.5)
        ).asProxy());

        autoCommand = AutoBuilder.buildAuto("four notes");
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.6) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        joystick.leftTrigger(0.5)
            .whileTrue(intake.runIntake().alongWith(shooter.runReverse()))
            .onFalse(intake.runInverseALittle().deadlineFor(shooter.runReverse()));

        joystick.rightBumper()
            .whileTrue(shooter.runForward());

        joystick.rightTrigger(0.5).and(joystick.rightBumper())
            .whileTrue(intake.runIntake());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Command autoCommand = Commands.none();
    public Command getAutonomousCommand() {
        return autoCommand;
    }
}
