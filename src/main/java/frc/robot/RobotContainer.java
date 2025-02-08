package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

public class RobotContainer {
    // Joysticks for driver control
    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private boolean isHalfSpeed = false; // Toggle for speed reduction

    // Subsystems for drive and scoring
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/8387"));
    private final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();

    // Drive command using joystick input
    
    /* This is old drive command code that doesn't seem to work with the new swerve drive code. I am not sure why.
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(
        drivebase,
        () -> -MathUtil.applyDeadband(leftStick.getY() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(leftStick.getX() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(rightStick.getX() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.RIGHT_X_DEADBAND),
        leftStick::getTrigger,
        () -> false,
        () -> false,
        () -> false
    );

    // Field-oriented drive command
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(leftStick.getY() * (isHalfSpeed ? 0.5 : 1.0) / 3.0, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(leftStick.getX() * (isHalfSpeed ? 0.5 : 1.0) / 3.0, OperatorConstants.LEFT_X_DEADBAND),
        () -> rightStick.getX(),
        () -> rightStick.getY()
    );

    // Angular velocity drive mode
    Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(leftStick.getY() * (isHalfSpeed ? -0.5 : -1.0), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(leftStick.getX() * (isHalfSpeed ? -0.5 : -1.0), OperatorConstants.LEFT_X_DEADBAND),
        () -> rightStick.getX() * (isHalfSpeed ? -0.5 : -1.0)
    );

    // Simulation drive command
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(leftStick.getY() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(leftStick.getX() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.LEFT_X_DEADBAND),
        () -> rightStick.getRawAxis(2)
    );
    */

      /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> -MathUtil.applyDeadband(leftStick.getY() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.LEFT_Y_DEADBAND),
                                                                    () -> -MathUtil.applyDeadband(leftStick.getX() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.LEFT_X_DEADBAND))
                                                            .withControllerRotationAxis(() -> rightStick.getX() * (isHalfSpeed ? 0.5 : 1.0))
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

/**
 * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
 */
SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> rightStick.getX() * (isHalfSpeed ? 0.5 : 1.0),
                                                                                                                () -> rightStick.getY() * (isHalfSpeed ? 0.5 : 1.0))
                                                                                                                .headingWhile(true);

/**
 * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
 */
SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                .allianceRelativeControl(false);

SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                                () -> -MathUtil.applyDeadband(leftStick.getY() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.LEFT_Y_DEADBAND),
                                                                                () -> -MathUtil.applyDeadband(leftStick.getX() * (isHalfSpeed ? 0.5 : 1.0), OperatorConstants.LEFT_X_DEADBAND))
                                                                        .withControllerRotationAxis(() -> rightStick.getRawAxis(2) * (isHalfSpeed ? 0.5 : 1.0))
                                                                        .deadband(OperatorConstants.DEADBAND)
                                                                        .scaleTranslation(0.8)
                                                                        .allianceRelativeControl(true);
// Derive the heading axis with math!
SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                        .withControllerHeadingAxis(() -> Math.sin(rightStick.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                                                                                            () -> Math.cos(rightStick.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                                                                        .headingWhile(true);

    public RobotContainer() {
        configureBindings();
    }

    // Configures joystick button bindings
    private void configureBindings() {
        if (!DriverStation.isTest()) {
            // Toggle half-speed mode when left stick trigger is pressed
            new Trigger(leftStick::getTrigger).onTrue(Commands.runOnce(() -> isHalfSpeed = !isHalfSpeed));
            
            // Right stick trigger to launch scoring mechanism
            new Trigger(rightStick::getTrigger).whileTrue(Commands.runOnce(() -> scoringSubsystem.launch()));
            
            // Elevator movement controls - current moveTo parameters are placeholders
            new Trigger(() -> rightStick.getRawButton(2)).whileTrue(Commands.runOnce(() -> scoringSubsystem.pull()));
            new Trigger(() -> rightStick.getRawButton(3)).whileTrue(Commands.runOnce(() -> scoringSubsystem.moveTo(0)));
            new Trigger(() -> rightStick.getRawButton(4)).whileTrue(Commands.runOnce(() -> scoringSubsystem.moveTo(25)));
            new Trigger(() -> rightStick.getRawButton(5)).whileTrue(Commands.runOnce(() -> scoringSubsystem.moveTo(50)));
            new Trigger(() -> rightStick.getRawButton(6)).whileTrue(Commands.runOnce(() -> scoringSubsystem.moveTo(100)));
        }
    }

    // Returns the autonomous command
    public Command getAutonomousCommand() {
        return drivebase.getAutonomousCommand("New Auto");
    }

    // Reconfigures joystick bindings
    public void setDriveMode() {
        configureBindings();
    }

    // Sets the motor brake mode
    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}