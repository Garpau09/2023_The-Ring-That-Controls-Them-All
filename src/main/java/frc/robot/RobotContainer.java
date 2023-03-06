package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClawConstants.ClawPoses;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final XboxController m_Operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton driver_Back = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton driver_leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton kRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final JoystickButton kB = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton kA = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton kStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    // private final JoystickButton kX = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton kY = new JoystickButton(driver, XboxController.Button.kY.value);

    /* operator Buttons */
    private final JoystickButton operator_X = new JoystickButton(m_Operator, XboxController.Button.kX.value);
    private final JoystickButton operator_A = new JoystickButton(m_Operator, XboxController.Button.kA.value);
    private final JoystickButton operator_B = new JoystickButton(m_Operator, XboxController.Button.kB.value);
    private final JoystickButton operator_Y = new JoystickButton(m_Operator, XboxController.Button.kY.value);
    //private final JoystickButton operator_rightBumper = new JoystickButton(m_Operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton operator_leftBumper = new JoystickButton(m_Operator, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve mSwerve = new Swerve();
    private final ClawSubsystem mClawSubsystem = new ClawSubsystem();
    private final GripperSubsystem mGripperSubsystem = new GripperSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {


        mSwerve.setDefaultCommand(
                new TeleopSwerve(
                        mSwerve,
                        () -> Math.pow(-driver.getRawAxis(translationAxis), 3),
                        () -> Math.pow(-driver.getRawAxis(strafeAxis), 3),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> driver_leftBumper.getAsBoolean()));

        mGripperSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> mGripperSubsystem
                                .driveGripper(m_Operator.getRightTriggerAxis() - m_Operator.getLeftTriggerAxis()),
                        mGripperSubsystem));

        // Configure the button bindings

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        /* Driver Buttons */
        driver_Back.onTrue(new InstantCommand(() -> mSwerve.zeroGyro()));
        
        operator_A.onTrue(new ClawCommand(mClawSubsystem, ClawPoses.LOW_SCORE));
        operator_Y.onTrue(new ClawCommand(mClawSubsystem, ClawPoses.HIGH_SCORE));
        operator_X.onTrue(new ClawCommand(mClawSubsystem, ClawPoses.LOADING));
        operator_B.onTrue(new ClawCommand(mClawSubsystem, ClawPoses.MID_SCORE));
        operator_leftBumper.onTrue(new ClawCommand(mClawSubsystem, ClawPoses.TRANSPORT));
    }

    /**
     * Use this to pass the autonomous c
     * command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return  null;
    }
}
