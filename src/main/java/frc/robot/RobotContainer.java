// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TurretControl;
import frc.robot.commands.VisionFollow;
import frc.robot.commands.Brake;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.ElevatorControl;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Using "import static an.enum.or.constants.inner.class.*;" helps reduce verbosity
// this replaces "DoubleSolenoid.Value.kForward" with just kForward
// further reading is available at https://www.geeksforgeeks.org/static-import-java/
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // ************  OI Controller  ***************
  private static final Joystick driverStick = new Joystick(Constants.DRIVER_STICK_PORT);
  private static final Joystick opStick = new Joystick(Constants.OP_STICK_PORT);

  // ************  Subsystems  **************
  private DriveTrain m_driveTrain = new DriveTrain();
  // private Elevator m_elevator = new Elevator();
  //private Turret m_turret = new Turret();

  //private final ColorSensor m_colorSensor = new ColorSensor();
  //private final Limelight m_limelight = new Limelight();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // ************   Commands    ***************
  //public final VisionFollow m_visionFollow = new VisionFollow(m_limelight, m_driveTrain);
  //public final Brake m_brake = new Brake(m_driveTrain);
  public final Pneumatics m_pneumatics = new Pneumatics();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveTrain.setDefaultCommand(new DriveArcade(
          () -> -driverStick.getY(),
          () -> driverStick.getZ(),
          m_driveTrain));

    //m_turret.setDefaultCommand(new TurretControl(m_turret, () -> opStick.getTwist()));
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // DRIVER STICK BUTTONS
      //final JoystickButton lockonButton = new JoystickButton(driverStick, 2);
      //  lockonButton.whenHeld(m_visionFollow);
      //final JoystickButton brakeButton = new JoystickButton(driverStick, Constants.DRIVER_BRAKE_BUTTON);
        //brakeButton.whenHeld(m_brake);
  

    // OPERATOR STICK BUTTONS
      // final JoystickButton elevatorDownButton = new JoystickButton(opStick, Constants.OP_ELEVATOR_UP_BUTTON);
      //   elevatorDownButton.whenPressed(new ElevatorControl(m_elevator90, Constants.OP_ELEVATOR_UP_BUTTON));
      // final JoystickButton elevatorUpButton = new JoystickButton(opStick, Constants.OP_ELEVATOR_DOWN_BUTTON);
      //   elevatorUpButton.whenPressed(new ElevatorControl(m_elevator, Constants.OP_ELEVATOR_DOWN_BUTTON));    }
      //FIXME: Make constant
      // FIXME: Must make this a command
      final JoystickButton collectorUpButton = new JoystickButton(opStick, 4);
        collectorUpButton.whenPressed(() -> m_pneumatics.exampleDoublePH.set(kForward));
      final JoystickButton collectorDownButton = new JoystickButton(opStick, 5);
        collectorDownButton.whenPressed(() -> m_pneumatics.exampleDoublePH.set(kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
