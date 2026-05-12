// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // driving declarations
  WPI_TalonSRX left1 = new WPI_TalonSRX(4);
  WPI_VictorSPX left2 = new WPI_VictorSPX(6);
  WPI_TalonSRX right1 = new WPI_TalonSRX(16);
  WPI_VictorSPX right2 = new WPI_VictorSPX(2);
  SlewRateLimiter speedfilter = new SlewRateLimiter(2.5);
  SlewRateLimiter rotationfilter = new SlewRateLimiter(2.5);

  DifferentialDrive drivetrain = new DifferentialDrive(left1, right1);

  // Controller Scheme Declarations
  XboxController xboxController = new XboxController(0);
  Joystick exampleJoystick = new Joystick(1);

  boolean useFlightstick = false;

  // Turn Signal Declarations
  WPI_VictorSPX leftRSL = new WPI_VictorSPX(10);  // Use your specific CAN IDs
  WPI_VictorSPX rightRSL = new WPI_VictorSPX(11);

  boolean leftRSLActive = false;
  boolean rightRSLActive = false;


  int blinkCounter = 0;
  boolean blinkState = false;
  int brakeTimer = 0;

  WPI_VictorSPX horn = new WPI_VictorSPX(1);

  private final RobotContainer m_robotContainer;

  // Inside your Robot or Subsystem class
AddressableLED m_led = new AddressableLED(0); // PWM Port 0
AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(80); // 15 LEDs long

public void ledInit() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
}

public void setStaticColor(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
}

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_robotContainer = new RobotContainer();

    // 1. Set Master Inversions
    left1.setInverted(false); 
    right1.setInverted(true);

    // 2. Set Followers and force them to match the Master's direction
    left2.follow(left1);
    right2.follow(right1);
    
    // This tells the Victors to ignore their own inversion settings
    // and just do whatever the Talon SRX is doing.
    left2.setInverted(com.ctre.phoenix.motorcontrol.InvertType.FollowMaster);
    right2.setInverted(com.ctre.phoenix.motorcontrol.InvertType.FollowMaster);

    // 3. Re-enable the safety configurations you had
    // // left1.configContinuousCurrentLimit(40);
    // // left1.current
    // // right1.configContinuousCurrentLimit(40);
    // left1.enableCurrentLimit(true);
    // right1.enableCurrentLimit(true);
    left1.configOpenloopRamp(0.5);
    right1.configOpenloopRamp(0.5);

    // ADD THESE LINES TO ACTIVATE THE LEDS
    ledInit(); 
    setStaticColor(0, 255, 0); // Sets them to Green for testing
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   * 
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double speed;
    double rotation;

    // 1. Controller Selection
    if (exampleJoystick.isConnected()) {
        speed = -exampleJoystick.getY(); 
        rotation = -exampleJoystick.getZ(); 
    } else {
      double forward = xboxController.getRightTriggerAxis();
      double reverse = xboxController.getLeftTriggerAxis();
      
      speed = forward - reverse;
        rotation = -xboxController.getLeftX();
    }

    // 2. Drive Drivetrain
    double filteredSpeed = speedfilter.calculate(speed);
    double filteredRotation = rotationfilter.calculate(rotation);
    drivetrain.arcadeDrive(filteredSpeed, filteredRotation);  

    // 3. Handle Toggle Buttons (Bumpers)
    if (xboxController.getBackButtonPressed()) leftRSLActive = !leftRSLActive;
    if (xboxController.getStartButtonPressed()) rightRSLActive = !rightRSLActive;

    // 4. Blink Logic (Runs 50 times a second)
    // Using a class-level counter (add 'int blinkCounter = 0;' to your class variables)
    blinkCounter++;
    boolean blinkOn = (blinkCounter % 40 < 10); // Simple 200ms blink cycle

    // 5. Brake Detection
    boolean isCurrentlyBraking = false;
    if (Math.abs(speed) < 0.05 && Math.abs(filteredSpeed) > 0.1) {
        isCurrentlyBraking = true;
    } else if ((speed > 0 && filteredSpeed < -0.1) || (speed < 0 && filteredSpeed > 0.1)) {
        isCurrentlyBraking = true;
    }

    if (isCurrentlyBraking) {
      brakeTimer = 25; // Set/Reset timer to 25 loops (0.5 seconds)
  } else if (brakeTimer > 0) {
      brakeTimer--; // Countdown if we aren't actively braking anymore
  }

    // 6. FINAL LIGHT OUTPUT (The Priority List)
    double leftOutput = 0.0;
    double rightOutput = 0.0;

    boolean showBrakeLight = (brakeTimer > 0);

    if (showBrakeLight) {
        leftOutput = 1.0;
        rightOutput = 1.0;
    } else {
        // TURN SIGNALS are Priority #2: Blinking
        leftOutput = (leftRSLActive && blinkOn) ? 1.0 : 0.0;
        rightOutput = (rightRSLActive && blinkOn) ? 1.0 : 0.0;
    }

    // 7. Send to Victors (Only one set of .set() calls!)
    leftRSL.set(leftOutput);
    rightRSL.set(rightOutput);

    if (xboxController.getAButton())
    {
      horn.set(0.5);
    }
    else if (xboxController.getYButton() && xboxController.getBackButton() && xboxController.getBButton())
    {
      horn.set(1.0);
    }
    else
    {
      horn.set(0);
    }

    // 2. Reverse Beeper Logic
    // If the robot is moving backwards (filteredSpeed is negative)
    if (filteredSpeed < -0.1 && xboxController.getLeftTriggerAxis() > 0.5) {
        // Pulse the horn at the same rate as the blinker
        horn.set(blinkOn ? 0.45 : 0.0);
    }

  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
