# FRC Team 6901 - 2026 Rebuilt

Welcome to the **2026 Rebuilt** repository for FIRST Robotics Competition (FRC) Team 6901! This repository contains the complete source code and documentation for our competition robot built for the 2026 season.

## About FRC Team 6901

Team 6901 is a dedicated robotics team committed to excellence in engineering, innovation, and teamwork. We design, build, and program competitive robots to compete in the FRC competition. Our team consists of passionate students who work together to solve complex engineering challenges and push the boundaries of what's possible in robotics.

This year's robot, **2026 Rebuilt**, features an advanced swerve drivetrain, precision shooting mechanisms, and automated game piece handling systems designed to dominate the competition.

## Quick Installation

### Prerequisites

- **Java Development Kit (JDK) 17** or later
- **Gradle** (included via wrapper)
- **VS Code** with WPILib extensions
- **Git**

### Setup Instructions

1. **Clone the Repository**
   ```bash
   git clone https://github.com/frc-6901/2026-Rebuilt.git
   cd 2026-Rebuilt
   ```

2. **Build the Project**
   ```bash
   ./gradlew build
   ```

3. **Deploy to RoboRIO**
   ```bash
   ./gradlew deploy
   ```

4. **Run Simulations**
   ```bash
   ./gradlew simulateJava
   ```

For detailed setup instructions, refer to the [WPILib Documentation](https://docs.wpilib.org/).

## Controller Keybinds

### Driver Controller (Xbox Controller - Port 0)

| Button/Stick | Action |
|---|---|
| **Left Stick** | Field-centric drive movement (Y-axis: forward/back, X-axis: strafe) |
| **Right Stick** | Rotation control (X-axis: rotate) |
| **A Button** | Toggle intake on/off |
| **X Button** | Outtake (expel game pieces) |
| **Y Button** | Reset vision pose |
| **B Button** | Brake (hold wheels in place) |
| **Left Bumper** | Point wheels toward hub |
| **Right Bumper** | Reset field-centric heading |
| **POV Up** | Retract slapdown |
| **POV Down** | Deploy slapdown |
| **POV Left** | Fine-tune slapdown power (forward) |
| **POV Right** | Fine-tune slapdown power (reverse) |
| **Left Stick (Press)** | Reset slapdown position |

### Operator Controller (Xbox Controller - Port 1)

| Button/Stick | Action |
|---|---|
| **Left Trigger** | Auto-aim and shoot |
| **Right Trigger** | Preset shoot (speed controlled by right stick Y-axis) |
| **Left Bumper** | Toggle intake on/off |
| **POV Up** | Prime shooter (5-second spin-up) |
| **POV Down** | Stop all subsystems |
| **POV Left** | Index game pieces forward |
| **POV Right** | Index game pieces reverse |

## Code Overview

### Architecture

The codebase follows the WPILib Command-Based Robot framework, organizing functionality into **Commands** (actions) and **Subsystems** (robot mechanisms).

### Subsystems

Subsystems represent the physical mechanisms of the robot and manage their hardware interactions:

- **CommandSwerveDrivetrain** - Advanced swerve drivetrain providing omnidirectional movement and precise rotation control
- **VisionSubsystem** - Handles camera inputs and vision processing for auto-aiming and field localization
- **ShooterSubsystem** - Controls the shooter mechanism with variable RPM control for different shot types
- **IntakeSubsystem** - Manages intake motor for collecting and expelling game pieces
- **IndexerSubsystem** - Controls the indexer mechanism for routing game pieces
- **SlapdownSubsystem** - Handles pneumatic slapdown mechanism for game piece manipulation
- **KickerSubsystem** - Controls the kicker mechanism for final game piece delivery

### Commands

Commands represent robot actions and are triggered by controller inputs or autonomous routines:

#### Core Commands

- **AutoAimShootCommand** - Uses vision data to automatically aim and shoot at the target
- **PresetShootCommand** - Shoots at a preset RPM specified by the operator
- **PrimeShooterCommand** - Spins up the shooter to prepare for a shot
- **IntakeCommand** - Activates the intake to draw game pieces into the robot
- **OuttakeCommand** - Reverses the intake to expel game pieces
- **ToggleIntakeCommand** - Toggles the intake between running and stopped states
- **ToggleSlapdownCommand** - Toggles the slapdown mechanism between deployed and retracted
- **RotateToHubCommand** - Rotates the robot to face the hub for precise shooting
- **StopSubsystemsCommand** - Stops all subsystems safely

### Default Commands

Default commands run continuously when no other commands override them:

- **Drivetrain**: Applies driver controller input for field-centric driving
- **Shooter**: Idle (stopped)
- **Intake**: Idle (stopped)
- **Indexer**: Idle (stopped)
- **Kicker**: Idle (stopped)

### Autonomous Routines

Autonomous commands are managed by PathPlanner and can be selected from the SmartDashboard during the match. Available named commands include:

- `intake` - Run intake
- `stopIntake` - Stop intake
- `shoot20RPS` - Shoot at 20 RPS
- `autoAimShoot` - Auto-aim and shoot
- `rotateToHub` - Rotate to face hub
- `slapdownTrigger` - Toggle slapdown
- `stopSubsystems` - Stop all subsystems

## Project Structure

```
src/main/
├── java/frc/robot/
│   ├── Main.java                 # Entry point
│   ├── Robot.java                # Main robot class
│   ├── RobotContainer.java       # Subsystem and command setup
│   ├── Constants.java            # Robot configuration constants
│   ├── Telemetry.java            # Telemetry logging
│   ├── commands/                 # Command implementations
│   └── subsystems/               # Subsystem implementations
└── deploy/                       # Deployed files (pathplanner configs, etc.)
```

## Building and Deploying

### Local Build
```bash
./gradlew build
```

### Deploy to RoboRIO
```bash
./gradlew deploy
```

### Simulation Mode
```bash
./gradlew simulateJava
```

## Dependencies

- **WPILib** - FRC robot framework
- **Phoenix 6** - CTRE motor controllers
- **REVLib** - REV robotics libraries
- **PathPlannerLib** - Autonomous path planning
- **PhotonLib** - Vision processing

See `vendordeps/` for detailed dependency information.

## Contributing

When contributing to this codebase:

1. Create a feature branch from `main`
2. Follow WPILib coding standards and Java conventions
3. Add appropriate JavaDoc comments to new public methods
4. Test your changes in simulation or on the test robot
5. Submit a pull request for review

## License

This code is provided under the same license as WPILib. See `WPILib-License.md` for details.

## Contact

For questions or more information about FRC Team 6901, please reach out to the team leadership or visit our website.

---

**Go Team 6901! 🤖**
