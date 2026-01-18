# 🤖 2026 Robot Code - Ri3D Big Sky Robotics 🤖
Code for Big Sky Robotics Team from Montana State University ⛰️
## Features
### [State Machines](https://github.com/Montana-State-Ri3D/2026-Ri3D/tree/master/src/main/java/frc/robot/stateMachines)
#### [SuperStructure](https://github.com/Montana-State-Ri3D/2026-Ri3D/blob/master/src/main/java/frc/robot/subsystems/SuperStructure.java)
Finite State Machine for positions and velocities of all mechanisms. Numbers are tunable for quick testing.
#### [SuperStateMachine](https://github.com/Montana-State-Ri3D/2026-Ri3D/blob/master/src/main/java/frc/robot/stateMachines/SuperStateMachine.java)
Finite State Machine for robot game action (intaking, scoring, climbing, ect.). Calls action state machines, such as [ScoreStateMachine](https://github.com/Montana-State-Ri3D/2026-Ri3D/blob/master/src/main/java/frc/robot/stateMachines/ScoreStateMachine.java), or directly sets SuperStructure states
#### Action State Machines
Sequential state machines to handle compound actions cleanly while logging current state. Uses 2930 State Machine library. Includes [ScoreStateMachine](https://github.com/Montana-State-Ri3D/2026-Ri3D/blob/master/src/main/java/frc/robot/stateMachines/ScoreStateMachine.java) and [IntakeStateMachine](https://github.com/Montana-State-Ri3D/2026-Ri3D/blob/master/src/main/java/frc/robot/stateMachines/IntakeStateMachine.java).
Example: When scoring, robot prepares by spinning up RPM and rotating to the hub, then when those conditions are met, indexes gamepieces to the flywheel.
### [Drivetrain](https://github.com/Montana-State-Ri3D/2026-Ri3D/tree/master/src/main/java/frc/robot/subsystems/drive)
- State-based control (controller, path follow, drive to pose, rotate to angle)
- choreo path following
- Mecanum drive
### [Autonomous](https://github.com/Montana-State-Ri3D/2026-Ri3D/tree/master/src/main/java/frc/robot/autonomous)
- [Auto Manager](https://github.com/Montana-State-Ri3D/2026-Ri3D/blob/master/src/main/java/frc/robot/autonomous/AutoManager.java) to create sendable chooser for paths
- [AutoStateMachine](https://github.com/Montana-State-Ri3D/2026-Ri3D/blob/master/src/main/java/frc/robot/autonomous/AutoStateMachine.java) to handle all auto logic
## Development
- Become a member of the MSU Ri3D github organization
- Clone Repo in WPILib VSCode: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
- Assign yourself to an issue: https://github.com/Montana-State-Ri3D/2026-Ri3D/issues
- Create a branch for your issue
- Link the branch in your issue
- Develop code
- Create pull request for your branch: https://github.com/Montana-State-Ri3D/2026-Ri3D/pulls
- Get another developer to review your code
- The other developer will merge into main when they approve your code 🔥
## Credits
6328 Mechanical Advantage 🟦🟨
- Vision template: https://docs.advantagekit.org/getting-started/template-projects/vision-template/
- Utils: https://github.com/Mechanical-Advantage/RobotCode2025Public/tree/main/src/main/java/org/littletonrobotics/frc2025/util

2930 Sonic Squirrels
- Utils: https://github.com/FRC-Sonic-Squirrels/2025-Robot-Code/tree/main/src/main/java/frc/lib/team2930

WPILib
- Foundational libraries
