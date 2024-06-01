# 6423-drivebase-template
#### Iron Patriot's drivebase template

### Structure
 - [Main.java](src/main/java/frc/robot/Main.java)
 - [Robot.java](src/main/java/frc/robot/Robot.java)
 - [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java)
 - [Constants.java](src/main/java/frc/robot/Constants.java)
    - [ControllerConstants](src/main/java/frc/robot/Constants.java)
    - [DriveConstants](src/main/java/frc/robot/Constants.java)
 - [subsystems](src/main/java/frc/robot/subsystems/)
    - [LEDSubsystem.java](src/main/java/frc/robot//subsystems/LEDSubsystem.java)
    - [Drive](src/main/java/frc/robot/subsystems/Drive/)
        - [Drive.java](src/main/java/frc/robot/subsystems/Drive/Drive.java)
        - [Gyro](src/main/java/frc/robot/subsystems/Drive/Gyro)
            - [GyroIO.java](src/main/java/frc/robot/subsystems/Drive/Gyro/GyroIO.java)
            - [GyroIOSim.java](src/main/java/frc/robot/subsystems/Drive/Gyro/GyroIOSim.java)
            - [NavxIO.java](src/main/java/frc/robot/subsystems/Drive/Gyro/NavxIO.java)
        - [Module](src/main/java/frc/robot/subsystems/Drive/Module/)
            - [ModuleIO.java](src/main/java/frc/robot/subsystems/Drive/Module/ModuleIO.java)
            - [ModuleIOSim.java](src/main/java/frc/robot/subsystems/Drive/Module/ModuleIOSim.java)
            - [NeoCoaxialModule.java](src/main/java/frc/robot/subsystems/Drive/Module/NeoCoaxialModule.java)
        

### Included vendor deps
 - Advantage Kit 3.1.1
 - NavX 2024.1.0
 - Path Planner Lib 2024.1.3
 - REVLib 2024.2.0
 - WPILib New Commands 1.0.0