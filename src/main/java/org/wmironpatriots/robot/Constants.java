// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wmironpatriots.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

// * constants should be imported like this
// ? import static frc.robot.Constants.SubsytemConstants.*;
// ! some of these values will need to be tuned again
public final class Constants {

    public static final class ControllerConstants {
        public static final double DRIVER_DEADBAND= 0.025;
        public static final double OPERATOR_DEADBAND = 0.025;
        
        public static final double DRIVER_TRIGGER_DEADBAND = 0.5;
        public static final double OPERATOR_TRIGGER_DEADBAND = 0.5;
    }

    public static final class DriveConstants {
        /** volts per meter per second */
        public static final double kS = 0.1849;
        public static final double kV = 2.5108;
        public static final double kA = 0.24017;

        public static final double MODULEDRIVE_P = 0.1;
        public static final double MODULEDRIVE_I = 0;
        public static final double MODULEDRIVE_D = 0;
        public static final double MODULEPIVOT_P = 0.5;
        public static final double MODULEPIVOT_I = 0.01;
        public static final double MODULEPIVOT_D = 0.09;

        // * Module Constants
        public static final double FRONTLEFT_ABS_ENCODER_OFFSET = 317.;
        public static final double FRONTRIGHT_ABS_ENCODER_OFFSET = 246;
        public static final double BACKLEFT_ABS_ENCODER_OFFSET = 236;
        public static final double BACKRIGHT_ABS_ENCODER_OFFSET = 275;

        public static final Rotation2d[] ABS_ENCODER_OFFSETS = {
            Rotation2d.fromDegrees(FRONTLEFT_ABS_ENCODER_OFFSET),
            Rotation2d.fromDegrees(FRONTRIGHT_ABS_ENCODER_OFFSET),
            Rotation2d.fromDegrees(BACKLEFT_ABS_ENCODER_OFFSET),
            Rotation2d.fromDegrees(BACKRIGHT_ABS_ENCODER_OFFSET)
        };

        // ! What the fuck
        public static final double MODULE_DIST_METERS = Units.inchesToMeters(16.6);
        public static final Measure<Angle> TOLERANCE = Degrees.of(3);
        public static final Translation2d m_frontLeftLoc =
            new Translation2d(MODULE_DIST_METERS, MODULE_DIST_METERS);
        public static final Translation2d m_frontRightLoc =
            new Translation2d(MODULE_DIST_METERS, -MODULE_DIST_METERS);
        public static final Translation2d m_backLeftLoc =
            new Translation2d(-MODULE_DIST_METERS, MODULE_DIST_METERS);
        public static final Translation2d m_backRightLoc =
            new Translation2d(-MODULE_DIST_METERS, -MODULE_DIST_METERS);

        public static final Translation2d[] MODULE_OFFSET = {
        m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc
        };

        // * Robot movement Constants
        // ! Tune
        public static final double MAX_ANGULAR_SPEED = 720;
        public static final double MAX_LINEAR_SPEED = 8;
        // ! Tune
        public static final double ROBOT_ROTATION_P = 1; // ! 4.5
        public static final double ROBOT_ROTATION_I = 0;
        public static final double ROBOT_ROTATION_D = 0;
        public static final double ROBOT_TRANSLATION_P = 1;
        public static final double ROBOT_TRANSLATION_I = 0;
        public static final double ROBOT_TRANSLATION_D = 0;

        // * Motor constants
        public static final int FRONTLEFT_DRIVE = 1;
        public static final int FRONTRIGHT_DRIVE = 3;
        public static final int BACKLEFT_DRIVE = 5;
        public static final int BACKRIGHT_DRIVE = 7;
        public static final int FRONTLEFT_PIVOT = 2;
        public static final int FRONTRIGHT_PIVOT = 4;
        public static final int BACKLEFT_PIVOT = 6;
        public static final int BACKRIGHT_PIVOT = 8;

        public static final int FRONTLEFT_ABS_ENCODER = 0;
        public static final int FRONTRIGHT_ABS_ENCODER = 1;
        public static final int BACKLEFT_ABS_ENCODER = 2;
        public static final int BACKRIGHT_ABS_ENCODER = 3;

        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 65;
        public static final int PIVOT_MOTOR_CURRENT_LIMIT = 25;

        public static final double DRIVE_GEAR_RATIO = 6.55;

        public static final double WHEEL_DIAMETER_INCHES = 4;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
        public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
        public static final double RPM_TO_METERS_PER_SEC =DRIVE_ROTATIONS_TO_METERS / 60;
    }
    
}
