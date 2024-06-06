// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wmironpatriots.lib.IronUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IronController extends CommandXboxController {
    public final double joystickDeadband, triggerDeadband;

    /**
     * Creates a new Iron Controller
     *
     * @param port port controller is connected too
     * @param joystickDeadband Joystick deadzone
     * @param triggerDeadband Trigger deadzone
     */
    public IronController(int port, double joystickDeadband, double triggerDeadband) {
        super(port);

        this.joystickDeadband = joystickDeadband;
        this.triggerDeadband = triggerDeadband;
    }

    /**
     * Joystick deadband
     *
     * @param rawAxis 
     * @return double
     */
    public double getJoystickAxis(int rawAxis) {
        return MathUtil.applyDeadband(
            Math.abs(Math.pow(super.getRawAxis(rawAxis), 2)) * Math.signum(super.getRawAxis(rawAxis)),
            joystickDeadband
        );
    }

    /**
     * Trigger deadband
     * 
     * @param rawAxis
     * @return double
     */
    public double triggerDeadbandOutput(int rawAxis) {
        return MathUtil.applyDeadband(super.getRawAxis(rawAxis), triggerDeadband); 
    }

    /**
     * Returns circular angle of the joystick
     * 
     * @param yAxis Joystick Y
     * @param xAxis Joystick X
     * @return {@link Rotation2d}
     */
    public Rotation2d flickStickOutput(int yAxis, int xAxis) {
        double y = getJoystickAxis(yAxis);
        double x = getJoystickAxis(xAxis);
        return (y <= joystickDeadband && x <= joystickDeadband) ? new Rotation2d() : Rotation2d.fromRadians(Math.atan2(y, x));
    }

    /**
     * Returns circular angle of the joystick
     * 
     * @param defaultOut Output when x & y is at 0
     * @param yAxis Joystick Y
     * @param xAxis Joystick X
     * @return {@link Rotation2d}
     */
    public Rotation2d getJoystickCircularAngle(Rotation2d defaultOut, int yAxis, int xAxis) {
        double y = getJoystickAxis(yAxis);
        double x = getJoystickAxis(xAxis);
        return (y == 0.0 && x == 0.0) ? defaultOut : Rotation2d.fromRadians(Math.atan2(y, x));
    }

    /**
     * Rumble Controller
     *
     * @param rmb
     * @param n
     * @return {@link Command}
     */
    public Command rumbleController(GenericHID.RumbleType rmb, double n) {
        return new InstantCommand(() -> super.getHID().setRumble(rmb, n));
    }
}