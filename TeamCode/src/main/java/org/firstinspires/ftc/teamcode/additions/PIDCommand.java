/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.firstinspires.ftc.teamcode.additions;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.Arrays;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * A command that controls an output with a {@link PIDController}. Runs forever by default - to add
 * exit conditions and/or other behavior, subclass this class. The controller calculation and output
 * are performed synchronously in the command's execute() method.
 */
public class PIDCommand extends CommandBase {

    protected final PIDController m_controller;
    protected DoubleSupplier m_measurement;
    protected DoubleSupplier m_setpoint;
    protected DoubleConsumer m_useOutput;

    /**
     * Creates a new PIDCommand, which controls the given output with a PIDController.
     *
     * @param controller        the controller that controls the output.
     * @param measurementSource the measurement of the process variable
     * @param setpointSource    the controller's setpoint
     * @param useOutput         the controller's output
     * @param requirements      the subsystems required by this command
     */
    public PIDCommand(@NonNull PIDController controller, @NonNull DoubleSupplier measurementSource,
                      @NonNull DoubleSupplier setpointSource, @NonNull DoubleConsumer useOutput, Subsystem... requirements) {

        m_controller = controller;
        m_useOutput = useOutput;
        m_measurement = measurementSource;
        m_setpoint = setpointSource;
        m_requirements.addAll(Arrays.asList(requirements));
    }

    /**
     * Creates a new PIDCommand, which controls the given output with a PIDController.
     *
     * @param controller        the controller that controls the output.
     * @param measurementSource the measurement of the process variable
     * @param setpoint          the controller's setpoint
     * @param useOutput         the controller's output
     * @param requirements      the subsystems required by this command
     */
    public PIDCommand(PIDController controller, DoubleSupplier measurementSource,
                      double setpoint, DoubleConsumer useOutput, Subsystem... requirements) {
        this(controller, measurementSource, () -> setpoint, useOutput, requirements);
    }

    @Override
    public void initialize() {
        m_controller.reset();
    }

    @Override
    public void execute() {
        m_useOutput.accept(
                m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        m_useOutput.accept(0);
    }

    /**
     * Returns the PIDController used by the command.
     *
     * @return The PIDController
     */
    public PIDController getController() {
        return m_controller;
    }
}
