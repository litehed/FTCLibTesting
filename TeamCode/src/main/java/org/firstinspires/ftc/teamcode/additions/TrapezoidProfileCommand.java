package org.firstinspires.ftc.teamcode.additions;


import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Consumer;

/**
 * A command that runs a {@link TrapezoidProfile}. Useful for smoothly controlling mechanism motion.
 */
public class TrapezoidProfileCommand extends CommandBase {
    private final TrapezoidProfile m_profile;
    private final Consumer<TrapezoidProfile.State> m_output;

    private final ElapsedTime m_timer = new ElapsedTime();

    /**
     * Creates a new TrapezoidProfileCommand that will execute the given {@link TrapezoidProfile}.
     * Output will be piped to the provided consumer function.
     *
     * @param profile      The motion profile to execute.
     * @param output       The consumer for the profile output.
     * @param requirements The subsystems required by this command.
     */
    public TrapezoidProfileCommand(@NonNull TrapezoidProfile profile,
                                   @NonNull Consumer<TrapezoidProfile.State> output, Subsystem... requirements) {
        m_profile = profile;
        m_output = output;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
    }

    @Override
    public void execute() {
        m_output.accept(m_profile.calculate(m_timer.seconds()));
    }

    @Override
    public boolean isFinished() {
        return m_timer.seconds() == m_profile.totalTime();
    }
}


