package org.ice.util.sendable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * {@link SubsystemBase} subclass that uses {@link AnnotatedSendable}
 */
public class AnnotatedSubsystemBase extends SubsystemBase implements AnnotatedSendable {

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        AnnotatedSendable.super.initSendable(builder);
    }
}
