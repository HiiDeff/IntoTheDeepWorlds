package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.Condition;

/**
 * A task tells if the operation should continue or terminate. This should only be used in a
 * SeriesTask.
 */
public class ConditionalTask implements Task {
    private final Condition condition;

    public ConditionalTask(Condition condition) {
        this.condition = condition;
    }

    @Override
    public boolean perform() {
        // For other tasks, TRUE means task is done, FALSE means task not done. For
        // ConditionalTask, the task is performed only once. The meaning of the return value is
        // different from other tasks. TRUE means the next task in a SeriesTask can continue.
        // Otherwise, the SeriesTask should be terminated.
        return condition.shouldContinue();
    }
}
