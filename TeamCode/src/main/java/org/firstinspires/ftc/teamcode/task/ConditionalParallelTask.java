package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.Condition;

/*
 * Parallel task honor unbounded tasks with a condition which can decide to terminal all tasks
 * early.
 */
public class ConditionalParallelTask extends ParallelTask {

    private final Condition condition;

    public ConditionalParallelTask(Condition condition, Task... tasks) {
        super(tasks);
        this.condition = condition;
    }

    @Override
    public boolean perform() {
        if (condition.shouldContinue()) {
            return super.perform();
        }
        cancel();
        return true;
    }
}
