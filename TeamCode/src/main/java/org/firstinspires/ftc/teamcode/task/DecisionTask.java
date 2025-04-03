package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.Condition;

/**
 * A task that runs the first task if a condition is true, and the second if false
 */
public class DecisionTask implements Task {
    private final Condition condition;
    private final Task conditionTrueTask, conditionFalseTask;
    private boolean isTrue = false, started = false;

    public DecisionTask(Condition condition, Task conditionTrueTask, Task conditionFalseTask) {
        this.condition = condition;
        this.conditionTrueTask = conditionTrueTask;
        this.conditionFalseTask = conditionFalseTask;
    }

    @Override
    public boolean perform() {
        if(!started) {
            started = true;
            isTrue = condition.shouldContinue();
        }
        if(isTrue) {
            return conditionTrueTask.perform();
        } else {
            return conditionFalseTask.perform();
        }
    }

    @Override
    public void cancel(){
        if(!started) return;
        if(isTrue) {
            conditionTrueTask.cancel();
        } else {
            conditionFalseTask.cancel();
        }
    }
}
