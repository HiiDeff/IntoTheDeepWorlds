package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/*
 * Parallel task honor unbounded tasks.
 */
public class ParallelTask implements Task {

    private final Set<Task> tasks;
    private ElapsedTime timer;

    public ParallelTask(Task... tasks) {
        this.tasks = new HashSet<>(Arrays.asList(tasks));
    }

    @Override
    public boolean perform() {
        if (timer == null) {
            timer = new ElapsedTime();
        }
        Set<Task> remove = new HashSet<>();
        int unfinishedUnboundedTasks = 0;
        for (Task task : tasks) {
//            Log.i("allendebug", task.getClass().toString());
            if (task.perform()) {
                remove.add(task);
            } else if (task instanceof UnboundedTask) {
                unfinishedUnboundedTasks++;
            }
        }
        for (Task task : remove) tasks.remove(task);
        if (tasks.size() == unfinishedUnboundedTasks) {
            cancel();
            return true;
        }
        return false;
    }

    @Override
    public void cancel() {
        for (Task task : tasks) {
            task.cancel();
        }
    }

    public ParallelTask add(Task task) {
        tasks.add(task);
        return this;
    }
}
