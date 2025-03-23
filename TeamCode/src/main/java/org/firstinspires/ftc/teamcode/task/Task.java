package org.firstinspires.ftc.teamcode.task;

public interface Task {

    // Returns true if the task is done. Otherwise, returns false.
    boolean perform();

    default void cancel(){}

}
