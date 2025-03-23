package org.firstinspires.ftc.teamcode.task;

public class SleepTask extends TimedTask {

    private boolean canceled = false;

    public SleepTask(int milliseconds) {
        setFinishTimeMillis(milliseconds);
    }

    @Override
    protected boolean performInternal() {
        // This task is time based, base class logic takes care of it unless it is cancelled.
        return canceled;
    }

    @Override
    public void cancel() {
        canceled = true;
    }
}
