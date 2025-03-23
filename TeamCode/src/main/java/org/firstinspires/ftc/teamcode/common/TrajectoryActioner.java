package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public interface TrajectoryActioner {
    public Action trajectory2Action(TrajectoryActionBuilder builder);
}
