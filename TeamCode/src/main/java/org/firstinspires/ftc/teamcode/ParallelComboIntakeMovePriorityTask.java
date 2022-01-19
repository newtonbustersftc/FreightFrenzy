package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

public class ParallelComboIntakeMovePriorityTask implements RobotControl {
    String taskName = "ParallelComboPriorityTask";

    ArrayList<RobotControl> taskList = new ArrayList<>();
    boolean[] doneList; //update doneList to be true if the task is cleaned up; don't need to be isDone() anymore
    int counterDone; //+1 for every True in doneList
    Pose2d lastPos;

    @Override
    public void prepare() {
        doneList = new boolean[taskList.size()];
        counterDone = 0;

        for (int i = 0; i < taskList.size(); i++) {
            taskList.get(i).prepare();
            doneList[i] = false;
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < taskList.size(); i++) {
            if (doneList[i] == false) {
                RobotControl task = taskList.get(i);

                if (!task.isDone()) {
                    task.execute();
                } else {
                    Logger.logFile("parallel priority task completed:" + task);
                    task.cleanUp();

                    doneList[i] = true;
                            counterDone++;
                    Logger.logFile("counterDone:"+counterDone);
                    i = (i==0) ? 1 : 0;
                    taskList.get(i).cleanUp();

                    break;
                }
            }
        }
    }

    @Override
    public void cleanUp() {
        Logger.logFile("parallel cleannup");
    }

    @Override
    public boolean isDone() {
        if(counterDone == 1) {
            Logger.logFile("Suppose intake got freight and exit");
        }
        return counterDone == 1;
    }

    public void setTaskList(ArrayList<RobotControl> taskList) {
        this.taskList = taskList;
    }

    public void setTaskName(String taskName) {
        this.taskName = taskName;
    }

    public String toString() {
        return taskName;
    }

    public void addTask(RobotControl task) {
        taskList.add(task);
    }
}