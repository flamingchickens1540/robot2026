package org.team1540.robot2026.util.logging;

import com.sun.management.OperatingSystemMXBean;
import java.lang.management.ManagementFactory;
import org.littletonrobotics.junction.Logger;

public class PerformanceLogger {
    private static final String tableKey = "LoggedRobot/Performance/";

    public static void log() {
        LoggedTracer.reset();

        OperatingSystemMXBean osBean = ManagementFactory.getPlatformMXBean(OperatingSystemMXBean.class);

        double totalCpuLoad = osBean.getCpuLoad();
        double processCpuLoad = osBean.getProcessCpuLoad();

        Logger.recordOutput(tableKey + "TotalCPU", totalCpuLoad);
        Logger.recordOutput(tableKey + "ProcessCPU", processCpuLoad);

        double freeMemoryMB = osBean.getFreeMemorySize() / 1048576.0; // Convert bytes to MB
        double totalMemoryMB = osBean.getTotalMemorySize() / 1048576.0;
        double usedMemoryMB = totalMemoryMB - freeMemoryMB;
        double memoryUsage = usedMemoryMB / totalMemoryMB;

        Logger.recordOutput(tableKey + "FreeMemoryMB", freeMemoryMB);
        Logger.recordOutput(tableKey + "UsedMemoryMB", usedMemoryMB);
        Logger.recordOutput(tableKey + "MemoryUsage", memoryUsage);

        LoggedTracer.record("PerformanceLogger");
    }
}
