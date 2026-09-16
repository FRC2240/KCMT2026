package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Global singelton thread that enables the high-speed collection of odometry
 * data from status signals.
 */
public class OdometryThread extends Thread {
  private static OdometryThread instance = null;

  private ReentrantLock lock = new ReentrantLock();

  private ArrayList<BaseStatusSignal> signals = new ArrayList<>();
  private ArrayList<Queue<Double>> dataQueues = new ArrayList<>();
  private ArrayList<Queue<Double>> timestampQueues = new ArrayList<>();

  private OdometryThread() {
    setDaemon(true);
  }

  public static OdometryThread getInstance() {
    if (instance == null) {
      instance = new OdometryThread();
    }

    return instance;
  }

  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(50);
    
    lock();
    try {
      signals.add(signal);
      dataQueues.add(queue);
    } finally {
      unlock();
    }

    return queue;
  }

  public Queue<Double> createTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(50);

    lock();
    try {
      timestampQueues.add(queue);
    } finally {
      unlock();
    }

    return queue;
  }

  @Override
  public void run() {
    while (true) {
      lock();
      
      try {
        BaseStatusSignal.waitForAll(2.0 / 250.0, signals);

        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;

        for (BaseStatusSignal signal : signals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (signals.size() > 0) {
          timestamp -= totalLatency / signals.size();
        }

        for (int i = 0; i < signals.size(); i++) {
          dataQueues.get(i).offer(signals.get(i).getValueAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }

      } finally {
        unlock();
      }
    }
  }


  /**
   * Locks all queues so they can be read and put into the pose estimator
   */
  public void lock() {
    lock.lock();
  }

  /**
   * Unlocks the queues to resume data collection
   */
  public void unlock() {
    lock.unlock();
  }

}
