package frc.robot.util;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class ProximitySensor {

  /* Singleton */

  private static ProximitySensor instance = null;

  public static ProximitySensor getInstance() {
    if (instance == null) instance = new ProximitySensor();
    return instance;
  }

  /* Implementation */

  private LaserCan laserCan;

  public ProximitySensor() {
    laserCan = new LaserCan(22);
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("LaserCAN configuration failed!");
    }
  }

  public double getDistanceMM() {
    LaserCan.Measurement measurement = laserCan.getMeasurement();

    if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
      return 0;

    return measurement.distance_mm;
  }

  public boolean hasObject() {
    return getDistanceMM() <= 75;
  }

  public boolean isNoteIndexed() {
    return this.hasObject() && getDistanceMM() > 50;
  }
}
