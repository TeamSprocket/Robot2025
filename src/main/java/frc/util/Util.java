/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import java.util.ArrayList;
import java.util.List;

/**
 * This class contains miscellaneous utility functions that can be used anywhere
 * in the robot project.
 */
public final class Util {
  /**
   * Return 0 if the value is within -range..range. Otherwise return the value.
   */
  public static double deadband(double value, double range) {
    if(value < range && value > -range)
      return 0;
    else
      return value;
  }

  /**
   * Return 0 if the value is within -range..range. Otherwise return the value.
   */
  public static double deadband(double value, double minValue, double maxValue) {
    if(value > minValue && value < maxValue)
      return 0;
    else
      return value;
  }
  
  /**
   * Limits the value to the given min and max
   */
  public static double minmax(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }

  /**
   * Returns if the value is in the given range (between the given min and max)
   */
  public static boolean inRange(double value, double min, double max) {
    return (value > min && value < max);
  }


  /**
   * @return Largest numerical parameter 
   */
  public static double max(double... nums) {
    double max = -1.0 * Double.MAX_VALUE;
    for (double num : nums) {
      if (num > max) {
        max = num;
      }
    }

    return max;
  }




  /**
   * Corrects an input based on a polynomial function.
   */
  public static double polynomialCorrect(double input, double... coefficients) {
    double output = 0;
    double term = 1;

    for(double c : coefficients) {
      output += c * term;
      term *= input;
    }

    return output;
  }

  public static double scale(double val, double fromLow, double fromHigh, double toLow, double toHigh) {
    double normalizedVal = (val - fromLow) / (fromHigh - fromLow);
    return normalizedVal * (toHigh - toLow) + toLow;
  }

  
  public static double average(double[] values) {
    double sum = 0.0;
    for (double val: values) {
      sum += val;
    }
    return (sum / values.length);
  }

  public static double average(List<Double> values) {
    double sum = 0.0;
    for (int i = 0; i < values.size(); i++) {
      sum += values.get(i);
    }
    return (sum / values.size());
  }
  
  public static double average(ArrayList<Double> values) {
    double sum = 0.0;
    for (int i = 0; i < values.size(); i++) {
      sum += values.get(i);
    }
    return (sum / values.size());
  }
  

  public static double signedSquare(double value) {
    if (value == 0) {
      return 0.0;
    }

    double valSign = value / Math.abs(value);
    return valSign * Math.pow(value, 2.0);
  }

  public static double getSign(double value) {
    return value / Math.abs(value);
  }

  public static double distance(double x, double y, double targetX, double targetY) {
    double distance = Math.pow(targetX - x, 2) + Math.pow(targetY - y, 2);
    distance = Math.sqrt(distance);
    return distance;
  }
}