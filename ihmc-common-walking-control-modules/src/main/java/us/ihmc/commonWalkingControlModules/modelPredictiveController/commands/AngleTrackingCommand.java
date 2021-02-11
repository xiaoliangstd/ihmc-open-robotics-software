package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.ContinuousModelPredictiveController;
import us.ihmc.euclid.interfaces.Settable;

public class AngleTrackingCommand implements Settable<AngleTrackingCommand>
{
   private int startIndex;

   private double startValue;
   private double startRate;
   private double finalValue;
   private double finalRate;

   private double duration;

   private double omega;

   private double weight = ContinuousModelPredictiveController.orientationTrackingWeight;

   public void setStartIndex(int startIndex)
   {
      this.startIndex = startIndex;
   }

   public void setStartValue(double startValue)
   {
      this.startValue = startValue;
   }

   public void setStartRate(double startRate)
   {
      this.startRate = startRate;
   }

   public void setFinalValue(double finalValue)
   {
      this.finalValue = finalValue;
   }

   public void setFinalRate(double finalRate)
   {
      this.finalRate = finalRate;
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public double getStartValue()
   {
      return startValue;
   }

   public double getStartRate()
   {
      return startRate;
   }

   public double getFinalValue()
   {
      return finalValue;
   }

   public double getFinalRate()
   {
      return finalRate;
   }

   public int getStartIndex()
   {
      return startIndex;
   }

   public double getDuration()
   {
      return duration;
   }

   public double getWeight()
   {
      return weight;
   }

   public double getOmega()
   {
      return omega;
   }

   public void set(AngleTrackingCommand other)
   {
      setStartIndex(other.startIndex);
      setStartValue(other.startValue);
      setStartRate(other.startRate);
      setFinalValue(other.finalValue);
      setFinalRate(other.finalRate);
      setDuration(other.duration);
      setOmega(other.omega);
      setWeight(other.weight);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof AngleTrackingCommand)
      {
         AngleTrackingCommand other = (AngleTrackingCommand) object;
         if (weight != other.weight)
            return false;
         if (startIndex != other.startIndex)
            return false;
         if (startValue != other.startValue)
            return false;
         if (startRate != other.startRate)
            return false;
         if (finalValue != other.finalValue)
            return false;
         if (finalRate != other.finalRate)
            return false;
         if (duration != other.duration)
            return false;
         if (omega != other.omega)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": weight: " + weight + ", start index: " + startIndex + ", start value: " + startValue
            + ", start rate: " + startRate + ", final value: " + finalValue + ", final rate: " + finalRate + ", omega: " + omega;
   }
}
