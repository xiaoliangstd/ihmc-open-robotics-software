package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.ContinuousMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.ContinuousModelPredictiveController;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.function.DoubleConsumer;

public class OrientationTrackingCommand implements MPCCommand<OrientationTrackingCommand>
{
   private int commandId;

   private final AngleTrackingCommand yawTrackingCommand = new AngleTrackingCommand();
   private final AngleTrackingCommand pitchTrackingCommand = new AngleTrackingCommand();
   private final AngleTrackingCommand rollTrackingCommand = new AngleTrackingCommand();

   private double weight = ContinuousModelPredictiveController.orientationTrackingWeight;

   private DoubleConsumer costToGoConsumer;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.ORIENTATION_TRACKING;
   }

   public void clear()
   {
      costToGoConsumer = null;
   }

   public void setStartOrientation(FrameQuaternionReadOnly initialOrientation)
   {
      yawTrackingCommand.setStartValue(initialOrientation.getYaw());
      pitchTrackingCommand.setStartValue(initialOrientation.getPitch());
      rollTrackingCommand.setStartValue(initialOrientation.getRoll());
   }

   public void setFinalOrientation(FrameQuaternionReadOnly finalOrientation)
   {
      yawTrackingCommand.setFinalValue(finalOrientation.getYaw());
      pitchTrackingCommand.setFinalValue(finalOrientation.getPitch());
      rollTrackingCommand.setFinalValue(finalOrientation.getRoll());
   }

   public void setStartAngularRate(FrameVector3DReadOnly initialAngularRate)
   {
      yawTrackingCommand.setStartRate(initialAngularRate.getX());
      pitchTrackingCommand.setStartRate(initialAngularRate.getY());
      rollTrackingCommand.setStartRate(initialAngularRate.getZ());
   }

   public void setFinalAngularRate(FrameVector3DReadOnly finalAngularRate)
   {
      yawTrackingCommand.setFinalRate(finalAngularRate.getX());
      pitchTrackingCommand.setFinalRate(finalAngularRate.getY());
      rollTrackingCommand.setFinalRate(finalAngularRate.getZ());
   }

   public void setSegmentNumber(int segmentNumber, ContinuousMPCIndexHandler indexHandler)
   {
      yawTrackingCommand.setStartIndex(indexHandler.getYawCoefficientsStartIndex(segmentNumber));
      pitchTrackingCommand.setStartIndex(indexHandler.getPitchCoefficientsStartIndex(segmentNumber));
      rollTrackingCommand.setStartIndex(indexHandler.getRollCoefficientsStartIndex(segmentNumber));
   }

   public void setSegmentDuration(double duration)
   {
      yawTrackingCommand.setDuration(duration);
      pitchTrackingCommand.setDuration(duration);
      rollTrackingCommand.setDuration(duration);
   }

   public void setOmega(double omega)
   {
      yawTrackingCommand.setOmega(omega);
      pitchTrackingCommand.setOmega(omega);
      rollTrackingCommand.setOmega(omega);
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
      yawTrackingCommand.setWeight(weight);
      pitchTrackingCommand.setWeight(weight);
      rollTrackingCommand.setWeight(weight);
   }

   public AngleTrackingCommand getYawTrackingCommand()
   {
      return yawTrackingCommand;
   }

   public AngleTrackingCommand getPitchTrackingCommand()
   {
      return pitchTrackingCommand;
   }

   public AngleTrackingCommand getRollTrackingCommand()
   {
      return rollTrackingCommand;
   }

   public double getWeight()
   {
      return weight;
   }

   public double getOmega()
   {
      return yawTrackingCommand.getOmega();
   }

   public void setCostToGoConsumer(DoubleConsumer costToGoConsumer)
   {
      this.costToGoConsumer = costToGoConsumer;
   }

   public DoubleConsumer getCostToGoConsumer()
   {
      return costToGoConsumer;
   }

   public void setCostToGo(double costToGo)
   {
      if (costToGoConsumer != null)
         costToGoConsumer.accept(costToGo);
   }

   public void set(OrientationTrackingCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      yawTrackingCommand.set(other.yawTrackingCommand);
      pitchTrackingCommand.set(other.pitchTrackingCommand);
      rollTrackingCommand.set(other.rollTrackingCommand);
      setOmega(other.getOmega());
      setWeight(other.getWeight());
      setCostToGoConsumer(other.getCostToGoConsumer());
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof OrientationTrackingCommand)
      {
         OrientationTrackingCommand other = (OrientationTrackingCommand) object;
         if (commandId != other.commandId)
            return false;
         if (weight != other.weight)
            return false;
         if (!yawTrackingCommand.equals(other.yawTrackingCommand))
            return false;
         if (!pitchTrackingCommand.equals(other.pitchTrackingCommand))
            return false;
         if (!rollTrackingCommand.equals(other.rollTrackingCommand))
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
      return getClass().getSimpleName() + ": weight: " + weight + ", \nyaw tracking command\n: " + yawTrackingCommand.toString()
             + ",  \npitch tracking command\n: " + pitchTrackingCommand.toString() + ", \nroll tracking command\n:" + rollTrackingCommand.toString();
   }
}
