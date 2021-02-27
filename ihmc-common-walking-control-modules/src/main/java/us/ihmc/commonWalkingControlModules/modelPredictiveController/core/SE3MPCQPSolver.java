package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is the wrapper class for the quadratic program solver. It receives all the MPC Commands, converting them to quadratic costs or linear constraints.
 * It then submits these costs and constraints to a quadratic program solver, and computes the optimal solution. It then provides these solution coefficients
 * as an output.
 */
public class SE3MPCQPSolver extends LinearMPCQPSolver
{
   private final SE3MPCIndexHandler indexHandler;

   private final YoDouble firstOrientationVariableRegularization = new YoDouble("firstOrientationVariableRegularization", registry);
   private final YoDouble secondOrientationVariableRegularization = new YoDouble("secondOrientationVariableRegularization", registry);
   private final YoDouble firstOrientationRateVariableRegularization = new YoDouble("firstOrientationRateVariableRegularization", registry);
   private final YoDouble secondOrientationRateVariableRegularization = new YoDouble("secondOrientationRateVariableRegularization", registry);

   private final MomentumOrientationInputCalculator orientationInputCalculator;

   public SE3MPCQPSolver(SE3MPCIndexHandler indexHandler, double dt, double gravityZ, double mass, YoRegistry parentRegistry)
   {
      this(indexHandler, dt, gravityZ, mass, false, parentRegistry);
   }

   public SE3MPCQPSolver(SE3MPCIndexHandler indexHandler, double dt, double gravityZ, double mass, boolean useBlockInverse, YoRegistry parentRegistry)
   {
      super(indexHandler, dt, gravityZ, useBlockInverse, parentRegistry);
      this.indexHandler = indexHandler;

      orientationInputCalculator = new MomentumOrientationInputCalculator(indexHandler, mass, gravityZ);

      firstOrientationVariableRegularization.set(1e-8);
      secondOrientationVariableRegularization.set(1e-8);
      firstOrientationRateVariableRegularization.set(1e-6);
      secondOrientationRateVariableRegularization.set(1e-6);
   }

   public void setFirstOrientationVariableRegularization(double value)
   {
      firstOrientationVariableRegularization.set(value);
   }

   public void setSecondOrientationVariableRegularization(double value)
   {
      secondOrientationVariableRegularization.set(value);
   }

   public void setFirstOrientationRateVariableRegularization(double value)
   {
      firstOrientationRateVariableRegularization.set(value);
   }

   public void setSecondOrientationRateVariableRegularization(double value)
   {
      secondOrientationRateVariableRegularization.set(value);
   }

   public void addValueRegularization()
   {
      super.addValueRegularization();

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int var = indexHandler.getOrientationStartIndices(segmentId);
         int ticks = indexHandler.getOrientationTicksInSegment(segmentId);
         for (int tick = 0; tick < ticks; tick++)
         {
            int end = var + 3;
            for (; var < end; var++)
               solverInput_H.add(var, var, firstOrientationVariableRegularization.getDoubleValue());
            end += 3;
            for (; var < end; var++)
               solverInput_H.add(var, var, secondOrientationVariableRegularization.getDoubleValue());
         }
      }
   }

   public void addRateRegularization()
   {
      super.addRateRegularization();

      double firstOrientationCoefficientFactor = dt * dt / firstOrientationRateVariableRegularization.getDoubleValue();
      double secondOrientationCoefficientFactor = dt * dt / secondOrientationRateVariableRegularization.getDoubleValue();

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int var = indexHandler.getOrientationStartIndices(segmentId);
         int ticks = indexHandler.getOrientationTicksInSegment(segmentId);
         for (int tick = 0; tick < ticks; tick++)
         {
            int end = var + 3;
            for (; var < end; var++)
            {
               solverInput_H.add(var, var, firstOrientationCoefficientFactor);
               solverInput_f.add(var, 0, -solverOutput.get(var, 0) / firstOrientationCoefficientFactor);
            }
            end += 3;
            for (; var < end; var++)
            {
               solverInput_H.add(var, var, secondOrientationCoefficientFactor);
               solverInput_f.add(var, 0, -solverOutput.get(var, 0) / secondOrientationCoefficientFactor);
            }
         }
      }
   }

   public void submitMPCCommand(MPCCommand<?> command)
   {
      if (command.getCommandType() == MPCCommandType.ORIENTATION_DYNAMICS)
      {
         submitOrientationDynamicsCommand((DiscreteMomentumOrientationCommand) command);
         return;
      }

      super.submitMPCCommand(command);
   }

   public void submitOrientationDynamicsCommand(DiscreteMomentumOrientationCommand command)
   {
      boolean success = orientationInputCalculator.compute(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }
}
