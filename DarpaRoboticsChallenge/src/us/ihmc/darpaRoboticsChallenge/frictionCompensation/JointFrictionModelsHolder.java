package us.ihmc.darpaRoboticsChallenge.frictionCompensation;

import java.util.EnumMap;

import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;

import us.ihmc.utilities.frictionModels.FrictionModel;
import us.ihmc.utilities.frictionModels.FrictionState;
import us.ihmc.utilities.frictionModels.JointFrictionModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;

public abstract class JointFrictionModelsHolder
{
   private final String name;
   private final DoubleYoVariable stictionTransitionVelocity;
   private final AlphaFilteredYoVariable filteredVelocity;
   private final DoubleYoVariable alphaForFilteredVelocity;
   private final DoubleYoVariable forceThreshold;

   protected final DoubleYoVariable frictionForce;
   protected final EnumYoVariable<FrictionState> frictionCompensationState;
   protected final EnumYoVariable<FrictionModel> activeFrictionModel;
   protected final EnumMap<FrictionModel, JointFrictionModel> frictionModels;

   public JointFrictionModelsHolder(String name, YoVariableRegistry registry, double alpha, double forceThreshold, double stictionTransitionVelocity)
   {
      this.name = name;
      alphaForFilteredVelocity = new DoubleYoVariable(name + "_alphaForFilteredVelocity", registry);
      alphaForFilteredVelocity.set(alpha);
      frictionModels = new EnumMap<FrictionModel, JointFrictionModel>(FrictionModel.class);
      frictionCompensationState = new EnumYoVariable<FrictionState>(name + "_frictionCompensationState", registry, FrictionState.class);
      activeFrictionModel = new EnumYoVariable<FrictionModel>(name + "_activeFrictionModel", registry, FrictionModel.class);
      frictionForce = new DoubleYoVariable(name + "_frictionForce", registry);
      this.stictionTransitionVelocity = new DoubleYoVariable(name + "_stictionTransitionVelocity", registry);
      this.stictionTransitionVelocity.set(stictionTransitionVelocity);
      this.forceThreshold = new DoubleYoVariable(name + "_forceThreshold", registry);
      this.forceThreshold.set(forceThreshold);
      filteredVelocity = new AlphaFilteredYoVariable(name + "_alphaFilteredVelocity", registry, alphaForFilteredVelocity);
      filteredVelocity.update(0.0);
   }

   /**
    * This method computes an equivalent joint velocity in case the joint is in stiction, but the operator is commanding a movement or a force.
    * In case of force, the equivalent velocity is computed as the stiction velocity with the sign of the desired force.
    * In case of velocity the equivalent velocity is the desired velocity. 
    * As current Joint velocity use the less noisy. 
    * In case of stiction the discrimination between force or velocity mode is done based on the requested force value, so force control is predominant.
    * 
    */
   protected Double selectFrictionStateAndFrictionVelocity(double requestedForce, double currentJointVelocity, double requestedJointVelocity)
   {
      double velocityForFrictionCalculation;
      filteredVelocity.update(currentJointVelocity);

      if (activeFrictionModel.getEnumValue() == FrictionModel.OFF)
      {
         frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
         frictionForce.set(0.0);
         return null;
      }

      if (requestedJointVelocity == 0.0 && Math.abs(requestedForce) < forceThreshold.getDoubleValue())
      {
         frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
         frictionForce.set(0.0);
         return null;
      }

      if (Math.abs(filteredVelocity.getDoubleValue()) > stictionTransitionVelocity.getDoubleValue())
      {
         frictionCompensationState.set(FrictionState.OUT_STICTION);
         velocityForFrictionCalculation = currentJointVelocity;
      }
      else
      {
         if (Math.abs(requestedForce) < forceThreshold.getDoubleValue())
         {
            frictionCompensationState.set(FrictionState.IN_STICTION_VELOCITY_MODE);
            velocityForFrictionCalculation = requestedJointVelocity;
         }
         else
         {
            frictionCompensationState.set(FrictionState.IN_STICTION_FORCE_MODE);
            velocityForFrictionCalculation = stictionTransitionVelocity.getDoubleValue() * Math.signum(requestedForce);
         }
      }

      return velocityForFrictionCalculation;
   }

   public void setActiveFrictionModel(FrictionModel requestedFrictionModel)
   {
      activeFrictionModel.set(requestedFrictionModel);
      checkIfExistFrictionModelForThisJoint(requestedFrictionModel);
   }

   public FrictionModel getActiveFrictionModel()
   {
      return activeFrictionModel.getEnumValue();
   }

   public double getCurrentFrictionForce()
   {
      return frictionForce.getDoubleValue();
   }

   public FrictionState getCurrentFrictionState()
   {
      return frictionCompensationState.getEnumValue();
   }

   protected JointFrictionModel getActiveJointFrictionModel()
   {
      return frictionModels.get(activeFrictionModel.getEnumValue());
   }

   protected abstract void checkIfExistFrictionModelForThisJoint(FrictionModel requestedFrictionModel);
}
