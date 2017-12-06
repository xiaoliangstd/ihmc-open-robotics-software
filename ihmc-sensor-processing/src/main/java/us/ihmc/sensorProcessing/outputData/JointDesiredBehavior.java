package us.ihmc.sensorProcessing.outputData;

/**
 * Mutable implementation if the {@link JointDesiredBehaviorReadOnly} interface.
 */
public class JointDesiredBehavior implements JointDesiredBehaviorReadOnly
{
   private JointDesiredControlMode controlMode;
   private double stiffness;
   private double damping;
   private double masterGain;
   private double velocityScaling;

   public JointDesiredBehavior(JointDesiredControlMode controlMode, double stiffness, double damping, double masterGain,
                               double velocityScaling)
   {
      this.controlMode = controlMode;
      this.stiffness = stiffness;
      this.damping = damping;
      this.masterGain = masterGain;
      this.velocityScaling = velocityScaling;
   }

   public void set(JointDesiredBehaviorReadOnly other)
   {
      setControlMode(other.getControlMode());
      setStiffness(other.getStiffness());
      setDamping(other.getDamping());
      setMasterGain(other.getMasterGain());
      setVelocityScaling(other.getVelocityScaling());
   }

   public void setControlMode(JointDesiredControlMode controlMode)
   {
      this.controlMode = controlMode;
   }

   public void setStiffness(double stiffness)
   {
      this.stiffness = stiffness;
   }

   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   public void setMasterGain(double masterGain)
   {
      this.masterGain = masterGain;
   }

   public void setVelocityScaling(double velocityScaling)
   {
      this.velocityScaling = velocityScaling;
   }

   @Override
   public JointDesiredControlMode getControlMode()
   {
      return controlMode;
   }

   @Override
   public double getStiffness()
   {
      return stiffness;
   }

   @Override
   public double getDamping()
   {
      return damping;
   }

   @Override
   public double getMasterGain()
   {
      return masterGain;
   }

   @Override
   public double getVelocityScaling()
   {
      return velocityScaling;
   }

}
