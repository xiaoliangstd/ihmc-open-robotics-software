package us.ihmc.robotics.math.trajectories;

import org.apache.commons.math3.util.Precision;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CubicPolynomialTrajectoryGenerator extends PolynomialTrajectoryGenerator
{
   private static final int numberOfCoefficients = 4;

   private final DoubleProvider initialVelocityProvider;
   private final DoubleProvider finalVelocityProvider;

   public CubicPolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider finalPositionProvider,
         DoubleProvider trajectoryTimeProvider, YoRegistry parentRegistry)
   {
      this(namePrefix, initialPositionProvider, () -> 0.0, finalPositionProvider, () -> 0.0, trajectoryTimeProvider,
            parentRegistry);
   }

   public CubicPolynomialTrajectoryGenerator(String namePrefix, DoubleProvider initialPositionProvider, DoubleProvider initialVelocityProvider,
         DoubleProvider finalPositionProvider, DoubleProvider finalVelocityProvider, DoubleProvider trajectoryTimeProvider, YoRegistry parentRegistry)
   {
      super(namePrefix, initialPositionProvider, finalPositionProvider, trajectoryTimeProvider, numberOfCoefficients, parentRegistry);

      this.initialVelocityProvider = initialVelocityProvider;
      this.finalVelocityProvider = finalVelocityProvider;
   }

   @Override
   protected void setPolynomial()
   {
      if (Precision.equals(0.0, trajectoryTime.getDoubleValue()))
      {
         polynomial.setLinear(0.0, initialPositionProvider.getValue(), initialVelocityProvider.getValue());
      }
      else
      {
         polynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), initialPositionProvider.getValue(), initialVelocityProvider.getValue(),
                             finalPositionProvider.getValue(), finalVelocityProvider.getValue());
      }
   }
}
