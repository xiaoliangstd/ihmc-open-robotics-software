package us.ihmc.robotics.linearAlgebra.careSolvers;

public class KleinmanCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new KleinmanCARESolver();
   }
}
