package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Random;

public class CARESolverTestTools
{
   public static DenseMatrix64F generateRandomSymmetricPDMatrix(Random random, int size)
   {
      DenseMatrix64F matrix = generateRandomSymmetricMatrix(random, size);
      MatrixTools.addDiagonal(matrix, size);

      return matrix;
   }


   public static DenseMatrix64F generateRandomSymmetricMatrix(Random random, int size)
   {
      DenseMatrix64F a = new DenseMatrix64F(size, size);
      double[] aValues = RandomNumbers.nextDoubleArray(random, size * size, 1.0);
      a.setData(aValues);

      DenseMatrix64F matrix =  new DenseMatrix64F(size, size);
      CommonOps.transpose(a, matrix);
      CommonOps.addEquals(matrix, a);
      CommonOps.scale(0.5, a);

      return matrix;
   }
   public static DenseMatrix64F generateRandomDiagonalMatrix(Random random, int size, boolean positiveSemiDefinite)
   {
      DenseMatrix64F a = new DenseMatrix64F(size, size);
      double lowerBound = positiveSemiDefinite ? 0.0 : 1e-3;
      double[] aValues = RandomNumbers.nextDoubleArray(random, size, lowerBound, 100.0);
      for (int i = 0; i < size; i++)
      {
         a.set(i, i, aValues[i]);
      }
      return a;
   }
}
