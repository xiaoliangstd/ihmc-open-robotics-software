package us.ihmc.robotics;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrix3x3;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.matrixlib.MatrixTools;

public class MatrixMissingTools
{
   /**
    * Sets a block of a matrix
    *
    * @param dest            Set a block of this matrix
    * @param destStartRow    Row index of the top left corner of the block to set
    * @param destStartColumn Column index of the top left corner of the block to set
    * @param src             Get a block of this matrix
    * @param srcStartRow     Row index of the top left corner of the block to use from otherMatrix
    * @param srcStartColumn  Column index of the top left corner of the block to use from otherMatrix
    * @param numberOfRows    Row size of the block
    * @param numberOfColumns Column size of the block
    * @param scale           Scale the block from otherMatrix by this value
    */
   public static void setMatrixBlock(DMatrix1Row dest, int destStartRow, int destStartColumn, DMatrix src, int srcStartRow, int srcStartColumn,
                                     int numberOfRows, int numberOfColumns, double scale)
   {
      if (numberOfRows == 0 || numberOfColumns == 0)
         return;

      if (dest.getNumRows() < numberOfRows || dest.getNumCols() < numberOfColumns)
         throw new IllegalArgumentException("dest is too small, min size: [rows: " + numberOfRows + ", cols: " + numberOfColumns + "], was: [rows: "
                                            + dest.getNumRows() + ", cols: " + dest.getNumCols() + "]");
      if (src.getNumRows() < numberOfRows + srcStartRow || src.getNumCols() < numberOfColumns + srcStartColumn)
         throw new IllegalArgumentException("src is too small, min size: [rows: " + (numberOfRows + srcStartRow) + ", cols: "
                                            + (numberOfColumns + srcStartColumn) + "], was: [rows: " + src.getNumRows() + ", cols: " + src.getNumCols() + "]");

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            dest.unsafe_set(destStartRow + i, destStartColumn + j, scale * src.unsafe_get(srcStartRow + i, srcStartColumn + j));
         }
      }
   }

   public static void setDiagonalValues(DMatrix1Row mat, double diagonalValue, int rowStart, int colStart)
   {
      if (rowStart >= mat.getNumRows())
         throw new IllegalArgumentException("Row start cannot exceed the number of rows.");
      if (colStart >= mat.getNumCols())
         throw new IllegalArgumentException("Col start cannot exceed the number of columns.");

      int width = (mat.getNumRows() - rowStart) < (mat.getNumCols() - colStart) ? (mat.getNumRows() - rowStart) : (mat.getNumCols() - colStart);

      int index = colStart;
      for (int i = 0; i < rowStart && i < width; i++)
         index += mat.getNumCols();

      for (int i = 0; i < width; i++, index += mat.getNumCols() + 1)
      {
         mat.data[index] = diagonalValue;
      }
   }

   public static void fast2x2Inverse(DMatrixRMaj matrix, DMatrixRMaj inverseToPack)
   {
      double determinantInverse = 1.0 / (matrix.get(0, 0) * matrix.get(1, 1) - matrix.get(0, 1) * matrix.get(1, 0));
      inverseToPack.set(0, 0, determinantInverse * matrix.get(1, 1));
      inverseToPack.set(1, 1, determinantInverse * matrix.get(0, 0));
      inverseToPack.set(0, 1, -determinantInverse * matrix.get(0, 1));
      inverseToPack.set(1, 0, -determinantInverse * matrix.get(1, 0));
   }
   public static void setDiagonal(DMatrix3x3 mat, double diagonalValue)
   {
      for (int row = 0; row < 3; row++)
      {
         for (int col = 0; col < 3; col++)
         {
            if (row == col)
               mat.unsafe_set(row, col, diagonalValue);
            else
               mat.unsafe_set(row, col, 0.0);
         }
      }
   }

   public static void setMatrixBlock(DMatrix dest, int destStartRow, int destStartColumn, DMatrix3x3 src, double scale)
   {
      setMatrixBlock(dest, destStartRow, destStartColumn, src, 0, 0, 3, 3, scale);
   }

   public static void setMatrixBlock(DMatrix dest,
                                     int destStartRow,
                                     int destStartColumn,
                                     DMatrix src,
                                     int srcStartRow,
                                     int srcStartColumn,
                                     int numberOfRows,
                                     int numberOfColumns,
                                     double scale)
   {
      if (numberOfRows == 0 || numberOfColumns == 0)
         return;

      if (dest.getNumRows() < numberOfRows || dest.getNumCols() < numberOfColumns)
         throw new IllegalArgumentException(
               "dest is too small, min size: [rows: " + numberOfRows + ", cols: " + numberOfColumns + "], was: [rows: " + dest.getNumRows() + ", cols: "
               + dest.getNumCols() + "]");
      if (src.getNumRows() < numberOfRows + srcStartRow || src.getNumCols() < numberOfColumns + srcStartColumn)
         throw new IllegalArgumentException(
               "src is too small, min size: [rows: " + (numberOfRows + srcStartRow) + ", cols: " + (numberOfColumns + srcStartColumn) + "], was: [rows: "
               + src.getNumRows() + ", cols: " + src.getNumCols() + "]");

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            dest.unsafe_set(destStartRow + i, destStartColumn + j, scale * src.unsafe_get(srcStartRow + i, srcStartColumn + j));
         }
      }
   }

   public static DMatrixRMaj createVector(int size, double fillValue)
   {
      DMatrixRMaj vector = new DMatrixRMaj(size, 1);
      CommonOps_DDRM.fill(vector, fillValue);
      return vector;
   }

   public static DMatrixRMaj createRowVector(double... values)
   {
      DMatrixRMaj vector = new DMatrixRMaj(1, values.length);
      for (int i = 0; i < values.length; i++)
      {
         vector.set(i, values[i]);
      }

      return vector;
   }

   public static boolean epsilonEquals(DMatrix1Row a, DMatrix1Row b, double epsilon)
   {
      if (a.numRows != b.numRows)
         return false;
      if (a.numCols != b.numCols)
         return false;
      for (int i = 0; i < a.getNumElements(); i++)
      {
         if (!EuclidCoreTools.epsilonEquals(a.get(i), b.get(i), epsilon))
            return false;
      }
      return true;
   }

   public static SimpleMatrix toSkewSymmetricMatrix(DMatrix1Row vector)
   {
      SimpleMatrix skewSymmetric = new SimpleMatrix(vector.getNumElements(), vector.getNumElements());
      skewSymmetric.set(0, 0, 0.0);
      skewSymmetric.set(0, 1, -vector.get(2));
      skewSymmetric.set(0, 2, vector.get(1));

      skewSymmetric.set(1, 0, vector.get(2));
      skewSymmetric.set(1, 1, 0.0);
      skewSymmetric.set(1, 2, -vector.get(0));

      skewSymmetric.set(2, 0, -vector.get(1));
      skewSymmetric.set(2, 1, vector.get(0));
      skewSymmetric.set(2, 2, 0.0);
      return skewSymmetric;
   }


   public static void toSkewSymmetricMatrix(DMatrix1Row vector, DMatrixRMaj skewSymmetricToPack)
   {
      skewSymmetricToPack.set(0, 0, 0.0);
      skewSymmetricToPack.set(0, 1, -vector.get(2));
      skewSymmetricToPack.set(0, 2, vector.get(1));

      skewSymmetricToPack.set(1, 0, vector.get(2));
      skewSymmetricToPack.set(1, 1, 0.0);
      skewSymmetricToPack.set(1, 2, -vector.get(0));

      skewSymmetricToPack.set(2, 0, -vector.get(1));
      skewSymmetricToPack.set(2, 1, vector.get(0));
      skewSymmetricToPack.set(2, 2, 0.0);
   }

   public static void unsafe_add(DMatrixRMaj matrix, int row, int col, double value)
   {
      matrix.data[ row * matrix.numCols + col ] += value;
   }

   public static void addMatrixBlock(DMatrix1Row dest, int destStartRow, int destStartColumn, DMatrix1Row src)
   {
      addMatrixBlock(dest, destStartRow, destStartColumn, src, 1.0);
   }

   public static void addMatrixBlock(DMatrix1Row dest, int destStartRow, int destStartColumn, DMatrix1Row src, double scale)
   {
      MatrixTools.addMatrixBlock(dest, destStartRow, destStartColumn, src, 0, 0, src.getNumRows(), src.getNumCols(), scale);
   }
}
