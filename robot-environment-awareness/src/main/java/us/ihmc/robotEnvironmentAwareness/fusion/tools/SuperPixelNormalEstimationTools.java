package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotEnvironmentAwareness.fusion.data.SuperPixelData;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

import java.util.List;
import java.util.stream.Stream;

public class SuperPixelNormalEstimationTools
{
   public static void updateUsingPCA(SuperPixelData superPixel, List<Point3D> points, boolean addPointsInParallel)
   {
      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
      Stream<Point3D> pointStream = addPointsInParallel ? points.parallelStream() : points.stream();
      pointStream.forEach(pca::addDataPoint);

      pca.compute();

      Point3D center = new Point3D();
      Vector3D normal = new Vector3D();
      Vector3D standardDeviation = new Vector3D();

      pca.getMean(center);
      pca.getThirdVector(normal);
      pca.getStandardDeviation(standardDeviation);

      if (normal.getZ() < 0.0)
         normal.negate();

      superPixel.setCenter(center);
      superPixel.setNormal(normal);
      superPixel.setStandardDeviation(standardDeviation);
   }
}
