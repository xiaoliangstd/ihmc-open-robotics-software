package us.ihmc.ihmcPerception.lineSegmentDetector;

import boofcv.struct.calib.CameraPinholeBrown;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

public class LineSegmentToPlanarRegionAssociator
{
   private CameraPinholeBrown camIntrinsics;
   private PoseReferenceFrame regionFrame;
   private PoseReferenceFrame cameraFrame;

   public LineSegmentToPlanarRegionAssociator(ReferenceFrame sensorFrame)
   {
      regionFrame = new PoseReferenceFrame("RegionFrame", ReferenceFrame.getWorldFrame());
      cameraFrame = new PoseReferenceFrame("CameraFrame", sensorFrame);
      cameraFrame.setOrientationAndUpdate(new RotationMatrix(0, -1, 0, 0, 0, -1, 1, 0, 0));
   }

   private Mat curLines;

   public void loadParams(CameraPinholeBrown intrinsics)
   {
      this.camIntrinsics = intrinsics;
   }

   public ArrayList<Point> projectPlanarRegion(PlanarRegion region, Point2D regionMidPoint)
   {
      regionFrame.setPoseAndUpdate(region.getTransformToWorld());

      List<Point2D> concaveHull = region.getConcaveHull();

      ArrayList<Point> pointList = new ArrayList<>();

      int regionSize = 0;

      for (int i = 0; i < concaveHull.size(); i++)
      {
         FramePoint3D vertex = new FramePoint3D(regionFrame, concaveHull.get(i).getX(), concaveHull.get(i).getY(), 0);
         vertex.changeFrame(cameraFrame);

         System.out.println(vertex);

         if (vertex.getZ() >= 0)
         {
            double px = camIntrinsics.cx + camIntrinsics.fx * vertex.getX() / vertex.getZ();
            double py = camIntrinsics.cy + camIntrinsics.fy * vertex.getY() / vertex.getZ();
            regionMidPoint.add(px, py);
            regionSize += 1;
            pointList.add(new Point(px, py));
         }
      }
      regionMidPoint.scale(1 / (double) regionSize);
      return pointList;
   }

   public void drawPolygonOnImage(Mat img, ArrayList<Point> pointList, Point2D regionMidPoint, int id)
   {
      if (pointList.size() > 2)
      {

         MatOfPoint points = new MatOfPoint();
         points.fromList(pointList);

         MatOfInt hull = new MatOfInt();
         Imgproc.convexHull(points, hull);

         Point[] hullPoints = new Point[hull.rows()];

         List<Integer> hullContourIdxList = hull.toList();
         Point[] polygonArray = points.toArray();
         for (int i = 0; i < hullContourIdxList.size(); i++)
         {
            hullPoints[i] = polygonArray[hullContourIdxList.get(i)];
         }

         MatOfPoint convexPoly = new MatOfPoint(hullPoints);
         List<MatOfPoint> ppt = new ArrayList<>();
         ppt.add(convexPoly);

         Imgproc.fillPoly(img, ppt, new Scalar(id * 123 % 255, id * 321 % 255, id * 135 % 255));
         Imgproc.circle(img, new Point(regionMidPoint.getX(), regionMidPoint.getY()), 3, new Scalar(255, 140, 255), -1);
      }
   }

   public void drawLineRegionAssociation(Mat img, ArrayList<Point> pointList, Point2D regionMidPoint, int id)
   {
      if (pointList.size() > 2)
      {

         if (regionMidPoint.getX() < camIntrinsics.getWidth() && regionMidPoint.getY() < camIntrinsics.getHeight() && regionMidPoint.getX() >= 0
             && regionMidPoint.getY() >= 0)
         {
            // Mat mask = Mat.zeros(curImg.rows() + 2, curImg.cols() + 2, CvType.CV_8U);
            // Imgproc.floodFill(curImgSegment, mask, new Point((int)regionMidPoint.getX(), (int)regionMidPoint.getY()),
            //         new Scalar(100 + region.getRegionId()*123 % 155, 100 + region.getRegionId()*321 % 155, 100 + region.getRegionId()*135 % 155),
            //         new Rect(), new Scalar(4,4,4), new Scalar(4,4,4), 4);

            ArrayList<Point> regionSegment = getSegmentFromLines(curLines, regionMidPoint);

            // for (Point2D segment : regionSegment) {
            //     Imgproc.line(img, new Point(segment.getX(), segment.getY()), new Point(regionMidPoint.getX(), regionMidPoint.getY()),
            //             new Scalar(255, 100, 10), 2);
            //     // Imgproc.line(img, new Point(segment.getSecondEndpointX(), segment.getSecondEndpointY()), new Point(regionMidPoint.getX(), regionMidPoint.getY()),
            //     //         new Scalar(255, 100, 10), 2);
            //     Imgproc.circle(img, new Point(segment.getX(), segment.getY()), 8, new Scalar(255, 0, 255), -1);
            // }
            drawPolygonOnImage(img, regionSegment, regionMidPoint, id);
         }
         Imgproc.circle(img, new Point(regionMidPoint.getX(), regionMidPoint.getY()), 3, new Scalar(255, 140, 255), -1);
      }
   }

   public int compare(LineSegment2D a, LineSegment2D b, Point2D centroid)
   {
      double delta = a.distance(centroid) - b.distance(centroid);
      if (delta > 0.00001)
         return 1;
      if (delta < -0.00001)
         return -1;
      return 0;
   }

   public ArrayList<Point> getSegmentFromLines(Mat curLines, Point2D centroid)
   {
      ArrayList<Point> segments = new ArrayList<>();
      PriorityQueue<LineSegment2D> queue = new PriorityQueue<>(10, (a, b) -> compare(a, b, centroid));
      for (int i = 0; i < curLines.rows(); i++)
      {
         double[] line = curLines.get(i, 0);
         LineSegment2D ls = new LineSegment2D(line[0], line[1], line[2], line[3]);
         if (ls.distanceSquared(centroid) < 8000 && ls.length() > 30)
         {
            queue.add(ls);
         }
      }

      System.out.println("Queue Size:" + queue.size());

      ArrayList<Line2D> polygon = new ArrayList<Line2D>();

      for (LineSegment2D ls : queue)
      {
         System.out.println("Distance:" + ls.distance(centroid));

         segments.add(new Point(ls.getFirstEndpointX(), ls.getFirstEndpointY()));
         segments.add(new Point(ls.getSecondEndpointX(), ls.getSecondEndpointY()));

         // Point2D closest = (Point2D) ls.orthogonalProjectionCopy(centroid);
         // boolean present = false;
         // ArrayList<Point2D> projections = new ArrayList<>();
         // Line2D line = new Line2D(ls);
         // Point2D proj = (Point2D) line.orthogonalProjectionCopy(centroid);
         // for(Point2D p : projections){
         //     if(proj.distanceSquared(p) < 10){
         //         present = true;
         //     }
         // }
         // if(!present){
         //     projections.add(proj);
         //     polygon.add(line);
         // }
         // if(polygon.size() >= 4)break;
      }

      // if(polygon.size() >= 4){
      //     for(int i = 0; i<polygon.size()-1; i++){
      //         System.out.println( (i+1) % polygon.size());
      //         Point2D corner = (Point2D) polygon.get(i).intersectionWith(polygon.get( i+1 ));
      //         segments.add(corner);
      //     }
      // }
      return segments;
   }

   public void setCamIntrinsics(CameraPinholeBrown camIntrinsics)
   {
      this.camIntrinsics = camIntrinsics;
   }

   public void setCurLines(Mat curLines)
   {
      this.curLines = curLines;
   }
}