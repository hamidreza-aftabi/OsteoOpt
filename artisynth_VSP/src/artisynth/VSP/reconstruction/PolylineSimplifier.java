package artisynth.VSP.reconstruction;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/**
 * Implements the RDP algorithm for simplifying a polyline given constraints on
 * the number of resulting segments and minimum segment length.
 */
public class PolylineSimplifier {

   private static class Range {
      private Range(int first, int last) {
         this.first = first;
         this.last = last;
      }
      int first;
      int last;
   }

   private static double getMinDistance (ArrayList<Point3d> polyline) {
      double min = Double.POSITIVE_INFINITY;
      for (int i=0; i<polyline.size()-1; i++) {
         double d = polyline.get(i).distance (polyline.get(i+1));
         if (d < min) {
            min = d;
         }
      }
      return min;
   }

   public static double getSquareSegmentDistance (
      Vector3d v, Vector3d v1, Vector3d v2) {

      Vector3d d12 = new Vector3d();
      Vector3d d10 = new Vector3d();
      d12.sub (v2, v1);
      d10.sub (v, v1);
      double d12sqr = d12.normSquared();
      Vector3d vx = new Vector3d(v1);
      double t = 0;
      if (d12sqr > 0) {
         t = d10.dot(d12)/d12sqr;
         if (t > 1.0) {
            vx.set (v2);
         }
         else if (t > 0) {
            vx.scaledAdd (t, d12);
         }
      }
      return v.distanceSquared(vx);
   }

   public static ArrayList<Point3d> simplifyDouglasPeucker (
      ArrayList<Point3d> oldList, double sqrTol) {
      BitSet bitSet = new BitSet(oldList.size());
      bitSet.set(0);
      bitSet.set(oldList.size()-1);

      List<Range> stack = new ArrayList<Range>();
      stack.add(new Range(0, oldList.size()-1));

      while (!stack.isEmpty()) {
         Range range = stack.remove(stack.size() - 1);

         int index = -1;
         double maxDsqr = 0f;

         // find index of point with maximum square distance from first and
         // last point
         for (int i=range.first+1; i<range.last; ++i) {
            double dsqr = getSquareSegmentDistance (
               oldList.get(i), oldList.get(range.first), oldList.get(range.last));

            if (dsqr > maxDsqr) {
               index = i;
               maxDsqr = dsqr;
            }
         }

         if (maxDsqr > sqrTol) {
            bitSet.set(index);
            stack.add(new Range(range.first, index));
            stack.add(new Range(index, range.last));
         }
      }

      ArrayList<Point3d> newList = new ArrayList<>();
      for (int index = bitSet.nextSetBit(0);
           index >= 0; index = bitSet.nextSetBit(index+1)) {
         newList.add (new Point3d(oldList.get(index)));
      }
      return newList;
   }

   public static ArrayList<Point3d> bisectSimplifyDouglasPeucker (
      ArrayList<Point3d> polyline, double minDistance, int maxSegments) {

      double minEpsilon = 0;
      double maxEpsilon = 0;
      double prevEpsilon = 0;
      int iter = 0;
      int numNewPoints = 0;
      double tempEpsilon, epsilon, shortestDistance = 0;
      ArrayList<Point3d> newline;

      // Finding maximum epsilon in the beginning
      for (int i =0; i < polyline.size()-2; i++ ) {
         tempEpsilon = getSquareSegmentDistance (
            polyline.get(i+1), polyline.get(0), polyline.get(polyline.size()-1));
         if (tempEpsilon > maxEpsilon) {
            maxEpsilon = tempEpsilon;
         }
      }

      while (true) {
         iter += 1;
         epsilon = (minEpsilon + maxEpsilon) / 2;
         if (Math.abs(epsilon - prevEpsilon) < 0.0001) {
            epsilon += 0.0002;
            return simplifyDouglasPeucker (polyline, epsilon);
         }
         
         newline = simplifyDouglasPeucker (polyline, epsilon);
         prevEpsilon = epsilon;
         numNewPoints = newline.size();
         shortestDistance = getMinDistance(newline);

         if ((numNewPoints-1 == maxSegments && shortestDistance > minDistance) ||
             (shortestDistance == minDistance && numNewPoints-1 < maxSegments)) {
            return simplifyDouglasPeucker (polyline, epsilon);
         }
         else if (numNewPoints-1 > maxSegments ||
                  shortestDistance < minDistance) {
            minEpsilon = epsilon;
         }
         else {
            maxEpsilon = epsilon;
         }
         if (iter > 100) {
            System.out.println ("Too Many Iters");
            return newline;
         }
      }
   }

  
}

