package artisynth.VSP.reconstruction;

import java.io.IOException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;
import java.util.List;

import artisynth.core.util.ArtisynthIO;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;
import maspack.util.ReaderTokenizer;

/**
 * Support class for testing PolylineSimplifier.
 */
public class PolylineSimplifierTest {
   private static class Range {
      private Range(int first, int last) {
         this.first = first;
         this.last = last;
      }

      int first;
      int last;
   }


   private double getMinDistance(NumericList list) {
      double min = 1000;
      double distance = 0;
      NumericListKnot arrayOfKnots[] = new NumericListKnot[list.getNumKnots ()];
      Iterator<NumericListKnot> itr = list.iterator ();
      int j = 0;
      while (itr.hasNext ()) {
         arrayOfKnots[j] = itr.next ();
         j++;
      }
      for (int i=0; i < list.getNumKnots () - 1; i++) {
         distance = arrayOfKnots[i].v.distance(arrayOfKnots[i+1].v);
         if (distance < min) {
            min = distance;
         }
      }
      return min;
   }

   public double getSquareSegmentDistance(VectorNd v, VectorNd v2, VectorNd v3) {
      double x0, y0, z0, x1, y1, z1, x2, y2, z2, dx, dy, dz, t;

      x1 = v2.get (0);
      y1 = v2.get (1);
      z1 = v2.get (2);
      x2 = v3.get (0);
      y2 = v3.get (1);
      z2 = v3.get (2);
      x0 = v.get (0);
      y0 = v.get (1);
      z0 = v.get (2);

      dx = x2 - x1;
      dy = y2 - y1;
      dz = z2 - z1;

      if (dx != 0.0d || dy != 0.0d || dz != 0.0d) {
         t = ((x0 - x1) * dx + (y0 - y1) * dy + (z0 - z1) * dz)
            / (dx * dx + dy * dy + dz * dz);

         if (t > 1.0d) {
            x1 = x2;
            y1 = y2;
            z1 = z2;
         } else if (t > 0.0d) {
            x1 += dx * t;
            y1 += dy * t;
            z1 += dz * t;
         }
      }
      else {
         t = 0;
      }
      

      dx = x0 - x1;
      dy = y0 - y1;
      dz = z0 - z1;
      return dx * dx + dy * dy + dz * dz;
   }

   public void simplifyDouglasPeucker(
      NumericList oldList, double sqTolerance, NumericList newList) {
      int numberOfKnots = oldList.getNumKnots ();
      BitSet bitSet = new BitSet(numberOfKnots);
      bitSet.set(0);
      bitSet.set(numberOfKnots - 1);

     
      //Creating array of numericListKnots
      NumericListKnot arrayOfKnots[] = new NumericListKnot[numberOfKnots];
      Iterator<NumericListKnot> itr = oldList.iterator ();
      int j = 0;
      while (itr.hasNext ()) {
         arrayOfKnots[j] = itr.next ();
         j++;
      }
     
      List<Range> stack = new ArrayList<Range>();
      stack.add(new Range(0, oldList.getNumKnots () - 1));

      while (!stack.isEmpty()) {
         Range range = stack.remove(stack.size() - 1);

         int index = -1;
         double maxSqDist = 0f;

         // find index of point with maximum square distance from first and
         // last point
         for (int i = range.first + 1; i < range.last; ++i) {
            double sqDist = getSquareSegmentDistance(
               arrayOfKnots[i].v, arrayOfKnots[range.first].v,
               arrayOfKnots[range.last].v);

            if (sqDist > maxSqDist) {
               index = i;
               maxSqDist = sqDist;
            }
         }

         if (maxSqDist > sqTolerance) {
            bitSet.set(index);
            stack.add(new Range(range.first, index));
            stack.add(new Range(index, range.last));
         }
      }

      newList.clear ();
      for (int index = bitSet.nextSetBit(0);
           index >= 0;
           index = bitSet.nextSetBit(index+1)) {
         newList.add (arrayOfKnots[index].v, arrayOfKnots[index].t);
      }
   }

   public void bisectSimplifyDouglasPeucker (
      NumericList oldList, double minDistance,
      int maxSegments, NumericList newList) {
      double minEpsilon = 0;
      double maxEpsilon = 0;
      double prevEpsilon = 0;
      int iter = 0;
      int numNewPoints = 0;
      double tempEpsilon, epsilon, shortestDistance = 0;
      NumericList simplified;
      // Creating array of NumericKnots
      NumericListKnot arrayOfKnots[] =
         new NumericListKnot[oldList.getNumKnots ()];
      Iterator<NumericListKnot> itr = oldList.iterator ();
      int j = 0;
      while (itr.hasNext ()) {
         arrayOfKnots[j] = itr.next ();
         j++;
      }
      // Finding maximum epsilon in the beginning
      for (int i =0; i < oldList.getNumKnots () -2; i++ ) {
         tempEpsilon = getSquareSegmentDistance(
            arrayOfKnots[i+1].v, arrayOfKnots[0].v,
            arrayOfKnots[oldList.getNumKnots()-1].v);
         if (tempEpsilon > maxEpsilon) {
            maxEpsilon = tempEpsilon;
         }
      }
  
      while (true) {
         iter += 1;
         epsilon = (minEpsilon + maxEpsilon) / 2;
         if (Math.abs(epsilon - prevEpsilon) < 0.0001) {
            epsilon += 0.0002;
            simplifyDouglasPeucker(oldList, epsilon, newList);
            return;
         }
         
         simplifyDouglasPeucker(oldList, epsilon, newList);
         prevEpsilon = epsilon;
         numNewPoints = newList.getNumKnots ();
         shortestDistance = getMinDistance(newList);

         if ((numNewPoints-1 == maxSegments && shortestDistance > minDistance) ||
             (shortestDistance == minDistance && numNewPoints-1 < maxSegments)) {
            simplifyDouglasPeucker(oldList, epsilon, newList);
            return;
         }
         else if (numNewPoints - 1> maxSegments ||
                  shortestDistance < minDistance) {
            minEpsilon = epsilon;
         }
         else {
            maxEpsilon = epsilon;
         }
         if (iter > 100) {
            System.out.println ("Too Many Iters");
            return;
         }
      }
   }

   public static ArrayList<Point3d> numlistToPolyline (NumericList numlist) {
      ArrayList<Point3d> line = new ArrayList<>();
      Iterator<NumericListKnot> itr = numlist.iterator ();
      while (itr.hasNext ()) {
         line.add (new Point3d (itr.next().v));
      }
      return line;
   }

   public static NumericList polylineToNumlist (ArrayList<Point3d> polyline) {
      NumericList list = new NumericList(3);
      VectorNd vec3 = new VectorNd(3);
      for (int k=0; k<polyline.size(); k++) {
         vec3.set (polyline.get(k));
         list.add (vec3, k);
      }
      return list;      
   }

   public static void main (String[] args) throws IOException {
      ArrayList<Point3d> clippedLine = new ArrayList<>();
      ReaderTokenizer rtok = ArtisynthIO.newReaderTokenizer("clist.txt");
      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         rtok.pushBack();
         Point3d pnt = new Point3d();
         pnt.x = rtok.scanNumber();
         pnt.y = rtok.scanNumber();
         pnt.z = rtok.scanNumber();
         clippedLine.add (pnt);
      }
      rtok.close();

      ArrayList<Point3d> rdpLine =
         PolylineSimplifier.bisectSimplifyDouglasPeucker (clippedLine, 20, 5);
      System.out.println ("new simp: " + rdpLine.size());

      PolylineSimplifierTest nsimp = new PolylineSimplifierTest();
      NumericList clist = polylineToNumlist (clippedLine);
      NumericList rlist = new NumericList(3);
      nsimp.bisectSimplifyDouglasPeucker (clist, 20, 5, rlist);  
      ArrayList<Point3d> rdpLineChk = numlistToPolyline (rlist);
      System.out.println ("old simp: " + rdpLineChk.size());

      if (rdpLine.size() == rdpLineChk.size()) {
         for (int i=0; i<rdpLine.size(); i++) {
            if (!rdpLine.get(i).equals (rdpLineChk.get(i))) {
               System.out.println ("Not equal at i=" + i);
               break;
            }
         }
      }
      else {
         System.out.println ("Not equal");
      }

   }
  
}





