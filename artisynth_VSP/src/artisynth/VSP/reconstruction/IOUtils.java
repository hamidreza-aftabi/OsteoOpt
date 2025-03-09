package artisynth.VSP.reconstruction;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import artisynth.VSP.reconstruction.MeshManager.MarkerFormat;
import artisynth.VSP.reconstruction.MeshManager.PlaneFormat;
import artisynth.core.util.ArtisynthIO;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.util.ReaderTokenizer;

/**
 * Utilities for reading and writing certain reconstruction objects.
 */
public class IOUtils {

   // hook in case we need to translate marker and plane inputs
   //static Point3d myPosOffset = new Point3d(0.5810, -0.1080, -2.4350);
   static Point3d myPosOffset = null;

   public static RigidTransform3d readResectionPlane (
      ReaderTokenizer rtok, MeshManager.PlaneFormat fmt) throws IOException {

      RigidTransform3d TPW = new RigidTransform3d();
      if (fmt == MeshManager.PlaneFormat.BLENDER) {
         AxisAngle axisAng = new AxisAngle();
         axisAng.axis.x = rtok.scanNumber();
         axisAng.axis.y = rtok.scanNumber();
         axisAng.axis.z = rtok.scanNumber();
         axisAng.angle = Math.toRadians(rtok.scanNumber());
         TPW.setRotation (axisAng);
      }
      else {
         Vector3d nrm = new Vector3d();
         nrm.x = rtok.scanNumber();
         nrm.y = rtok.scanNumber();
         nrm.z = rtok.scanNumber();
         TPW.R.setZDirection (nrm);
      }
      TPW.p.x = rtok.scanNumber();
      TPW.p.y = rtok.scanNumber();      
      TPW.p.z = rtok.scanNumber();
      return TPW;
   }

   public static RigidTransform3d[] readResectionPlanes (
      File file, MeshManager.PlaneFormat fmt, Vector3d mandibleCentroid)       
      throws IOException {
      
      ArrayList<RigidTransform3d> planes = new ArrayList<>();
      ReaderTokenizer rtok = null;
      try {
         rtok = ArtisynthIO.newReaderTokenizer (file);
         while (rtok.nextToken() == ReaderTokenizer.TT_NUMBER) {
            rtok.pushBack();
            planes.add (readResectionPlane (rtok, fmt));
         }
      }
      catch (IOException e) {
         throw e;
      }
      finally {
         if (rtok != null) {
            rtok.close();
         }
      }
      if (fmt == MeshManager.PlaneFormat.SLICER) {
         for (RigidTransform3d TPW : planes) {
            TPW.p.sub (mandibleCentroid);
         }
      }
      if (myPosOffset != null) {
         for (RigidTransform3d TPW : planes) {
            TPW.p.add (myPosOffset);
         }
      }
      return planes.toArray(new RigidTransform3d[0]);
   }

   public static void writePlane (
      PrintWriter pw, RigidTransform3d plane,
      MeshManager.PlaneFormat fmt, Vector3d mandibleCentroid) {
      
      Point3d pnt = new Point3d (plane.p);
      if (fmt == MeshManager.PlaneFormat.SLICER) {
         pnt.add (mandibleCentroid);
      }
      if (fmt == MeshManager.PlaneFormat.BLENDER) {
         AxisAngle axisAng = new AxisAngle();
         plane.R.getAxisAngle (axisAng);
         pw.println (axisAng.axis + " " + Math.toDegrees(axisAng.angle));
         pw.println (pnt);
      }
      else {
         Vector3d zdir = new Vector3d();
         plane.R.getColumn (2, zdir);
         pw.println (zdir);
         pw.println (pnt);
      }
   }

   public static void writePlanes (
      File file, RigidTransform3d[] planes, 
      MeshManager.PlaneFormat fmt, Vector3d mandibleCentroid) throws IOException {

      PrintWriter pw = null;
      try {
         pw = ArtisynthIO.newIndentingPrintWriter(file);
         for (int i=0; i<planes.length; i++) {
            writePlane (pw, planes[i], fmt, mandibleCentroid);
         }
      }
      catch (IOException e) {
         throw e;
      }
      finally {
         pw.close();
      }
   }

   public static ArrayList<Point3d> readMarkerPoints (
      File file, MeshManager.MarkerFormat fmt, Vector3d mandibleCentroid)       
      throws IOException {
      
      ArrayList<Point3d> points = new ArrayList<>();
      ReaderTokenizer rtok = null;
      try {
         rtok = ArtisynthIO.newReaderTokenizer (file);
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            rtok.pushBack();
            Point3d pnt = new Point3d();
            pnt.x = rtok.scanNumber();
            pnt.y = rtok.scanNumber();
            pnt.z = rtok.scanNumber();
            if (fmt == MeshManager.MarkerFormat.SLICER) {
               pnt.sub (mandibleCentroid);
            }
            if (myPosOffset != null) {
               pnt.add (myPosOffset);
            }
            points.add (pnt);
         }
      }
      catch (IOException e) {
         throw e;
      }
      finally {
         if (rtok != null) {
            rtok.close();
         }
      }
      return points;
   }

   public static void writeMarkerPoints (
      File file, ArrayList<Point3d> points,
      MeshManager.MarkerFormat fmt, Vector3d mandibleCentroid) 
      throws IOException {

      PrintWriter pw = null;
      try {
         pw = ArtisynthIO.newIndentingPrintWriter(file);
         Point3d pos = new Point3d();
         for (int i=0; i<points.size(); i++) {
            pos.set (points.get(i));
            if (fmt == MeshManager.MarkerFormat.SLICER) {
               pos.add (mandibleCentroid);
            }
            pw.println (pos);
         }
      }
      catch (IOException e) {
         throw e;
      }
      finally {
         pw.close();
      }
   }
}
