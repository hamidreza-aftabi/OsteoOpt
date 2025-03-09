package artisynth.VSP.reconstruction;

import java.awt.Color;

import java.io.PrintWriter;
import java.io.IOException;
import java.util.Deque;
import java.util.ArrayList;

import artisynth.core.util.ScanToken;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import maspack.matrix.Vector3d;
import maspack.util.ReaderTokenizer;
import maspack.util.NumberFormat;

import maspack.matrix.*;
import maspack.util.*;
import maspack.geometry.*;
import maspack.properties.*;
import maspack.render.*;
import maspack.render.Renderer.*;

import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;

public class OcclusalPlane extends RigidBody {

   private static double INF = Double.POSITIVE_INFINITY;

   PointList<Point> myMarkers;
   Point3d[] myLocs = new Point3d[3];
   RigidTransform3d myTMW = new RigidTransform3d();

   boolean myMaskUpdateMarkers = false;

   public static double DEFAULT_MESH_WIDTH = 100;
   protected double myMeshWidth = DEFAULT_MESH_WIDTH;

   public static double DEFAULT_MESH_HEIGHT = 100;
   protected double myMeshHeight = DEFAULT_MESH_HEIGHT;

   public static PropertyList myProps =
      new PropertyList (OcclusalPlane.class, Frame.class);

   static {
      myProps.remove ("velocity");
      myProps.remove ("targetPosition");
      myProps.remove ("targetOrientation");
      myProps.remove ("targetVelocity");
      myProps.remove ("targetActivity");
      myProps.remove ("force");
      myProps.remove ("transForce");
      myProps.remove ("moment");
      myProps.remove ("externalForce");
      myProps.remove ("frameDamping");
      myProps.remove ("rotaryDamping");
      myProps.add (
         "meshWidth", "width of the mesh along the x axis",
         DEFAULT_MESH_WIDTH);
      myProps.add (
         "meshHeight", "height of the mesh along the y axis",
         DEFAULT_MESH_HEIGHT);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }  

   public double getMeshWidth() {
      return myMeshWidth;
   }

   public void setMeshWidth (double width) {
      myMeshWidth = width; 
      updateSurfaceMesh();
   }

   public double getMeshHeight() {
      return myMeshHeight;
   }

   public void setMeshHeight (double height) {
      myMeshHeight = height;
      updateSurfaceMesh(); 
   }

   public void setMeshSize (double w, double h) {
      setMeshWidth (w);
      setMeshHeight (h);
   }

   public void scaleMesh (double s) {
      setMeshSize (s*getMeshWidth(), s*getMeshHeight());
      updateSurfaceMesh();
   }

   protected void initializeChildComponents() {
      super.initializeChildComponents();
      myMarkers =
         new PointList<Point>(Point.class, "markers", "mkr");
      add (myMarkers);
   }

   public OcclusalPlane () {
      initializeChildComponents();
      myLocs[0] = new Point3d();
      myLocs[1] = new Point3d();
      myLocs[2] = new Point3d();
      setDynamic (false);
   }

   public OcclusalPlane (
      Point3d p0, Point3d p1, Point3d p2, double w, double h) {
      this();
      myMarkers.add (new Point(p0));
      myMarkers.add (new Point(p1));
      myMarkers.add (new Point(p2));
      RenderProps props = new RenderProps(myMarkers.getRenderProps());
      props.setAlpha (1.0);
      props.setPointColor (new Color (0.8f, 0, 0.8f));
      myMarkers.setRenderProps (props);
      updatePoseFromMarkers();
      setMeshSize (w, h);
   }

   public OcclusalPlane (Point3d p0, Point3d p1, Point3d p2) {
      this();
      myMarkers.add (new Point(p0));
      myMarkers.add (new Point(p1));
      myMarkers.add (new Point(p2));
      RenderProps props = new RenderProps(myMarkers.getRenderProps());
      props.setAlpha (1.0);
      props.setPointColor (new Color (0.8f, 0, 0.8f));
      myMarkers.setRenderProps (props);
      updatePoseFromMarkers();
   }

   void updateSurfaceMesh() {
      PolygonalMesh mesh = null;

      double w = getMeshWidth();
      double h = getMeshHeight();
      RigidTransform3d TPW = getPose();
      // Vector3d center = new Vector3d();
      // center.inverseTransform (TPW.R, TPW.p);
      // center.negate();
      mesh = MeshFactory.createCircularSector (w/2, Math.PI, 1, 30);
      mesh.scale (h/(w/2), 1.0, 0);
      mesh.transform (new RigidTransform3d (0, h/2, 0, -Math.PI/2, 0, 0));
      //mesh.transform (new RigidTransform3d (center.x, h/2+center.y, 0));
      //mesh.transform (new RigidTransform3d (0, h/2, 0));
      setSurfaceMesh (mesh);
   }

   public void componentChanged (ComponentChangeEvent e) {
      super.componentChanged(e);
      if (e instanceof GeometryChangeEvent &&
          e.getComponent() instanceof Point) {
         updatePoseFromMarkers();
      }
   }

   public void addTransformableDependencies (
      TransformGeometryContext context, int flags) {
   }

   @Override
   protected void updateSlavePosStates() {
      super.updateSlavePosStates();
      if (!myMaskUpdateMarkers && !isScanning()) {
         updateMarkersFromPose();         
      }
   } 

   private RigidTransform3d projectToZAxis (Vector3d nrm, Vector3d pnt) {
      RigidTransform3d TPW = new RigidTransform3d();
      Plane plane = new Plane (nrm, pnt);
      Point3d isect = new Point3d();
      plane.intersectLine (isect, Vector3d.Z_UNIT, Vector3d.ZERO);
      TPW.p.set (isect);
      TPW.R.setZDirection (nrm);
      return TPW;
   }

   private void projectPoseToZAxis () {
      RigidTransform3d TPW = getPose();
      Vector3d nrm = new Vector3d();
      TPW.R.getColumn (2, nrm);
      TPW = projectToZAxis (nrm, TPW.p);
      setPose (TPW);
   }

   public void transformGeometry (
      GeometryTransformer gtr, TransformGeometryContext context, int flags) {
      
      super.transformGeometry (gtr, context, flags);
      projectPoseToZAxis();
      updateMarkersFromPose();
      updateSurfaceMesh();
   }   

   protected void updatePoseFromMarkers() {
      Point3d cent = new Point3d();
      Point3d p0 = myMarkers.get(0).getPosition();
      Point3d p1 = myMarkers.get(1).getPosition();
      Point3d p2 = myMarkers.get(2).getPosition();
      // compute marker centroid
      cent.add (p0);
      cent.add (p1);
      cent.add (p2);
      cent.scale (1/3.0);

      Vector3d nrm = new Vector3d();
      Vector3d u01 = new Vector3d();
      Vector3d u02 = new Vector3d();
      u01.sub (p1, p0);
      u02.sub (p2, p0);
      nrm.cross (u01, u02);
      if (nrm.z < 0) {
         nrm.negate();
      }
      RigidTransform3d TPW = projectToZAxis (nrm, cent);
      Point3d pos = new Point3d();
      for (int i=0; i<3; i++) {
         myLocs[i].inverseTransform (TPW, myMarkers.get(i).getPosition());
      }
      updateSurfaceMesh();
      myMaskUpdateMarkers = true;
      setPose (TPW);
      myMaskUpdateMarkers = false;
   }

   protected void updateMarkersFromPose() {

      RigidTransform3d TPW = getPose();
      RigidTransform3d TMW = new RigidTransform3d(TPW);
      Vector3d zdir = new Vector3d();
      TPW.R.getColumn (2, zdir);
      // project TMW.p onto zdir
      //TMW.p.scale (TPW.p.dot(zdir), zdir);
      Point3d pos = new Point3d();
      for (int i=0; i<3; i++) {
         pos.transform (TMW, myLocs[i]);
         myMarkers.get(i).setPosition(pos);
         myLocs[i].inverseTransform (TPW, pos);
      }
   }

   public RenderProps createRenderProps() {
      // make the mesh visible from both sides and transparent
      RenderProps props = super.createRenderProps();
      props.setFaceStyle (FaceStyle.FRONT_AND_BACK);
      props.setFaceColor (new Color (0.6f, 0.6f, 1f));
      //props.setAlpha (0.6);
      return props;
   }

   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

      pw.println ("locs=[");
      IndentingPrintWriter.addIndentation (pw, 2);
      for (int i=0; i<3; i++) {
         myLocs[i].write (pw, fmt, /*withBrackets=*/false); 
         pw.println ("");
      }
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");
      super.writeItems (pw, fmt, ancestor);
   }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      if (scanAttributeName (rtok, "locs")) {
         rtok.scanToken ('[');
         for (int i=0; i<3; i++) {
            myLocs[i].scan (rtok);
         }
         rtok.scanToken (']');
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }
}
