package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.mechmodels.MechSystemSolver.*;
import artisynth.core.renderables.*;
import maspack.collision.IntersectionContour;
import maspack.spatialmotion.*;
import maspack.geometry.ConvexPolygon2d;
import maspack.geometry.MeshFactory;
import maspack.geometry.Polygon2d;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.*;
import maspack.collision.*;
import maspack.geometry.Vertex2d;
import maspack.matrix.NumericalException;
import maspack.util.*;
import maspack.matrix.*;
import maspack.matrix.Point2d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector3i;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.*;
import maspack.util.PathFinder;

/**
 * Worker component for creating mandible cutting guides. At present, the guide
 * is not complete, but consists of two cut boxes that are aligned with the
 * left and right resection planes and that can then be exported to create and
 * more complete guide mesh within Blender.
 */
public class MandibleGuideBuilder extends WorkerComponentBase {

   private static double INF = Double.POSITIVE_INFINITY;

   IntersectionContour myContour;
   RigidTransform3d myTSW;

   class PolylinePnt {
      int idx;
      double s;

      PolylinePnt (int idx, double s) {
         this.idx = idx;
         this.s = s;
      }

      PolylinePnt (PolylinePnt pnt) {
         this.idx = pnt.idx;
         this.s = pnt.s;
      }

      void eval (Point3d pnt, Polyline line) {
         if (idx >= line.numVertices()-1) {
            pnt.set (line.getVertex (idx).getPosition());
         }
         else {
            Vertex3d vtx0 = line.getVertex(idx);
            Vertex3d vtx1 = line.getVertex(idx+1);
            pnt.combine (1-s, vtx0.getPosition(), s, vtx1.getPosition());
         }
      }
   }

   IntersectionContour myContourL;
   int myCurvePlaneIdxL; // plate curve index where it intersects left plane
   IntersectionContour myContourR;
   int myCurvePlaneIdxR; // plate curve index where it intersects right plane

   // offsets needed to center the blade guide mesh
   static double GUIDE_OFF_Y = 15.8503;
   static double GUIDE_OFF_Z = -1.5254;

   static Vector3i DEFAULT_BLADE_BOX_RES = new Vector3i (5, 20, 4);
   protected Vector3i myBladeBoxRes = new Vector3i(DEFAULT_BLADE_BOX_RES);

   static Vector3d DEFAULT_BLADE_BOX_WIDTHS =
      new Vector3d (9.5130, 40.1592, 6.3860);
   protected Vector3d myBladeBoxWidths = new Vector3d(DEFAULT_BLADE_BOX_WIDTHS);

   static double DEFAULT_MANDIBLE_CLEARANCE = 1.0;
   protected double myMandibleClearance = DEFAULT_MANDIBLE_CLEARANCE;

   static double DEFAULT_BOTTOM_MARGIN = 5.0;
   protected double myBottomMargin = DEFAULT_BOTTOM_MARGIN;

   static double DEFAULT_HANDLE_RADIUS = 5.0;
   protected double myHandleRadius = DEFAULT_HANDLE_RADIUS;

   static double DEFAULT_HANDLE_EXTENSION = 3.0;
   protected double myHandleExtension = DEFAULT_HANDLE_EXTENSION;

   static boolean DEFAULT_HANDLE_POINTS_VISIBLE = true;
   protected boolean myHandlePointsVisible = DEFAULT_HANDLE_POINTS_VISIBLE;

   public static PropertyList myProps =
      new PropertyList (MandibleGuideBuilder.class, WorkerComponentBase.class);

   static {
      myProps.add (
         "bladeBoxWidths",
         "x,y,z widths of the blade box base",
         DEFAULT_BLADE_BOX_WIDTHS);
      myProps.add (
         "bladeBoxRes",
         "x,y,z mesh resolutions of the blade box",
         DEFAULT_BLADE_BOX_RES);
      myProps.add (
         "mandibleClearance",
         "space between the mandible and the blade box",
         DEFAULT_MANDIBLE_CLEARANCE);
      myProps.add (
         "bottomMargin",
         "distance of the blade box below the mandible bottom",
         DEFAULT_BOTTOM_MARGIN);
      myProps.add (
         "handleRadius",
         "radius of the handle connecting the flanges",
         DEFAULT_HANDLE_RADIUS);
      myProps.add (
         "handleExtension",
         "extension of the handle beyond its end points",
         DEFAULT_HANDLE_EXTENSION);
      myProps.add (
         "handlePointsVisible",
         "handle control points are visible",
         DEFAULT_HANDLE_POINTS_VISIBLE);
      myProps.add (
         "allPartsVisible",
         "all mandible guide parts are visible",
         true, "NW");
      myProps.add (
         "guideVisible isGuideVisible",
         "completed mandible guide is visible",
         true, "NW");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   PlateBuilder getPlateBuilder() {
      return ((MandibleRecon)getRoot()).getPlateBuilder();
   }

   double getScrewSpacing() {
      return getPlateBuilder().getScrewSpacing();
   }

   public MandibleGuideBuilder () {
      setRenderProps (defaultRenderProps(this));
   }

   public MandibleGuideBuilder (String name) {
      this();
      setName (name);
   }

   public Vector3d getBladeBoxWidths () {
      return myBladeBoxWidths;
   }

   public void setBladeBoxWidths (Vector3d widths) {
      myBladeBoxWidths.set (widths);
   }

   public Vector3i getBladeBoxRes () {
      return myBladeBoxRes;
   }

   public void setBladeBoxRes (Vector3i res) {
      myBladeBoxRes.set (res);   }
   
   public double getMandibleClearance () {
      return myMandibleClearance;
   }

   public void setMandibleClearance (double spacing) {
      myMandibleClearance = spacing;
   }
   
   public double getBottomMargin () {
      return myBottomMargin;
   }

   public void setBottomMargin (double m) {
      myBottomMargin = m;
   }
   
   public double getHandleRadius () {
      return myHandleRadius;
   }

   public void setHandleRadius (double r) {
      if (r != myHandleRadius) {
         myHandleRadius = r;
         updateHandle();
      }
   }

   public double getHandleExtension () {
      return myHandleExtension;
   }

   public void setHandleExtension (double ext) {
      if (myHandleExtension != ext) {
         myHandleExtension = ext;
         updateHandle();
      }
   }
   
   public boolean hasCutBoxes() {
      return (getRigidBody ("mandibleCutBoxL") != null &&
              getRigidBody ("mandibleCutBoxR") != null);
   }

   public boolean areCutBoxesVisible() {
      return (isVisible (getRigidBody ("mandibleCutBoxL")) &&
              isVisible (getRigidBody ("mandibleCutBoxR")));
   }

   public boolean setCutBoxesVisible (boolean enable) {
      boolean changed = false;
      changed |= setVisible (getRigidBody ("mandibleCutBoxL"), enable);
      changed |= setVisible (getRigidBody ("mandibleCutBoxR"), enable);
      return changed;
   }

   public boolean clearCutBoxes() {
      RigidBody bodyL = getRigidBody ("mandibleCutBoxL");
      RigidBody bodyR = getRigidBody ("mandibleCutBoxR");
      RigidBody expMandible = getRigidBody ("expandedMandible");
      if (bodyL != null) {
         removeRigidBody ("mandibleCutBoxL");
      }
      if (bodyR != null) {
         removeRigidBody ("mandibleCutBoxR");
      }
      return bodyL != null || bodyR != null;
   }

   private RigidBody addCutBoxPart (
      String name, PolygonalMesh bladeBox, PolygonalMesh bladeInset,
      RigidTransform3d TPW) {

      RigidBody body = setRigidBody (name, bladeBox.clone());
      body.addMesh (bladeInset.clone(), /*hasMass=*/false, /*collidable=*/false);
      body.setPose (TPW);
      //body.setDynamic (true);
      body.setInertialDamping (10);
      RenderProps.setFaceColor (body, ReconAppRoot.GUIDE_COLOR);
      return body;
   }

   public PolygonalMesh getCutBoxesMesh () {
      RigidBody bodyL = getRigidBody ("mandibleCutBoxL");
      RigidBody bodyR = getRigidBody ("mandibleCutBoxR");
      PolygonalMesh mesh = new PolygonalMesh();
      if (bodyL != null && bodyR != null) {
         for (PolygonalMesh m : bodyL.getSurfaceMeshes()) {
            mesh.addMesh (m, /*respectTransforms=*/true);
         }
         for (PolygonalMesh m : bodyR.getSurfaceMeshes()) {
            mesh.addMesh (m, /*respectTransforms=*/true);
         }
      }
      return mesh;
   }

   private PolygonalMesh createBladeBox () {
      Vector3i res = myBladeBoxRes;
      Vector3d widths = myBladeBoxWidths;

      return MeshFactory.createBox (
         widths.x, widths.y, widths.z, Point3d.ZERO, res.z, res.y, res.z);      
   }

   private PolygonalMesh createBladeInset() throws IOException {
      Vector3i res = myBladeBoxRes;
      Vector3d widths = myBladeBoxWidths;

      String meshPath =
         PathFinder.getSourceRelativePath (
            this, "geometry/bladeGuideUnion30.stl");
      PolygonalMesh mesh = new PolygonalMesh (meshPath);
      // need to flip the blade inset about z since y axis now points up
      mesh.transform (
         new RigidTransform3d (0, GUIDE_OFF_Y, GUIDE_OFF_Z, Math.PI, 0, 0));
         //new RigidTransform3d (0, GUIDE_OFF_Y, GUIDE_OFF_Z));
      return mesh;
   }

   public void createCutBoxes (
      PolygonalMesh mandibleMesh, RigidTransform3d[] TPWs,
      PolylineMesh plateCurve) throws IOException {

      clearCutBoxes();
      clearFlanges();
      clearGuide();

      // find the points on the plate curve that intersect the left and right
      // cut planes.
      Point3d leftPnt = new Point3d();
      myCurvePlaneIdxL = intersectPlaneWithPlateCurve (
         leftPnt, new Plane(TPWs[0]), plateCurve);
      
      Point3d rightPnt = new Point3d();
      myCurvePlaneIdxR = intersectPlaneWithPlateCurve (
         rightPnt, new Plane(TPWs[1]), plateCurve);

      // use the distance of these from the origin to infer the amount we need
      // to scale the mandible to provide the requested clearance. We scale the
      // mandible, rather than growing it, because growing can cause self
      // intersections in the mesh, particularly if it has inside voids.
      double c = getMandibleClearance();
      double s = (2+c/leftPnt.norm()+c/rightPnt.norm())/2;
      PolygonalMesh expandedMesh = mandibleMesh.clone();
      expandedMesh.scale (s);
      // Growing mesh - don't do this because it creates self-intersections:
      // PolygonalMesh expandedMesh =
      //    MeshFactory.extrudeAlongVertexNormals (
      //       mandibleMesh, getMandibleClearance());
      RigidBody expMandible = setRigidBody ("expandedMandible", expandedMesh);

      PolygonalMesh bladeInset = createBladeInset();
      PolygonalMesh bladeBox = createBladeBox();

      RigidTransform3d TLW = findCutBoxPose (
         expandedMesh, TPWs[0], plateCurve, /*rightPlane=*/false);
      bladeBox.setMeshToWorld (TLW);
      bladeInset.setMeshToWorld (TLW);
      RigidBody guideL = addCutBoxPart (
         "mandibleCutBoxL", bladeBox, bladeInset, TLW);

      RigidTransform3d TRW = findCutBoxPose (
         expandedMesh, TPWs[1], plateCurve, /*rightPlane=*/true);
      bladeBox.setMeshToWorld (TRW);
      bladeInset.setMeshToWorld (TRW);
      RigidBody guideR = addCutBoxPart (
         "mandibleCutBoxR", bladeBox, bladeInset, TRW);
   }

   IntersectionContour findCrossSection (
      RigidTransform3d TSW, double[] yzbounds,
      PolylineMesh plateCurve, int idx, RotationMatrix3d RLW) {

      Vector3d xdir = estimateCurveDirection (plateCurve, idx, /*winLen=*/20.0);
      xdir.negate();
      TSW.p.set (plateCurve.getVertex(idx).getPosition());

      Vector3d ydir = new Vector3d();
      RLW.getColumn (1, ydir);
      TSW.R.setXYDirections (xdir, ydir);

      RigidTransform3d TPW = new RigidTransform3d(TSW);
      TPW.R.mulRotY (-Math.PI/2);

      PolygonalMesh expandedMesh = getRigidBodyMesh ("expandedMandible");
      PolygonalMesh cutPlane = createCutPlaneMesh();
      cutPlane.setMeshToWorld (TPW);
      IntersectionContour contour =
         ComputeUtils.findPrimaryIsectContour (expandedMesh, cutPlane);

      // estimate bounds. x/y correspond to z/y in the lattice
      double ymin = INF;
      double ymax = -INF;
      double zmin = INF;
      double zmax = -INF;
      Point3d pc = new Point3d();
      for (Point3d p : contour) {
         pc.inverseTransform (TPW, p);
         if (pc.y < getScrewSpacing()) {
            // check bounds over lower part of contour
            if (pc.y > ymax) {
               ymax = pc.y;
            }
            if (pc.y < ymin) {
               ymin = pc.y;
               zmin = pc.x;
            }
            if (pc.x > zmax) {
               zmax = pc.x;
            }
         }
      }
      yzbounds[0] = ymin;
      yzbounds[1] = ymax;
      yzbounds[2] = zmin;
      yzbounds[3] = zmax;
      return contour;
   }

   int intersectPlaneWithPlateCurve (
      Point3d linePnt, Plane plane, PolylineMesh plateCurve) {

      // find the plane intersection point on the plateCurve
      for (int i=0; i<plateCurve.numVertices()-1; i++) {
         Point3d p0 = plateCurve.getVertex(i).getPosition();
         Point3d p1 = plateCurve.getVertex(i+1).getPosition();
         double d0 = plane.distance (p0);
         double d1 = plane.distance (p1);
         if (d0*d1 <= 0) {
            // crossing found
            d0 = Math.abs(d0);
            d1 = Math.abs(d1);
            double s = d0/(d0+d1);
            linePnt.combine (1-s, p0, s, p1);
            return i;
         }
      }
      throw new NumericalException (
         "plate curve does not intersect resection plane");
   }

   RigidTransform3d findCutBoxPose (
      PolygonalMesh expandedMesh, RigidTransform3d TPW,
      PolylineMesh plateCurve, boolean rightPlane) {

      RigidTransform3d TCBW = new RigidTransform3d(); // cut box to world

      Plane plane = new Plane (TPW);
      Point3d linePnt = new Point3d();
      intersectPlaneWithPlateCurve (linePnt, plane, plateCurve);

      // estimate the surface normal at the line point and project it to the
      // plane
      Vector3d nrm = ComputeUtils.estimateSurfaceNormal (linePnt, expandedMesh);
      nrm.scaledAdd (-nrm.dot(plane.getNormal()), plane.getNormal()); 
      nrm.normalize();

      // find interesection contour between the mandible and resection plane
      PolygonalMesh cutPlane = createCutPlaneMesh();
      cutPlane.setMeshToWorld (TPW);
      IntersectionContour contour =
         ComputeUtils.findPrimaryIsectContour (expandedMesh, cutPlane);
      if (contour == null) {
         throw new NumericalException (
            "No intersection contour between mandible and resection plane");
      }
      if (!rightPlane) {
         myContourR = contour;
      }
      else {
         myContourL = contour;
      }

      // transform the contour into 2d in the plane and take its convex hull
      ArrayList<Point2d> points2d = new ArrayList<>();
      Point3d pp = new Point3d(); // point in plane coords
      for (Point3d p : contour) {
         pp.inverseTransform (TPW, p);
         points2d.add (new Point2d (pp.x, pp.y));
      }
      ConvexPolygon2d cpoly = Polygon2d.simpleConvexHull (points2d);
      // find the nearest edge on this convex hull to linePnt, and use this to
      // determine the y direction for the cut box.
      pp.inverseTransform (TPW, linePnt);
      Point2d linePnt2d = new Point2d (pp.x, pp.y);
      Point2d near2d = new Point2d();
      Vertex2d evtx = cpoly.nearestEdge (near2d, linePnt2d);
      Vector2d edir = evtx.getEdge();
      edir.normalize();
      Vector3d ydir = new Vector3d (edir.x, edir.y, 0);
      ydir.transform (TPW);

      // x direction is given by the plane normal or its negative
      Vector3d xdir = new Vector3d ();
      TPW.R.getColumn (2, xdir);
      if (!rightPlane) {
         xdir.negate();
      }
      else {
         ydir.negate();
      }
      TCBW.R.setXYDirections (xdir, ydir);

      // For the origin, start by setting it to the nearest point to the
      // linePnt on the convex hull.
      Point3d origin = new Point3d(near2d.x, near2d.y, 0);
      origin.transform (TPW);
      // Shift it along z to clear the blade box from the mandible:
      Vector3d zdir = new Vector3d();
      TCBW.R.getColumn (2, zdir);
      origin.scaledAdd (myBladeBoxWidths.z/2 + myMandibleClearance, zdir);
      // Find the bounds of the contour, wrt to origin, in the y direction.
      double ymin = INF;
      double ymax = -INF;
      Vector3d delp = new Vector3d();
      for (Point3d p : contour) {
         delp.sub (p, origin);
         double y = delp.dot (ydir);
         if (y < ymin) {
            ymin = y;
         }
         if (y > ymax) {
            ymax = y;
         }
      }
      // shift the origin along y so that the blade box is bottomMargin below
      // the maximum y bounds.
      origin.scaledAdd (ymin-myBottomMargin+myBladeBoxWidths.y/2, ydir);
      TCBW.p.set (origin);

      return TCBW;
   }

   // renderable interface

   private void renderContour (Renderer renderer, IntersectionContour contour) {
      if (contour != null) {
         renderer.setLineWidth (3);
         renderer.setColor (Color.RED);         
         renderer.beginDraw (DrawMode.LINE_LOOP);
         for (Point3d p : contour) {
            renderer.addVertex (p);
         }
         renderer.endDraw();
      }
   }                             

   public void render (Renderer renderer, int flags) {
      renderContour (renderer, myContour);
      if (myTSW != null) {
         renderer.drawAxes (
            myTSW, AxisDrawStyle.ARROW, 
            10, 2, 0, false);
      }
      //renderContour (renderer, myContourR);
   }

   public void alignToMandible() {
      RigidBody guideL = getRigidBody ("mandibleCutBoxL");
      RigidBody guideR = getRigidBody ("mandibleCutBoxR");
      RigidBody expMandible = getRigidBody ("expandedMandible");

      // need to set default collison parameters because the model components
      // will likely not have been in place when the model was first
      // initialized at load time.
      myMech.setPenetrationTol (-1);
      CollisionManager cm = myMech.getCollisionManager();
      cm.setRigidRegionTol (-1);
      cm.setRigidPointTol (-1);
      cm.setContactNormalLen (-1);

      guideL.setDynamic(true);
      guideR.setDynamic(true);
      myMech.setCollisionBehavior (guideL, expMandible, true, 0.3);
      myMech.setCollisionBehavior (guideR, expMandible, true, 0.3);

      // add planar joints to the mechmodel to ensure that the boxes move only
      // in their cutplanes. Constraint planes should be aligned with x
      // axes of the boxes (so rotate by PI/2 about y).
      RigidTransform3d TDW = new RigidTransform3d();
      TDW.set (guideL.getPose());
      TDW.mulRotY (Math.PI/2);
      PlanarJoint constraintL = new PlanarJoint (guideL, TDW);
      TDW.set (guideR.getPose());
      TDW.mulRotY (Math.PI/2);
      PlanarJoint constraintR = new PlanarJoint (guideR, TDW);

      myMech.addBodyConnector (constraintL);
      myMech.addBodyConnector (constraintR);

      Integrator savedIntegrator = myMech.getIntegrator();
      myMech.setIntegrator (Integrator.ConstrainedBackwardEuler);

      guideL.setVelocity (new Twist());
      guideR.setVelocity (new Twist());

      getPlateBuilder().setPlateFemDynamic (false);

      int nsteps = 10;
      double t0 = 0;
      for (int i=0; i<nsteps; i++) {
         double t1 = t0 + 0.01;
         Vector3d zforce = new Vector3d();
         guideL.getPose().R.getColumn (2, zforce);
         zforce.scale (-5);
         guideL.setExternalForce (new Wrench (zforce, Vector3d.ZERO));
         
         guideR.getPose().R.getColumn (2, zforce);
         zforce.scale (-5);
         guideR.setExternalForce (new Wrench (zforce, Vector3d.ZERO));
         
         myMech.preadvance (t0, t1, 0);
         myMech.advance (t0, t1, 0);
         t0 = t1;
      }

      getPlateBuilder().setPlateFemDynamic (true);

      guideL.zeroExternalForces();
      guideR.zeroExternalForces();

      myMech.removeBodyConnector (constraintL);
      myMech.removeBodyConnector (constraintR);

      myMech.clearCollisionBehavior (guideL, expMandible);
      myMech.clearCollisionBehavior (guideR, expMandible);
      guideL.setDynamic(false);
      guideR.setDynamic(false);

      myMech.setIntegrator (savedIntegrator);
   }

   /**
    * Given a starting vertex v0 located at index {@code idx0} on a curve,
    * advance along the curve to locate the index of a vertex that is {@code
    * dist} away from {@code v0}. The advance occurs in a positive index
    * direction if {@code dist > 0} and a negative index direction otherwise.
    */
   int advanceAlongCurve (PolylineMesh curve, int idx0, double dist) {
      Vertex3d v0 = curve.getVertex(idx0);
      int idx = idx0;
      if (dist > 0) {
         while (idx < curve.numVertices()-1) {
            if (curve.getVertex(idx).distance(v0) > dist) {
               break;
            }
            idx++;            
         }
      }
      else {
         while (idx > 0) {
            if (curve.getVertex(idx).distance(v0) > -dist) {
               break;
            }
            idx--;
         }
      }
      return idx;
   }

   /**
    * Estimate the curve direction for at the vertex at index {@code idx},
    * based on a window of length {@code winLen}.
    */
   Vector3d estimateCurveDirection (PolylineMesh curve, int idx, double winLen) {
      int idx0 = advanceAlongCurve (curve, idx, -winLen/2);
      int idx1 = advanceAlongCurve (curve, idx, winLen/2);

      Vector3d dir = new Vector3d();
      Point3d pprev = curve.getVertex(idx0).getPosition();
      for (int k=idx0+1; k<=idx1; k++) {
         Point3d p = curve.getVertex(k).getPosition();
         Vector3d u = new Vector3d();
         u.sub (p, pprev);
         u.normalize();
         dir.add (u);
         pprev = p;
      }
      dir.scale (1.0/(idx1-idx0));
      return dir;
   }

   void fitLatticeVertices (
      SubdivisionMesh sdmesh, PolylineMesh plateCurve,
      int idx, boolean left) {

      double bbwx = myBladeBoxWidths.x;
      double screwSpacing = getScrewSpacing();

      int idxL, idxR;
      if (left) {
         idxL = advanceAlongCurve (plateCurve, idx, -bbwx/2-2*screwSpacing);
         idxR = advanceAlongCurve (plateCurve, idx, 3*bbwx/2);
      }
      else {
         idxR = advanceAlongCurve (plateCurve, idx, bbwx/2+2*screwSpacing);
         idxL = advanceAlongCurve (plateCurve, idx, -3*bbwx/2);
      }

      RigidTransform3d TSW = new RigidTransform3d();
      double[] yzbounds = new double[4];

      Point3d vpnt = new Point3d();
      double bbwz = myBladeBoxWidths.z;
      VertexList<VertexComponent> vtxs =
         sdmesh.getLattice().getVertexComponents();         
      RigidTransform3d TLW = sdmesh.getPose();

      findCrossSection (TSW, yzbounds, plateCurve, idxL, TLW.R);
      //myTSW = new RigidTransform3d (TSW);

      double ymin = yzbounds[0];
      double ymax = yzbounds[1];
      double zmin = yzbounds[2] - myMandibleClearance;
      double zmax = yzbounds[3];

      double ytop = 0.75*screwSpacing;

      // vertex 8 
      vpnt.set (0, ytop, zmin);
      vpnt.transform (TSW);
      vtxs.get(8).setWorldPosition (vpnt);
      // vertex 9
      vpnt.set (0, ymin-myBottomMargin, zmin);
      vpnt.transform (TSW);
      vtxs.get(9).setWorldPosition (vpnt);
      // vertex 10
      vpnt.set (0, ytop, zmax+bbwz/2);
      vpnt.transform (TSW);
      vtxs.get(10).setWorldPosition (vpnt);
      // vertex 11
      vpnt.set (0, ymin, zmax+bbwz/2);
      vpnt.transform (TSW);
      vtxs.get(11).setWorldPosition (vpnt);

      findCrossSection (TSW, yzbounds, plateCurve, idxR, TLW.R);

      ymin = yzbounds[0];
      ymax = yzbounds[1];
      zmin = yzbounds[2] - myMandibleClearance;
      zmax = yzbounds[3];

      // vertex 1
      vpnt.set (0, ytop, zmin);
      vpnt.transform (TSW);
      vtxs.get(1).setWorldPosition (vpnt);
      // vertex 0
      vpnt.set (0, ymin-myBottomMargin, zmin);
      vpnt.transform (TSW);
      vtxs.get(0).setWorldPosition (vpnt);
      // vertex 5
      vpnt.set (0, ytop, zmax+bbwz/2);
      vpnt.transform (TSW);
      vtxs.get(5).setWorldPosition (vpnt);
      // vertex 4
      vpnt.set (0, ymin, zmax+bbwz/2);
      vpnt.transform (TSW);
      vtxs.get(4).setWorldPosition (vpnt);

      findCrossSection (TSW, yzbounds, plateCurve, idx, TLW.R);

      ymin = yzbounds[0];
      ymax = yzbounds[1];
      zmin = yzbounds[2] - myMandibleClearance;
      zmax = yzbounds[3];

      // vertex 2
      vpnt.set (0, ytop, zmin);
      vpnt.transform (TSW);
      vtxs.get(2).setWorldPosition (vpnt);
      // vertex 3
      vpnt.set (0, ymin-myBottomMargin, zmin);
      vpnt.transform (TSW);
      vtxs.get(3).setWorldPosition (vpnt);

      sdmesh.updateSmoothedMesh();
   }

   public void addFlangeBox (
      String name, RigidBody guide, IntersectionContour contour,
      PolylineMesh plateCurve, int curveIdx, boolean left) {

      Vector3d zdir = new Vector3d();
      RigidTransform3d TBW = guide.getPose();
      TBW.R.getColumn (2, zdir);

      // Find the bounds of the contour, wrt to origin, in the z direction.
      double zmin = INF;
      double zmax = -INF;
      Vector3d delp = new Vector3d();
      for (Point3d p : contour) {
         delp.sub (p, TBW.p);
         double z = delp.dot (zdir);
         if (z < zmin) {
            zmin = z;
         }
         if (z > zmax) {
            zmax = z;
         }
      }
      double zdepth = zmax-zmin;

      // find plate curve point and transform it into box coodinates
      // to find height of the flange box
      Point3d curvePnt =
         new Point3d(plateCurve.getVertex(curveIdx).getPosition());
      curvePnt.inverseTransform (TBW);
      double bbwx = myBladeBoxWidths.x;
      double bbwy = myBladeBoxWidths.y;
      double bbwz = myBladeBoxWidths.z;

      double ytop = 0.75*getScrewSpacing();

      double wx = 2*(bbwx + getScrewSpacing());
      double wy = bbwy/2+curvePnt.y+ytop;
      double wz = bbwz + zdepth;

      PolygonalMesh mesh = MeshFactory.createQuadBox (
         wx, wy, wz, new Point3d(), 2, 1, 1);
      SubdivisionMesh sdmesh = new SubdivisionMesh (mesh);
      sdmesh.setName (name);
      sdmesh.getLattice().getFaceList().setSelectable(false);
      myMech.addRenderable (sdmesh);

      RigidTransform3d TLW = new RigidTransform3d(guide.getPose());
      if (left) {
         TLW.mulXyz (-3*bbwx/2+wx/2, -bbwy/2+wy/2, bbwz/2-wz/2);
      }
      else {
         TLW.mulXyz (3*bbwx/2-wx/2, -bbwy/2+wy/2, bbwz/2-wz/2);
      }
      sdmesh.setPose (TLW);

      fitLatticeVertices (sdmesh, plateCurve, curveIdx, left);

      RenderProps.setSphericalPoints (sdmesh.getLattice(), 1.0, Color.CYAN);
      RenderProps.setFaceColor (sdmesh, ReconAppRoot.GUIDE_COLOR);
      //RenderProps.setDrawEdges (sdmesh, true);
      //RenderProps.setEdgeColor (sdmesh, Color.BLUE);
      RenderProps.setShading (sdmesh.getLattice().getFaceList(), Shading.NONE);
   }

   SubdivisionMesh getFlangeL() {
      return (SubdivisionMesh)myMech.renderables().get ("mandibleFlangeL");
   }

   SubdivisionMesh getFlangeR() {
      return (SubdivisionMesh)myMech.renderables().get ("mandibleFlangeR");
   }

   public void addHandle() {

      SubdivisionMesh sdmeshL = getFlangeL();
      SubdivisionMesh sdmeshR = getFlangeR();

      Vector3d zdirR = new Vector3d();
      sdmeshR.getPose().R.getColumn (2, zdirR);
      Point3d p0 = new Point3d();
      p0.add (sdmeshR.getLatticeVertex(10).getWorldPosition());
      p0.add (sdmeshR.getLatticeVertex(11).getWorldPosition());
      p0.scale (0.5);

      Vector3d zdirL = new Vector3d();
      sdmeshL.getPose().R.getColumn (2, zdirL);
      Point3d p2 = new Point3d();      
      p2.add (sdmeshL.getLatticeVertex(5).getWorldPosition());
      p2.add (sdmeshL.getLatticeVertex(4).getWorldPosition());
      p2.scale (0.5);

      // move p0 and p2 inward on their lattices
      Vector3d u02 = new Vector3d();
      u02.sub (p2, p0);
      u02.normalize();
      p0.scaledAdd (-myHandleRadius, u02);
      p2.scaledAdd (myHandleRadius, u02);

      Vector3d zdir = new Vector3d();
      zdir.add (zdirL, zdirR);
      zdir.normalize();
      Point3d p1 = new Point3d();      
      p1.add (p0, p2);
      p1.scale (0.5);
      p1.scaledAdd (p0.distance(p2)/2, zdir);

      addHandlePoints (p0, p1, p2);
      updateHandle();
   }

   private double sqr (double x) {
      return x*x;
   }

   RigidTransform3d computePointCenter () {
      Point3d p0 = new Point3d(getHandlePoints().get(0).getPosition());
      Point3d p1 = new Point3d(getHandlePoints().get(1).getPosition());
      Point3d p2 = new Point3d(getHandlePoints().get(2).getPosition());

      Vector3d v01 = new Vector3d();
      v01.sub (p1, p0);
      Vector3d v02 = new Vector3d();
      v02.sub (p2, p0);

      Vector3d zdir = new Vector3d();
      zdir.cross (v01, v02);
      // XXX test for colinearity
      zdir.normalize();

      RigidTransform3d TPW = new RigidTransform3d();
      TPW.p.set (p0);
      TPW.R.setZDirection (zdir);

      // find circle points in the plane
      Point3d pp = new Point3d();
      pp.inverseTransform (TPW, p0);
      double x0 = pp.x;
      double y0 = pp.y;
      pp.inverseTransform (TPW, p1);
      double x1 = pp.x;
      double y1 = pp.y;
      pp.inverseTransform (TPW, p2);
      double x2 = pp.x;
      double y2 = pp.y;

      // linear eqn to find the center
      double a1 = 2*(x1-x0);
      double b1 = 2*(y1-y0);
      double c1 = x1*x1 + y1*y1 - x0*x0 - y0*y0;

      double a2 = 2*(x2-x0);
      double b2 = 2*(y2-y0);
      double c2 = x2*x2 + y2*y2 - x0*x0 - y0*y0;

      double denom = a1*b2 - a2*b1;

      double xc = ( b2*c1 - b1*c2)/denom;
      double yc = (-a2*c1 + a1*c2)/denom;

      Point3d pc = new Point3d(xc, yc, 0);
      pc.transform (TPW);

      TPW.p.set (pc);
      Vector3d xdir = new Vector3d();
      xdir.sub (p0, pc);
      xdir.normalize();
      TPW.R.setZXDirections (zdir, xdir);
      return TPW;
   }

   /**
    * Updates the handle mesh from the guide points.
    */
   public void updateHandle() {
      if (hasHandlePoints()) {
         RigidTransform3d TCW = computePointCenter();
         Point3d p0 = getHandlePoints().get(0).getPosition();
         Point3d p2 = getHandlePoints().get(2).getPosition();

         Point3d pp = new Point3d(p2);
         pp.inverseTransform (TCW);
         double ang = Math.atan2 (pp.y, pp.x);
         if (ang < 0) {
            ang += 2*Math.PI;
         }
         double r = p0.distance(TCW.p);
         double extang = myHandleExtension/r;
         PolygonalMesh handle = MeshFactory.createToroidalSection (
            r, myHandleRadius-1, -extang, ang+extang,
            30, 20, /*capmin=*/true, /*capmax=*/true);
         handle.transform (TCW);
         RigidBody body = setRigidBody ("mandibleHandle", handle);
         RenderProps.setFaceColor (body, ReconAppRoot.GUIDE_COLOR);
      }
   }

   public boolean hasFlanges() {
      return getFlangeL() != null && getFlangeR() != null && hasHandlePoints();
   }

   public boolean areFlangesVisible() {
      return (isVisible (getFlangeL()) &&
              isVisible (getFlangeR()) &&
              isVisible (getRigidBody ("mandibleHandle")));
   }

   public boolean setFlangesVisible (boolean enable) {
      boolean changed = false;
      changed |= setVisible (getFlangeL(), enable);
      changed |= setVisible (getFlangeR(), enable);
      changed |= setVisible (getRigidBody ("mandibleHandle"), enable);
      return changed;
   }

   public void addFlanges (PolylineMesh plateCurve) {
      clearFlanges();
      clearGuide();

      addFlangeBox (
         "mandibleFlangeL", getRigidBody ("mandibleCutBoxL"), myContourL,
         plateCurve, myCurvePlaneIdxL, /*left=*/true);
      addFlangeBox (
         "mandibleFlangeR", getRigidBody ("mandibleCutBoxR"), myContourR,
         plateCurve, myCurvePlaneIdxR, /*left=*/false);
      //addScrews (plateCurve, myCurvePlaneIdxL, /*left=*/true);
      //addScrews (plateCurve, myCurvePlaneIdxR, /*left=*/false);
      addHandle();
   }

   public boolean clearFlanges() {
      boolean removed = false;
      removed |= myMech.removeRenderable (getFlangeL());
      removed |= myMech.removeRenderable (getFlangeR());
      removed |= removeRigidBody ("mandibleHandle");
      removed |= clearHandlePoints();
      //removed |= clearGuideScrews();
      return removed;
   }

   // handle points

   public boolean hasHandlePoints() {
      return getHandlePoints().size() == 3;
   }

   public boolean getHandlePointsVisible () {
      return myHandlePointsVisible;
   }

   public boolean doSetHandlePointsVisible (boolean enable) {
      return RenderableComponentBase.setVisible (getHandlePoints(), enable);
   }
   
   public boolean doGetHandlePointsVisible () {
      return RenderableComponentBase.isVisible(getHandlePoints());
   }
   
   public boolean setHandlePointsVisible (boolean enable) {
      if (enable != myHandlePointsVisible) {
         myHandlePointsVisible = enable;
         return RenderableComponentBase.setVisible (getHandlePoints(), enable);
      }
      else {
         return false;
      }
   }
   
   public boolean clearHandlePoints() {
      if (hasHandlePoints()) {
         getHandlePoints().removeAll();
         return true;
      }
      else {
         return false;
      }
   }

   public void addHandlePoints (
      Point3d p0, Point3d p1, Point3d p2) {
      PointList<Point> pnts = getHandlePoints();
      pnts.removeAll();
      pnts.add (new Point(p0));
      pnts.add (new Point(p1));
      pnts.add (new Point(p2));
   }

   public PointList<Point> getHandlePoints() {
      return (PointList<Point>)myMech.get ("mandibleHandlePoints");
   }

   // mandible parts

   public boolean hasSomeParts() {
      return hasCutBoxes() || hasFlanges();
   }

   public boolean getAllPartsVisible() {
      return (areCutBoxesVisible() && areFlangesVisible() &&
              (!getHandlePointsVisible() || doGetHandlePointsVisible()));
   }

   public boolean setAllPartsVisible (boolean visible) {
      boolean changed = false;
      changed |= setCutBoxesVisible (visible);
      changed |= setFlangesVisible (visible);
      if (!visible) {
         changed |= doSetHandlePointsVisible(false);
      }
      else {
         changed |= doSetHandlePointsVisible(myHandlePointsVisible);         
      }
      return changed;
   }

   // mandible cutting guide

   public boolean hasGuide() {
      return getRigidBody ("mandibleGuide") != null;
   }

   public RigidBody getGuide() {
      return getRigidBody ("mandibleGuide");
   }

   public boolean isGuideVisible() {
      return isVisible (getRigidBody ("mandibleGuide"));
   }

   public boolean setGuideVisible (boolean enable) {
      return setVisible (getRigidBody ("mandibleGuide"), enable);
   }
   
   public boolean clearGuide() {
      return removeRigidBody ("mandibleGuide");
   }

   public void createGuide() {
      clearGuide();

      RigidBody cutBoxL = getRigidBody ("mandibleCutBoxL");
      RigidBody cutBoxR = getRigidBody ("mandibleCutBoxR");
      SubdivisionMesh flangeL = getFlangeL();
      SubdivisionMesh flangeR = getFlangeR();
      RigidBody handle = getRigidBody ("mandibleHandle");
      RigidBody expandedMandible = getRigidBody ("expandedMandible");

      // union handle, flanges, andflangeL.getSmoothedMesh() cutboxs

      PolygonalMesh flangeMeshL = flangeL.getSmoothedMesh().clone();
      flangeMeshL.triangulate();
      PolygonalMesh flangeMeshR = flangeR.getSmoothedMesh().clone();
      flangeMeshR.triangulate();

      PolygonalMesh mesh = new PolygonalMesh();
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();

      PolygonalMesh meshL =
         intersector.findUnion (cutBoxL.getSurfaceMesh(), flangeMeshL);
      PolygonalMesh meshR =
         intersector.findUnion (cutBoxR.getSurfaceMesh(), flangeMeshR);

      mesh.addMesh (meshL);
      mesh.addMesh (meshR);
      mesh = intersector.findUnion (mesh, handle.getSurfaceMesh());

      // remove blade slots
      PolygonalMesh slot = (PolygonalMesh)cutBoxL.getMeshComp(1).getMesh();
      mesh = intersector.findDifference01 (mesh, slot);
      slot = (PolygonalMesh)cutBoxR.getMeshComp(1).getMesh();
      mesh = intersector.findDifference01 (mesh, slot);

      // remove expandedMandible
      mesh = intersector.findDifference01 (
         mesh, expandedMandible.getSurfaceMesh());

      PlateBuilder plateBuilder = getPlateBuilder();

      if (plateBuilder.numSelectedScrews() > 0) {
         // cut screw holes
         PolygonalMesh screwMesh = new PolygonalMesh();
         for (FixedMeshBody screw : plateBuilder.getScrews()) {
            if (screw.isSelected()) {
               screwMesh.addMesh (
                  (PolygonalMesh)screw.getMesh(), /*respectTransforms=*/true);
            }
         }
         mesh = intersector.findDifference01 (mesh, screwMesh);
      }
      
      RigidBody guide = setRigidBody ("mandibleGuide", mesh);
      RenderProps.setFaceColor (guide, ReconAppRoot.GUIDE_COLOR);
      setGuideVisible (true);
      setFlangesVisible (false);
      setHandlePointsVisible (false);
      setCutBoxesVisible (false);
      getRoot().getMeshManager().setExpandedMandibleVisible (false);
   }
}
