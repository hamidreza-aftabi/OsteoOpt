package artisynth.VSP.reconstruction;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ScanToken;
import maspack.collision.IntersectionContour;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVIntersector;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.TriLineIntersection;
import maspack.geometry.Vertex3d;
import maspack.interpolation.CubicHermiteSpline3d;
import maspack.matrix.Line;
import maspack.matrix.NumericalException;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.util.InternalErrorException;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

/**
 * Worker component for creating donor segments given a plate curve, some
 * constraints on the desired number of segments and their minimum spacing, and
 * a given donor mesh.
 */
public class SegmentGenerator extends WorkerComponentBase {

   private static double INF = Double.POSITIVE_INFINITY;

   private boolean myUseMinScapulaSegHeight = true;

   /**
    * Describes the orientation of scapula-based segments with respect to the
    * scapula outer edge.
    */
   public enum ScapulaOrientation {
      /**
       * Outer edge is at the top, clip plane is at the bottom
       */
      VERTICAL_UP,

      /**
       * Outer edge is at the bottom, clip plane is at the top
       */
      VERTICAL_DOWN
   }

   /**
    * Container for tracking the transforms associated with a
    * fibula donor segment.
    */
   private class FibulaDonorSegment {
      RigidTransform3d myTPS0;  // transform from cut plane 0 to the segment
      RigidTransform3d myTPS1;  // transform from cut plane 1 to the segment
      RigidTransform3d myTSW_M; // segment-to-world transform, in mandible space
      RigidTransform3d myTSW_D; // segment-to-world transform, in donor space
      
      public FibulaDonorSegment() {
         myTPS0 = new RigidTransform3d();
         myTPS1 = new RigidTransform3d();
         myTSW_M = new RigidTransform3d();
         myTSW_D = new RigidTransform3d();
      }
   }

   // last point advanced to along a fibula donor after segments are
   // created. Used for sizing the donor guide.
   Point3d myDonorEndPoint = new Point3d();

   // property definitions:

   static double DEFAULT_MIN_SEG_LENGTH = 10;
   double myMinSegLength = DEFAULT_MIN_SEG_LENGTH;

   public static int DEFAULT_MAX_SEGMENTS = 1;
   int myMaxSegments = DEFAULT_MAX_SEGMENTS;

   public static int DEFAULT_NUM_SEGMENTS = 1;
   int myNumSegments = DEFAULT_NUM_SEGMENTS;

   static double DEFAULT_SEGMENT_SEPARATION = 15;
   double mySegmentSeparation = DEFAULT_SEGMENT_SEPARATION;

   static boolean DEFAULT_CREATE_LEFT_TO_RIGHT = true;
   boolean myCreateLeftToRight = DEFAULT_CREATE_LEFT_TO_RIGHT;

   static ScapulaOrientation DEFAULT_SCAPULA_ORIENTATION =
      ScapulaOrientation.VERTICAL_DOWN;
   ScapulaOrientation myScapulaOrientation = DEFAULT_SCAPULA_ORIENTATION;

   static double DEFAULT_SCAPULA_HEIGHT_REDUCTION = 15;
   double myScapulaHeightReduction = DEFAULT_SCAPULA_HEIGHT_REDUCTION;

   public static PropertyList myProps =
      new PropertyList (
         SegmentGenerator.class, WorkerComponentBase.class);

   static {
      myProps.add (
         "minSegLength", "minimum length for each fibular segment",
         DEFAULT_MIN_SEG_LENGTH);
      myProps.add (
         "maxSegments", "maximum number of fibular segments",
         DEFAULT_MAX_SEGMENTS);
      myProps.add (
         "numSegments", "explicit number of fibular segments",
         DEFAULT_NUM_SEGMENTS);
      myProps.add (
         "segmentSeparation",
         "separation distance between donor segments",
         DEFAULT_SEGMENT_SEPARATION);
      myProps.add (
         "createLeftToRight",
         "create donor segments from left to right",
         DEFAULT_CREATE_LEFT_TO_RIGHT);
      myProps.add (
         "scapulaOrientation",
         "orientation of the scapula edge with respect to the donor segment",
         DEFAULT_SCAPULA_ORIENTATION);
      myProps.add (
         "scapulaHeightReduction",
         "reduction in the height of the scapula segments",
         DEFAULT_SCAPULA_HEIGHT_REDUCTION);
 }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public double getMinSegLength () {
      return myMinSegLength;
   }

   public void setMinSegLength (double len) {
      this.myMinSegLength = len;
   }

   public int getMaxSegments () {
      return myMaxSegments;
   }

   public void setMaxSegments (int max) {
      this.myMaxSegments = max;
   }

   public int getNumSegments () {
      return myNumSegments;
   }

   public void setNumSegments (int num) {
      this.myNumSegments = num;
   }

   public double getSegmentSeparation () {
      return mySegmentSeparation;
   }

   public void setSegmentSeparation (double sep) {
      this.mySegmentSeparation = sep;
   }

   public boolean getCreateLeftToRight () {
      return myCreateLeftToRight;
   }

   public void setCreateLeftToRight (boolean leftToRight) {
      this.myCreateLeftToRight = leftToRight;
   }

   public ScapulaOrientation getScapulaOrientation () {
      return myScapulaOrientation;
   }

   public void setScapulaOrientation (ScapulaOrientation orient) {
      this.myScapulaOrientation = orient;
   }

   public double getScapulaHeightReduction () {
      return myScapulaHeightReduction;
   }

   public void setScapulaHeightReduction (double hr) {
      this.myScapulaHeightReduction = hr;
   }

   // trim plane accessors

   public int numTrimPlanes() {
      return getTrimPlanes().size();
   }

   public RenderableComponentList<FixedMeshBody> getTrimPlanes() {
      return (RenderableComponentList<FixedMeshBody>)myMech.get ("trimPlanes");
   }

   public boolean removeTrimPlane (FixedMeshBody planeBody) {
      return getTrimPlanes().remove (planeBody);
   }

   public void clearTrimPlanes () {
      getTrimPlanes().removeAll();
   }

   public void addTrimPlane (FixedMeshBody planeBody) {
      getTrimPlanes().add (planeBody);
   }

   public FixedMeshBody addTrimPlane (RigidTransform3d TPW) {
      PolygonalMesh mesh = getRoot().getMeshManager().createRenderPlaneMesh();
      FixedMeshBody mbody = new FixedMeshBody(mesh);
      addTrimPlane (mbody);
      mbody.setPose (TPW);
      return mbody;
   }

   // other accessors

   public Point3d getDonorEndPoint() {
      return myDonorEndPoint;
   }

   public void setDonorEndPoint(Point3d pnt) {
      myDonorEndPoint.set (pnt);
   }

   // constructors

   /**
    * Note: need a public no-args constructor for ArtiSynth save/load.
    */
   public SegmentGenerator () {
      setRenderProps (defaultRenderProps(this));
   }

   public SegmentGenerator (String name) {
      this();
      setName (name);
   }

   // RDP line creation

   /**
    * Queries if the RDP line currently exists.
    */
   boolean hasRDPLine() {
      return numRDPPoints() > 0;
   }

   /**
    * Queries if the RDP line is visible.
    */
   public boolean isRDPLineVisible() {
      return isVisible (getRDPFrames()) && isVisible (getRDPSegments());
   }

   /**
    * Sets whether the RDP line is visible.
    */
   public boolean setRDPLineVisible (boolean enable) {
      boolean changed = false;
      changed |= setVisible (getRDPFrames(), enable);
      changed |= setVisible (getRDPSegments(), enable);
      return changed;
   }

   /**
    * Find the plate marker that is most "in between" the resection planes.
    */
   public Point3d findMiddlePlateMarker (
      PointList<FrameMarker> markers, Plane planeL, Plane planeR) {

      double minDistDiff = INF;
      FrameMarker midMkr = null;
      for (FrameMarker mkr : markers) {
         Point3d pos = mkr.getPosition();
         double distL = planeL.distance (pos);
         double distR = planeR.distance (pos);
         if (distL > 0 && distR > 0) {
            // inside the resect region
            if (Math.abs (distL-distR) < minDistDiff) {
               minDistDiff = Math.abs (distL-distR);
               midMkr = mkr;
            }
         }
      }
      return midMkr != null ? new Point3d(midMkr.getPosition()) : null;
   }

   /**
    * Returns true if the direction of the last two points of a point list are
    * heading *into* a plane (i.e., if p0 and p1 are the second-to-last and
    * last points, {@code (p1-p0) . nrm < 0}),
    */
   private boolean headingIntoPlane (ArrayList<Point3d> plist, Plane plane) {
      Vector3d dir = new Vector3d();
      if (plist.size() > 1) {
         int k = plist.size()-1;
         dir.sub (plist.get(k), plist.get(k-1));
         return plane.getNormal().dot(dir) < 0;
      }
      else {
         return false;
      }
   }

   /**
    * Computes a PolylineMesh representing the RDP line.
    */
   public ArrayList<Point3d> computeRDPLine (
      PolylineMesh curveMesh, Plane planeL, Plane planeR) {

      double distance = getMinSegLength();
      int segments = getMaxSegments();

      // find the points in the plate line that are clipped by the resection
      // region, with the end points just outside the region
      boolean insideResectRegion = false;
      ArrayList<Point3d> clippedPlateLine = new ArrayList<>();

      for (int k=0; k<curveMesh.numVertices(); k++) {
         Point3d pos = curveMesh.getVertex(k).getPosition();
         if (!insideResectRegion) {
            if (planeL.distance (pos) >= 0) {
               if (k == 0) {
                  break; // plane line doesn't extend beyond resection region
               }
               clippedPlateLine.add (curveMesh.getVertex(k-1).getPosition());
               insideResectRegion = true;
            }
         }
         else {
            clippedPlateLine.add (pos);
            // clip against R when pos is inside R *and* the curve is also
            // heading into R - the latter guards against situations where
            // planeR also intersects the curve on the left side.
            if (planeR.distance (pos) < 0 && 
                headingIntoPlane (clippedPlateLine, planeR)) {
               insideResectRegion = false;
               break;
            }
         }
      }
      if (clippedPlateLine.size() == 0 || insideResectRegion) {
         throw new IllegalArgumentException (
            "Plate line does not adequately extend beyond the resection region");
      }

      // Create RDP line from the clipped plate line
      ArrayList<Point3d> rdpLine = 
         PolylineSimplifier.bisectSimplifyDouglasPeucker (
            clippedPlateLine, distance, segments);

      // project rdpLine end points onto the resection planes
      Point3d p0 = rdpLine.get(0);
      planeL.project (p0, p0);
      Point3d pl = rdpLine.get(rdpLine.size()-1);
      planeR.project (pl, pl);

      return rdpLine;
   }

   private RigidBody createRDPFrame (
      Point3d p0, Point3d p1, Point3d p2, PolygonalMesh mandibleMesh) {
      RigidBody frame = RigidBody.createSphere (null, 1.5, 1000, 20);
      // figure out an orientation for the frame - line it up with
      // nearest mandible surface normal.
      Vector3d ydir = new Vector3d();
      Vector3d xdir = new Vector3d();
      if (p0 == null) {
         ydir.sub (p1, p2);
      }
      else if (p2 == null) {
         ydir.sub (p0, p1);
      }
      else {
         ydir.sub (p0, p2);
      }
      ydir.normalize();
      if (xdir.equals (Vector3d.ZERO)) {
         xdir = ComputeUtils.estimateSurfaceNormal (p1, mandibleMesh);
      }
      xdir.scaledAdd (-xdir.dot(ydir), xdir);
      xdir.normalize();
      RigidTransform3d TFW = new RigidTransform3d();
      TFW.R.setXYDirections (xdir, ydir);
      TFW.R.getColumn (1, ydir);
      TFW.p.set (p1);
      frame.setPose (TFW);
      frame.setDynamic (false);
      return frame;
   }

   /**
    * Creates a FixedMeshBody representing the RDP line.
    *
    * @param curveMesh mesh representing the plate curve as a single polyline
    * @param planeL left resection plane
    * @param planeR right resection plane
    */
   public void createRDPLine (
      PolylineMesh curveMesh, PolygonalMesh mandibleMesh, 
      Plane planeL, Plane planeR) {
      ArrayList<Point3d> rdpLine =
         computeRDPLine (curveMesh, planeL, planeR);
      clearRDPLine();
      RenderableComponentList<RigidBody> frames = getRDPFrames();
      PointList<FrameMarker> pnts = getRDPPoints();
      AxialSpringList<AxialSpring> segs = getRDPSegments();
      FrameMarker mkr0 = null;
      Point3d p0 = null;
      for (int i=0; i<rdpLine.size(); i++) {
         Point3d p1 = rdpLine.get(i);
         Point3d p2 = (i<rdpLine.size()-1 ? rdpLine.get(i+1) : null);
         RigidBody frame = createRDPFrame (p0, p1, p2, mandibleMesh);
         frames.add (frame);
         FrameMarker mkr1 = new FrameMarker(frame, new Point3d());
         pnts.add (mkr1);
         if (i > 0) {
            AxialSpring spr = new AxialSpring();
            spr.setFirstPoint (mkr0);
            spr.setSecondPoint (mkr1);
            segs.add (spr);
         }
         mkr0 = mkr1;
         p0 = p1;
      }
      
   }

   RenderableComponentList<RigidBody> getRDPFrames() {
      return (RenderableComponentList<RigidBody>)myMech.get ("RDPLineFrames");
   }

   PointList<FrameMarker> getRDPPoints() {
      return (PointList<FrameMarker>)myMech.get ("RDPLinePoints");
   }

   int numRDPPoints() {
      return getRDPPoints().size();
   }

   AxialSpringList<AxialSpring> getRDPSegments() {
      return (AxialSpringList<AxialSpring>)myMech.get ("RDPLineSegments");
   }

   /**
    * Remove the RDP line.
    */
   public boolean clearRDPLine() {
      if (numRDPPoints() > 0) {
         getRDPFrames().clear();
         getRDPPoints().clear();
         getRDPSegments().clear();
         return true;
      }
      else {
         return false;
      }
   }

   // support for donor segments

   /**
    * Queries if the donor segments are present.
    */
   boolean hasSegments() {
      return getSegments().size() > 0;
   }

   /**
    * Returns the list of donor segments.
    */
   RenderableComponentList<DonorSegmentBody> getSegments() {
      return
         (RenderableComponentList<DonorSegmentBody>)myMech.get(
            "donorSegments");
   }

   /**
    * Queries if the donor segments are visible.
    */
   boolean getSegmentsVisible() {
      return hasSegments() && isVisible (getSegments());
   }

   /**
    * Sets whether the donor segments are visible.
    */
   boolean setSegmentsVisible (boolean enable) {
      return setVisible (getSegments(), enable);
   }

   /**
    * Determines any extra separation needed before the start of a donor
    * segment in order to ensure that its first cut plane is sufficiently far
    * from either the top of the donor, or the previous segment.
    */
   private double computeExtraSegSeparation0 (
      Point3d p0, RigidTransform3d TPW0_D, PolygonalMesh donorMesh, double ztop) {

      // find the intersection contour between the cut plane nad the donor, and
      // check (in donor coordinates) if the z coordinate of any of its points
      // extends above the z coordinate of p0.

      PolygonalMesh cutPlane = createCutPlaneMesh();
      cutPlane.setMeshToWorld (TPW0_D);
      IntersectionContour contour = 
         ComputeUtils.findPrimaryIsectContour (donorMesh, cutPlane);
      if (contour == null) {
         throw new NumericalException (
            "No intersection contour between donor and cut plane 0");
      }
      RigidTransform3d TDW = donorMesh.getMeshToWorld();
      // find max z value in donor coordinates
      double maxz = -INF;
      Point3d p_d = new Point3d(); // point in donor space
      for (Point3d p : contour) {
         p_d.inverseTransform (TDW, p);
         if (p_d.z > maxz) {
            maxz = p_d.z;
         }
      }
      // if maxz > p0.z in donor space, need to add more separation
      p_d.inverseTransform (TDW, p0);
      if (maxz > p_d.z) {
         if (Math.abs(ztop-maxz) < 1e-8) {
            // contour might have cut off at the top; need to compute
            // separation differently

            // intersect with a plane whose z axis is aligned with TDW
            RigidTransform3d TPW = new RigidTransform3d (TPW0_D);
            TPW.R.set (TDW.R); // align cut plane with donor
            cutPlane.setMeshToWorld (TPW);
            contour = ComputeUtils.findPrimaryIsectContour (donorMesh, cutPlane);
            if (contour == null) {
               throw new NumericalException (
                  "No intersection contour between donor and cut plane 0");
            }            
            // find contour radius
            Point3d cent = new Point3d();
            contour.computeCentroid (cent);
            double r = ComputeUtils.computeContourRadius (contour, cent);
            // set sep to 2*r*sin(theta), where theta is the angle between the
            // z axes of TPW0_D and TDW
            Vector3d zdirD = new Vector3d();
            Vector3d zdirP = new Vector3d();
            TDW.R.getColumn (2, zdirD);
            TPW0_D.R.getColumn (2, zdirP);
            Vector3d xprod = new Vector3d();
            xprod.cross (zdirD, zdirP);
            return 2*r*xprod.norm(); // sin(theta) = length of xprod            
         }
         else {
            return maxz - p_d.z;
         }
      }
      else {
         return 0;
      }
   }

   /**
    * Determines any extra separation needed before that start of another donor
    * segment to ensure that the last cut plane of the current segment will not
    * intersect it.
    */
   private double computeExtraSegSeparation1 (
      Point3d p1, RigidTransform3d TPW1_D, PolygonalMesh donorMesh) {

      // find the intersection contour between the cut plane nad the donor, and
      // check (in donor coordinates) if the z coordinate of any of its points
      // extends below the z coordinate of p1.

      PolygonalMesh cutPlane = createCutPlaneMesh();
      cutPlane.setMeshToWorld (TPW1_D);
      IntersectionContour contour =
         ComputeUtils.findPrimaryIsectContour (donorMesh, cutPlane);
      if (contour == null) {
         throw new NumericalException (
            "No intersection contour between donor and cut plane 1");
      }
      RigidTransform3d TDW = donorMesh.getMeshToWorld();
      // find min z value in donor coordinates
      double minz = INF;
      Point3d p_d = new Point3d(); // point in donor space
      for (Point3d p : contour) {
         p_d.inverseTransform (TDW, p);
         if (p_d.z < minz) {
            minz = p_d.z;
         }
      }
      // if minz < p1.z in donor space, add the diff to the separation
      p_d.inverseTransform (TDW, p1);
      if (minz < p_d.z) {
         return p_d.z - minz;
      }
      else {
         return 0;
      }
   }

   /**
    * Compute the coordinate frame of a donor segment, wrt some mesh, using its
    * corresponding end points on the RDP line. The origin is given by the
    * midpoint the points, the z direction is parallel to pos0 - pos1, and the
    * y direction is set to be as close as possible to the average of the
    * estimated surface normals at pos0 and pos1.
    */
   public static RigidTransform3d computeSegmentFrame (
      Point3d pos0, Point3d pos1, PolygonalMesh mesh) {

      Point3d segCenter = new Point3d();
      segCenter.add (pos0, pos1);
      segCenter.scale (0.5);
      Vector3d zvec = new Vector3d();
      zvec.sub (pos0, pos1);
      zvec.normalize();     
      // compute average of the estimate surface normals at pos0 and pos1
      Vector3d yvec = new Vector3d();      
      yvec.set (ComputeUtils.estimateSurfaceNormal (pos0, mesh));
      yvec.add (ComputeUtils.estimateSurfaceNormal (pos1, mesh));
      // remove y component parallel to z
      yvec.scaledAdd (-yvec.dot(zvec), zvec);
      yvec.normalize();
      // Compute new segment coordinate frame TSW
      RigidTransform3d TSW = new RigidTransform3d();
      TSW.R.setYZDirections (yvec, zvec);
      TSW.p.set (segCenter);
      return TSW;
   }

   /**
    * Project a point {@code p1} to the surface of the donor mesh, in place,
    * and return the distance that the point was moved.  The projection is done
    * along a ray from the centerline of the donor mesh to {@code p1}; if this
    * fails for some reason, an unrestricted projection to the mesh surface is
    * used instead.
    *
    * @param p1 on input, point to project; on output, projected point.
    * @param donorMesh donor mesh
    * @param donorCurve donor centerline curve
    * @param TDW transform from donor to world coordinates
    * @return distance that p1 was moved
    */
   private double projectToDonor (
      Point3d p1, PolygonalMesh donorMesh,
      CubicHermiteSpline3d donorCurve, RigidTransform3d TDW) {

      Point3d p1_d = new Point3d(p1);
      p1_d.inverseTransform (TDW);
      
      Point3d cp1_d = new Point3d(donorCurve.eval (p1_d.z));
      Vector3d ray = new Vector3d();
      ray.sub (p1_d, cp1_d);
      ray.transform (TDW);
      Point3d pp = BVFeatureQuery.nearestPointAlongRay (donorMesh, p1, ray);
      if (pp != null) {
         double d = p1.distance(pp);
         p1.set (pp);
         return d;
      }
      ray.negate();
      pp = BVFeatureQuery.nearestPointAlongRay (donorMesh, p1, ray);
      if (pp != null) {
         double d = p1.distance(pp);
         p1.set (pp);
         return d;
      }
      double d = BVFeatureQuery.distanceToMesh (p1, donorMesh, p1);
      if (d >= 0) {
         System.out.println ("WARNING: projectToDonor failed along ray");
         return d;
      }
      else {
         System.out.println (
            "WARNING: projectToDonor failed for point "+p1);
         return -1;
      }
   }

   /**
    * Find a point {@code p1} along the (fibula) donor segment that is {@code
    * len} distance away from a previous point {@code p0}. We start by moving
    * in a direction parallel to the donor centerline. We then project {@code
    * p1} to the donor surface and move it so that is still {@code len} away
    * from {@code p1}, and iterate this until {@code p1} is sufficiently close
    * to the donor surface.
    */
   public Point3d findNextDonorPoint(
      Point3d p0, double len, PolygonalMesh donorMesh,
      CubicHermiteSpline3d donorCurve) {

      Point3d p1 = new Point3d();

      // "_d" denotes quantities in donor space
      RigidTransform3d TDW = donorMesh.getMeshToWorld();
      Point3d p0_d = new Point3d(p0);
      p0_d.inverseTransform(TDW);

      Point3d cp0_d = new Point3d(donorCurve.eval(p0_d.z));
      Point3d cp1_d = new Point3d(donorCurve.eval(p0_d.z - len));
      Vector3d u = new Vector3d();
      u.sub(cp1_d, cp0_d);
      u.transform(TDW);
      p1.scaledAdd(len / u.norm(), u, p0);
      int maxIters = 300;
     
      Point3d closestP1 = new Point3d(p1);  // Store the initial point as the closest
      double closestDist = Double.MAX_VALUE; // Initialize the closest distance to a large value

      for (int iter = 1; iter <= maxIters; iter++) {
          double dist = projectToDonor(p1, donorMesh, donorCurve, TDW);
          if (dist == -1) {
              throw new NumericalException("project to donor failed");
          }

          // make sure || p1-p0 || = len
          u.sub(p1, p0);
          p1.scaledAdd(len / u.norm(), u, p0);

          // Check if the current point is closer to the surface than the previous closest
          if (dist < closestDist) {
              closestDist = dist;
              closestP1.set(p1);  // Update the closest point
          }

          // Commented out original logic for returning the point within tolerance
          // if (dist <= tol) {
          //     return p1;
          // }
      }

      // Return the closest point found during the iterations
      return closestP1;

      // The following line is unreachable because the method already returns above
      // throw new NumericalException("findNextDonorPoint: iteration count exceeded");
}

   
   
   /**
    * Compute the average z direction of two transforms.
    */
   private static Vector3d averageZDir (
      RigidTransform3d T0, RigidTransform3d T1) {
      Vector3d z0 = new Vector3d();
      Vector3d z1 = new Vector3d();
      T0.R.getColumn (2, z0);
      T1.R.getColumn (2, z1);
      z0.add (z1);
      z0.normalize();
      return z0;      
   }

   /**
    * Compute a local transform for a plane that given a plane point and normal
    * in world coordinates.
    *
    * @param TLW transform from local to world coordinates
    * @param nrm plane normal in world coordinates
    * @param pnt plane point in world coordinates
    * @return transform from plane to world coordinates
    */
   private RigidTransform3d getPlaneTrans (
      RigidTransform3d TLW, Vector3d nrm, Point3d pos) {

      RigidTransform3d TPW = new RigidTransform3d();
      TPW.R.set (TLW.R);
      TPW.R.setZDirection (nrm);
      TPW.p.set (pos);
      RigidTransform3d TPL = new RigidTransform3d();
      TPL.mulInverseLeft (TLW, TPW); 
      return TPL;
   }

   /**
    * Create the coordinate frames for each donor segment in mandible space,
    * together with the cut plane positions relative to the segment.
    */
   public ArrayList<FibulaDonorSegment> createSegmentSpecs (
      PointList<FrameMarker> rdpLine, PolygonalMesh mandibleMesh,
      Plane resectPlane0, Plane resectPlane1) {

      ArrayList<FibulaDonorSegment> segs = new ArrayList<>();
      // rdpLine gives the end points for the segments.
      ArrayList<Point3d> linePnts = new ArrayList<>();
      for (Point p : rdpLine) {
         linePnts.add (p.getPosition());
      }
      if (!getCreateLeftToRight()) {
         // reverse points from the RDP line as well as the resection planes
         Collections.reverse (linePnts);
      }
      // compute segment coordinate frames and lengths
      RigidTransform3d TSW_prev = null;
      for (int i=0; i<linePnts.size()-1; i++) {
         Point3d pos0 = linePnts.get(i);
         Point3d pos1 = linePnts.get(i+1);
         FibulaDonorSegment spec = new FibulaDonorSegment();
         RigidTransform3d TSW = computeSegmentFrame (pos0, pos1, mandibleMesh);
         if (TSW_prev != null) {
            alignYZ (TSW, TSW_prev);
         }
         spec.myTSW_M.set (TSW);
         segs.add (spec);
         TSW_prev = TSW;
      }
      // compute segment cut planes based on the resection planes and the RDP
      // line
      for (int i=0; i<segs.size(); i++) {
         Point3d pos0 = linePnts.get(i);
         Point3d pos1 = linePnts.get(i+1);
         FibulaDonorSegment spec = segs.get(i);
         RigidTransform3d TSW_M = spec.myTSW_M;
         if (i == 0) {
            // plane 0 is the left resection plance
            spec.myTPS0.set (
               getPlaneTrans (TSW_M, resectPlane0.getNormal(), pos0));
         }
         else {
            // plane 0 is perpendicular to the average RDP line direction
            // between this segment and the previous one
            Vector3d nrm = averageZDir (TSW_M, segs.get(i-1).myTSW_M);
            nrm.negate();
            spec.myTPS0.set (getPlaneTrans (TSW_M, nrm, pos0));
         }
         if (i == segs.size()-1) {
            // plane 1 is the right resection plance
            spec.myTPS1.set (
               getPlaneTrans (TSW_M, resectPlane1.getNormal(), pos1));
         }
         else {
            // plane 1 is perpendicular to the average RDP line direction
            // between this segment and the next one
            Vector3d nrm = averageZDir (TSW_M, segs.get(i+1).myTSW_M);
            spec.myTPS1.set (getPlaneTrans (TSW_M, nrm, pos1));
         }
      }
      return segs;
   }

   public void adjustDonorSetback (double inc) {
      for (DonorSegmentBody sbody : getSegments()) {
         sbody.adjustDonorSetback (inc);
      }
   }

   /**
    * Create fibula donor segments. Finds the pose of each donor segment, in both
    * donor and mandible space, together with the pose of each of its two cut
    * planes, expresssed with respect to the segment. Each segment is expressed
    * by an instance of DonorSegmentBody and stored in a dedicated component
    * list.
    */
   public void createFibulaSegments (
      Point3d startPoint, RigidBody clippedDonor,
      CubicHermiteSpline3d donorCurve, PolygonalMesh mandible,
      FixedMeshBody resectBody0, FixedMeshBody resectBody1) {

      if (!getCreateLeftToRight()) {
         // reverse resection planes
         FixedMeshBody tmp = resectBody0; 
         resectBody0 = resectBody1; 
         resectBody1 = tmp;
      }
      Plane resectPlane0 = new Plane(resectBody0.getPose());
      Plane resectPlane1 = new Plane(resectBody1.getPose());
      clearSegments();

      PolygonalMesh donorMesh = clippedDonor.getSurfaceMesh();
      double defaultSep = getSegmentSeparation();
      
      // compute the segment specifications in mandible space
      ArrayList<FibulaDonorSegment> segments =
         createSegmentSpecs (
            getRDPPoints(), mandible, resectPlane0, resectPlane1);
      double ztop = donorCurve.getLastKnot().getS0();

      // For each segment, find a corresponding placement with respect to the
      // donor. Essentially, starting at {@code startPoint}, we place each
      // segment along the donor, moving in a direction (roughly) parallel to
      // the centerline, adding extra separation as needed to ensure that the
      // cut planes do not overlap.
      Point3d p0 = new Point3d (startPoint);
      for (int i=0; i<segments.size(); i++) {
         FibulaDonorSegment seg = segments.get(i);
         double len = seg.myTPS0.p.distance(seg.myTPS1.p);
         // find segment end point along the fibula
         Point3d p1 = findNextDonorPoint (p0, len, donorMesh, donorCurve);
         // use this to compute the segment transform
         seg.myTSW_D.set (computeSegmentFrame (p0, p1, donorMesh));
         // compute any extra sepration that we need to apply to p0
         RigidTransform3d TPW0_D = new RigidTransform3d();
         TPW0_D.mul (seg.myTSW_D, seg.myTPS0);
         double sep = computeExtraSegSeparation0 (p0, TPW0_D, donorMesh, ztop);
         if (sep > 0) {
            p0 = findNextDonorPoint (p0, sep, donorMesh, donorCurve);
            // redo with new p0
            p1 = findNextDonorPoint (p0, len, donorMesh, donorCurve);
            seg.myTSW_D.set (computeSegmentFrame (p0, p1, donorMesh));
         }
         // recompute p0 with required separation applied to p1
         RigidTransform3d TPW1_D = new RigidTransform3d();
         TPW1_D.mul (seg.myTSW_D, seg.myTPS1);
         sep = computeExtraSegSeparation1 (p1, TPW1_D, donorMesh);
         sep += defaultSep;
         p0 = findNextDonorPoint (p1, sep, donorMesh, donorCurve);

         DonorSegmentBody sbody;
         sbody = new DonorSegmentBody (
            seg.myTSW_M, seg.myTSW_D, clippedDonor, seg.myTPS0, seg.myTPS1);

         // if (i == 0) {
         //    sbody = new DonorSegmentBody (
         //       seg.myTSW_M, seg.myTSW_D, clippedDonor, resectBody0, seg.myTPS1);
         // }
         // else if (i < segments.size()-1) {
         //    sbody = new DonorSegmentBody (
         //       seg.myTSW_M, seg.myTSW_D, clippedDonor, seg.myTPS0, seg.myTPS1);
         // }
         // else {
         //    sbody = new DonorSegmentBody (
         //       seg.myTSW_M, seg.myTSW_D, clippedDonor, seg.myTPS0, resectBody1);
         // }
         
         // compute and add pose offset TSM for easy manipulation
         sbody.setPoseOffset (computePoseOffset(sbody, donorMesh));
         getSegments().add (sbody);
         sbody.updateDonorMesh();
      }
      myDonorEndPoint.set (p0);
   }

   /**
    * Compute an offset info the segment that makes it easier to manipulate
    */
   RigidTransform3d computePoseOffset (
      DonorSegmentBody seg, PolygonalMesh donorMesh) {
      PolygonalMesh cutPlane = createCutPlaneMesh();
      RigidTransform3d TDW0 = seg.getPoseD0();
      cutPlane.setMeshToWorld (TDW0);
      IntersectionContour contour = 
         ComputeUtils.findPrimaryIsectContour (donorMesh, cutPlane);
      if (contour == null) {
         throw new NumericalException (
            "No intersection contour between donor and plane at D0");
      }
      Vector3d cent = new Vector3d();
      Vector3d ydir = new Vector3d();
      TDW0.R.getColumn (1, ydir);
      contour.computeCentroid(cent);
      cent.sub (TDW0.p);
      return new RigidTransform3d (0, cent.dot(ydir), 0);
   }

   /**
    * Create the donor segments. Each segment is expressed by an instance of
    * DonorSegmentBody and stored in a dedicated component list.
    */
   // public void createDonorSegmentsX (
   //    Point3d startPoint, RigidBody clippedDonor,
   //    CubicHermiteSpline3d donorCurve, PolygonalMesh mandible,
   //    Plane resectPlaneL, Plane resectPlaneR) {

   //    clearSegments();
   //    findDonorCutPlanes (
   //       startPoint, clippedDonor, donorCurve,
   //       mandible, resectPlaneL, resectPlaneR);
   // }

   /**
    * Find top and bottom points on the intersection contour between a given
    * surface mesh and a plane.
    */
   private Point3d[] findTopBotIsectPoints (
      PolygonalMesh mesh, RigidTransform3d TPW) {

      PolygonalMesh cutPlane = createCutPlaneMesh();
      cutPlane.setMeshToWorld (TPW);
      IntersectionContour contour = 
         ComputeUtils.findPrimaryIsectContour (mesh, cutPlane);
      if (contour != null) {
         double minZ = INF;
         Point3d minP = null;         
         double maxZ = -INF;
         Point3d maxP = null;         
         for (Point3d p : contour) {
            if (p.z < minZ) {
               minZ = p.z;
               minP = p;
            }
            if (p.z > maxZ) {
               maxZ = p.z;
               maxP = p;
            }
         }
         return new Point3d[] { maxP, minP };
      }
      else {
         return null;
      }
   }

   double projectToSurface (PolygonalMesh mesh, Point3d p1, Vector3d ray) {
      double mind = INF;
      Point3d pr = new Point3d (p1);
      Point3d pp = BVFeatureQuery.nearestPointAlongRay (mesh, p1, ray);
      if (pp != null) {
         double d = p1.distance(pp);
         if (d < mind) {
            mind = d;
            pr.set (pp);
         }
      }
      ray.negate();
      pp = BVFeatureQuery.nearestPointAlongRay (mesh, p1, ray);
      if (pp != null) {
         double d = p1.distance(pp);
         if (d < mind) {
            mind = d;
            pr.set (pp);
         }
      }
      p1.set (pr);
      return (mind != INF ? mind : 0);
   }


   /**
    * Create scapula segments, based on the resection planes and the
    * donor markers. Currently we assume that there is only one segment.
    */
   public void createScapulaSegments (
      PointList<FrameMarker> donorMarkers,
      PointList<FrameMarker> plateMarkers,
      RigidBody donor, RigidBody mandible,
      FixedMeshBody resectBodyL, FixedMeshBody resectBodyR) {

      RigidTransform3d TLPW = resectBodyL.getPose();
      RigidTransform3d TRPW = resectBodyR.getPose();

      clearSegments();
      clearTrimPlanes();
      Point3d[] pntsL = findTopBotIsectPoints (mandible.getSurfaceMesh(), TLPW);
      if (pntsL == null) {
         throw new NumericalException (
            "No intersection found between mandible and left resection plane");
      }
      Point3d[] pntsR = findTopBotIsectPoints (mandible.getSurfaceMesh(), TRPW);
      if (pntsR == null) {
         throw new NumericalException (
            "No intersection found between mandible and right resection plane");
      }
      if (myUseMinScapulaSegHeight) {
         Vector3d ur = new Vector3d();
         ur.sub (pntsR[0], pntsR[1]);
         Vector3d ul = new Vector3d();
         ul.sub (pntsL[0], pntsL[1]);
         double hr = pntsR[0].distance (pntsR[1]) - myScapulaHeightReduction;
         double hl = pntsL[0].distance (pntsL[1]) - myScapulaHeightReduction;
         if (hr > hl) {
            hr = hl;
         }
         else if (hr < hl) {
            hl = hr;
         }
         pntsR[0].scaledAdd (hr/ur.norm(), ur, pntsR[1]);
         pntsL[0].scaledAdd (hl/ul.norm(), ul, pntsL[1]);
      }
      int edgeIdx; // end point indices corresponding to scapula edge
      int trimIdx; // end point indices corresponding to the trim plane
      double scapYDir;
      switch (myScapulaOrientation) {
         case VERTICAL_DOWN: {
            // edge on the botton, trim on the top
            edgeIdx = 1; 
            trimIdx = 0;
            scapYDir = 1;
            break;
         }
         case VERTICAL_UP: {
            // edge on the top, trim on the bottom
            edgeIdx = 0;
            trimIdx = 1;
            scapYDir = -1;
            break;
         }
         default: {
            throw new InternalErrorException (
               "Scapula orientation "+myScapulaOrientation+" not implemented");
         }
      }
      

      // find center point and z direction for edge coords in mandible space
      Point3d pntC = new Point3d();
      pntC.add (pntsR[edgeIdx], pntsL[edgeIdx]);
      pntC.scale (0.5);
      Vector3d zdir = new Vector3d();
      zdir.sub (pntC, pntsR[edgeIdx]);
      double len = zdir.norm();
      zdir.scale (1/len);

      // create mandible space coordinate frame TMW from edge coords
      RigidTransform3d TMW = new RigidTransform3d();
      TMW.p.set (pntC);
      Vector3d xdir = new Vector3d();
      xdir.set (0, 0, 1); // start with x dir set to world z
      xdir.scaledAdd (-xdir.dot(zdir), zdir);
      TMW.R.setZXDirections (zdir, xdir);

      // find center point and z direction on scapula
      Point3d pntL = new Point3d();
      Point3d pntR = new Point3d();
      if (getCreateLeftToRight()) {
         pntL.set (donorMarkers.get(0).getPosition());
         pntR.set (donorMarkers.get(donorMarkers.size()-1).getPosition());
      }
      else {
         pntR.set (donorMarkers.get(0).getPosition());
         pntL.set (donorMarkers.get(donorMarkers.size()-1).getPosition());
      }
      zdir.sub (pntL, pntR);
      zdir.normalize();

      pntC.scaledAdd (-len, zdir, pntL);

      // create scapula space coordinate frame TDW
      RigidTransform3d TSW = new RigidTransform3d(donor.getPose());
      RigidTransform3d TDW = new RigidTransform3d();
      Vector3d cvec = new Vector3d();
      cvec.sub (TSW.p, pntC);
      Vector3d xprod = new Vector3d();
      xprod.cross (zdir, cvec);
      Vector3d ydir = new Vector3d();
      TSW.R.getColumn (2, ydir);
      if (xprod.dot (ydir) < 0) {
         ydir.negate();
      }
      ydir.scale (scapYDir);
      ydir.scaledAdd (-ydir.dot(zdir), zdir);
      TDW.R.setYZDirections (ydir, zdir);

      // refine so that pntR is nearer to the surface
      TDW.R.getColumn (0, xdir);
      pntR.scaledAdd (-2*len, zdir, pntL);
      projectToSurface (donor.getSurfaceMesh(), pntR, xdir);
      pntC.combine (0.5, pntL, 0.5, pntR);
      zdir.sub (pntL, pntR);
      zdir.normalize();
      ydir.scaledAdd (-ydir.dot(zdir), zdir);
      TDW.R.setYZDirections (ydir, zdir);
      TDW.p.set (pntC);

      // create the trim plane
      Plane planeL = new Plane(TLPW);
      Plane planeR = new Plane(TRPW);
      Point3d midPnt = findMiddlePlateMarker (plateMarkers, planeL, planeR);
      if (midPnt == null) {
         throw new NumericalException (
            "No plate markers in the resection area");
      }
      ydir.set (ComputeUtils.estimateSurfaceNormal (
                   midPnt, mandible.getSurfaceMesh()));
      RigidTransform3d TPW = new RigidTransform3d();
      pntC.add (pntsR[trimIdx], pntsL[trimIdx]);
      pntC.scale (0.5);
      xdir.sub (pntC, pntsR[trimIdx]);
      xdir.normalize();
      ydir.scaledAdd (-ydir.dot(xdir), xdir);
      ydir.normalize();
      TPW.p.set (pntC);
      TPW.R.setXYDirections (xdir, ydir);
      
      FixedMeshBody trimPlane = addTrimPlane (TPW);

      // create the actual donor segment
      DonorSegmentBody sbody = new DonorSegmentBody (
         TMW, TDW, donor, resectBodyL, resectBodyR, trimPlane);
      getSegments().add (sbody);
      sbody.updateDonorMesh();

      // Rotate about the M zaxis to bring the trim part of the scapula closer
      // to the center of the trim plane.
      Vector3d ray = new Vector3d();
      ray.negate (ydir);

      ArrayList<TriLineIntersection> faceIsects = new ArrayList<>();
      BVIntersector bvi = new BVIntersector();
      Point3d scapPnt = null;
      // move pntC by 1 mm along plane z to clear the edge of the mesh
      TPW.R.getColumn (2, zdir);
      pntC.scaledAdd (scapYDir, zdir);
      bvi.intersectMeshLine (faceIsects, sbody.getMesh(), new Line(pntC, ray));
      if (faceIsects.size() == 0) {
         System.out.println (
            "WARNING: can't find nearest scapula point to trim plane center");
      }
      else {
         scapPnt = new Point3d();
         double smax = -INF;
         double smin = INF;
         Vector3d disp = new Vector3d();
         for (TriLineIntersection ti : faceIsects) {
            disp.sub (ti.points[0], pntC);
            double s = disp.dot(ray);
            if (s < smin) {
               smin = s;
            }
            if (s > smax) {
               smax = s;
            }
         }
         scapPnt.scaledAdd ((smin+smax)/2, ray, pntC);
      }
      
      //Point3d scapPnt = BVFeatureQuery.nearestPointAlongRay (
      // sbody.getMesh(), pntC, ray);
      if (scapPnt != null) {
         // find vectors delc and dels from the origin of TMW to the plane
         // center and nearest scapula points. Project them into into x-y plane
         // of TMW, normalize them, and take their cross product to find
         // sine of the required rotation about z.
         Vector3d delc = new Vector3d();
         Vector3d dels = new Vector3d();
         TMW.R.getColumn (2, zdir);
         dels.sub (scapPnt, TMW.p);
         delc.sub (TPW.p, TMW.p);
         dels.scaledAdd (-dels.dot(zdir), zdir);
         delc.scaledAdd (-delc.dot(zdir), zdir);
         dels.normalize();
         delc.normalize();
         xprod.cross (delc, dels);
         // now rotate in D space to obtain the correction
         TDW.R.mulRotZ (Math.asin (xprod.dot (zdir)));
         //sbody.setPoseD (TDW);
         sbody.setPoseD0 (TDW);
      }
      else {
         System.out.println (
            "WARNING: can't find nearest scapula point to trim plane center");
      }

      // move the segment pose to the centroid
      Point3d cent = new Point3d();
      sbody.getMesh().computeCentroid (cent);
      sbody.setPoseOffset (new RigidTransform3d (cent.x, cent.y, cent.z));
   }

   /**
    * Remove all donor segments.
    */
   boolean clearSegments () {
      if (hasSegments()) {
         getSegments().clear();
         return true;
      }
      else {
         return false;
      }
   }

   /**
    * Traverse a list of donor segments, either in left-to-right or 
    * right-to-left, and change their orientations so that for any two segments,
    * their y axes are aligned with respect to the average of their z axes.
    */
   void alignSegments (boolean leftToRight) {
      if (hasSegments()) {
         ArrayList<DonorSegmentBody> segments = new ArrayList<>();
         segments.addAll (getSegments());
         if (leftToRight != getCreateLeftToRight()) {
            Collections.reverse (segments);
         }
         for (int i=1; i<segments.size(); i++) {
            DonorSegmentBody seg = segments.get(i);
            seg.alignYZ (segments.get(i-1));
         }
      }
   }

   /**
    * Modify the orientation of transform {@code T1} so that the y axes of both
    * {@code T0} and {@code T1} are both aligned with respect to the average z
    * direction of both transforms; i.e., each transform's y axis has the same
    * rotation about about the average z direction.
    */
   public static void alignYZ (RigidTransform3d T1, RigidTransform3d T0) {
      Vector3d y0 = new Vector3d();
      Vector3d x1 = new Vector3d();
      Vector3d y1 = new Vector3d();

      T0.R.getColumn (1, y0);
      T1.R.getColumn (0, x1);
      T1.R.getColumn (1, y1);

      Vector3d zavg = averageZDir (T1, T0);
      Vector3d u = new Vector3d();
      u.cross (y0, zavg);

      // rotate y1 about z1 such that y1 . u = 0. If the rotation is performed
      // by y1' = -s x1 + c y1, where s and c are the sine and cosine of the
      // rotation, we want -s (x1 . u) + c (y1 . u) = 0, or s a = c b, where a
      // = x1 . u and b = y1 . u
      double a = x1.dot (u);
      double b = y1.dot (u);
      // solve for s and c
      double mag = Math.sqrt (a*a + b*b);
      if (mag != 0) {
         double s = b/mag;
         double c = a/mag;
         RotationMatrix3d RZ =
            new RotationMatrix3d (c, -s, 0,  s, c, 0,  0, 0,1);
         T1.R.mul (RZ);
      }
   }

   /**
    * Write out the transforms for all the segments.
    */
   void writeSegmentTransforms (PrintWriter pw, NumberFormat fmt)
      throws IOException {
      for (DonorSegmentBody body : getSegments()) {
         body.writeTransforms (pw, fmt);
      }
   }

   // I/O method overrides to write and scan quantities not stored as
   // properties
 
   /**
    * {@inheritDoc}
    */
   @Override
   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor) 
      throws IOException {
      pw.println ("donorEndPoint=[" + myDonorEndPoint.toString(fmt)+"]");
      super.writeItems (pw, fmt, ancestor);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {
      rtok.nextToken();
      if (scanAttributeName (rtok, "donorEndPoint")) {
         myDonorEndPoint.scan (rtok);
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

   public void cutPlaneGeometryChanged (FixedMeshBody planeBody) {
      for (DonorSegmentBody sbody : getSegments()) {
         if (sbody.containsPlaneBody (planeBody)) {
            sbody.updateLocalPlanePoses();
            sbody.updateDonorMesh();
            sbody.updateWorldPlanePoses();
         }
      }
   }

}

