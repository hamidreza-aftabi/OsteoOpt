package artisynth.VSP.reconstruction;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Deque;

import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.util.ScanToken;
import maspack.collision.SurfaceMeshIntersector;
import maspack.collision.IntersectionContour;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.CubicHermiteSpline3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector3i;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.util.InternalErrorException;
import maspack.util.NumberFormat;
import maspack.util.PathFinder;
import maspack.util.ReaderTokenizer;

/**
 * Worker component for creating donor cutting guides.
 */
public class DonorGuideBuilder extends WorkerComponentBase {

   public enum DonorRecessType {
      CYLINDER,
      DONOR
   };

   protected boolean myHasScrewHoles = false;
   protected boolean myHasImplantHoles = false;

   // offsets needed to center the blade guide mesh
   static double GUIDE_OFF_Y = -26.3642;
   static double GUIDE_OFF_Z = -1.49926;

   static Vector3i DEFAULT_GUIDE_BASE_RES = new Vector3i (8, 4, 35);
   protected Vector3i myGuideBaseRes = new Vector3i(DEFAULT_GUIDE_BASE_RES);

   static Vector3d DEFAULT_GUIDE_BASE_WIDTHS = new Vector3d (24, 12, -1);
   protected Vector3d myGuideBaseWidths = new Vector3d(DEFAULT_GUIDE_BASE_WIDTHS);

   static Vector3i DEFAULT_BLADE_BOX_RES = new Vector3i (5, 30, 4);
   protected Vector3i myBladeBoxRes = new Vector3i(DEFAULT_BLADE_BOX_RES);

   static Vector3d DEFAULT_BLADE_BOX_WIDTHS =
      new Vector3d (9.61872, 57.7085, 6.37235);
   protected Vector3d myBladeBoxWidths = new Vector3d(DEFAULT_BLADE_BOX_WIDTHS);

   static double DEFAULT_SETBACK_FRACTION = 0.3;
   protected double mySetbackFraction = DEFAULT_SETBACK_FRACTION;

   static DonorRecessType DEFAULT_RECESS_TYPE = DonorRecessType.CYLINDER;
   protected DonorRecessType myRecessType = DEFAULT_RECESS_TYPE;

   static double DEFAULT_RECESS_CLEARANCE = 1.0;
   protected double myRecessClearance = DEFAULT_RECESS_CLEARANCE;

   static double DEFAULT_TOP_FLANGE_DEPTH = 0.0;
   protected double myTopFlangeDepth = DEFAULT_TOP_FLANGE_DEPTH;

   static double DEFAULT_MIN_FLANGE_WIDTH = 0.0;
   protected double myMinFlangeWidth = DEFAULT_MIN_FLANGE_WIDTH;

   public static PropertyList myProps =
      new PropertyList (DonorGuideBuilder.class, WorkerComponentBase.class);

   static {
      myProps.add (
         "guideBaseWidths",
         "x,y,z widths of the donor guide base",
         DEFAULT_GUIDE_BASE_WIDTHS);
      myProps.add (
         "guideBaseRes",
         "x,y,z mesh resolutions of the donor guide base",
         DEFAULT_GUIDE_BASE_RES);
      myProps.add (
         "bladeBoxWidths",
         "x,y,z widths of the blade box base",
         DEFAULT_BLADE_BOX_WIDTHS);
      myProps.add (
         "bladeBoxRes",
         "x,y,z mesh resolutions of the blade box",
         DEFAULT_BLADE_BOX_RES);
      myProps.add (
         "setbackFraction", 
         "fraction of guide-centerline distance by which guide is set back",
         DEFAULT_SETBACK_FRACTION);
      myProps.add (
         "recessType",
         "type of recess used to accommodate the donor",
         DEFAULT_RECESS_TYPE);
      myProps.add (
         "recessClearance",
         "clearance distance when using a DONOR recess type",
         DEFAULT_RECESS_CLEARANCE);
      myProps.add (
         "topFlangeDepth",
         "depth of the top flange used for implant drilling",
         DEFAULT_TOP_FLANGE_DEPTH);
      myProps.add (
         "minFlangeWidth",
         "width of the min flange used for implant drilling",
         DEFAULT_MIN_FLANGE_WIDTH);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public DonorGuideBuilder () {
      super();
   }

   public DonorGuideBuilder (String name) {
      this();
      setName (name);
      // String meshPath =
      //    PathFinder.getSourceRelativePath (this, "geometry/bladeGuideUnion.stl");
      // myBladeGuide = new PolygonalMesh (meshPath);
   }

   public Vector3d getGuideBaseWidths () {
      return myGuideBaseWidths;
   }

   public void setGuideBaseWidths (Vector3d widths) {
      myGuideBaseWidths.set (widths);
   }

   public Vector3i getGuideBaseRes () {
      return myGuideBaseRes;
   }

   public void setGuideBaseRes (Vector3i res) {
      myGuideBaseRes.set (res);
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
      myBladeBoxRes.set (res);
   }

   public double getSetbackFraction () {
      return mySetbackFraction;
   }

   public void setSetbackFraction (double frac) {
      mySetbackFraction = frac;
   }

   public DonorRecessType getRecessType() {
      return myRecessType;
   }

   public void setRecessType (DonorRecessType type) {
      myRecessType = type;
   }

   public double getTopFlangeDepth() {
      return myTopFlangeDepth;
   }

   public void setTopFlangeDepth (double d) {
      myTopFlangeDepth = d;
   }

   public double getMinFlangeWidth() {
      return myMinFlangeWidth;
   }

   public void setMinFlangeWidth (double d) {
      myMinFlangeWidth = d;
   }

   public double getRecessClearance() {
      return myRecessClearance;
   }

   public void setRecessClearance (double clearance) {
      myRecessClearance = clearance;
   }

   public boolean hasGuide() {
      return (
         getMeshBody ("donorGuide") != null &&
         getMeshBody ("donorGuideBackup") != null);
   }

   public FixedMeshBody getGuide() {
      return getMeshBody ("donorGuide");
   }

   private FixedMeshBody getBackupGuide() {
      return getMeshBody ("donorGuideBackup");
   }

   public boolean isGuideVisible() {
      return isVisible (getMeshBody ("donorGuide"));
   }

   public boolean setGuideVisible (boolean enable) {
      return setVisible (getMeshBody ("donorGuide"), enable);
   }

   public boolean clearGuide() {
      boolean changed = false;
      changed |= removeMeshBody ("donorGuideBackup");
      changed |= removeMeshBody ("donorGuide");
      myHasScrewHoles = false;
      myHasImplantHoles = false;
      return changed;
   }

   /**
    * Computes a vector from the donor center curve to a given point, in world
    * coordinates.
    */
   private Vector3d getCurveToPoint (
      Vector3d pnt, CubicHermiteSpline3d curve, RigidTransform3d TCW) {

      Point3d pnt_d = new Point3d (pnt);
      pnt_d.inverseTransform (TCW);
      Point3d ctr_d = new Point3d (curve.eval (pnt_d.z));
      Vector3d dir = new Vector3d();
      dir.sub (pnt_d, ctr_d);
      dir.transform (TCW);
      return dir;      
   }

   private RigidTransform3d getBladeBoxTransform (
      RigidTransform3d TPW, RigidTransform3d TBW) {

      // start by finding the transform TBBB in base coordinates
      RigidTransform3d TBBB = new RigidTransform3d();

      // transform the plane into the coordinates of the base
      RigidTransform3d TPB = new RigidTransform3d();
      TPB.mulInverseLeft (TBW, TPW);
      Plane plane = new Plane (TPB);

      // the origin is the intersection of the plane with the center line
      // parallel to z and passing through (0, guideBaseWidths.y/2, 0):
      Point3d cpnt = new Point3d (0, myGuideBaseWidths.y/2, 0);
      Point3d isect = new Point3d();
      plane.intersectLine (isect, Vector3d.Z_UNIT, cpnt);
      TBBB.p.set (isect);

      // y direction of TBBB is given by the intersection of the plane with the
      // zxplane running through cpnt.
      Plane zxplane = new Plane (Vector3d.Y_UNIT, cpnt);
      Vector3d ydir = new Vector3d();
      plane.intersectPlane (isect, ydir, zxplane);
      if (ydir.x < 0) {
         ydir.negate();
      }
      // x dir is parallel to the plane normal
      Vector3d xdir = new Vector3d();
      xdir.set (plane.getNormal());
      if (xdir.z < 0) {
         xdir.negate();
      }
      TBBB.R.setXYDirections (xdir, ydir);
      // now create TBBW from TBBB 
      RigidTransform3d TBBW = new RigidTransform3d();
      TBBW.mul (TBW, TBBB);
      return TBBW;
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
         PathFinder.getSourceRelativePath (this, "geometry/bladeGuideUnion.stl");
      PolygonalMesh mesh = new PolygonalMesh (meshPath);
      mesh.transform (new RigidTransform3d (0, GUIDE_OFF_Y, GUIDE_OFF_Z));
      return mesh;
   }

   private PolygonalMesh createCylinderRecessMesh (
      double maxRadius, double baseLen, double maxYlen, RigidTransform3d TBW) {

      double boxwy = myGuideBaseWidths.y;
      // find coordinate frame for the cylinder
      RigidTransform3d TCW = new RigidTransform3d (TBW);
      Vector3d ydir = new Vector3d();
      TCW.R.getColumn (1, ydir);
      double yoffset =
         boxwy/2 + maxRadius -
         (1-mySetbackFraction)*maxYlen - myRecessClearance;
      TCW.p.scaledAdd (-yoffset, ydir);

      // find the wedge angle of the recess with respect to the box edge
      double cos = (yoffset-boxwy/2)/maxRadius;
      // extra careful: make sure cos is valid in acos call
      double ang = Math.acos (Math.max(0, Math.min(1, cos)));
      // create a wedge mesh to produce the recess
      int nh = myGuideBaseRes.z;
      int nr = (int)Math.ceil(maxRadius*nh/baseLen);
      int nang = 30;
      PolygonalMesh mesh = MeshFactory.createCylindricalWedge (
         maxRadius, baseLen+2, 2*ang, nr, nh, nang);
      //mesh = MeshFactory.createCylinder (
      //  maxRadius, baseLen+2, 2*nang, nr, nh);
      // rotate mesh about z so wedge is centered on the y axis
      mesh.transform (new RigidTransform3d (0, 0, 0, Math.PI/2, 0, 0));
      mesh.setMeshToWorld (TCW);      
      return mesh;
   }

   private PolygonalMesh createDonorRecessMesh (
      PolygonalMesh clippedDonor, CubicHermiteSpline3d donorCurve,
      RigidTransform3d TDW) {
      PolygonalMesh mesh = clippedDonor.clone();
      Point3d pos = new Point3d();
      Vector3d ucent = new Vector3d(); // vector from center to pos
      double ztol = 2.0;
      double zmax = donorCurve.getLastKnot().getS0();
      for (Vertex3d vtx : mesh.getVertices()) {
         pos.set (vtx.getPosition());
         ucent.sub (pos, donorCurve.eval (pos.z));         
         double mag = ucent.norm();
         if (mag > 0) {
            pos.scaledAdd (myRecessClearance/mag, ucent);
         }
         // extend recess beyond clipped donor in +z direction
         if (Math.abs(pos.z-zmax) < ztol) {
            pos.z += 1.0;
         }
         vtx.setPosition (pos);
      }
      mesh.notifyVertexPositionsModified();
      mesh = ComputeUtils.remeshByCrossSection (mesh, TDW, ztol/2, 20, 15, 2);
      return mesh;
   }

   /**
    * Create the small box that marks the top of the donor guide.
    */
   private PolygonalMesh createTopMarker (
      RigidTransform3d TBW, double baseLen, double maxYlen) {

      Vector3d widths = myGuideBaseWidths;
      Vector3i res = myGuideBaseRes;
      // max recess indentation relative to base origin
      double yrecess =
         -widths.y/2 + (1-mySetbackFraction)*maxYlen + myRecessClearance;
      double wx = widths.x/1.6;
      double wy = Math.min (widths.y/2 - yrecess, 3.0);
      if (wy <= 0) {
         System.out.println (
            "WARNING: recess extends to far into base to add top marker");
         return null;
      }
      PolygonalMesh mesh = MeshFactory.createBox (
         wx, wy, 1.0, Point3d.ZERO,         
         (int)Math.ceil(res.x*wx/widths.x), (int)Math.ceil(res.y*wy/widths.y), 1);
      RigidTransform3d TMW = new RigidTransform3d (TBW);
      TMW.mulXyz (0.0, (yrecess+widths.y/2)/2, baseLen/2);
      mesh.setMeshToWorld (TMW);
      return mesh;
   }

   /**
    * Intersect mesh with a series of planes aligned with the x-y plane of TBW
    * and placed along the z axis of TBW between [-range/2, range/2].  For each
    * intersection, find maximum x and y values of these intersexctions (with
    * respect to the B frame), or in other words, the upper x and y bounds of
    * the mesh in the plane. Then return the maximum and minimum of both these
    * bounds in an array of length 4:
    *
    * xybounds[0] = minmaxx;
    * xybounds[1] = maxmaxx;
    * xybounds[2] = minmaxy;
    * xybounds[3] = maxmaxy;
    */
   private double[] estimateXYBounds (
      PolygonalMesh mesh, RigidTransform3d TBW, double zrange) {

      double minmaxx = 0, maxmaxx = 0;
      double minmaxy = 0, maxmaxy = 0;

      int nsamps = 20;
      RigidTransform3d TPW = new RigidTransform3d();
      Vector3d zdir = new Vector3d();
      TBW.R.getColumn (2, zdir);
      PolygonalMesh cutplane = createCutPlaneMesh();
      Point3d pb = new Point3d();
      for (int i=0; i<nsamps; i++) {
         TPW.set (TBW);
         TPW.p.scaledAdd (-zrange/2 + i*(zrange/(nsamps-1)), zdir);
         cutplane.setMeshToWorld (TPW);
         IntersectionContour contour =
            ComputeUtils.findPrimaryIsectContour (mesh, cutplane);
         if (contour != null) {
            double maxx = -INF;
            double maxy = -INF;
            for (Point3d pworld : contour) {
               pb.inverseTransform (TBW, pworld);
               if (pb.x > maxx) {
                  maxx = pb.x;
               }
               if (pb.y > maxy) {
                  maxy = pb.y;
               }
            }
            if (i == 0) {
               minmaxx = maxx;
               maxmaxx = maxx;
               minmaxy = maxy;
               maxmaxy = maxy;
            }
            else {
               minmaxx = Math.min(maxx,minmaxx);
               maxmaxx = Math.max(maxx,maxmaxx);
               minmaxy = Math.min(maxy,minmaxy);
               maxmaxy = Math.max(maxy,maxmaxy);
            }
         }
      }
      return new double[] { minmaxx, maxmaxx, minmaxy, maxmaxy };
   }

   /**
    * Adds a top flange to the basic guide box.
    */
   public PolygonalMesh addTopFlange (
      PolygonalMesh mesh, double boxwx, double boxwy, double baseLen,
      double maxRadius, double maxYlen, double[] xybounds) {

      Vector3i res = myGuideBaseRes;
      double minmaxx = xybounds[0];
      double maxmaxx = xybounds[1];
      double minmaxy = xybounds[2];
      double maxmaxy = xybounds[3];
      double height = maxmaxx; // start point of flange along x
      double depth = myTopFlangeDepth;
      if (myRecessType == DonorRecessType.DONOR) {
         depth += (maxmaxx-minmaxx);
         height = minmaxx;
      }
      else {
         // height is maxRadius*sin(phi), where phi is the wedge angle for the
         // cylinder recess. See createCylinderRecessMesh().
         double rc =
            maxRadius - (1-mySetbackFraction)*maxYlen - myRecessClearance;
         height = Math.sqrt (Math.max(0, maxRadius*maxRadius - rc*rc));
      }
      depth = Math.max (depth, boxwx/2-height+1.0);
      double widthy = Math.max (myMinFlangeWidth, boxwy + maxmaxy);
      int resx = (int)Math.ceil (res.x*(depth/boxwx));
      int resy = (int)Math.ceil (res.y*(widthy/boxwy));
      PolygonalMesh flange = MeshFactory.createBox (
         depth, widthy, baseLen, new Point3d(), resx, resy, res.z);
      RigidTransform3d TFW = new RigidTransform3d();
      TFW.p.set (height+depth/2, -(widthy-boxwy)/2, 0);
      if (!mesh.meshToWorldIsIdentity()) {
         TFW.mul (mesh.getMeshToWorld(), TFW);
      }
      flange.setMeshToWorld (TFW);
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      mesh = intersector.findUnion (mesh, flange);
      return mesh;
   }

   public PolygonalMesh createFibulaGuide (
      PolygonalMesh clippedDonor, CubicHermiteSpline3d donorCurve,
      double maxRadius, Collection<DonorSegmentBody> donorSegments,
      Point3d endPoint) throws IOException {

      RigidTransform3d TCW = clippedDonor.getMeshToWorld();

      // set up coordinate frame for the box

      PolygonalMesh bladeInset = createBladeInset();
      PolygonalMesh bladeBox = createBladeBox();

      Vector3i res = myGuideBaseRes;
      Vector3d widths = myGuideBaseWidths;

      double z0 = donorCurve.getLastKnot().getS0();
      Point3d endPoint_d = new Point3d (endPoint);
      endPoint_d.inverseTransform (TCW);
      double zl = endPoint_d.z;
      
      // compute origin and z direction based on first and last z values for
      // the donor curve.
      Vector3d cl0_d = donorCurve.eval (z0);
      Vector3d cll_d = donorCurve.eval (zl);
      Vector3d zdir = new Vector3d();
      zdir.sub (cl0_d, cll_d);
      double len = zdir.norm();
      zdir.normalize();
      zdir.transform (TCW);
      Point3d ctr = new Point3d();
      ctr.add (cl0_d, cll_d); // compute center in donor coords ...
      ctr.scale (0.5);
      ctr.transform (TCW); // ... and transform to world

      // compute the y direction by averaging the vectors from the centerline
      // to the cut-plane centers of each of the donor segments.
      RigidTransform3d TBW = new RigidTransform3d();
      Vector3d ydir = new Vector3d();
      Vector3d cvec = new Vector3d();
      double maxYlen = 0;
      for (DonorSegmentBody body : donorSegments) {
         cvec = getCurveToPoint (body.getTPW_D(0).p, donorCurve, TCW);
         cvec.scaledAdd (-cvec.dot(zdir), cvec);
         maxYlen = Math.max (cvec.norm(), maxYlen);
         ydir.add (cvec);
         cvec = getCurveToPoint (body.getTPW_D(1).p, donorCurve, TCW);
         cvec.scaledAdd (-cvec.dot(zdir), cvec);
         maxYlen = Math.max (cvec.norm(), maxYlen);
         ydir.add (cvec);
      }
      // make perpendicular to z
      ydir.scaledAdd (-ydir.dot(zdir), zdir);
      ydir.normalize();
      TBW.R.setYZDirections (ydir, zdir);
      TBW.p.set (ctr);
      double[] xybounds = estimateXYBounds (clippedDonor, TBW, (z0-zl)*0.99);
      TBW.p.scaledAdd (widths.y/2+maxYlen*mySetbackFraction, ydir, ctr);
      //TBW.p.scaledAdd (widths.y/2, ydir, ctr);

      // TDW same as TBW but centered on the donor instead of the donor base:
      RigidTransform3d TDW = new RigidTransform3d (TBW);
      TDW.p.set (ctr);
      TDW.mulRotZ (-Math.PI/2);

      double baseLen = (widths.z <= 0 ? len : widths.z);
      PolygonalMesh mesh = MeshFactory.createBox (
         widths.x, widths.y, baseLen, new Point3d(), res.x, res.y, res.z);
      if (myTopFlangeDepth > 0) {
         mesh = addTopFlange (
            mesh, widths.x, widths.y, baseLen, maxRadius, maxYlen, xybounds);
      }
      mesh.setMeshToWorld (TBW);

      ArrayList<RigidTransform3d> TBBWlist = new ArrayList<>();
      // find the poses for the blade boxes
      for (DonorSegmentBody seg : donorSegments) {
         TBBWlist.add (getBladeBoxTransform (seg.getTPW_D(0), TBW));
         TBBWlist.add (getBladeBoxTransform (seg.getTPW_D(1), TBW));
      }
      // union the mesh with the blade boxes and remove the blade insets
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      for (int i=0; i<TBBWlist.size(); i++) {
         bladeBox.setMeshToWorld (TBBWlist.get(i));
         mesh = intersector.findUnion (mesh, bladeBox);
      }
      for (RigidTransform3d TBBW : TBBWlist) {
         bladeInset.setMeshToWorld (TBBW);
         mesh = intersector.findDifference01 (mesh, bladeInset);
      }
      // add the top marker
      PolygonalMesh marker = createTopMarker (TBW, baseLen, maxYlen);
      if (marker != null) {
         mesh = intersector.findUnion (mesh, marker);
      }
      // remove the recess in which to set the donor
      PolygonalMesh recess;
      switch (myRecessType) {
         case CYLINDER: {
            recess = createCylinderRecessMesh (
               maxRadius, baseLen, maxYlen, TBW);
            break;
         }
         case DONOR: {
            recess = createDonorRecessMesh (clippedDonor, donorCurve, TDW);
            break;
         }
         default:
            throw new InternalErrorException (
               "Unimplemented recess type "+myRecessType);
      }
      mesh = intersector.findDifference01 (mesh, recess);
      mesh.inverseTransform (TBW);

      FixedMeshBody body = setMeshBody ("donorGuide", mesh);
      body.setPose (TBW);
      RenderProps.setFaceColor (body, ReconAppRoot.GUIDE_COLOR);

      body = setMeshBody ("donorGuideBackup", mesh.clone());
      body.setPose (TBW);
      RenderProps.setVisible (body, false);
      myHasScrewHoles = false;
      myHasImplantHoles = false;
      return mesh;
   }

   public boolean hasHoles() {
      return myHasScrewHoles || myHasImplantHoles;
   }

   public PolygonalMesh addScrewHoles (PolygonalMesh screwMesh) {

      PolygonalMesh baseMesh = (PolygonalMesh)getBackupGuide().getMesh();
      
      RigidTransform3d TBW = new RigidTransform3d();
      baseMesh.getMeshToWorld (TBW);
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      PolygonalMesh mesh = intersector.findDifference01 (baseMesh, screwMesh);
      mesh.inverseTransform (TBW);
      
      FixedMeshBody body = setMeshBody ("donorGuide", mesh);
      body.setPose (TBW);
      RenderProps.setFaceColor (body, ReconAppRoot.GUIDE_COLOR);
      myHasScrewHoles = true;
      
      return mesh;     
   }

   public DonorSegmentBody nearestDonorSegment (
      RenderableComponentList<DonorSegmentBody> segments, Point3d pnt) {
      DonorSegmentBody nearest = null;
      double dist = INF;
      for (DonorSegmentBody seg : segments) {
         double d = seg.getMesh().distanceToPoint (pnt);
         if (d < dist) {
            dist = d;
            nearest = seg;
         }
      }
      return nearest;
   }

   public void addImplantHoles (
      RenderableComponentList<DonorSegmentBody> segments,
      RenderableComponentList<RigidBody> implants) {

      PolygonalMesh baseMesh = (PolygonalMesh)getMeshBodyMesh ("donorGuide");
      RigidTransform3d TBW = new RigidTransform3d();
      baseMesh.getMeshToWorld (TBW);

      PolygonalMesh holeMesh = new PolygonalMesh();
      for (RigidBody implant : implants) {
         RigidTransform3d TIW = implant.getPose();
         DonorSegmentBody donorSeg = nearestDonorSegment (
            segments, new Point3d(TIW.p));
         PolygonalMesh hole = MeshFactory.createCylinder (2.0, 20.0, 30, 1, 5);
         hole.transform (new RigidTransform3d (0, 0, 10));
         // transform from screw to donor
         RigidTransform3d TSD = new RigidTransform3d();
         TSD.mulInverseRight (
            donorSeg.getPoseD(), donorSeg.getMesh().getMeshToWorld());
         TSD.mul (TIW);
         hole.transform (TSD);
         holeMesh.addMesh (hole, /*respectTransform=*/false);
      }

      if (holeMesh.numVertices() > 0) {
         SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
         PolygonalMesh mesh = intersector.findDifference01 (baseMesh, holeMesh);
         mesh.inverseTransform (TBW);
      
         FixedMeshBody body = setMeshBody ("donorGuide", mesh);
         body.setPose (TBW);
         RenderProps.setFaceColor (body, ReconAppRoot.GUIDE_COLOR);
         myHasImplantHoles = true;
      }
   }

   public void clearHoles () {

      PolygonalMesh baseMesh = (PolygonalMesh)getBackupGuide().getMesh();
      
      RigidTransform3d TBW = new RigidTransform3d();
      baseMesh.getMeshToWorld (TBW);
      FixedMeshBody body = setMeshBody ("donorGuide", baseMesh);
      body.setPose (TBW);
      RenderProps.setFaceColor (body, ReconAppRoot.GUIDE_COLOR);
      myHasScrewHoles = false;
      myHasImplantHoles = false;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor) 
      throws IOException {
      pw.println ("hasScrewHoles=" + myHasScrewHoles);
      super.writeItems (pw, fmt, ancestor);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {
      rtok.nextToken();
      if (scanAttributeName (rtok, "hasScrewHoles")) {
         myHasScrewHoles = rtok.scanBoolean();
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok,tokens);
   }

}
