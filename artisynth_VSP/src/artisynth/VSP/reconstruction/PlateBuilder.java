package artisynth.VSP.reconstruction;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dList;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.materials.AnisotropicLinearMaterial;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponentList;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.CubicHermiteSpline3d;
import maspack.interpolation.Interpolation.Order;
import maspack.interpolation.LinearSpline3d;
import maspack.interpolation.NumericList;
import maspack.matrix.Matrix6d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.util.IntHolder;
import maspack.util.InternalErrorException;

/**
 * Worker component for creating the plate on the mandible, including the
 * markers and associated interpolating curve, an FEM model intended to create
 * more realistic plate geometry based on this curve, and screw hole positions.
 */
public class PlateBuilder extends WorkerComponentBase {

   // not a property yet ...
   public static int DEFAULT_NUM_PLATE_CURVE_SEGS = 500;
   
   
  public LinearSpline3d screwLine;

   // property definitions:

   static boolean DEFAULT_MARKERS_LEFT_TO_RIGHT = true;
   boolean myMarkersLeftToRight = DEFAULT_MARKERS_LEFT_TO_RIGHT;

   public static int DEFAULT_NUM_PLATE_ELEMS = 20;
   protected int myNumPlateElems = DEFAULT_NUM_PLATE_ELEMS;

   public static double DEFAULT_PLATE_THICKNESS = 2;
   protected double myPlateThickness = DEFAULT_PLATE_THICKNESS;

   public static double DEFAULT_PLATE_WIDTH = 7;
   protected double myPlateWidth = DEFAULT_PLATE_WIDTH;

   public static double DEFAULT_SPRING_STIFFNESS = 10;
   protected double mySpringStiffness = DEFAULT_SPRING_STIFFNESS;
 
   public static int DEFAULT_NUM_SCREWS = 1;
   protected int myNumScrews = DEFAULT_NUM_SCREWS;

   public static double DEFAULT_SCREW_OFFSET = 0;
   protected double myScrewOffset = DEFAULT_SCREW_OFFSET;

   public static double DEFAULT_SCREW_SPACING = 10;
   protected double myScrewSpacing = DEFAULT_SCREW_SPACING;

   public static double DEFAULT_SCREW_RADIUS = 1.3;
   protected double myScrewRadius = DEFAULT_SCREW_RADIUS;

   public static double DEFAULT_SCREW_LENGTH = 15;
   protected double myScrewLength = DEFAULT_SCREW_LENGTH;

   public static int DEFAULT_NUM_SCREW_SIDES = 10;
   protected int myNumScrewSides = DEFAULT_NUM_SCREW_SIDES;

   public static PropertyList myProps =
      new PropertyList (PlateBuilder.class, WorkerComponentBase.class);

   static {
      myProps.add (
         "markersLeftToRight", "direction of the plate markers", 
         DEFAULT_MARKERS_LEFT_TO_RIGHT);
      myProps.add (
         "numPlateElems", "number of elements in the plate FEM",
         DEFAULT_NUM_PLATE_ELEMS);
      myProps.add (
         "plateWidth", "width of the plate FEM",
         DEFAULT_PLATE_WIDTH);
      myProps.add (
         "plateThickness", "thickness of the plate FEM",
         DEFAULT_PLATE_THICKNESS);
      myProps.add (
         "springStiffness", "stiffness for the deforming springs",
         DEFAULT_SPRING_STIFFNESS);
      myProps.add (
         "numScrews", "number of screws to add",
         DEFAULT_NUM_SCREWS);
      myProps.add (
         "screwOffset", "screw hole offset relative to plate center",
         DEFAULT_SCREW_OFFSET);
      myProps.add (
         "screwSpacing", "spacing between screw holes",
         DEFAULT_SCREW_SPACING);
      myProps.add (
         "screwRadius", "radius of the screw holes",
         DEFAULT_SCREW_RADIUS);
      myProps.add (
         "screwLength", "length of the screw hole mesh",
         DEFAULT_SCREW_LENGTH);
      myProps.add (
         "numScrewSides", "number of sides in the screw holes mesh",
         DEFAULT_NUM_SCREW_SIDES);
      myProps.add (
         "screwsVisible", "sets whether the screws are visible",
         false);
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   // property accessors

   public boolean getMarkersLeftToRight() {
      return myMarkersLeftToRight;
   }

   public void setMarkersLeftToRight (boolean enable) {
      myMarkersLeftToRight = enable;
   }

   public int getNumPlateElems() {
      return myNumPlateElems;
   }

   public void setNumPlateElems (int num) {
      myNumPlateElems = num;
   }

   public double getPlateWidth() {
      return myPlateWidth;
   }

   public void setPlateWidth (double width) {
      myPlateWidth = width;
   }

   public double getPlateThickness() {
      return myPlateThickness;
   }

   public void setPlateThickness (double thickness) {
      myPlateThickness = thickness;
   }

   public double getSpringStiffness() {
      return mySpringStiffness;
   }

   public void setSpringStiffness (double k) {
      if (k != mySpringStiffness) {
         if (myMech != null) {
            for (AxialSpring spr : myMech.axialSprings()) {
               spr.setMaterial (new LinearAxialMaterial (k, 0));
            }
         }
         mySpringStiffness = k;
      }
   }

   public int getNumScrews() {
      return myNumScrews;
   }

   public void setNumScrews (int num) {
      if (myNumScrews != num) {
         myNumScrews = num;
         if (hasScrews()) {
            buildScrews();
         }
      }
   }

   public double getScrewSpacing() {
      return myScrewSpacing;
   }

   public void setScrewSpacing (double spacing) {
      if (myScrewSpacing != spacing) {
         myScrewSpacing = spacing;
         if (hasScrews()) {
            buildScrews();
         }
      }
   }

   public double getScrewOffset() {
      return myScrewOffset;
   }

   public void setScrewOffset (double offset) {
      if (myScrewOffset != offset) {
         myScrewOffset = offset;
         if (hasScrews()) {
            buildScrews();
         }
      }
   }

   public double getScrewRadius() {
      return myScrewRadius;
   }

   public void setScrewRadius (double rad) {
      if (myScrewRadius != rad) {
         myScrewRadius = rad;
         if (hasScrews()) {
            buildScrews();
         }
      }
   }

   public double getScrewLength() {
      return myScrewLength;
   }

   public void setScrewLength (double len) {
      if (myScrewLength != len) {
         myScrewLength = len;
         if (hasScrews()) {
            buildScrews();
         }
      }
   }

   public int getNumScrewSides() {
      return myNumScrewSides;
   }

   public void setNumScrewSides (int num) {
      if (myNumScrewSides != num) {
         myNumScrewSides = num;
         if (hasScrews()) {
            buildScrews();
         }
      }
   }
   
   // constructors

   /**
    * Note: need a public no-args constructor for ArtiSynth save/load.
    */
   public PlateBuilder () {
      setRenderProps (defaultRenderProps(this));
   }

   public PlateBuilder (String name) {
      this();
      setName (name);
   }

   // support for plate markers

   public boolean hasPlateMarkers() {
      return getPlateMarkers().size() > 0;
   }

   /**
    * Adds a new plate marker to the mandible.
    *
    * @param pworld marker position in world coordinates
    */
   public void addPlateMarker (Point3d pworld) {
      RigidBody mandible = getRigidBody ("mandible");
      if (mandible == null) {
         throw new InternalErrorException ("mandible not present");
      }
      Point3d loc = new Point3d();
      loc.inverseTransform (mandible.getPose(), pworld);
      FrameMarker mkr = new FrameMarker (mandible, loc);
      getPlateMarkers().add (mkr);
   }

   /**
    * Removes a plate marker from the mandible.
    *
    * @param mkr marker to remove
    */
   public boolean removePlateMarker (FrameMarker mkr) {
      return getPlateMarkers().remove (mkr);
   }

   /**
    * Removes all plate markers.
    *
    * @return {@code true} if there were any markers present
    */
   public boolean clearPlateMarkers() {
      PointList<FrameMarker> markers = getPlateMarkers();
      if (markers.size() > 0) {
         markers.removeAll();
         return true;
      }
      else {
         return false;
      }
   }

   /**
    * Return the list of plate markers.
    *
    * @return plate marker list
    */
   public PointList<FrameMarker> getPlateMarkers() {
      return (PointList<FrameMarker>)myMech.get ("plateMarkers");
   }

   // Support for the plate curve

   /**
    * Creates the plate curve from the current set of plate markers
    * and stores it as a PolylineMesh in a MeshBody named "plateCurve".
    */
   public void createPlateCurve () {
      PointList<FrameMarker> mkrs = getPlateMarkers();
      if (mkrs.size() < 2) {
         throw new IllegalArgumentException (
            "plate marker list contains less than 2 markers");
      }
      NumericList numericList = new NumericList (3);
      numericList.setInterpolationOrder (Order.Cubic);
      for (int i=0; i<mkrs.size(); i++) {
         double s = i/(double)(mkrs.size()-1);
         numericList.add (mkrs.get(i).getPosition(), s);
      }
      int nsegs = DEFAULT_NUM_PLATE_CURVE_SEGS;
      ArrayList<Point3d> interpolatedPnts = new ArrayList<>(nsegs+1);
      int[][] indices = new int[1][nsegs+1];
      VectorNd vec3 = new VectorNd(3);
      for (int k=0; k<=nsegs; k++) {
         double s = k/(double)nsegs;
         numericList.interpolate (vec3, s);         
         interpolatedPnts.add (new Point3d (vec3));
         indices[0][k] = k;
      }
      if (!getMarkersLeftToRight()) {
         Collections.reverse (interpolatedPnts);
      }
      PolylineMesh mesh = new PolylineMesh();
      mesh.set (interpolatedPnts.toArray(new Point3d[0]));
      MeshComponent mbody = setMeshBody ("plateCurve", mesh);
      RenderProps.setLineStyle (mbody, LineStyle.CYLINDER);
      RenderProps.setLineColor (mbody, ReconAppRoot.LINE_COLOR);
   }

   /**
    * Returns the plate curve, or {@code null} if there is no curve.
    */
   public FixedMeshBody getPlateCurve () {
      return getMeshBody ("plateCurve");
   }
   
   /**
    * Removes the plate curve, if any.
    *
    * @param {@code true} if a plate curve was present
    */
   public boolean removePlateCurve() {
      FixedMeshBody mbody = getMeshBody ("plateCurve");
      if (mbody != null) {
         myMech.removeMeshBody (mbody);
         return true;
      }
      else {
         return false;
      }
   }
  
   // Support for the plate FEM

   /**
    * Create nodes for a single FEM element face.
    *
    * @param fem FEM model to add the nodes to
    * @param TNW transform to map the node positions into world coordinates
    */
   private FemNode3d[] createFemNodes (FemModel3d fem, RigidTransform3d TNW) {
      double w= myPlateWidth;
      double t = myPlateThickness;
      Point3d[] pnts = new Point3d[] {
         new Point3d ( w/2, -t/2, 0), 
         new Point3d (-w/2, -t/2, 0), 
         new Point3d (-w/2,  t/2, 0), 
         new Point3d ( w/2,  t/2, 0)
      };
      FemNode3d[] nodes = new FemNode3d[4];
      for (int i=0; i<4; i++) {
         pnts[i].transform (TNW);
         nodes[i] = new FemNode3d (pnts[i]);
         fem.addNode (nodes[i]);
      }
      return nodes;      
   }

   /**
    * Queries if the plate FEM currently exists.
    */
   boolean hasPlateFem() {
      return getPlateFem() != null;
   }

   /**
    * Returns the plate FEM.
    */
   private FemModel3d getPlateFem() {
      return (FemModel3d)myMech.models().get("plateFem");
   }

   /**
    * Queries if the plate FEM is visible.
    */
   boolean isPlateFemVisible() {
      FemModel3d fem = getPlateFem();
      if (fem == null) {
         return false;
      }
      RenderableComponentList<AxialSpring> sprs = myMech.axialSprings();
      return (isVisible(fem) && isVisible(sprs));
   }

   /**
    * Sets whether the plate FEM is visible.
    */
   boolean setPlateFemVisible (boolean enable) {
      boolean changed = false;
      FemModel3d fem = getPlateFem();
      if (fem != null) {
         RenderableComponentList<AxialSpring> sprs = myMech.axialSprings();
         changed |= setVisible (fem, enable);
         changed |= setVisible (sprs, enable);
      }
      return changed;
   }

   public void setPlateFemDynamic (boolean enable) {
      FemModel3d fem = getPlateFem();
      if (fem != null) {
         for (FemNode3d n : fem.getNodes()) {
            n.setDynamic (enable);
         }
      }
   }

   /**
    * Queries if the plate FEM curve is visible.
    */
   boolean isPlateFemCurveVisible() {
      Spline3dBody curve = getCurve("femPlateCurve");
      return (isVisible(curve));
   }

   /**
    * Sets whether the plate FEM curve is visible.
    */
   boolean setPlateFemCurveVisible (boolean enable) {
      boolean changed = false;
      Spline3dBody curve = getCurve("femPlateCurve");
      if (curve != null) {
         changed |= setVisible (curve, enable);
      }
      return changed;
   }

   /**
    * Removes the plate FEM.
    */
   void clearPlateFem () {
      FemModel3d fem = (FemModel3d)myMech.models().get("plateFem");
      RigidBody recon = myMech.rigidBodies().get("reconstruction");
      if (fem != null) {
         if (recon != null) {
            myMech.clearCollisionBehavior (fem, recon);
         }
         myMech.removeModel (fem);
      }
      PointList<FrameMarker> markers =
         (PointList<FrameMarker>)myMech.get ("deformMarkers");
      markers.clear();
      myMech.clearAxialSprings();
      removeCurve ("femPlateCurve");
   }

   void addAxialSpring (FrameMarker mkr, FemNode3d node) {
      AxialSpring spr = new AxialSpring (mySpringStiffness, 0, 0);
      myMech.attachAxialSpring (mkr, node, spr);
   }

   /**
    * Create the plate FEM. This entire construction consists of:
    * <ul>
    * 
    * <li>A spline curve that creates a smoother approximation of the plate
    * curve and which stands off from the mandible by the distance {@code
    * standOff}.</li>

    * <li>An FEM model consisting of a single band of thin hex elements
    * arranged along, and centered on, the spline curve.</li>
    *
    * <li>A set of axial springs connecting the mandible-facing FEM nodes to
    * corresponding positions along the plate curve. These are used
    * to pull the FEM into position.</li>
    *
    * </ul>
    */
   void buildPlateFem (
      Collection<FrameMarker> markers, PolylineMesh plateCurve, 
      PolygonalMesh mandibleMesh, RigidBody recon, double standoff) {

      clearPlateFem ();

      // Fit each plate marker, find a corresponding point projected away from
      // the mandible surface by 'standoff', together with the surface normal
      // at the point.
      BVFeatureQuery query = new BVFeatureQuery();
      ArrayList<Point3d> points = new ArrayList<>();
      ArrayList<Vector3d> normals = new ArrayList<>();
      for (FrameMarker mkr : markers) {
         Point3d pos = new Point3d(mkr.getPosition());
         Vector3d nrm = ComputeUtils.estimateSurfaceNormal (pos, mandibleMesh);
         normals.add (nrm);
         pos.scaledAdd (standoff, nrm, pos);
         points.add (pos);
      }
      // Fit the point and normal values to single-segment cubic splines
      CubicHermiteSpline3d femCurve = new CubicHermiteSpline3d();
      double[] svals = new double[] {0, 1, 2};
      //femCurve.setSingleSegment (points, 0, 1);
      femCurve.setMultiSegment (points, svals);
      CubicHermiteSpline3d femNormals = new CubicHermiteSpline3d();
      //femNormals.setSingleSegment (normals, 0, 1);
      femNormals.setMultiSegment (normals, svals);

      // Now approximate both curves using a linear spline so that we can
      // compute values based on distance along the curve
      int nsamps = 200;
      LinearSpline3d femCurveL = new LinearSpline3d();
      femCurveL.setUsingDistance (femCurve.getSampledValues(nsamps));
      double femLen = femCurveL.getSLength();
      LinearSpline3d femNormalsL = new LinearSpline3d();
      femNormalsL.set (
         femCurveL.getSValues(), femNormals.getSampledValues(nsamps));

      FemModel3d fem = new FemModel3d ("plateFem");
      AnisotropicLinearMaterial material = new AnisotropicLinearMaterial ();
      Matrix6d stiffTensor = material.getStiffnessTensor ();
      fem.setDensity (1000);
      fem.setStiffnessDamping (10);
      fem.setParticleDamping (0.1);

      IntHolder lastIdx = new IntHolder();
      RigidTransform3d TNW = new RigidTransform3d();
      FemNode3d[] nprev = null;
      for (int i=0; i<=myNumPlateElems; i++) {
         // create coordinate frame for node cross section
         double s = i*femLen/myNumPlateElems;
         TNW.p.set (femCurveL.eval(s, lastIdx));
         Vector3d zdir = femCurveL.evalDx(s, lastIdx);
         zdir.normalize();
         zdir.negate();
         Vector3d ydir = femNormalsL.eval(s, lastIdx);
         ydir.scaledAdd (-ydir.dot(zdir), zdir);
         TNW.R.setYZDirections (ydir, zdir);
         FemNode3d[] nodes = createFemNodes (fem, TNW);
         if (i > 0) {
            fem.addElement (
               new HexElement (
                  nodes[0], nodes[1], nodes[2], nodes[3],
                  nprev[0], nprev[1], nprev[2], nprev[3])); 
         }
         nprev = nodes;
      }
      RenderProps.setVisible (fem.getNodes(), false);
      RenderProps.setFaceColor (fem, ReconAppRoot.PLATE_COLOR);
      fem.setSurfaceRendering (SurfaceRender.Shaded);
      myMech.addModel (fem);

      // Create corresponding linear spline for the plateCurve
      LinearSpline3d plateCurveL = new LinearSpline3d();
      points.clear();
      for (Vertex3d vtx : plateCurve.getVertices()) {
         points.add (vtx.getPosition());
      }
      plateCurveL.setUsingDistance (points);
      double plateLen = plateCurveL.getSLength();

      // add deformation markers to the reconstuction mesh and axial springs to
      // deform the FEM
      PointList<FrameMarker> deformMarkers =
         (PointList<FrameMarker>)myMech.get ("deformMarkers");
      for (int i=0; i<=myNumPlateElems; i++) {
         // create coordinate frame for node cross section
         double s = i*plateLen/myNumPlateElems;
         Point3d pnt = new Point3d (plateCurveL.eval(s, lastIdx));
         // project to surface of recon:
         Point3d loc = new Point3d();
         recon.getSurfaceMesh().distanceToPoint (loc, pnt);
         loc.inverseTransform (recon.getPose(), loc);
         FrameMarker mkr = new FrameMarker (recon, loc);
         deformMarkers.add (mkr);
         addAxialSpring (mkr, fem.getNode(i*4));
         addAxialSpring (mkr, fem.getNode(i*4+1));
      }

      myMech.setCollisionBehavior (fem, recon, true);

      Spline3dBody sbody = setCurve (
         "femPlateCurve", femCurve, RigidTransform3d.IDENTITY);
      RenderProps.setVisible (sbody, false);
   }

   // plate screw support

   /**
    * Returns the screw mesh list. This is a dedicated list of FixedMeshBody
    * named "screws".
    */
   RenderableComponentList<FixedMeshBody> getScrews() {
      return (RenderableComponentList<FixedMeshBody>)myMech.get("screws");
   }

   /**
    * Returns {@code true} if any screws are currently present.
    */
   boolean hasScrews() {
      return getScrews().size() > 0;
   }

   /**
    * Queries whether the screws are currently visible.
    */
   public boolean getScrewsVisible() {
      return hasScrews() && isVisible (getScrews());
   }

   /**
    * Sets whether the screws are visible.
    */
   public boolean setScrewsVisible (boolean enable) {
      return setVisible (getScrews(), enable);
   }

   /**
    * Removes all screws.
    */      
   void clearScrews () {
      getScrews().clear();
   }

   /**
    * Compute the normal of the plate element face that is pressing towards the
    * mandible. The normal is directed *away* from the mandible.
    */
   private Vector3d elemFaceNormal (FemModel3d fem, int idx) {
      Vector3d xprod = new Vector3d();

      Point3d p0 = fem.getNode(4*idx).getPosition();
      Vector3d d01 = new Vector3d();
      Vector3d d04 = new Vector3d();
      Vector3d d05 = new Vector3d();
      d01.sub (fem.getNode(4*idx+1).getPosition(), p0);
      d04.sub (fem.getNode(4*idx+4).getPosition(), p0);
      d05.sub (fem.getNode(4*idx+5).getPosition(), p0);
      xprod.cross (d04, d05);
      xprod.crossAdd (d05, d01, xprod);
      xprod.normalize();
      return xprod;      
   }

   /**
    * Creates the plate screws and stores them in a dedicated list
    * of FixedMeshBody named "screws".
    */
   void buildScrews () {

      clearScrews();
      if (myNumScrews < 1) {
         // just in case :)
         return;
      }
      
      FemModel3d fem = (FemModel3d)myMech.models().get("plateFem");
      RenderableComponentList<FixedMeshBody> screws = getScrews();

      // Find the centers of each vertical FEM edge facing the mandible.
      int numElems = fem.numElements();
      ArrayList<Point3d> points = new ArrayList<>();

      
      for (int i=0; i<=numElems; i++) {
         Point3d pnt = new Point3d();
         // since we know the exact structure of the FEM, we know that the
         // mandible-facing vertical edges are delimited by nodes 0, 1, 4, 5,
         // 8, 9, etc.
         pnt.add (
            fem.getNode(4*i).getPosition(), fem.getNode(4*i+1).getPosition());
         pnt.scale (0.5);
         points.add (pnt);
      }
      

      PointList<FrameMarker> rdpMarkers = (PointList<FrameMarker>)myMech.get("RDPLinePoints");
      FemElement3dList<FemElement3d> elements = fem.getElements();

      // Create an ArrayList to hold the positions of the FrameMarkers
      
      ArrayList<Point3d> rdpPoints0 = new ArrayList<>();
      ArrayList<Point3d> rdpPoints1 = new ArrayList<>();

      LinearSpline3d screwLineRDP1 = new LinearSpline3d();
      LinearSpline3d screwLineRDP0 = new LinearSpline3d();

      double[] screwLineLenRDP = {0,0};
      

      rdpPoints0.add(rdpMarkers.get (0).getPosition());
      rdpPoints0.add(rdpMarkers.get (1).getPosition());

   
      screwLineRDP0.setUsingDistance (rdpPoints0);
      double screwLineLenRDP0 = screwLineRDP0.getSLength();
      screwLineLenRDP[0] = screwLineLenRDP0;
  
         
      
       if (myNumScrews == 2) {
        
          rdpPoints1.add(rdpMarkers.get (1).getPosition());
          rdpPoints1.add(rdpMarkers.get (2).getPosition());
         
          screwLineRDP1.setUsingDistance (rdpPoints1);
          double screwLineLenRDP1 = screwLineRDP1.getSLength();
          
          screwLineLenRDP[1] =  screwLineLenRDP1;
         
      }

      
      screwLine = new LinearSpline3d();
      screwLine.setUsingDistance (points);
      double screwLineLen = screwLine.getSLength();
      double[] dists = screwLine.getSValues();
      double[] midElemDists = new double[numElems];
     
      
      ArrayList<Vector3d> normals = new ArrayList<>();
      for (int i=0; i<numElems; i++) {
         midElemDists[i] = (dists[i]+dists[i+1])/2;
         normals.add (elemFaceNormal (fem, i));
      }
      LinearSpline3d screwNormals = new LinearSpline3d();
      screwNormals.set (midElemDists, normals);

      int nums = myNumScrews;

      // create a template mesh for each screw
      PolygonalMesh baseMesh = MeshFactory.createCylinder (
         myScrewRadius, myScrewLength, myNumScrewSides,
         /*nr=*/1, (int)Math.ceil(myScrewLength/(2*myScrewRadius)));
      baseMesh.transform (new RigidTransform3d (0, 0, 0, 0, 0, Math.PI/2));
      RigidTransform3d TSW = new RigidTransform3d();         

      IntHolder lastIdx = new IntHolder();
      // create {@code nums} screws evenly spaced along the screwline
      for (int i=0; i<nums; i++) {
         
         // Original code
         //double d = d0 + i*spacing; // screw location along the screwline
         
         //mine
         //double d = screwLineLen/2;
         
         //mine new version 
            double d = screwLineLenRDP[i]/2;
         
            
            //updated
            if (i==0) {
               
               Vector3d pos0 = screwLineRDP0.eval(d, lastIdx);

               Point3d closestPoint0 = null;
               double closestDistance0 = Double.MAX_VALUE;

               for (FemElement3d element0 : elements) {
                   Point3d centroid0 = new Point3d();
                   element0.computeCentroid(centroid0);
                   double distance0 = centroid0.distance(pos0);
                   if (distance0 < closestDistance0) {
                       closestDistance0 = distance0;
                       closestPoint0 = centroid0;
                   }
               }
               
               
               //TSW.p.set (new Vector3d (closestPoint0.x, closestPoint0.y, closestPoint0.z));
               //TSW.p.set (new Vector3d (pos0.x, pos0.y, pos0.z));
               
               //For B Defect 
               TSW.p.set (new Vector3d ((rdpMarkers.get (0).getPosition().x + rdpMarkers.get (1).getPosition().x)/2 ,  (rdpMarkers.get (0).getPosition().y + rdpMarkers.get (1).getPosition().y )/2 , (rdpMarkers.get (0).getPosition().z + rdpMarkers.get (1).getPosition().z)/2 ));   

               
               //For S Defect 
               //TSW.p.set (new Vector3d ((rdpMarkers.get (0).getPosition().x + rdpMarkers.get (1).getPosition().x)/2 ,  (rdpMarkers.get (0).getPosition().y + rdpMarkers.get (1).getPosition().y - 10)/2 , (rdpMarkers.get (0).getPosition().z + rdpMarkers.get (1).getPosition().z)/2 ));   

               
               //For RB Defect 
               //TSW.p.set (new Vector3d ((rdpMarkers.get (0).getPosition().x + rdpMarkers.get (1).getPosition().x)/2 ,  ((rdpMarkers.get (0).getPosition().y + rdpMarkers.get (1).getPosition().y )+3)/2 , ((rdpMarkers.get (0).getPosition().z + rdpMarkers.get (1).getPosition().z)-7)/2 ));   

            }
            
            if (i==1) {
               
               Vector3d pos1 =  screwLineRDP1.eval(d, lastIdx);

               Point3d closestPoint1 = null;
               double closestDistance1 = Double.MAX_VALUE;

               for (FemElement3d element1 : elements) {
                   Point3d centroid1 = new Point3d();
                   element1.computeCentroid(centroid1);
                   double distance1 = centroid1.distance(pos1);
                   if (distance1 < closestDistance1) {
                       closestDistance1 = distance1;
                       closestPoint1 = centroid1;
                   }
               }
               
               
               //TSW.p.set (new Vector3d (closestPoint1.x, closestPoint1.y, closestPoint1.z));
               //TSW.p.set (new Vector3d (pos1.x, pos1.y, pos1.z));
          
               TSW.p.set (new Vector3d ((rdpMarkers.get (1).getPosition().x + rdpMarkers.get (2).getPosition().x)/2 ,  ((rdpMarkers.get (1).getPosition().y + rdpMarkers.get (2).getPosition().y)+5)/2 , ((rdpMarkers.get (1).getPosition().z + rdpMarkers.get (2).getPosition().z)-10)/2 ));   


            }

           
            d = screwLineLen/2;

            Vector3d zdir = screwLine.evalDx(d, lastIdx);
            zdir.normalize();
            zdir.negate();
            System.out.println("Z direction: " + zdir.toString());

            Vector3d ydir = screwNormals.eval(d);
            ydir.normalize();
            System.out.println("Y direction: " + ydir.toString());
            
            
            //For S Defect 
            //Vector3d ydir1 = new Vector3d (0, -1, -.2);
            //Vector3d zdir1 = new Vector3d (0.9821693523530379, -0.1877767306047194, 0.00912484201597602);
            

            
           // for B Defect
           
            Vector3d ydir1 = new Vector3d (-0.8831891864902558, -0.46611507271598007, -0.0520922245028643);
            Vector3d zdir1 = new Vector3d ( 0.44297740956143233, -0.8709352841458438, -0.21270342133600262);
           
            
            
           //For RB Defect 
           
           //Vector3d ydir1 = new Vector3d (-0.7241536065000872, -0.5816421632264681, 0.017010903582818565);
           //Vector3d zdir1 = new Vector3d ( 0.3011589228424498, -0.8178761688010696, -0.4902875438960889);
           
            
                 
            TSW.R.setYZDirections (ydir1, zdir1);
            FixedMeshBody body = new FixedMeshBody (baseMesh.clone());
            body.setPose (TSW);
            screws.add (body);
            
         
      }
   }

   /**
    * Creates a composite mesh of all the screws in mandible space.
    *
    * @return composite mesh of screws in mandible space
    */
   PolygonalMesh getMandibleScrewMesh() {
      PolygonalMesh screwMesh = new PolygonalMesh();
      RenderableComponentList<FixedMeshBody> screws = getScrews();
      for (FixedMeshBody body : getScrews()) {
         screwMesh.addMesh (
            (PolygonalMesh)body.getMesh(), /*respectTransform=*/true);
      }
      return screwMesh;
   }

   /**
    * Creates a mesh of the plate FEM surface, with screw holesif screws are
    * defined.
    *
    * @return mesh of plate FEM surface with screw holes
    */
   PolygonalMesh getPlateFemSurfaceWithHoles() {
      PolygonalMesh surface = getPlateFem().getSurfaceMesh();
      if (hasScrews()) {
         PolygonalMesh screws = getMandibleScrewMesh();
         SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
         return intersector.findDifference01 (surface, screws);
      }
      else {
         return surface.clone();
      }
   }

   /**
    * Returns the number of screws currently selected in the ArtiSynth viewer.
    *
    * @return number of currently selected screws
    */
   int numSelectedScrews() {
      int num = 0;
      for (FixedMeshBody screw : getScrews()) {
         if (screw.isSelected()) {
            num++;
         }
      }
      return num;
   }

   /**
    * Creates a composite mesh of all the screws in donor space. Only
    * screws whose axis intersects one of the donor segments are included.
    *
    * @param segments list of donor segments
    * @param selectedScrewsOnly if {@code true}, further restricts the
    * screws to only those that are selected
    * @return composite mesh of the screws in donor space
    */
   PolygonalMesh getDonorScrewMesh (
      RenderableComponentList<DonorSegmentBody> donorBodies,
      boolean selectedScrewsOnly) {
      PolygonalMesh screwMesh = new PolygonalMesh();
      RenderableComponentList<FixedMeshBody> screws = getScrews();
      BVFeatureQuery query = new BVFeatureQuery();
      for (FixedMeshBody screw : getScrews()) {
         if (selectedScrewsOnly && !screw.isSelected()) {
            continue;
         }
         RigidTransform3d TSW = screw.getPose();
         Vector3d ydir = new Vector3d();
         TSW.R.getColumn (1, ydir);
         Point3d pnt = new Point3d(TSW.p);
         // add the screw if it intersects one of the donor bodies
         for (DonorSegmentBody donorSeg : donorBodies) {
            Point3d nearPnt = new Point3d();
            if (query.nearestFaceAlongLine (
                   nearPnt, null, donorSeg.getMesh(),
                   pnt, ydir, -30, 30) != null) {
               PolygonalMesh smesh = (PolygonalMesh)screw.getMesh().clone();
               // transform from screw to donor
               RigidTransform3d TSD = new RigidTransform3d();
               TSD.mulInverseRight (
                  donorSeg.getPoseD(), donorSeg.getMesh().getMeshToWorld());
               TSD.mul (TSW);
               smesh.transform (TSD);
               screwMesh.addMesh (smesh, /*respectTransform=*/false);
            }
         }
      }
      return screwMesh;
   }

   /**
    * Rendering method - currently used for diagnostic rendering only.
    */
   public void render (Renderer renderer, int flags) {
      // CubicHermiteSpline3d curve = myPointCurve;
      // CubicHermiteSpline3d normalCurve = myNormalCurve;
      // if (curve != null && normalCurve != null) {
      //    int nsegs = 50;
      //    for (int i=0; i<=nsegs; i++) {
      //       double s = i/(double)nsegs;
      //       Point3d pnt0 = new Point3d(curve.eval(s));
      //       Vector3d u = curve.evalDx(s);
      //       Point3d pnt1 = new Point3d(pnt0);
      //       Vector3d nrm = new Vector3d(normalCurve.eval(s));
      //       u.normalize();
      //       nrm.scaledAdd (-nrm.dot(u), u);
      //       nrm.normalize();
      //       pnt1.scaledAdd (10, nrm);
      //       renderer.drawArrow (pnt0, pnt1, 0.2, false);
      //    }
      // }
      // LinearSpline3d curve = myPointCurve;
      // LinearSpline3d normalCurve = myNormalCurve;
      // if (curve != null && normalCurve != null) {
      //    int nsegs = 100;
      //    for (int i=0; i<=nsegs; i++) {
      //       double s = curve.getS0() + i*curve.getSLength()/(double)nsegs;
      //       Point3d pnt0 = new Point3d(curve.eval(s));
      //       Point3d pnt1 = new Point3d(pnt0);
      //       Vector3d nrm = new Vector3d(normalCurve.eval(s));
      //       nrm.normalize();
      //       pnt1.scaledAdd (10, nrm);
      //       renderer.drawArrow (pnt0, pnt1, 0.2, false);
      //    }
      // }
   }
}
