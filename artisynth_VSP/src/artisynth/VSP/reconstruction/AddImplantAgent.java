/**
 * Copyright (c) 2014, by the Authors: John Lloyd (UBC), Tracy Wilkinson (UBC) and

 * ArtiSynth Team Members
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Component;
import java.awt.Font;
import java.awt.Cursor;
import java.awt.event.*;
import java.util.*;

import javax.swing.*;
import javax.swing.event.*;

import maspack.render.*;
import maspack.render.GL.GLViewer;
import maspack.render.Renderer.PointStyle;
import maspack.geometry.*;
import maspack.matrix.*;
import maspack.util.*;
import maspack.properties.*;
import artisynth.core.driver.*;
import artisynth.core.renderables.*;
import artisynth.core.gui.*;
import artisynth.core.gui.editorManager.*;
import artisynth.core.gui.selectionManager.*;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.workspace.RootModel;

/**
 * Responsible for adding implants to a reconstruction model.
 */
public class AddImplantAgent extends AddComponentAgent<RigidBody> {

   private MechModel myModel;
   private DicomPlaneViewer myOcclusalSlice;
   private RenderableComponentList<Implant> myImplantList;
   private PointList<FrameMarker> myPlateMarkers;
   private ImplantsManager myImplantsManager;
   private MeshManager myMeshManager;
   
   private static HashMap<Class,ModelComponent> myPrototypeMap;
   private static RootModel myLastRootModel = null;
   
   protected void setInitialState() {
      setState (State.SelectingLocation);
   }

   protected void initializePrototype (
      ModelComponent comp, ComponentList<?> container, Class type) {
   }

   protected void resetState() {
      setState (State.SelectingLocation);
   }

   private enum State {
      SelectingLocation,
   };

   private State myState = State.SelectingLocation;

   private void setState (State state) {
      switch (state) {
         case SelectingLocation: {
            myInstructionBox.setText ("Pick location in viewer");
            installLocationListener();
            break;
         }
         default: {
            throw new InternalErrorException ("Unhandled state " + state);
         }
      }
      myState = state;
   }

   public AddImplantAgent (
      Main main, MechModel model,
      MeshManager meshManager,
      ImplantsManager implantsManager,
      PointList<FrameMarker> plateMarkers,
      DicomPlaneViewer oslice) {
      super (main, implantsManager.getImplants(), model);
      myMeshManager = meshManager;
      myImplantsManager = implantsManager;
      myImplantList = implantsManager.getImplants();
      myPlateMarkers = plateMarkers;
      myModel = model;
      myOcclusalSlice = oslice;
   }

   protected void createDisplay() {
      createDisplayFrame ("Add Particles");

      addComponentType (
         RigidBody.class,
         /*excludedProps=*/new String[] {
            "name", "position", "orientation", "velocity", 
            "targetPosition", "targetOrientation", "targetVelocity", 
            "targetActivity", "force", "transForce", "moment", "externalForce",
            "axisLength", "axisDrawStyle", "frameDamping", "rotaryDamping",
            "inertialDamping", "inertiaMethod", "density", "mass",
            "inertia", "centerOfMass", "dynamic", "collidable",
            "gridSurfaceRendering" });
         
      createComponentList (
         "Existing implants:",
         new ComponentListWidget<Implant> (
            myImplantList, myModel));
      // createSeparator();
      // createTypeSelector();
      //createNameField();
      createPropertyFrame("Default implant properties:");
      // createSeparator();
      // createProgressBox();
      createInstructionBox();
      createOptionPanel ("Done");
   }

   protected HashMap<Class,ModelComponent> getPrototypeMap() {
      RootModel root = myMain.getRootModel();
      if (root != null && root != myLastRootModel) {
         myPrototypeMap = new HashMap<Class,ModelComponent>();
         myLastRootModel = root;
      }
      return myPrototypeMap;
   }
   
   private Vector3d findPlateTangent (Point3d pnt) {
      // find the nearest point on the plate markers
      double nearestDist = Double.POSITIVE_INFINITY;
      int nearestIdx = -1;
      int nmkrs = myPlateMarkers.size();
      if (nmkrs < 3) {
         return null;
      }
      for (int i=0; i<nmkrs; i++) {
         double d = pnt.distance (myPlateMarkers.get(i).getPosition());
         if (d < nearestDist) {
            nearestDist = d;
            nearestIdx = i;
         }
      }
      Vector3d tan = new Vector3d();
      Point3d p0;
      Point3d p1;
      if (nearestIdx == 0) {
         p1 = myPlateMarkers.get(1).getPosition();
         p0 = myPlateMarkers.get(0).getPosition();
      }
      else if (nearestIdx == nmkrs-1) {
         p1 = myPlateMarkers.get(nmkrs-1).getPosition();
         p0 = myPlateMarkers.get(nmkrs-2).getPosition();
      }
      else {
         p1 = myPlateMarkers.get(nearestIdx+1).getPosition();
         p0 = myPlateMarkers.get(nearestIdx-1).getPosition();
      }
      tan.sub (p1, p0);
      tan.normalize();
      return tan;
   }
        
   private Point3d rayIntersectsBody (
      RigidBody body, MouseRayEvent rayEvent, DoubleHolder minDist) {
      if (body.getSurfaceMesh() != null) {
         Point3d isectPoint = BVFeatureQuery.nearestPointAlongRay (
            body.getSurfaceMesh (),
            rayEvent.getRay().getOrigin(), rayEvent.getRay().getDirection());             
         if (isectPoint != null) {
            double d = isectPoint.distance (rayEvent.getRay().getOrigin());
            if (d < minDist.value) {
               minDist.value = d;
               return isectPoint;
            }
         }
      } 
      return null;
   }

   private Point3d rayIntersectsMesh (
      PolygonalMesh mesh, MouseRayEvent rayEvent, DoubleHolder minDist) {
      if (mesh != null) {
         Point3d isectPoint = BVFeatureQuery.nearestPointAlongRay (
            mesh, rayEvent.getRay().getOrigin(),
            rayEvent.getRay().getDirection());             
         if (isectPoint != null) {
            double d = isectPoint.distance (rayEvent.getRay().getOrigin());
            if (d < minDist.value) {
               minDist.value = d;
               return isectPoint;
            }
         }
      } 
      return null;
   }

   public void handleLocationEvent (GLViewer viewer, MouseRayEvent rayEvent) {
      DoubleHolder minDist = new DoubleHolder(Double.POSITIVE_INFINITY);

      ArrayList<PolygonalMesh> meshes = new ArrayList<>();
      RigidBody mandible = myMeshManager.getMandible();
      RigidBody maxilla = myMeshManager.getMaxilla();
      OcclusalPlane oplane = myMeshManager.getOcclusalPlane();

      RenderableComponentList<DonorSegmentBody> donorSegments =
         (RenderableComponentList<DonorSegmentBody>)myModel.get("donorSegments");

      if (mandible != null && mandible.getRenderProps().isVisible()) {
         meshes.add (mandible.getSurfaceMesh());
      }
      if (maxilla != null && maxilla.getRenderProps().isVisible()) {
         meshes.add (maxilla.getSurfaceMesh());
      }
      if (oplane != null &&
          oplane.getRenderProps().isVisible()) {
         meshes.add (oplane.getSurfaceMesh());
      }
      for (DonorSegmentBody seg : donorSegments) {
         meshes.add (seg.getMesh());
      }

      if (myOcclusalSlice != null &&
          myOcclusalSlice.getRenderProps().isVisible()) {
         meshes.add (myOcclusalSlice.getSurfaceMesh());
      }
      Point3d nearestIntersection = null;
      for (PolygonalMesh mesh : meshes) {
         Point3d isectPoint = rayIntersectsMesh (mesh, rayEvent, minDist);
         if (isectPoint != null) {
            nearestIntersection = isectPoint;
         }
      }     
      if (nearestIntersection != null) {
         createAndAddImplant (nearestIntersection);
      }
   }

   private void createAndAddImplant (Point3d pnt) {
      RigidBody implant = myImplantsManager.createImplant();

      Vector3d zdir = new Vector3d();
      OcclusalPlane oplane = myMeshManager.getOcclusalPlane();
      RigidTransform3d TPW = oplane.getPose();
      Plane plane = new Plane (TPW);
      TPW.R.getColumn (2, zdir);

      RigidTransform3d TIW = new RigidTransform3d();

      plane.project (pnt, pnt);
      double offset =
         myImplantsManager.getImplantLength()/2 + 
         myImplantsManager.getOcclusalOffset();
      pnt.scaledAdd (-offset, zdir);
      TIW.p.set (pnt);

      // Vector3d xprod = new Vector3d();
      // xprod.cross (tan, Vector3d.Z_UNIT);
      // xprod.cross (xprod, tan);
      //tan.negate();
      //TIW.R.setZXDirections (xprod, tan);      

      Vector3d tan = findPlateTangent (pnt);      
      if (tan != null) {
         tan.negate();
         TIW.R.setZXDirections (zdir, tan); 
      }
      else {
         TIW.R.setZDirection (zdir); 
      }
      implant.setPose (TIW);
      addComponent (new AddComponentsCommand (
         "add implant", implant,
         (MutableCompositeComponent<?>)myImplantList));

      setState (State.SelectingLocation);
   }

   protected boolean isContextValid() {
      return (ComponentUtils.withinHierarchy (myModel, myMain.getRootModel()));
   }

}
