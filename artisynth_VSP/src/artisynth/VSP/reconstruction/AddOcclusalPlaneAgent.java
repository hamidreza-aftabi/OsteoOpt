/**
 * Copyright (c) 2014, by the Authors: John Lloyd (UBC), Tracy Wilkinson (UBC) and
 * ArtiSynth Team Members
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.VSP.reconstruction;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.*;
import java.io.*;

import javax.swing.*;

import artisynth.core.driver.Main;
import artisynth.core.gui.SelectableComponentPanel;
import artisynth.core.gui.selectionManager.SelectionEvent;
import artisynth.core.gui.selectionManager.SelectionFilter;
import artisynth.core.gui.selectionManager.SelectionListener;
import artisynth.core.gui.editorManager.*;
import artisynth.core.gui.widgets.NodeNumberFileChooser;
import artisynth.core.util.ExtensionFileFilter;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.*;
import artisynth.core.femmodels.*;
import maspack.properties.EditingProperty;
import maspack.properties.HasProperties;
import maspack.properties.HostList;
import maspack.properties.PropTreeCell;
import maspack.util.InternalErrorException;
import maspack.util.ListView;
import maspack.widgets.*;
import maspack.geometry.*;
import maspack.matrix.*;
import maspack.widgets.DoubleField;
import maspack.widgets.GuiUtils;
import maspack.widgets.PropertyPanel;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;

public class AddOcclusalPlaneAgent extends FrameMarkerAgent {

   MeshManager myMeshManager;
   ImplantsManager myImplantsManager;

   public AddOcclusalPlaneAgent (
      Main main, MechModel mech,
      MeshManager meshManager, ImplantsManager implantsManager) {
      super (main, mech);
      myMeshManager = meshManager;
      myImplantsManager = implantsManager;
   }

   public void show (Rectangle popupBounds) {
      super.show (popupBounds);
   }

   public void dispose() {
      super.dispose();
      myEditManager.releaseEditLock();
   }

   protected void createDisplay() {
      createDisplayFrame ("Add occlusal plane");
      addComponentType (FrameMarker.class, new String[] { 
            "position", "velocity", "externalForce", "refPos", 
            "location", "name", "pointDamping", "displacement",
            "targetPosition", "targetVelocity", "targetActivity" });

      createInstructionBox();
      setInstructions ("Select three points for the plane");
      createOptionPanel ("Cancel");
   }

   protected void createAndAddMarker (Point3d pnt, Frame frame) {
      FrameMarker marker = new FrameMarker (pnt);
      marker.setFrame (frame);
      setProperties (marker, getPrototypeComponent (myComponentType));
      // update properties in the prototype as well ...
      setProperties (myPrototype, myPrototype);

      myMarkerList.add (marker);
      // addComponent (new AddComponentsCommand (
      //    "add FrameMarker", marker, myMarkerList));

      if (myMarkerList.size() == 3) {
         Point3d[] pnts = new Point3d[3];
         for (int i=0; i<3; i++) {
            pnts[i] = myMarkerList.get(i).getPosition();
         }
         OcclusalPlane oplane =
            myImplantsManager.createOcclusalPlane (pnts[0], pnts[1], pnts[2]);
         myMeshManager.addOcclusalPlane (oplane);
         myModel.clearFrameMarkers();
         myDisplay.setVisible (false);
         dispose();
         myMain.rerender();
      }
      //setState (State.SelectingLocation);
   }

   protected void resetDefaultProperties() {
   }

   public void actionPerformed (ActionEvent e) {
      String cmd = e.getActionCommand();
      if (cmd.equals ("Cancel")) {
         myModel.clearFrameMarkers();
         myDisplay.setVisible (false);
         dispose();
      }
      super.actionPerformed (e);
   }
}
