package artisynth.VSP.reconstruction;

import java.io.File;

import artisynth.VSP.reconstruction.MeshManager.MarkerFormat;
import artisynth.core.gui.widgets.PanelFileChooser;
import maspack.util.GenericFileFilter;
import maspack.widgets.BooleanSelector;
import maspack.widgets.EnumSelector;
import maspack.widgets.PropertyPanel;

/**
 * Chooser for files containing marker information, with an extra panel for
 * selecting the file format.
 */
public class MarkerFileChooser extends PanelFileChooser {

   private static final long serialVersionUID = 1L;
   
   GenericFileFilter myTxtFilter;
   EnumSelector myFormatSelector;
   BooleanSelector myContainsAngleMarkers;

   protected void addFormatSelector (MeshManager.MarkerFormat fmt) {
      myFormatSelector =
         new EnumSelector ("Format:", fmt, MeshManager.MarkerFormat.values());
   }

   protected void addContainsAngleMarkers (boolean contains) {
      myContainsAngleMarkers =
         new BooleanSelector (
            "Contains angle markers:", contains);
   }

   public MarkerFileChooser (
      MeshManager.MarkerFormat fmt, boolean containsAngleMarkers) {
      addFormatSelector (fmt);
      addContainsAngleMarkers (containsAngleMarkers);
      build();
   }

   protected void build () {
      setApproveButtonText("Save");
      
      myTxtFilter = new GenericFileFilter (
         "txt", "Marker files (*.txt)");
      addChoosableFileFilter (myTxtFilter);
      setFileFilter (myTxtFilter);

      PropertyPanel panel = createPropertyPanel();
      panel.addWidget (myFormatSelector);
      panel.addWidget (myContainsAngleMarkers);
   }

   public MeshManager.MarkerFormat getFormat() {
      return (MeshManager.MarkerFormat)myFormatSelector.getValue();
   }

   public void setFormat (MeshManager.MarkerFormat fmt) {
      myFormatSelector.setValue(fmt);
   }

   public boolean getContainsAngleMarkers() {
      return (Boolean)myContainsAngleMarkers.getValue();
   }      

   public void setContainsAngleMarkers (boolean contains) {
      myContainsAngleMarkers.setValue(contains);
   }

   public boolean isTxtFilterSelected() {
      return getFileFilter() == myTxtFilter;
   }

   /**
    * Returns the selected file, appending a .txt extension if there is no
    * extension and the .txt filter is selected.
    */
   public File getSelectedFileWithExtension() {
      File file = getSelectedFile();
      if (file.getName().indexOf ('.') == -1 && isTxtFilterSelected()) {
         file = new File(file.getPath() + ".txt");
      }
      return file;
   }

}
