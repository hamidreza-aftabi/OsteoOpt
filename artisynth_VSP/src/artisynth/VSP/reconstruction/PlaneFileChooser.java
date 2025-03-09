package artisynth.VSP.reconstruction;

import java.io.File;

import artisynth.VSP.reconstruction.MeshManager.PlaneFormat;
import artisynth.core.gui.widgets.PanelFileChooser;
import maspack.util.GenericFileFilter;
import maspack.widgets.EnumSelector;
import maspack.widgets.PropertyPanel;

/**
 * Chooser for files containing plane information, with an extra
 * panel for selecting the file format.
 */
public class PlaneFileChooser extends PanelFileChooser {

   private static final long serialVersionUID = 1L;
   
   GenericFileFilter myTxtFilter;
   EnumSelector myFormatSelector;

   protected void addFormatSelector (MeshManager.PlaneFormat fmt) {
      myFormatSelector =
         new EnumSelector ("Format:", fmt, MeshManager.PlaneFormat.values());
   }

   public PlaneFileChooser (MeshManager.PlaneFormat fmt) {
      addFormatSelector (fmt);
      build();
   }

   protected void build () {
      setApproveButtonText("Save");
      
      myTxtFilter = new GenericFileFilter (
         "txt", "Plane files (*.txt)");
      addChoosableFileFilter (myTxtFilter);
      setFileFilter (myTxtFilter);

      PropertyPanel panel = createPropertyPanel();
      panel.addWidget (myFormatSelector);
   }

   public MeshManager.PlaneFormat getFormat() {
      return (MeshManager.PlaneFormat)myFormatSelector.getValue();
   }

   public void setFormat (MeshManager.PlaneFormat fmt) {
      myFormatSelector.setValue(fmt);
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
