package de.unihannover.imes.imesyoubotshow;


import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import android.hardware.Camera;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;

public class rosDemoCaller extends RosActivity {
	
		  
	 public rosDemoCaller() {
	    super("androidDemoCaller", "androidDemoCaller");
	  }
	 @Override
	  protected void onCreate(Bundle savedInstanceState) {
	    super.onCreate(savedInstanceState);
	    
	    	
	   
	  }

	  @Override
	  public boolean onTouchEvent(MotionEvent event) {
	   
	    return true;
	  }

	  @Override
	  protected void init(NodeMainExecutor nodeMainExecutor) {
	    
	    NodeConfiguration nodeConfiguration =
	        NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
	    nodeConfiguration.setMasterUri(getMasterUri());
	 
	  }

}
