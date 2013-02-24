package de.unihannover.imes.imesyoubotshow.dummy;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ros.android.view.camera.RosCameraPreviewView;

import de.unihannover.imes.imesyoubotshow.rosDemoCaller;
import de.unihannover.imes.imesyoubotshow.R.drawable;

import android.R;
import android.widget.ImageView;

/**
 * Helper class for providing sample content for user interfaces created by
 * Android template wizards.
 * <p>
 * TODO: Replace all uses of this class before publishing your app.
 */
public class DummyContent {

	/**
	 * An array of sample (dummy) items.
	 */
	public static List<DummyItem> ITEMS = new ArrayList<DummyItem>();

	/**
	 * A map of sample (dummy) items, by ID.
	 */
	public static Map<String, DummyItem> ITEM_MAP = new HashMap<String, DummyItem>();

	static {
		// Add 3 sample items.
		addItem(new DummyItem("1", "Arm Motion Demo",  de.unihannover.imes.imesyoubotshow.R.drawable.omniwheel, de.unihannover.imes.imesyoubotshow.R.string.demo_armmotion));
		addItem(new DummyItem("2", "Base Motion Demo",de.unihannover.imes.imesyoubotshow.R.drawable.omniwheel, de.unihannover.imes.imesyoubotshow.R.string.demodescriptionplacer));
		addItem(new DummyItem("3", "Arm Base Motion Demo",de.unihannover.imes.imesyoubotshow.R.drawable.omniwheel, de.unihannover.imes.imesyoubotshow.R.string.demodescriptionplacer));
		addItem(new DummyItem("4", "Tracking Object HMI Demo", de.unihannover.imes.imesyoubotshow.R.drawable.omniwheel, de.unihannover.imes.imesyoubotshow.R.string.demodescriptionplacer));
		addItem(new DummyItem("5", "Simple Detection Arm Demo ", de.unihannover.imes.imesyoubotshow.R.drawable.omniwheel, de.unihannover.imes.imesyoubotshow.R.string.demodescriptionplacer));
		addItem(new DummyItem("6", "Tower of Hanoi", de.unihannover.imes.imesyoubotshow.R.drawable.omniwheel, de.unihannover.imes.imesyoubotshow.R.string.demodescriptionplacer));
		addItem(new DummyItem("7", "Tablet Control Demo", de.unihannover.imes.imesyoubotshow.R.drawable.omniwheel, de.unihannover.imes.imesyoubotshow.R.string.demodescriptionplacer));
		addItem(new DummyItem("8", "Kinect Control HMI Demo", de.unihannover.imes.imesyoubotshow.R.drawable.omniwheel, de.unihannover.imes.imesyoubotshow.R.string.demodescriptionplacer));
		
	}

	private static void addItem(DummyItem item) {
		ITEMS.add(item);
		ITEM_MAP.put(item.id, item);
	}

	/**
	 * A dummy item representing a piece of content.
	 */
	public static class DummyItem {
		public String id;
		public String content;
		public int description;
		public int resourceID;
		
		public DummyItem(String id, String content, int resourceId, int desc) {
			this.id = id;
			this.content = content;
			description = desc;
	    	this.resourceID = resourceId;
	   
	    	}
	    	
	    	
		

		@Override
		public String toString() {
			return content;
		}
	}
}
